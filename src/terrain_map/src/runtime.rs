use crate::{Annotation, Config, Error, Frame, MapSnapshot, Mapper};
use map_updates::{MapUpdate, SessionId};
use std::sync::Arc;
use tokio::sync::{mpsc, oneshot, watch};
use tokio::task::JoinHandle;

type Reply = oneshot::Sender<Result<Arc<MapSnapshot>, Error>>;
enum Operation {
    Frame(Frame),
    Update(MapUpdate),
    Upsert(Option<SessionId>, Annotation),
    Remove(Option<SessionId>, u64),
}
enum Command {
    Apply(Operation, Reply),
    Shutdown,
}

/// An accepted request continues even if this receipt is dropped.
pub struct Receipt(oneshot::Receiver<Result<Arc<MapSnapshot>, Error>>);
impl Receipt {
    pub async fn wait(self) -> Result<Arc<MapSnapshot>, Error> {
        self.0.await.map_err(|_| Error::Closed)?
    }
}

#[derive(Debug)]
pub struct RejectedFrame {
    pub error: Error,
    pub frame: Frame,
}

#[derive(Debug)]
pub struct RejectedUpdate {
    pub error: Error,
    pub update: MapUpdate,
}

#[derive(Clone)]
pub struct MapHandle {
    tx: mpsc::Sender<Command>,
    latest: watch::Receiver<Arc<MapSnapshot>>,
    max_points: usize,
}
impl MapHandle {
    pub fn subscribe(&self) -> watch::Receiver<Arc<MapSnapshot>> {
        self.latest.clone()
    }
    pub fn latest(&self) -> Arc<MapSnapshot> {
        Arc::clone(&self.latest.borrow())
    }
    /// Admission is nonblocking; a rejected input is returned for retry/drop policy.
    pub fn try_submit(&self, frame: Frame) -> Result<Receipt, RejectedFrame> {
        if frame.rays.len() > self.max_points {
            return Err(RejectedFrame {
                error: Error::InvalidFrame,
                frame,
            });
        }
        match self.tx.try_reserve() {
            Ok(permit) => {
                let (tx, rx) = oneshot::channel();
                permit.send(Command::Apply(Operation::Frame(frame), tx));
                Ok(Receipt(rx))
            }
            Err(e) => Err(RejectedFrame {
                error: if matches!(e, mpsc::error::TrySendError::Full(_)) {
                    Error::Busy
                } else {
                    Error::Closed
                },
                frame,
            }),
        }
    }
    /// Wait for queue capacity. Receipt acknowledgement reports commit/rejection.
    pub async fn submit(&self, frame: Frame) -> Result<Receipt, RejectedFrame> {
        if frame.rays.len() > self.max_points {
            return Err(RejectedFrame {
                error: Error::InvalidFrame,
                frame,
            });
        }
        match self.tx.reserve().await {
            Ok(permit) => {
                let (tx, rx) = oneshot::channel();
                permit.send(Command::Apply(Operation::Frame(frame), tx));
                Ok(Receipt(rx))
            }
            Err(_) => Err(RejectedFrame {
                error: Error::Closed,
                frame,
            }),
        }
    }
    /// Main producer path. Arc-backed observation payloads move without cloud copies.
    pub fn try_submit_update(&self, update: MapUpdate) -> Result<Receipt, RejectedUpdate> {
        if matches!(&update,MapUpdate::Observations(b) if b.point_count()>self.max_points) {
            return Err(RejectedUpdate {
                error: Error::Capacity,
                update,
            });
        }
        match self.tx.try_reserve() {
            Ok(permit) => {
                let (tx, rx) = oneshot::channel();
                permit.send(Command::Apply(Operation::Update(update), tx));
                Ok(Receipt(rx))
            }
            Err(e) => Err(RejectedUpdate {
                error: if matches!(e, mpsc::error::TrySendError::Full(_)) {
                    Error::Busy
                } else {
                    Error::Closed
                },
                update,
            }),
        }
    }
    pub async fn submit_update(&self, update: MapUpdate) -> Result<Receipt, RejectedUpdate> {
        if matches!(&update,MapUpdate::Observations(b) if b.point_count()>self.max_points) {
            return Err(RejectedUpdate {
                error: Error::Capacity,
                update,
            });
        }
        match self.tx.reserve().await {
            Ok(permit) => {
                let (tx, rx) = oneshot::channel();
                permit.send(Command::Apply(Operation::Update(update), tx));
                Ok(Receipt(rx))
            }
            Err(_) => Err(RejectedUpdate {
                error: Error::Closed,
                update,
            }),
        }
    }
    pub async fn upsert_annotation(
        &self,
        annotation: Annotation,
    ) -> Result<Arc<MapSnapshot>, Error> {
        annotation.validate()?;
        let session = self.latest().surfaces.session;
        self.request(Operation::Upsert(session, annotation)).await
    }
    pub async fn remove_annotation(&self, id: u64) -> Result<Arc<MapSnapshot>, Error> {
        let session = self.latest().surfaces.session;
        self.request(Operation::Remove(session, id)).await
    }
    /// Bind a UI command to the coordinate session of the displayed snapshot.
    pub async fn upsert_annotation_in_session(
        &self,
        session: SessionId,
        annotation: Annotation,
    ) -> Result<Arc<MapSnapshot>, Error> {
        annotation.validate()?;
        self.request(Operation::Upsert(Some(session), annotation))
            .await
    }
    pub async fn remove_annotation_in_session(
        &self,
        session: SessionId,
        id: u64,
    ) -> Result<Arc<MapSnapshot>, Error> {
        self.request(Operation::Remove(Some(session), id)).await
    }
    async fn request(&self, operation: Operation) -> Result<Arc<MapSnapshot>, Error> {
        let (tx, rx) = oneshot::channel();
        self.tx
            .send(Command::Apply(operation, tx))
            .await
            .map_err(|_| Error::Closed)?;
        Receipt(rx).wait().await
    }
}

/// Uses the caller's Tokio runtime. At most one blocking computation is in flight.
/// Explicit shutdown closes admission, drains accepted commands and joins the worker.
/// Dropping this owner alone does not cancel handles held elsewhere.
pub struct TerrainRuntime {
    handle: MapHandle,
    task: JoinHandle<Result<(), Error>>,
}
impl TerrainRuntime {
    pub fn start(config: Config) -> Result<Self, Error> {
        tokio::runtime::Handle::try_current().map_err(|_| Error::NoRuntime)?;
        let mapper = Mapper::new(config.clone())?;
        let initial = mapper.snapshot();
        let (tx, mut rx) = mpsc::channel(config.input_capacity);
        let (output, latest) = watch::channel(initial);
        let handle = MapHandle {
            tx,
            latest,
            max_points: config.max_points,
        };
        let task = tokio::spawn(async move {
            let mut mapper = mapper;
            while let Some(command) = rx.recv().await {
                match command {
                    Command::Shutdown => rx.close(),
                    Command::Apply(operation, reply) => {
                        let (returned, result, published) =
                            tokio::task::spawn_blocking(move || {
                                let result = match operation {
                                    Operation::Frame(frame) => mapper.update(&frame),
                                    Operation::Update(update) => mapper.apply_update(&update),
                                    Operation::Upsert(session, a) => {
                                        mapper.upsert_annotation_for(session, a)
                                    }
                                    Operation::Remove(session, id) => {
                                        mapper.remove_annotation_for(session, id)
                                    }
                                };
                                let published = match &result {
                                    Ok(snapshot) => Some(Arc::clone(snapshot)),
                                    Err(Error::SequenceGap { .. }) => Some(mapper.snapshot()),
                                    _ => None,
                                };
                                (mapper, result, published)
                            })
                            .await
                            .map_err(|e| Error::Worker(e.to_string()))?;
                        mapper = returned;
                        if let Some(snapshot) = published {
                            output.send_replace(snapshot);
                        }
                        let _ = reply.send(result);
                    }
                }
            }
            Ok(())
        });
        Ok(Self { handle, task })
    }
    pub fn handle(&self) -> MapHandle {
        self.handle.clone()
    }
    pub async fn shutdown(self) -> Result<(), Error> {
        // If the worker already failed, preserve its JoinError rather than only Closed.
        let _ = self.handle.tx.send(Command::Shutdown).await;
        self.task.await.map_err(|e| Error::Worker(e.to_string()))?
    }
}
