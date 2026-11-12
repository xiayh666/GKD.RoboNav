use crate::{Annotation, Config, Error, Frame, MapSnapshot, Mapper};
use std::sync::Arc;
use tokio::sync::{mpsc, oneshot, watch};
use tokio::task::JoinHandle;

type Reply = oneshot::Sender<Result<Arc<MapSnapshot>, Error>>;
enum Operation {
    Frame(Frame),
    Upsert(Annotation),
    Remove(u64),
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
    pub async fn upsert_annotation(
        &self,
        annotation: Annotation,
    ) -> Result<Arc<MapSnapshot>, Error> {
        annotation.validate()?;
        self.request(Operation::Upsert(annotation)).await
    }
    pub async fn remove_annotation(&self, id: u64) -> Result<Arc<MapSnapshot>, Error> {
        self.request(Operation::Remove(id)).await
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
                        let (returned, result) = tokio::task::spawn_blocking(move || {
                            let result = match operation {
                                Operation::Frame(frame) => mapper.update(&frame),
                                Operation::Upsert(a) => mapper.upsert_annotation(a),
                                Operation::Remove(id) => mapper.remove_annotation(id),
                            };
                            (mapper, result)
                        })
                        .await
                        .map_err(|e| Error::Worker(e.to_string()))?;
                        mapper = returned;
                        if let Ok(snapshot) = &result {
                            output.send_replace(Arc::clone(snapshot));
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
