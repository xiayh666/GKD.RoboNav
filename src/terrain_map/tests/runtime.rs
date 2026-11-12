use terrain_map::*;

fn frame(time: f64) -> Frame {
    Frame {
        time,
        center: [0.; 3],
        rays: vec![Ray {
            origin: [0.05; 3],
            endpoint: [1.05, 0.05, 0.05],
        }],
    }
}

// 不创建隐藏runtime，库应显式报告调用上下文错误。
#[test]
fn start_requires_caller_runtime() {
    assert!(matches!(
        TerrainRuntime::start(Config::default()),
        Err(Error::NoRuntime)
    ));
}

// 单线程执行器仍可处理计算请求；快照和回执必须使用同一提交版本。
#[tokio::test(flavor = "current_thread")]
async fn async_commit_annotation_and_shutdown() {
    let runtime = TerrainRuntime::start(Config::default()).unwrap();
    let handle = runtime.handle();
    let mut changes = handle.subscribe();
    let out = handle
        .submit(frame(1.))
        .await
        .unwrap()
        .wait()
        .await
        .unwrap();
    changes.changed().await.unwrap();
    assert_eq!(changes.borrow_and_update().revision, out.revision);
    let annotation = handle
        .upsert_annotation(Annotation {
            id: 1,
            min: [-1.; 3],
            max: [1.; 3],
            kind: AnnotationKind::Forbidden,
        })
        .await
        .unwrap();
    assert_eq!(annotation.map.version, 1);
    assert_eq!(annotation.annotation_version, 1);
    assert_eq!(handle.latest().revision, 2);
    assert!(matches!(
        handle.submit(frame(1.)).await.unwrap().wait().await,
        Err(Error::NonMonotonicTime)
    ));
    assert_eq!(handle.latest().revision, 2);
    handle.remove_annotation(1).await.unwrap();
    runtime.shutdown().await.unwrap();
    assert_eq!(
        handle.try_submit(frame(2.)).err().unwrap().error,
        Error::Closed
    );
}

// current_thread中无await时消费者不会抢跑，容量满必须返还原始帧供调用方决定。
#[tokio::test(flavor = "current_thread")]
async fn bounded_admission_returns_input_and_shutdown_drains() {
    let runtime = TerrainRuntime::start(Config {
        input_capacity: 1,
        ..Config::default()
    })
    .unwrap();
    let handle = runtime.handle();
    let accepted = handle.try_submit(frame(1.)).unwrap();
    let rejected = handle.try_submit(frame(2.)).err().unwrap();
    assert_eq!(rejected.error, Error::Busy);
    assert_eq!(rejected.frame.time, 2.);
    runtime.shutdown().await.unwrap();
    assert_eq!(accepted.wait().await.unwrap().map.version, 1);
    assert_eq!(handle.latest().map.version, 1);
}

// 丢弃回执不撤销已接收帧，watch可跳版本但必须得到完整最新状态。
#[tokio::test(flavor = "current_thread")]
async fn dropped_receipts_and_slow_subscribers_do_not_lose_commits() {
    let runtime = TerrainRuntime::start(Config {
        input_capacity: 4,
        ..Config::default()
    })
    .unwrap();
    let handle = runtime.handle();
    let mut changes = handle.subscribe();
    for i in 1..=4 {
        drop(handle.try_submit(frame(f64::from(i))).unwrap());
    }
    runtime.shutdown().await.unwrap();
    changes.changed().await.unwrap();
    let latest = changes.borrow_and_update().clone();
    assert_eq!(latest.map.version, 4);
    assert_eq!(
        latest.map.occupancy([1.05, 0.05, 0.05]),
        Occupancy::Occupied
    );
    assert!(changes.changed().await.is_err());
}
