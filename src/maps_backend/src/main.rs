use safe_drive::{
    context::Context, msg::common_interfaces::sensor_msgs, node::Node, qos::Profile,
};
use sensor_msgs::msg::NavSatFix;

use futures_util::{SinkExt, StreamExt};
use std::{sync::Arc};
use tokio::{
    net::TcpListener,
    sync::broadcast,
};
use tokio_tungstenite::accept_async;
use tungstenite::Message;

const MAPS_NODE: &str = "maps_node";

#[tokio::main]
async fn main() {
    // Set up ROS2 context and node
    let ctx: Arc<Context> = Context::new().unwrap();
    let node: Arc<Node> = ctx
        .create_node(MAPS_NODE, Some("/"), Default::default())
        .unwrap();

    // Set up GPS subscriber
    let mut gps_subscriber = node
        .create_subscriber::<NavSatFix>("/sensors/gps", Some(Profile::sensor_data()))
        .unwrap();

    // Set up broadcast channel to share GPS data with WebSocket clients
    let (tx, _rx) = broadcast::channel::<String>(16); // 16 is the buffer size

    // Spawn WebSocket server in background
    tokio::spawn(start_websocket_server(tx.clone()));

    // Main loop: receive GPS data and broadcast it
    loop {
        if let Ok(msg) = gps_subscriber.recv().await {
            let lat = msg.latitude;
            let lon = msg.longitude;
            let json = format!(r#"{{"lat": {}, "lon": {}}}"#, lat, lon);
            let _ = tx.send(json); // Ignore errors if no one is listening
        } else {
            eprintln!("❌ Failed to receive GPS data");
        }
    }
}

/// Handles WebSocket connections and broadcasts GPS data to each connected client.
async fn start_websocket_server(tx: broadcast::Sender<String>) {
    let listener = TcpListener::bind("127.0.0.1:9001").await.unwrap();
    println!("✅ WebSocket server running at ws://localhost:9001");

    loop {
        let (stream, _) = listener.accept().await.unwrap();
        let mut rx = tx.subscribe();
        
        let ws_stream = accept_async(stream).await.unwrap();
        let (mut write, _) = ws_stream.split();

        while let Ok(gps_data) = rx.recv().await {
            if write.send(Message::Text(gps_data.clone())).await.is_err() {
                eprintln!("❌ Client disconnected");
                break;
            }
        }
    }
}
