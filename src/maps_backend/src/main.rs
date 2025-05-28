use safe_drive::{
    context::Context, msg::common_interfaces::sensor_msgs, node::Node, topic::subscriber::Subscriber, qos::Profile
};

use std::sync::Arc;

use sensor_msgs::msg::NavSatFix;

const MAPS_NODE: &str = "maps_node";

#[tokio::main]
async fn main() {
    let ctx: Arc<Context> = Context::new().unwrap();
    let node: Arc<Node> = ctx
        .create_node(MAPS_NODE, Some("/"), Default::default())
        .unwrap();

    // Create subscriber
    let mut subscriber_sensor_qos = node
        .create_subscriber::<sensor_msgs::msg::NavSatFix>("/sensors/gps", Some(Profile::sensor_data()))
        .unwrap();
    
    loop {
        let maybe_message = subscriber_sensor_qos.recv().await;

        let message = match maybe_message {
            Ok(m) => m,
            Err(e) => {
                // handle error, for example:
                continue;
            },
        };
    }
}
