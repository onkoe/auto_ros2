//! # `lights_node`
//!
//! The lights node provides a service so that the client can make a request to turn the lights a given color.
//! The request information contains values representing:
//!
//! - RED
//! - GREEN
//! - BLUE
//!
//! and a boolean for FLASHING.

use std::{
    net::{IpAddr, Ipv4Addr},
    sync::Arc,
};

use custom_interfaces::srv::Lights;

use feedback::{prelude::RoverController, Led};

use ros2_client::Server;
use safe_drive::{
    context::Context,
    logger::Logger,
    node::{Node, NodeOptions},
    pr_error, pr_info, qos,
};

const LIGHTS_NODE_NAME: &str = "lights_node";

const LIGHTS_SERVICE_NAME: &str = "lights_service";

#[tokio::main(flavor = "multi_thread")]
async fn main() {
    // Initialize ros 2 context
    let ctx: Arc<Context> = Context::new().expect("init ros 2 context");

    // Create the logger
    let logger: Arc<Logger> = Arc::new(Logger::new(LIGHTS_NODE_NAME));

    // Create the lights node
    let mut node: Arc<Node> = ctx
        .create_node(LIGHTS_NODE_NAME, None, NodeOptions::new())
        .inspect_err(|e| pr_error!(logger, "Failed to create node: {e}"))
        .inspect(|_| {
            pr_info!(logger, "The `{LIGHTS_NODE_NAME}` has been created.");
        })
        .expect("create node");

    // Make the service server
    let server: Server<Lights> = node
        .create_server(LIGHTS_SERVICE_NAME, Some(qos::Profile::services_default()))
        .expect("create server");
    pr_info!(logger, "Server created! Starting request watcher...");

    // Info for the controller to use
    let ebox_ipaddr = IpAddr::V4(Ipv4Addr::new(192, 168, 1, 102));
    let ebox_port = 5003;
    let local_port = 6666; // we bind to this port

    // New instance of RoverController type, this should probably be a global
    // thing. Need ip address and port number
    let controller = RoverController::new(ebox_ipaddr, ebox_port, local_port)
        .await
        .expect("Failed to create Rover Controller");

    // Handle requests in the background
    let next = tokio::select! {
        _ = tokio::signal::ctrl_c() => (),
        _ = service_server_task(server, controller, Arc::clone(&logger)) => (),
    };
}

async fn service_server_task(
    server: Server<Lights>,
    controller: RoverController,
    logger: Arc<Logger>,
) {
    // wait for requests
    loop {
        let maybe_request = server.async_receive_request().await;

        let (id, request) = match maybe_request {
            Ok(a) => a,
            Err(e) => {
                pr_error!(logger, "Failed to get message from request! err: {e}");
                continue;
            }
        };

        // TODO: handle lights request
        let led = Led {
            red: 255,
            green: 0,
            blue: 0,
        };

        // try sending the LED color to the microcontroller
        if let Err(e) = controller.send_led(&led).await {
            pr_error!(logger, "Failed to send light color to Ebox! err: {e}");
        }

        // This is what is sent back to the client (returns true if successful, false if not)
        // let mut response = LightsResponse { success: false };

        // send a response back based on how that went
        let response = Lights::Response::new();
        server.async_send_response(id, response);
    }
}

// #[cfg(test)]
// mod tests;
