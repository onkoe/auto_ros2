//! Stuff related to sending things to the Rover.

// we need a type to store data about *where* to send info.

use std::net::{IpAddr, Ipv4Addr};

use tokio::net::UdpSocket;

use crate::{error::SendError, Arm, Led, Wheels};

/// An indicator of whether the request succeeded.
///
/// For more info on the error, see the wrapped [`reqwest::Error`].
///
/// WARNING: the microcontrollers will immediately change the existing values
/// to match what's sent over the network. In practice, this can result in some
/// weird behavior with the Rover! Be careful when sending these values.
type SendResult = Result<(), crate::error::SendError>;

/// Controls the Rover.
#[cfg_attr(feature = "python", pyo3::pyclass)]
pub struct RoverController {
    /// A socket to speak with the microcontroller that moves the Rover.
    socket: UdpSocket,
}

impl RoverController {
    /// Creates a new [`SendToRover`] with the given IP address and port.
    ///
    /// ## Example
    ///
    /// ```
    /// use feedback::prelude::*;
    /// use core::net::{IpAddr, Ipv4Addr};
    ///
    /// # #[tokio::main(flavor = "current_thread")]
    /// # async fn main() {
    /// // replace these with the actual ip/port of the microcontroller!
    /// //
    /// // the local port is what we bind to on the Orin.
    /// let (ebox_ip, ebox_port, local_port): (IpAddr, u16, u16) = (IpAddr::V4(Ipv4Addr::UNSPECIFIED), 1001, 6666);
    ///
    /// // this creates the controller. use the various `ctrl.send` methods to
    /// // use it properly!
    /// let ctrl: RoverController = futures_lite::future::block_on(
    ///         RoverController::new(ebox_ip, ebox_port, local_port)
    ///     )
    ///     .unwrap();
    /// # }
    /// ```
    #[tracing::instrument]
    pub async fn new(
        ebox_ip: IpAddr,
        ebox_port: u16,
        local_port: u16,
    ) -> Result<Self, std::io::Error> {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, local_port))
            .await
            .inspect_err(|e| tracing::warn!("Failed to bind to the local port! err: {e}"))
            .inspect(|_| tracing::debug!("Bound to port successfully."))?;

        // connect to the ebox
        socket
            .connect((ebox_ip, ebox_port))
            .await
            .inspect_err(|e| tracing::error!("Failed to connect to the ebox! err: {e}"))
            .inspect(|_| tracing::debug!("Connected to ebox successfully."))?;

        // socket was created successfully if we're still running!
        //
        // so... return a `Self`!
        Ok(Self { socket })
    }

    /// Attempts to send the given wheel speeds.
    #[tracing::instrument(skip(self))]
    pub async fn send_wheels(&self, wheels: &Wheels) -> SendResult {
        let mut message: [u8; 5] = [0x0; 5];

        // start with subsystem byte.
        // wheels is `0x01`, so...
        message[0] = Wheels::SUBSYSTEM_BYTE;

        // add the wheels part byte
        message[1] = Wheels::PART_BYTE;

        // each speed can be added directly...
        message[2] = wheels.left;
        message[3] = wheels.right;

        // add the checksum
        message[4] = wheels.checksum;

        // check the message validity
        crate::parse::parse(message.as_slice()).inspect_err(|e| {
            tracing::error!("Constructed message for the wheels was invalid! err: {e}")
        })?;
        tracing::debug!("Sending wheels message over UDP... {message:?}");

        // finally, we can send the message over UDP!
        //
        // FIXME: we should NOT be using UDP, as this can cause major errors in
        //        pathing!
        self.socket
            .send(&message)
            .await
            .inspect_err(|e| tracing::error!("Failed to send wheel speeds! err: {e}"))
            .inspect(|bytes_sent| tracing::debug!("Sent {bytes_sent} bytes!"))
            .map(|_bytes_sent| ())
            .map_err(SendError::SocketError)
    }

    /// Attempts to send the given light color.
    #[tracing::instrument(skip(self))]
    pub async fn send_led(&self, lights: &Led) -> SendResult {
        let mut message: [u8; 5] = [0x0; 5];

        // first, the subsystem byte
        message[0] = Led::SUBSYSTEM_BYTE;

        // then the part byte (lights)
        message[1] = Led::PART_BYTE;

        // now the three values - r, g, b!
        message[2] = lights.red;
        message[3] = lights.green;
        message[4] = lights.blue;

        // check the message validity
        crate::parse::parse(message.as_slice()).inspect_err(|e| {
            tracing::error!("Constructed message for the lights was invalid! err: {e}")
        })?;
        tracing::debug!("Sending lights message over UDP... {message:?}");

        // send the message
        self.socket
            .send(&message)
            .await
            .inspect_err(|e| tracing::error!("Failed to send light color! err: {e}"))
            .inspect(|bytes_sent| tracing::debug!("Sent {bytes_sent} bytes!"))
            .map(|_bytes_sent| ())
            .map_err(SendError::SocketError)
    }

    /// Attempts to send... all that arm stuff.
    #[tracing::instrument(skip(self))]
    pub async fn send_arm(&self, arm: &Arm) -> SendResult {
        let mut message: [u8; 8] = [0x0; 8];

        // subsystem
        // NOTE: there is no part byte for the arm!
        message[0] = Arm::SUBSYSTEM_BYTE;

        // now... add each field in order.
        message[1] = arm.bicep;
        message[2] = arm.forearm;
        message[3] = arm.base;
        message[4] = arm.wrist_pitch;
        message[5] = arm.wrist_roll;
        message[6] = arm.claw;

        // and the checksum! again, no clue if electrical uses this.
        //
        // just assuming it's correct.
        message[7] = arm.checksum;

        // check the message validity
        crate::parse::parse(message.as_slice()).inspect_err(|e| {
            tracing::error!("Constructed message for the arm was invalid! err: {e}")
        })?;
        tracing::debug!("Sending arm message over UDP... {message:?}");

        // send the message
        self.socket
            .send(&message)
            .await
            .inspect_err(|e| tracing::error!("Failed to send arm controls! err: {e}"))
            .inspect(|bytes_sent| tracing::debug!("Sent {bytes_sent} bytes!"))
            .map(|_bytes_sent| ())
            .map_err(SendError::SocketError)
    }

    // TODO: some helper fns for moving the rover (i.e. turning/etc.) might be
    // helpful in the future.
}

/// Handles the Python bindings.
#[cfg(feature = "python")]
mod python {
    use std::net::AddrParseError;

    use pyo3::{exceptions::PyException, prelude::*};

    use crate::{error::SendException, Arm, Led, Wheels};

    use super::RoverController;

    pyo3::create_exception!(error, IpParseException, PyException);
    pyo3::create_exception!(error, SocketConnectionException, PyException);

    #[pymethods]
    impl RoverController {
        /// Creates a new [`RoverController`].
        #[new]
        pub fn py_new(ebox_ip: String, ebox_port: u16, local_port: u16) -> PyResult<Self> {
            let addr = ebox_ip
                .parse()
                .inspect_err(|e| {
                    tracing::warn!("Failed to parse the given `ip` as an IP address! err: {e}")
                })
                .map_err(|e: AddrParseError| IpParseException::new_err(e.to_string()))?;

            futures_lite::future::block_on(Self::new(addr, ebox_port, local_port))
                .map_err(|e| SocketConnectionException::new_err(e.to_string()))
        }

        /// Attempts to send the given wheel speeds.
        #[pyo3(name = "send_wheels")]
        pub fn py_send_wheels(&self, wheels: Wheels) -> PyResult<()> {
            futures_lite::future::block_on(self.send_wheels(&wheels))
                .map_err(|e| SendException::new_err(e.to_string()))
        }

        /// Attempts to send the given light color.
        #[pyo3(name = "send_led")]
        pub fn py_send_led(&self, led: Led) -> PyResult<()> {
            futures_lite::future::block_on(self.send_led(&led))
                .map_err(|e| SendException::new_err(e.to_string()))
        }

        /// Attempts to send... all that arm stuff.
        #[pyo3(name = "send_arm")]
        pub fn py_send_arm(&self, arm: Arm) -> PyResult<()> {
            futures_lite::future::block_on(self.send_arm(&arm))
                .map_err(|e| SendException::new_err(e.to_string()))
        }
    }

    #[pymodule(submodule)]
    fn send(m: &Bound<'_, PyModule>) -> PyResult<()> {
        // add the rover controller
        m.add_class::<RoverController>()?;

        // and the exceptions here
        m.add("IpParseException", m.py().get_type::<IpParseException>())?;
        m.add(
            "SocketConnectionException",
            m.py().get_type::<SocketConnectionException>(),
        )?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::{
        net::Ipv4Addr,
        time::{Duration, Instant},
    };

    use super::RoverController;
    use crate::Led;

    #[tokio::test]
    async fn stuff_is_sent() {
        tracing_subscriber::fmt()
            .with_max_level(tracing::Level::DEBUG)
            .init();

        let controller = RoverController::new(Ipv4Addr::LOCALHOST.into(), 5003, 6666)
            .await
            .unwrap();

        // constantly send lights on background thread
        tokio::task::spawn(async move {
            let controller = controller;

            let lights = Led {
                red: 255,
                green: 0,
                blue: 0,
            };

            // send that shi forever
            loop {
                controller.send_led(&lights).await.unwrap();
                tracing::debug!("Sent lights.");
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
        });

        let mut buf = vec![0x0; 32];
        let start_time: Instant = Instant::now();
        let recvr_socket = tokio::net::UdpSocket::bind((Ipv4Addr::LOCALHOST, 5003))
            .await
            .unwrap();

        // try for 10s to get at least one message.
        //
        // early-return if we do get one to avoid the panic.
        while start_time.elapsed() < Duration::from_secs(10) {
            recvr_socket.recv(&mut buf).await.unwrap();

            if !buf.is_empty() {
                println!("oh hey, got some bytes: {buf:#?}");
                return;
            }
        }

        // we shoulda returned by now! so panic if the test makes it here.
        panic!("stuff wasn't sent! we ran outta time (10s).");
    }
}
