//! # `soro_gps`
//!
//! Connects to the GPS and provides its data in a readable format.
//!
//! ## Usage (Rust)
//!
//! In short, you can use the various methods on `Gps` to interact with the Rover's GPS system.
//!
//! ```no_run
//! # use core::net::IpAddr;
//! use soro_gps::Gps;
//!
//! // we can make a GPS from a given IP address and port.
//! //
//! // on the Rover, this is from the router (hopefully a static IP) and a port
//! let swift_ip: IpAddr = "192.168.1.222".parse().unwrap();
//! let swift_port: u16 = 55556;
//!
//! // here, we make the GPS! this will automatically initialize it.
//! let gps: Gps = Gps::new(swift_ip, swift_port).unwrap();
//!
//! // now, you can use its methods:
//! println!("coord: {:?}", gps.coord());
//! println!("height: {:?}", gps.height());
//! println!("error in mm: {:?}", gps.error());
//!
//! // dropping it (when it falls from scope) automatically runs the required cleanup.
//! ```
//!
//! ## Development
//!
//! To make changes, [install Rust](https://rustup.rs).
//!
//! ### Testing
//!
//! You'll want to run the tests before making releases. It's good to test everything concurrently, so consider using `cargo nextest` if you have it installed.

use core::net::{IpAddr, Ipv4Addr};
use std::time::{Duration, Instant};

use error::{GpsConnectionError, GpsReadError};
use tokio::net::UdpSocket;

pub mod error;

use sbp::messages::navigation::msg_pos_llh_cov::FixMode;

/// The best possible update time for the GPS.
///
/// This is 1/20th of a second.
pub const BEST_UPDATE_TIME: Duration = Duration::from_millis(1000 / 20);

/// A safe wrapper for the low-level GPS functions.
#[derive(Debug)]
pub struct Gps {
    /// A UDP socket to speak with the GPS directly.
    ///
    /// This allows us to get frames from the GPS.
    socket: UdpSocket,

    buf: [u8; 1024],
}

impl Gps {
    /// Attempts to initialize the GPS.
    ///
    /// ## Errors
    ///
    /// This constructor can fail if the GPS device is not available on the
    /// provided IP address and port.
    #[tracing::instrument]
    pub async fn new(
        gps_ip: IpAddr,
        gps_port: u16,
        socket_port: u16,
    ) -> Result<Self, GpsConnectionError> {
        // bind the socket to a local port
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 54555))
            .await
            .inspect_err(|e| {
                tracing::error!(
                    "Failed to bind to the given socket \
                        port (i.e. the port we talk to the GPS from). There may be another \
                        service on this port. err: {e}"
                );
            })?;

        // connect to the gps and check its file descriptor
        // socket.connect((gps_ip, gps_port)).await.inspect_err(|e| tracing::warn!("Failed to connect to the given IP and port. The GPS may not be accessible. err: {e}"))?;
        tracing::debug!("Connected to GPS. (ip: {gps_ip}, gps port: {gps_port})");

        Ok(Self {
            socket,
            buf: [0; 1024],
        })
    }

    /// Attempts to get the required GPS message.
    ///
    /// This can run forever! Run it on a background task if that won't work
    /// for your usage.
    #[tracing::instrument(skip(self))]
    pub async fn get(&mut self) -> Result<GpsInfo, GpsReadError> {
        tracing::trace!(
            "Attempting to get a GPS message. If this message \
            shows up without progress, no messages are being recv'd."
        );

        // grab data until we have the correct message types
        let mut last_time_got_data: Instant = Instant::now();
        let mut last_time_warned: Instant = Instant::now();
        loop {
            // if we haven't collected data in a while, we'll warn the user!
            if last_time_got_data.elapsed() > Duration::from_millis(1500)
                && last_time_warned.elapsed() > Duration::from_millis(1500)
            {
                tracing::warn!(
                    "Haven't heard from the GPS in {:.2} seconds!",
                    last_time_got_data.elapsed().as_secs_f32()
                );
                last_time_warned = Instant::now();
            }

            self.buf = [0; 1024];

            let (bytes_recvd, remote_addr) = self
                .socket
                .recv_from(&mut self.buf)
                .await
                .inspect_err(|e| tracing::error!("Failed to recv from GPS socket! err: {e}"))
                .inspect(|r| tracing::trace!("Got a message from the GPS! bytes: {}", r.0))
                .map_err(|_| GpsReadError::ReadFailed)?;

            // remember that we got some data
            tracing::trace!("got {} bytes from ip: {}", bytes_recvd, remote_addr);
            last_time_got_data = Instant::now();

            // try to parse whatever we got
            let all_parsed = sbp::iter_messages(self.buf.as_slice());

            for msg in all_parsed {
                // this might be the message we want, but we gotta check (below)
                let maybe_good_msg = match msg {
                    Ok(frame) => frame,
                    Err(e) => {
                        tracing::warn!("Failed to parse this message! err: {e}");
                        return Err(GpsReadError::ParseFailed(e));
                    }
                };

                // we'll take any of the following gps messages:
                //
                // - MsgPosLlh
                // - MsgPosLlhCov
                match maybe_good_msg {
                    // if we have covariances, we can just use those!
                    sbp::Sbp::MsgPosLlhCov(_) => {
                        if let Some(info) = parsing::msg_pos_llh_cov(&maybe_good_msg) {
                            return Ok(info);
                        }
                    }

                    // without covariances, we'll just say we don't have them
                    sbp::Sbp::MsgPosLlh(_) => {
                        if let Some(info) = parsing::msg_pos_llh(&maybe_good_msg) {
                            return Ok(info);
                        }
                    }

                    // other variants are untested.
                    //
                    // TODO: add additonal fallbacks if needed!
                    _ => (),
                };

                // TODO: use second Swift on Base Station to get better heading
                // data.
                //match maybe_good_msg { ... }

                // TODO: add (e.g.) heartbeat messages and other status info.
                //
                // publishing onto various `/sensors/gps/**` topics could
                // assist us in debugging far into the future ;D
                //
                //match maybe_good_msg { ... }
            }
        }
    }
}

/// Information from the GPS.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct GpsInfo {
    pub coord: Coordinate,
    pub height: Height,
    pub tow: TimeOfWeek,
    pub covariance: [f64; 9],
    pub fix_status: FixStatus,
}

impl core::fmt::Display for GpsInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "GpsInfo {{ coord(lat: {}, lon: {}), height: {} m above wsg84, tow: {} ms }}",
            self.coord.lat, self.coord.lon, self.height.0, self.tow.0
        )
    }
}

/// Some coordinate returned from the GPS.
///
/// The returned values are in degrees.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Coordinate {
    pub lat: f64,
    pub lon: f64,
}

/// Height, with the unit being "meters above a WGS84 ellipsoid".
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Height(pub f64);

/// Error in millimeters. Inside the `gps` library, this is calculated as the
/// average of the vertical and horizontal 'error'.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct ErrorInMm(pub f64);

/// The GPS time of week, given from the satellite. This is measured in
/// milliseconds since the start of this GNSS week.
///
/// For additional information, see
/// [the NOAA documentation](https://www.ngs.noaa.gov/CORS/Gpscal.shtml).
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct TimeOfWeek(pub u32);

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub enum FixStatus {
    Invalid,
    SinglePoint,
    Sbas,
    GroundBased,
}

/// Everything in this module is related to parsing the GPS messages into
/// something useful for maintainers of the `sensors::sensors_node`.
///
/// ## Why are there so many parsers?
///
/// Despite being so similar, the Swift GPS reports these as different message
/// types, so we have to parse each individually, with respect to each
/// variant's special properties.
///
/// We need to check for all of them due to our need for a fallback. At URC
/// 2025, we noticed that our GPS was only providing `MsgPosLlh`, despite
/// getting `MsgPosLlhCov` back in Oklahoma. By parsing both, we can be more
/// certain that we're parsing useful messages if we're receiving them - even
/// if they don't contain _everything_ we'd want... in a perfect world.
///
/// tl;dr: Swift has a lotta different messages, and they change depending on
/// time, region, and luck.
mod parsing {
    use crate::{Coordinate, FixStatus, GpsInfo, Height, TimeOfWeek};

    /// Parses from a `MsgPosLlhCov`.
    ///
    /// This is our preferred message types...
    #[tracing::instrument]
    pub fn msg_pos_llh_cov(raw_msg: &sbp::Sbp) -> Option<GpsInfo> {
        // ensure we got the right message type
        let sbp::Sbp::MsgPosLlhCov(m) = raw_msg else {
            tracing::error!("Provided wrong message type!");
            return None;
        };

        // grab the fix mode
        let fix_mode = match m.fix_mode() {
            Ok(fm) => fm,
            Err(e) => {
                tracing::error!("Failed to get fix mode from SwiftNav `MsgPosLlh`! err code: {e}");
                return None;
            }
        };

        // and convert that into a fix status
        use sbp::messages::navigation::msg_pos_llh_cov::FixMode;
        let fix_status = match fix_mode {
            FixMode::Invalid | FixMode::DeadReckoning => FixStatus::Invalid,
            FixMode::SinglePointPosition => FixStatus::SinglePoint,
            FixMode::SbasPosition => FixStatus::Sbas,
            FixMode::DifferentialGnss | FixMode::FloatRtk | FixMode::FixedRtk => {
                FixStatus::GroundBased
            }
        };
        // create the covariance.
        //
        // note: ROS 2 wants `ENU`, but Swift provides `NED`.
        // we'll move things around and flip down -> up...
        let position_covariance: [f64; 9] = [
            // east
            m.cov_e_e as f64,
            m.cov_n_e as f64,
            -m.cov_e_d as f64,
            // north
            m.cov_n_e as f64,
            m.cov_n_n as f64,
            -m.cov_n_d as f64,
            // up
            -m.cov_e_d as f64,
            -m.cov_n_d as f64,
            m.cov_d_d as f64,
        ];

        // create info from the basic stuff
        let info = GpsInfo {
            coord: Coordinate {
                lat: m.lat,
                lon: m.lon,
            },
            height: Height(m.height),
            tow: TimeOfWeek(m.tow),
            fix_status,
            covariance: position_covariance,
        };

        return Some(info);
    }

    /// Parses from a `MsgPosLlh`, which doesn't have covariances. That means
    /// the GPS isn't reporting how sure it is about its answer!
    ///
    /// We prefer position messages that provide that info instead, but this is
    /// here as fallback.
    #[tracing::instrument]
    pub fn msg_pos_llh(raw_msg: &sbp::Sbp) -> Option<GpsInfo> {
        // ensure we got the right message type
        let sbp::Sbp::MsgPosLlh(m) = raw_msg else {
            tracing::error!("Provided wrong message type!");
            return None;
        };

        // grab the fix mode
        let fix_mode = match m.fix_mode() {
            Ok(fm) => fm,
            Err(e) => {
                tracing::error!("Failed to get fix mode from SwiftNav `MsgPosLlh`! err code: {e}");
                return None;
            }
        };

        // and convert that into a fix status
        use sbp::messages::navigation::msg_pos_llh::FixMode;
        let fix_status = match fix_mode {
            FixMode::Invalid | FixMode::DeadReckoning => FixStatus::Invalid,
            FixMode::SinglePointPosition => FixStatus::SinglePoint,
            FixMode::SbasPosition => FixStatus::Sbas,
            FixMode::DifferentialGnss | FixMode::FloatRtk | FixMode::FixedRtk => {
                FixStatus::GroundBased
            }
        };

        // create info from the basic stuff.
        //
        // we'll report `covariance` as all zeroes
        let info = GpsInfo {
            coord: Coordinate {
                lat: m.lat,
                lon: m.lon,
            },
            height: Height(m.height),
            tow: TimeOfWeek(m.tow),
            fix_status,
            covariance: [0_f64; 9],
        };

        return Some(info);
    }
}
