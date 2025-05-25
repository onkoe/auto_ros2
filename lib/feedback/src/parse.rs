//! # Parse
//!
//! A module that parses a given slice into a valid message.

use std::f32::consts::PI;

use crate::{error::ParsingError, Arm, Imu, Led, Science, Wheels};

/// Any kind of message that should be sent to/from the rover.
#[derive(Debug, Clone, Copy)]
pub enum Message {
    Wheels(Wheels),
    Led(Led),
    Arm(Arm),
    Science(Science),
    Imu(Imu),
}

/// Parse an input slice into a valid message.
/// ```
/// # use feedback::parse::parse;
/// #
/// assert!(parse(&[0x09]).is_err());
/// ```
pub fn parse(input: &[u8]) -> Result<Message, ParsingError> {
    let input_len = input.len() as u32;

    // check if we have a subsystem byte
    if input_len < 1 {
        return Err(ParsingError::ZeroLengthSlice);
    }

    // we do! let's match on it
    let subsystem = input[0];

    match subsystem {
        Wheels::SUBSYSTEM_BYTE => {
            // you have to specify the part byte
            if input_len < 2 {
                return Err(ParsingError::NoEboxPart);
            }

            // it exists! now we can parse it
            // parse the second byte, input[1], to tell if it's wheel or leds
            let part = input[1];
            match part {
                // wheel part
                Wheels::PART_BYTE => {
                    check_length(input_len, subsystem, part, 5)?;

                    Ok(Message::Wheels(Wheels::new(input[2], input[3])))
                }

                // leds part
                Led::PART_BYTE => {
                    check_length(input_len, subsystem, part, 5)?;

                    Ok(Message::Led(Led {
                        red: input[2],
                        green: input[3],
                        blue: input[4],
                    }))
                }

                malformed_part => {
                    // invalid input
                    Err(ParsingError::InvalidSubsystem(malformed_part))
                }
            }
        }

        Arm::SUBSYSTEM_BYTE => {
            check_length(input_len, subsystem, 0x00, 8)?;

            let arm = Arm {
                bicep: input[1],
                forearm: input[2],
                base: input[3],
                wrist_pitch: input[4],
                wrist_roll: input[5],
                claw: input[6],
                checksum: input[7],
            };

            Ok(Message::Arm(arm))
        }

        Science::SUBSYSTEM_BYTE => {
            check_length(input_len, subsystem, 0x0, 7)?;

            let sci = Science {
                big_actuator: input[1],
                drill: input[2],
                small_actuator: input[3],
                test_tubes: input[4],
                camera_servo: input[5],
                checksum: input[6],
            };

            Ok(Message::Science(sci))
        }

        Imu::SUBSYSTEM_BYTE => {
            // we should have:
            //
            // - part byte
            // - 3 arrays with 3 four-byte values
            // - a four-byte temperature
            const EXPECTED_LENGTH: u32 = 1 + (3 * 3 * 4) + 4;
            check_length(input_len, subsystem, 0x0, EXPECTED_LENGTH)?;

            // note: each float is four bytes
            let imu = Imu {
                // accel, gyro, and compass math inspired by this library:
                // https://github.com/norlab-ulaval/ros2_icm20948/blob/5af0f48976623e2e1a5e8a1e65b659d8e5f513eb/ros2_icm20948/icm20948_node.py#L57C1-L64C1
                accel_x: f32::from_ne_bytes(input[1..5].try_into().unwrap()) * 9.81 / 2048.0,
                accel_y: f32::from_ne_bytes(input[5..9].try_into().unwrap()) * 9.81 / 2048.0,
                accel_z: f32::from_ne_bytes(input[9..13].try_into().unwrap()) * 9.81 / 2048.0,

                gyro_x: f32::from_ne_bytes(input[13..17].try_into().unwrap()) * PI / (16.4 * 180.0),
                gyro_y: f32::from_ne_bytes(input[17..21].try_into().unwrap()) * PI / (16.4 * 180.0),
                gyro_z: f32::from_ne_bytes(input[21..25].try_into().unwrap()) * PI / (16.4 / 180.0),

                compass_x: f32::from_ne_bytes(input[25..29].try_into().unwrap()) * 0.000001 / 0.15,
                compass_y: f32::from_ne_bytes(input[29..33].try_into().unwrap()) * 0.000001 / 0.15,
                compass_z: f32::from_ne_bytes(input[33..37].try_into().unwrap()) * 0.000001 / 0.15,

                // correct according to the sparkfun library:
                // https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/32cf0614bccf0d8cff32fbc4260097ac10e5b714/src/ICM_20948.cpp#L538C3-L538C43
                temp_c: ((f32::from_ne_bytes(input[37..41].try_into().unwrap()) - 21.0) / 333.87)
                    + 21.0,
            };

            Ok(Message::Imu(imu))
        }

        // otherwise, we got invalid input
        malformed_subsys => Err(ParsingError::InvalidSubsystem(malformed_subsys)),
    }
}

/// Checks if the given input length is equal to the expected length. If so, returns `Ok(())`.
/// Otherwise, returns a `ParsingError::LengthInconsistency` error.
///
/// This avoids some kinda annoying boilerplate in the `parse` function.
/// (please stabilize #74935 🥹)
const fn check_length(
    input_len: u32,
    subsystem: u8,
    part: u8,
    expected: u32,
) -> Result<(), ParsingError> {
    if input_len != expected {
        Err(ParsingError::LengthInconsistency {
            subsystem,
            part,
            length: input_len,
            expected_length: expected,
        })
    } else {
        Ok(())
    }
}

#[cfg(feature = "python")]
mod python {
    use pyo3::{exceptions::PyValueError, prelude::*};

    use crate::{Arm, Imu, Led, Science, Wheels};

    use super::Message;

    /// A PyO3-friendly version of the `Message` enum.
    #[doc(hidden)]
    #[cfg_attr(feature = "python", pyo3::pyclass)]
    #[derive(Debug, Clone, Copy)]
    pub enum PyMessage {
        Wheels { wheels: Wheels },
        Led { led: Led },
        Arm { arm: Arm },
        Science { science: Science },
        Imu { imu: Imu },
    }

    impl PyMessage {
        fn __str__(&self) -> PyResult<String> {
            Ok(format!("{:?}", self))
        }
    }

    /// this is some nonsense... but it's required nonsense.
    /// see [pyo3 issue #3748](https://github.com/PyO3/pyo3/issues/3748) for info
    impl From<PyMessage> for Message {
        fn from(val: PyMessage) -> Self {
            match val {
                PyMessage::Wheels { wheels } => Message::Wheels(wheels),
                PyMessage::Led { led } => Message::Led(led),
                PyMessage::Arm { arm } => Message::Arm(arm),
                PyMessage::Science { science } => Message::Science(science),
                PyMessage::Imu { imu } => Message::Imu(imu),
            }
        }
    }

    /// again. nonsense
    impl From<Message> for PyMessage {
        fn from(val: Message) -> Self {
            match val {
                Message::Wheels(wheels) => PyMessage::Wheels { wheels },
                Message::Led(led) => PyMessage::Led { led },
                Message::Arm(arm) => PyMessage::Arm { arm },
                Message::Science(science) => PyMessage::Science { science },
                Message::Imu(imu) => PyMessage::Imu { imu },
            }
        }
    }

    /// Parse an input slice into a valid message.
    #[cfg_attr(feature = "python", pyfunction(name = "parse"))]
    pub fn pyparse(input: &[u8]) -> PyResult<PyMessage> {
        super::parse(input)
            .map_err(|e| PyValueError::new_err(e.to_string()))
            .map(|t| t.into())
    }

    // export as module
    #[pymodule]
    fn parse(m: &Bound<'_, PyModule>) -> PyResult<()> {
        m.add_class::<PyMessage>()?;
        m.add_function(wrap_pyfunction!(pyparse, m)?)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use crate::{
        parse::{parse, Message},
        Imu,
    };

    #[test]
    fn parse_imu_msg() {
        let imu_msg: [&[u8]; 11] = [
            // subsystem byte
            //
            &[0x04],
            // accel
            &1.0241_f32.to_ne_bytes(),
            &5.135_f32.to_ne_bytes(),
            &0.153_f32.to_ne_bytes(),
            //
            // gyro
            &0.01523_f32.to_ne_bytes(),
            &0.6241_f32.to_ne_bytes(),
            &0.1_f32.to_ne_bytes(),
            //
            // compass
            &310_f32.to_ne_bytes(),
            &162.1_f32.to_ne_bytes(),
            &9.15602_f32.to_ne_bytes(),
            //
            // temp c
            &0_u32.to_ne_bytes(),
        ];

        let imu_msg = imu_msg.into_iter().flatten().copied().collect::<Vec<u8>>();

        // parse it
        let parsed_imu_msg = super::parse(&imu_msg).expect("parse should succeed");
        let Message::Imu(_) = parsed_imu_msg else {
            panic!("parser didn't recognize bytes as an imu message");
        };
    }

        // check a value in each
        assert_eq!(imu.accel_x, 1.0241_f32);
        assert_eq!(imu.gyro_y, 0.6241_f32);
        assert_eq!(imu.compass_z, 9.15602_f32);
    }
}
