use crate::{Arm, Science, Wheels};

pub trait Checksum<const T: usize> {
    /// Creates an array of the bytes that'll be checksummed.
    fn to_checksum_array(&self) -> [u8; T];

    /// Calculates the checksum of this `Message`.
    fn checksum(&self) -> u8 {
        // iterate over the bytes, sum them, and take the sum's last 8 bits
        (self
            .to_checksum_array()
            .iter()
            .map(|&x| x as u32)
            .sum::<u32>()
            & 0xFF) as u8
    }

    /// Check if the checksum is correct.
    fn is_checksum_correct(&self) -> bool;
}

impl Checksum<2> for Wheels {
    fn to_checksum_array(&self) -> [u8; 2] {
        [self.left, self.right]
    }

    fn is_checksum_correct(&self) -> bool {
        self.checksum == self.checksum()
    }
}
impl Checksum<6> for Arm {
    fn to_checksum_array(&self) -> [u8; 6] {
        [
            self.bicep,
            self.forearm,
            self.base,
            self.wrist_pitch,
            self.wrist_roll,
            self.claw,
        ]
    }

    fn is_checksum_correct(&self) -> bool {
        self.checksum == self.checksum()
    }
}

impl Checksum<5> for Science {
    fn to_checksum_array(&self) -> [u8; 5] {
        [
            self.big_actuator,
            self.drill,
            self.small_actuator,
            self.test_tubes,
            self.camera_servo,
        ]
    }

    fn is_checksum_correct(&self) -> bool {
        self.checksum == self.checksum()
    }
}
