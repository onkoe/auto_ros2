use thiserror::Error;

/// An error occuring while parsing a `Message`.
///
/// Doesn't appear in Python - they're cast to strings and raised
/// as `ValueError`s.
#[derive(Clone, Copy, Debug, Error, PartialEq, PartialOrd)]
pub enum ParsingError {
    #[error("You must supply a slice with a length greater than zero.")]
    ZeroLengthSlice,
    #[error("First byte doesn't code for a valid subsystem. Given: `{0:x}`.")]
    InvalidSubsystem(u8),
    #[error("The ebox subsystem must be given a second byte, input[1], to specify which part to control. None was given.")]
    NoEboxPart,
    #[error(
        "
        The given slice wasn't the correct length for subsystem \
        `{subsystem:x}` on part {part:x}. Expected a slice of length `{expected_length}`, but got a \
        slice of length `{length}`.
    "
    )]
    LengthInconsistency {
        subsystem: u8,
        part: u8,
        length: u32,
        expected_length: u32,
    },
    #[error("The given slice was malformed.")]
    MalformedMessage,
}

/// An error that can occur when sending messages to the Rover.
#[derive(Debug, Error)]
pub enum SendError {
    /// The validation of the message failed!
    #[error("Message validation failed! err: {0}")]
    MessageFailedValidation(#[from] ParsingError),

    /// Sending it with the socket resulted in an error.
    #[error("Failed to send a message! err: {0}")]
    SocketError(#[from] std::io::Error),
}

#[cfg(feature = "python")]
pyo3::create_exception!(error, SendException, pyo3::exceptions::PyException);
