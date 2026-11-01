//! This module provides error conversion functions for the ESP IDF Matter stack.

use esp_idf_svc::sys::EspError;

use rs_matter::error::{Error, ErrorCode};

/// Converts an ESP network error to an `rs-matter` error
pub fn to_net_error(err: EspError) -> Error {
    // TODO: The `rs-matter` error code is too generic
    log::error!("Network error: {err}");
    ErrorCode::NoNetworkInterface.into()
}

/// Converts an ESP persistence error to an `rs-matter` error
pub fn to_persist_error(err: EspError) -> Error {
    // TODO: The `rs-matter` error code is too generic
    log::error!("Persistence error: {err}");
    ErrorCode::StdIoError.into()
}
