//! This module provides the ESP IDF NVS implementation of the `KvBlobStore` trait for storing and loading BLOBs.

use core::fmt::Write;

use esp_idf_svc::nvs::{EspNvs, EspNvsPartition, NvsPartitionId};
use esp_idf_svc::sys::EspError;

use log::{debug, trace};

use rs_matter_stack::matter::error::Error;
use rs_matter_stack::matter::persist::KvBlobStore;

use crate::error::to_persist_error;

type SKey = heapless::String<5>;

/// A `KvBlobStore`` implementation that uses the ESP IDF NVS API
/// to store and load the BLOBs.
///
/// NOTE: Not async (yet)
pub struct EspKvBlobStore<T>(EspNvs<T>)
where
    T: NvsPartitionId;

impl<T> EspKvBlobStore<T>
where
    T: NvsPartitionId,
{
    /// Create a new KV BLOB store instance that would persist in namespace `esp-idf-matter`.
    pub fn new_default(nvs: EspNvsPartition<T>) -> Result<Self, EspError> {
        Self::new(nvs, "esp-idf-matter")
    }

    /// Create a new KV BLOB store instance.
    pub fn new(nvs: EspNvsPartition<T>, namespace: &str) -> Result<Self, EspError> {
        Ok(Self(EspNvs::new(nvs, namespace, true)?))
    }

    fn load<'a>(&self, key: u16, buf: &'a mut [u8]) -> Result<Option<&'a [u8]>, Error> {
        let mut skey = SKey::new();

        let data = self
            .0
            .get_blob(Self::skey(&mut skey, key), buf)
            .map_err(to_persist_error)?;

        debug!("Blob {key}: loaded {:?} bytes", data.map(|data| data.len()));
        trace!(
            "Blob {key} load details: loaded {:?} bytes {data:?}",
            data.map(|data| data.len())
        );

        Ok(data)
    }

    fn store(&mut self, key: u16, data: &[u8]) -> Result<(), Error> {
        let mut skey = SKey::new();

        self.0
            .set_blob(Self::skey(&mut skey, key), data)
            .map_err(to_persist_error)?;

        debug!("Blob {key}: stored {} bytes", data.len());
        trace!(
            "Blob {key} store details: stored {} bytes {data:?}",
            data.len()
        );

        Ok(())
    }

    fn remove(&mut self, key: u16) -> Result<(), Error> {
        let mut skey = SKey::new();

        self.0
            .remove(Self::skey(&mut skey, key))
            .map_err(to_persist_error)?;

        debug!("Blob {key}: removed");

        Ok(())
    }

    fn skey(skey: &mut SKey, key: u16) -> &str {
        skey.clear();
        write!(skey, "{key:04x}").unwrap();

        skey.as_str()
    }
}

impl<T> KvBlobStore for EspKvBlobStore<T>
where
    T: NvsPartitionId,
{
    fn load<'a>(&mut self, key: u16, buf: &'a mut [u8]) -> Result<Option<&'a [u8]>, Error> {
        EspKvBlobStore::load(self, key, buf)
    }

    fn store(&mut self, key: u16, data: &[u8], _buf: &mut [u8]) -> Result<(), Error> {
        EspKvBlobStore::store(self, key, data)
    }

    fn remove(&mut self, key: u16, _buf: &mut [u8]) -> Result<(), Error> {
        EspKvBlobStore::remove(self, key)
    }
}
