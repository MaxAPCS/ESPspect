//! Persists the last successfully bonded peer's BD_ADDR to NVS so we can
//! attempt to reconnect on boot. The cache is written from [`super::gap`]
//! on `GapEvent::AuthenticationCompleted` (a stack-wide hook — single
//! source of truth, profile modules stay unaware of NVS).
//!
//! Stored as a 6-byte blob in NVS namespace `bt_speaker`, key
//! `last_peer`. Pairing a different phone overwrites the entry.

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

use esp_idf_svc::bt::BdAddr;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use log::{info, warn};

const NAMESPACE: &str = "bt_speaker";
const KEY: &str = "last_peer";

pub struct PairCache {
    nvs: EspNvs<NvsDefault>,
}

impl PairCache {
    /// Open (or create) the `bt_speaker` namespace for read+write. Takes
    /// a clone of the NVS partition so the BT driver can still consume the
    /// original handle in `bt::init`.
    pub fn open(partition: EspDefaultNvsPartition) -> anyhow::Result<Self> {
        let nvs = EspNvs::new(partition, NAMESPACE, true)?;
        Ok(Self { nvs })
    }

    /// Return the last-stored peer address, or `None` on first boot.
    /// Logs `warn!` on genuine NVS errors so they don't hide behind None.
    pub fn read(&self) -> Option<BdAddr> {
        let mut buf = [0u8; 6];
        match self.nvs.get_blob(KEY, &mut buf) {
            Ok(Some(bytes)) if bytes.len() == 6 => Some(BdAddr::from_bytes(buf)),
            Ok(Some(_)) | Ok(None) => None,
            Err(e) => {
                warn!("read: {e}");
                None
            }
        }
    }

    /// Persist a peer address. `&self` is fine here — `EspNvs::set_blob`
    /// uses internal synchronization, so the cache can be shared through
    /// `Arc` without a `Mutex`.
    pub fn write(&self, addr: &BdAddr) {
        let bytes = addr.addr();
        match self.nvs.set_blob(KEY, &bytes) {
            Ok(()) => info!("stored {addr}"),
            Err(e) => warn!("write {addr}: {e}"),
        }
    }
}
