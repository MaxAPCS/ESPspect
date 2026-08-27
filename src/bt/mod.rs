//! Bluetooth classic stack — driver init + GAP + profile modules.
//!
//! - [`gap`] — device name + discoverable flag (stack-wide)
//! - [`a2dp`] — A2DP sink (external-codec)
//! - [`avrcp`] — AVRCP CT (transport control + metadata)
//! - [`hfp`] — HFP HF (SCO over HCI, Siri trigger)
//!
//! [`init`] constructs the shared `Arc<BtDriver>` that GAP and every
//! profile module clones at spawn time.

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

use std::sync::Arc;

use esp_idf_svc::bt::{BtClassic, BtDriver};
use esp_idf_svc::hal::modem::BluetoothModemPeripheral;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

pub mod a2dp;
pub mod avrcp;
pub mod gap;
pub mod hfp;
pub mod pair_cache;

/// Bring up the BT classic controller + Bluedroid stack and return a
/// shareable handle that every profile module clones.
///
/// Caller must already have called `reduce_bt_memory` on the modem
/// peripheral before this — that frees the BLE-controller heap and has
/// to happen before any threads or large allocations land.
pub fn init<M>(
    modem: M,
    nvs: EspDefaultNvsPartition,
) -> anyhow::Result<Arc<BtDriver<'static, BtClassic>>>
where
    M: BluetoothModemPeripheral + 'static,
{
    let driver = BtDriver::<BtClassic>::new(modem, Some(nvs))?;
    Ok(Arc::new(driver))
}
