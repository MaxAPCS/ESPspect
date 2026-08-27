//! GAP — set the BT-classic device name, make the controller discoverable,
//! and subscribe to peer-level events. Lives at the `bt::` level because
//! GAP is stack-wide: every profile (A2DP / AVRCP / HFP) is reachable
//! through the same visible name + discoverable flag, and every peer
//! authentication / link event surfaces here regardless of which profiles
//! the peer eventually uses.
//!
//! The single GAP subscription is also where we persist the last-bonded
//! BD_ADDR — on `AuthenticationCompleted` with success status we hand the
//! address to [`super::pair_cache`]. Profile modules know nothing about
//! NVS.

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

use std::sync::Arc;

use esp_idf_svc::bt::gap::{DiscoveryMode, EspGap, GapEvent};
use esp_idf_svc::bt::{BtClassic, BtDriver, BtStatus};
use esp_idf_svc::hal::delay::FreeRtos;
use log::info;

use crate::bt::pair_cache::PairCache;

/// Holds the `EspGap` for the program's lifetime. Dropping it unwinds the
/// GAP callbacks; we keep it in `main` alongside the profile sessions.
pub struct GapSession {
    _gap: EspGap<'static, BtClassic, Arc<BtDriver<'static, BtClassic>>>,
}

impl GapSession {
    /// Configure GAP (device name + discoverable) and subscribe to
    /// peer-level events. `pair_cache` is written on every successful
    /// authentication so it can be read at next boot for reconnect.
    pub fn spawn(
        bt: Arc<BtDriver<'static, BtClassic>>,
        name: &str,
        pair_cache: Arc<PairCache>,
    ) -> anyhow::Result<Self> {
        let gap = EspGap::new(bt)?;
        gap.set_device_name(name)?;
        gap.set_scan_mode(true, DiscoveryMode::Discoverable)?;
        info!("discoverable as '{name}'");

        gap.subscribe(move |event| match event {
            GapEvent::AuthenticationCompleted {
                bd_addr,
                status,
                device_name,
            } => {
                info!("auth complete {bd_addr} '{device_name}' status={status:?}");
                if status == BtStatus::Success {
                    pair_cache.write(&bd_addr);
                }
            }
            GapEvent::AclConnected {
                bd_addr, status, ..
            } => {
                info!("acl connected {bd_addr} status={status:?}");

                if status == BtStatus::HciConnectionExists {
                    FreeRtos::delay_ms(1000);
                    panic!("panic, connection is stale on boot, reboot for hot connect");
                }
            }
            GapEvent::AclDisconnected { bd_addr, .. } => {
                info!("acl disconnected {bd_addr}");
            }
            _ => {}
        })?;

        Ok(Self { _gap: gap })
    }
}
