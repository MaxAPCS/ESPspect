//! AVRCP CT (controller) — sends transport-control passthrough commands
//! (play/pause/next/prev) to the paired phone, and pulls back metadata
//! + playback notifications so the console can see track info live.
//!
//! AVRCP is pull/subscribe, not push. After the AG connects we have to:
//!
//! 1. Request the set of notifications the AG supports.
//! 2. Register for each notification we care about (`Playback`,
//!    `TrackChanged`). Notifications are one-shot — once one fires, we
//!    have to re-register.
//! 3. Request the initial metadata snapshot; thereafter re-request on
//!    every `TrackChanged`.
//!
//! Local `playing` state for the play/pause toggle is kept in sync from
//! the `Playback` notification so the toggle is always correct, even
//! when the user pauses on the phone instead of via the console.

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use enumset::EnumSet;
use esp_idf_svc::bt::avrc::controller::{AvrccEvent, EspAvrcc};
use esp_idf_svc::bt::avrc::{KeyCode, MetadataId, Notification, NotificationType, PlaybackStatus};
use esp_idf_svc::bt::{BtClassic, BtDriver};
use log::{info, warn};

/// AVRCP spec requires ≥ 100 ms between passthrough press and release;
/// 80 ms reads as snappy without violating that for typical paths after
/// transit. Tune up if "next" gets dropped by a particular AG.
const PASSTHROUGH_RELEASE_MS: u64 = 80;

/// Cloneable AVRCP-CT controller. The console (and any future transport-
/// command producer) hold their own clones; all clones drive the same
/// `EspAvrcc`.
#[derive(Clone)]
pub struct AvrcpController {
    inner: Arc<Inner>,
}

struct Inner {
    avrcc: EspAvrcc<'static, BtClassic, Arc<BtDriver<'static, BtClassic>>>,
    next_label: AtomicU8,
    /// Tracks the AG's current playback state. Set from `Playback`
    /// notifications; consulted by `play_pause()` to decide which key
    /// to send. AVRCP has separate `Play` and `Pause` passthroughs and
    /// no spec-level toggle.
    playing: AtomicBool,
}

impl AvrcpController {
    /// Bring up the AVRCP CT role on the shared `BtDriver` and subscribe
    /// the metadata/state callback.
    pub fn spawn(bt: Arc<BtDriver<'static, BtClassic>>) -> anyhow::Result<Self> {
        let avrcc = EspAvrcc::new(bt)?;
        let inner = Arc::new(Inner {
            avrcc,
            next_label: AtomicU8::new(0),
            playing: AtomicBool::new(false),
        });

        let cb_inner = Arc::clone(&inner);
        inner
            .avrcc
            .subscribe(move |event| Self::handle_event(&cb_inner, event))?;

        Ok(Self { inner })
    }

    /// Toggle play ↔ pause based on the AG's last-known state.
    pub fn play_pause(&self) {
        let was_playing = self.inner.playing.load(Ordering::Relaxed);
        let key = if was_playing {
            KeyCode::Pause
        } else {
            KeyCode::Play
        };
        info!("{}", if was_playing { "pause" } else { "play" });
        self.passthrough(key);
        // Optimistic local update — the AG's Playback notification will
        // correct us if the command didn't take.
        self.inner.playing.store(!was_playing, Ordering::Relaxed);
    }

    pub fn next_track(&self) {
        self.passthrough(KeyCode::Forward);
    }

    pub fn prev_track(&self) {
        self.passthrough(KeyCode::Backward);
    }

    fn passthrough(&self, key: KeyCode) {
        let label = self.inner.next_label();
        if let Err(e) = self.inner.avrcc.send_passthrough(label, key, true) {
            warn!("press {key:?}: {e}");
            return;
        }
        thread::sleep(Duration::from_millis(PASSTHROUGH_RELEASE_MS));
        if let Err(e) = self.inner.avrcc.send_passthrough(label, key, false) {
            warn!("release {key:?}: {e}");
        }
    }

    /// Metadata attributes we request on connect and after every track
    /// change. Title/Artist/Album cover the common case; PlayingTime
    /// adds duration for status displays.
    fn metadata_attrs() -> EnumSet<MetadataId> {
        MetadataId::Title | MetadataId::Artist | MetadataId::Album | MetadataId::PlayingTime
    }

    fn handle_event(inner: &Arc<Inner>, event: AvrccEvent) {
        match event {
            AvrccEvent::Connected(bd) => {
                info!("connected {bd}");
                // Kick off the metadata + notification subscription
                // pipeline. Order doesn't matter much — capabilities
                // arrives first in practice and we register from there.
                inner.request_capabilities();
                inner.request_metadata();
            }
            AvrccEvent::Disconnected(bd) => {
                info!("disconnected {bd}");
                // AG is gone; nothing to clean up locally — the next
                // Connected will redo the subscription dance.
            }
            AvrccEvent::NotificationCapabilities {
                allowed,
                capabilities,
            } => {
                info!("capabilities allowed={allowed} set={capabilities:?}");
                // Register for the events we care about that the AG
                // offers.
                for kind in [NotificationType::Playback, NotificationType::TrackChanged] {
                    if capabilities.contains(kind) {
                        inner.register_notification(kind);
                    }
                }
            }
            AvrccEvent::Notification(Notification::Playback(status)) => {
                // Sync our local guess to ground truth.
                inner
                    .playing
                    .store(status == PlaybackStatus::Playing, Ordering::Relaxed);
                info!("playback {status:?}");
                // Notifications are one-shot — re-arm.
                inner.register_notification(NotificationType::Playback);
            }
            AvrccEvent::Notification(Notification::TrackChanged) => {
                info!("track changed");
                // Pull fresh metadata for the new track.
                inner.request_metadata();
                inner.register_notification(NotificationType::TrackChanged);
            }
            AvrccEvent::Notification(other) => info!("notify {other:?}"),
            AvrccEvent::Metadata { id, text } => info!("{id:?}: {text}"),
            AvrccEvent::PlayStatus => info!("play status"),
            AvrccEvent::Volume(v) => info!("volume {v}"),
            AvrccEvent::Passthrough {
                key_code,
                response_code,
                ..
            } => info!("passthrough rsp key={key_code:?} code={response_code:?}"),
            other => info!("{other:?}"),
        }
    }
}

impl Inner {
    /// AVRCP transaction label is a 4-bit field; wrap at 0x0F.
    fn next_label(&self) -> u8 {
        self.next_label.fetch_add(1, Ordering::Relaxed) & 0x0F
    }

    fn request_capabilities(&self) {
        let label = self.next_label();
        if let Err(e) = self.avrcc.request_capabilities(label) {
            warn!("request_capabilities: {e}");
        }
    }

    fn request_metadata(&self) {
        let label = self.next_label();
        if let Err(e) = self
            .avrcc
            .request_metadata(label, AvrcpController::metadata_attrs())
        {
            warn!("request_metadata: {e}");
        }
    }

    /// Subscribe to a one-shot notification. The AG will fire the event
    /// once, after which we must re-register to keep receiving updates.
    fn register_notification(&self, kind: NotificationType) {
        let label = self.next_label();
        if let Err(e) = self.avrcc.register_notification(label, kind, 0) {
            warn!("register_notification {kind:?}: {e}");
        }
    }
}
