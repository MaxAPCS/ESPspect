//! esp32-bt-speaker: BT-classic Bluetooth speaker example for ESP32
//! in Rust. Profiles wired:
//!
//! - A2DP sink (external-codec, AAC + SBC) → decoder → I2S DAC
//! - AVRCP CT — play/pause/next/prev passthrough + metadata
//! - HFP HF — SCO RX/TX (mSBC), Siri trigger
//! - SPH0645 mic on I2S1 → DC blocker + gain → ring → HFP outgoing
//!
//! Thread graph:
//!
//! ```text
//!   BT host cb ──► DecoderMsg chan (32) ──► decoder task ──► I2SAudioOutput
//!     │                                                          ▲   │
//!     │                                                          │   ▼
//!     ▼                                  HFP RecvData ───────────┘  DAC
//!   AVRCP cb                              │
//!     │                                   ▼
//!     ▼                          PcmFrame chan (12)
//!   metadata logs                          ▲
//!                                          │
//!                                  HFP SendData ◄── mic ring ◄── mic task
//! ```
//!
//! Invariants:
//! - Every `PcmFrame` carries its own format; `I2SAudioOutput` reconfigs
//!   the DAC when it sees a format different from the current one.
//!   There is no separate "set format" event, and therefore no
//!   "restore A2DP format" coordination between profile modules.
//! - `DecoderMsg::SwitchCodec` precedes `Frame` messages for that codec
//!   (enforced by Bluedroid's own sequencing; FIFO preserves it).
//! - All cross-thread back-pressure paths drop on full, never block —
//!   the BT host callback must never wait on us.

#![allow(unknown_lints)]
#![allow(unexpected_cfgs)]

mod audio;
mod bt;
mod console;
mod stats;

#[cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]
fn main() -> anyhow::Result<()> {
    app::main()
}

#[cfg(not(all(esp32, esp_idf_bt_a2dp_use_external_codec)))]
fn main() -> anyhow::Result<()> {
    println!("FALLBACK MAIN: cfg gate did not match");
    panic!("requires ESP32 with CONFIG_BT_A2DP_USE_EXTERNAL_CODEC=y");
}

#[cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]
mod app {
    use std::sync::Arc;

    use esp_audio_codec::decoder::DefaultDecoders;
    use esp_idf_svc::bt::reduce_bt_memory;
    use esp_idf_svc::hal::delay::FreeRtos;
    use esp_idf_svc::hal::peripherals::Peripherals;
    use esp_idf_svc::log::EspLogger;
    use esp_idf_svc::nvs::EspDefaultNvsPartition;
    use log::info;

    use crate::audio::decoder::DecoderTask;
    use crate::audio::i2s_input::Mic;
    use crate::audio::i2s_output::I2SAudioOutput;
    use crate::audio::{I2SAudioOutputConfig, MicCaptureConfig};
    use crate::bt;
    use crate::bt::a2dp::A2dpSession;
    use crate::bt::avrcp::AvrcpController;
    use crate::bt::gap::GapSession;
    use crate::bt::hfp::HfpSession;
    use crate::bt::pair_cache::PairCache;
    use crate::console::Console;
    use crate::stats::Stats;

    /// Advertised device name over GAP.
    const DEVICE_NAME: &str = "esp-bt-speaker";

    pub fn main() -> anyhow::Result<()> {
        esp_idf_svc::sys::link_patches();
        EspLogger::initialize_default();

        let peripherals = Peripherals::take()?;
        let nvs = EspDefaultNvsPartition::take()?;
        let mut modem = peripherals.modem;
        reduce_bt_memory(unsafe { modem.reborrow() })?;

        // Pair cache: opens its own NVS namespace from a clone of the
        // partition handle. GAP writes to it on AuthenticationCompleted;
        // we read it after profiles spawn to try a boot reconnect to the
        // last bonded peer.
        let pair_cache = Arc::new(PairCache::open(nvs.clone())?);

        // Hold the default-decoder registration for the program lifetime —
        // makes AAC + SBC resolvable from `Decoder::aac()` / `Decoder::sbc()`.
        let _decoders = DefaultDecoders::register()
            .map_err(|e| anyhow::anyhow!("DefaultDecoders::register: {e:?}"))?;

        // Audio pipeline: I2S DAC writer ← decoder ← BT cb. The DAC is
        // PCM5102A on I2S0 with BCK=13, WS=12, DOUT=15. No MCLK.
        let audio_out = I2SAudioOutput::spawn(I2SAudioOutputConfig {
            i2s: peripherals.i2s0,
            bclk: peripherals.pins.gpio13,
            ws: peripherals.pins.gpio12,
            dout: peripherals.pins.gpio15,
        })?;
        let decoder = DecoderTask::spawn(audio_out.clone())?;

        // SPH0645 mic on I2S1 (BCK=26, WS=25, DIN=22). Drained by HFP's
        // outgoing-data callback while SCO is active.
        let mic = Mic::spawn(MicCaptureConfig {
            i2s: peripherals.i2s1,
            bclk: peripherals.pins.gpio26,
            ws: peripherals.pins.gpio25,
            din: peripherals.pins.gpio22,
        })?;

        Stats::spawn()?;

        // Bring up BT classic + GAP + profiles. One `Arc<BtDriver>`
        // shared across every profile.
        //
        // Order matters for SDP. Bluedroid emits a hard warning if
        // AVRCP CT is initialized after A2DP ("AVRC Controller is
        // expected to be initialized in advance of A2DP"), and iPhone
        // caches the SDP record set from the FIRST connection — any
        // profile not registered before GAP makes us discoverable
        // won't appear in that cache, and the peer will refuse to
        // open a channel for it on later sessions.
        //
        // So: profile registration first (AVRCP → A2DP → HFP), GAP
        // last to flip the discoverable bit only once SDP is complete.
        let bt_driver = bt::init(modem, nvs)?;
        let avrcp = AvrcpController::spawn(bt_driver.clone())?;
        let a2dp = A2dpSession::spawn(bt_driver.clone(), decoder)?;
        let hfp = HfpSession::spawn(bt_driver.clone(), audio_out, mic)?;
        let _gap = GapSession::spawn(bt_driver.clone(), DEVICE_NAME, pair_cache.clone())?;

        // Boot reconnect: fire A2DP + HFP connect directly. Cold path
        // hits power-on-to-music in ~4 s. Hot reboot races BTM's ACL
        // bookkeeping and surfaces HciConnectionExists; that wedge has
        // no clean in-process recovery, so the plan is to detect it in
        // GAP and panic — esp-idf auto-restarts and the next cold boot
        // brings us back in ~4 s.
        if let Some(addr) = pair_cache.read() {
            info!("reconnect attempt to {addr}");
            let _ = a2dp.connect(&addr);
            let _ = hfp.connect(&addr);
        }

        Console::spawn(avrcp, hfp.voice_handle())?;

        info!("ready — pair from phone (p/n/b for transport, s for Siri)");
        loop {
            FreeRtos::delay_ms(10_000);
        }
    }
}
