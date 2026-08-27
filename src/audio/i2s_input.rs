//! Mic-capture task — reads SPH0645 over I2S1 RX, converts 32-bit slot
//! samples to 16-bit PCM, applies a DC-blocker + digital gain, and pushes
//! into a ring buffer drained by the HFP outgoing-data callback.
//!
//! Mirrors the cpp `MicCapture` module
//! (`components/hfp_hf/src/MicCapture.cpp`).

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

use std::collections::VecDeque;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;

use esp_idf_svc::hal::gpio::AnyIOPin;
use esp_idf_svc::hal::i2s::config::{
    Config, DataBitWidth, SlotMode, StdClkConfig, StdConfig, StdGpioConfig, StdSlotConfig,
};
use esp_idf_svc::hal::i2s::I2sDriver;
use log::{info, warn};

use crate::audio::MicCaptureConfig;
use crate::stats;

/// Sample rate is fixed at SCO mSBC narrowband (16 kHz mono).
const SAMPLE_RATE_HZ: u32 = 16_000;

/// Ring buffer cap: 8 KB = ~250 ms of 16-bit-mono-16 kHz audio. Large
/// enough to absorb HFP outgoing-cb jitter without overrun under normal
/// conditions.
const RING_CAPACITY: usize = 8 * 1024;

/// How many i32 samples to pull per I2S read. 32 samples = 2 ms @ 16 kHz —
/// small enough to keep the ring topped up between HFP callbacks (~1 kHz),
/// big enough to amortise read overhead.
const SAMPLES_PER_READ: usize = 32;

/// Default mic gain (+14 dB), boosts SPH0645's -26 dBFS @ 94 dB SPL into
/// the ~-10 dBFS range iPhone's AEC expects.
const DEFAULT_GAIN_MILLI_DB: i32 = 14_000;

/// SPH0645 puts 24 valid data bits left-justified in bits [31:8] of a
/// 32-bit slot. Right-shifting by 16 lands the upper 16 of those 24 bits
/// as a signed i16 — the lower 8 bits of precision are sacrificed for
/// the simpler 16-bit pipeline downstream.
const SPH0645_DATA_SHIFT: u32 = 16;

/// Cloneable mic handle. Producer side (the task) and consumer side
/// (HFP outgoing cb, drain caller) share the same ring + gain state.
#[derive(Clone)]
pub struct Mic {
    inner: Arc<Inner>,
}

struct Inner {
    ring: Mutex<VecDeque<u8>>,
    gain_milli_db: AtomicI32,
}

impl Mic {
    /// Spawn the mic-capture task.
    pub fn spawn(config: MicCaptureConfig) -> std::io::Result<Self> {
        let inner = Arc::new(Inner {
            ring: Mutex::new(VecDeque::with_capacity(RING_CAPACITY)),
            gain_milli_db: AtomicI32::new(DEFAULT_GAIN_MILLI_DB),
        });
        let task_inner = Arc::clone(&inner);
        thread::Builder::new()
            .name("mic_capture".into())
            .stack_size(8 * 1024)
            .spawn(move || Self::event_loop(config, task_inner))?;
        Ok(Self { inner })
    }

    /// Drain exactly `dst.len()` bytes from the ring. Returns `dst.len()`
    /// on success, or 0 if the ring doesn't have enough — the HFP stack
    /// (`bta_hf_client_co.c`) rejects short reads, so all-or-nothing is
    /// the only correct semantic.
    pub fn read(&self, dst: &mut [u8]) -> usize {
        let mut ring = self.inner.ring.lock().unwrap();
        if ring.len() < dst.len() {
            return 0;
        }
        for byte in dst.iter_mut() {
            *byte = ring.pop_front().unwrap();
        }
        dst.len()
    }

    /// Drop everything currently buffered. Call on SCO connect so the
    /// remote AG doesn't get a quarter-second of stale samples.
    pub fn drain(&self) {
        self.inner.ring.lock().unwrap().clear();
    }

    /// Set digital gain in dB. Applied on the next mic sample. Public
    /// API for future runtime tuning (e.g. console key, OTA config);
    /// currently no caller — gain is fixed to `DEFAULT_GAIN_MILLI_DB`
    /// at spawn time.
    #[allow(dead_code)]
    pub fn set_gain_db(&self, db: f32) {
        let milli = (db * 1000.0).round() as i32;
        self.inner.gain_milli_db.store(milli, Ordering::Relaxed);
    }

    fn event_loop(config: MicCaptureConfig, inner: Arc<Inner>) {
        let MicCaptureConfig { i2s, bclk, ws, din } = config;

        // SPH0645 wire format: 32-bit slot, 24-bit data left-justified,
        // mono on the left channel (SEL pin tied to GND).
        let std_config = StdConfig::new(
            Config::new(),
            StdClkConfig::from_sample_rate_hz(SAMPLE_RATE_HZ),
            StdSlotConfig::philips_slot_default(DataBitWidth::Bits32, SlotMode::Mono),
            StdGpioConfig::default(),
        );

        let mut drv = match I2sDriver::new_std_rx(i2s, &std_config, bclk, din, None::<AnyIOPin>, ws)
        {
            Ok(d) => d,
            Err(e) => {
                warn!("new_std_rx: {e}");
                return;
            }
        };
        if let Err(e) = drv.rx_enable() {
            warn!("rx_enable: {e}");
            return;
        }
        info!("SPH0645 up on I2S1 @ {SAMPLE_RATE_HZ} Hz / 32-bit slot / mono-left");

        let mut dc = DcBlocker::default();
        let mut raw = [0u8; SAMPLES_PER_READ * 4];

        loop {
            let n_read = match drv.read(&mut raw, u32::MAX) {
                Ok(n) => n,
                Err(e) => {
                    warn!("read: {e}");
                    continue;
                }
            };
            let gain_linear =
                10f32.powf(inner.gain_milli_db.load(Ordering::Relaxed) as f32 / 20_000.0);
            let mut local_peak: u16 = 0;
            let mut out = [0u8; SAMPLES_PER_READ * 2];
            let mut out_len = 0;
            for chunk in raw[..n_read].chunks_exact(4) {
                let s32 = i32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                let s16 = (s32 >> SPH0645_DATA_SHIFT) as i16;
                let filtered = dc.process(s16);
                let scaled = (filtered as f32 * gain_linear).round();
                let clipped = if scaled >= i16::MAX as f32 {
                    stats::MIC_CLIP_COUNT.fetch_add(1, Ordering::Relaxed);
                    i16::MAX
                } else if scaled <= i16::MIN as f32 {
                    stats::MIC_CLIP_COUNT.fetch_add(1, Ordering::Relaxed);
                    i16::MIN
                } else {
                    scaled as i16
                };
                let abs = clipped.unsigned_abs();
                if abs > local_peak {
                    local_peak = abs;
                }
                let bytes = clipped.to_le_bytes();
                out[out_len] = bytes[0];
                out[out_len + 1] = bytes[1];
                out_len += 2;
            }

            // Atomic peak update (max-of-window since last stats dump).
            let prev = stats::MIC_PEAK_ABS.load(Ordering::Relaxed);
            if u32::from(local_peak) > prev {
                stats::MIC_PEAK_ABS.store(u32::from(local_peak), Ordering::Relaxed);
            }

            // Push into ring; drop on overflow.
            let mut ring = inner.ring.lock().unwrap();
            for &b in &out[..out_len] {
                if ring.len() >= RING_CAPACITY {
                    stats::MIC_RING_OVERFLOWS.fetch_add(1, Ordering::Relaxed);
                    break;
                }
                ring.push_back(b);
            }
        }
    }
}

/// One-pole DC-blocking high-pass: y[n] = x[n] - x[n-1] + R * y[n-1].
/// R=0.995 puts -3 dB at ~12.7 Hz @ 16 kHz — comfortably below the voice
/// band, removes the SPH0645's DC bias and any sub-audio mic-rumble.
struct DcBlocker {
    x_prev: f32,
    y_prev: f32,
}

impl Default for DcBlocker {
    fn default() -> Self {
        Self {
            x_prev: 0.0,
            y_prev: 0.0,
        }
    }
}

impl DcBlocker {
    fn process(&mut self, x: i16) -> i16 {
        const R: f32 = 0.995;
        let x_f = x as f32;
        let y = x_f - self.x_prev + R * self.y_prev;
        self.x_prev = x_f;
        self.y_prev = y;
        y.clamp(i16::MIN as f32, i16::MAX as f32) as i16
    }
}
