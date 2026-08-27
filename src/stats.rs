//! Pipeline observability counters. Each counter is cleared by the 1 Hz
//! logger thread after every dump, so values are deltas-per-second.
//! Counters are public statics — the hot paths in the BT cb, decoder,
//! i2s_output, and mic loops increment them with `Relaxed` ordering, and
//! the [`Stats`] task drains them once a second.

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

use std::sync::atomic::{AtomicU32, Ordering};
use std::thread;
use std::time::Duration;

use log::{info, warn};

pub static DROPPED_SENDS: AtomicU32 = AtomicU32::new(0);
pub static I2S_WRITE_ERRS: AtomicU32 = AtomicU32::new(0);
pub static DECODE_ERRS: AtomicU32 = AtomicU32::new(0);
pub static FRAMES_DECODED_DELTA: AtomicU32 = AtomicU32::new(0);
pub static PCM_PEAK_ABS: AtomicU32 = AtomicU32::new(0);

pub static MIC_PEAK_ABS: AtomicU32 = AtomicU32::new(0);
pub static MIC_CLIP_COUNT: AtomicU32 = AtomicU32::new(0);
pub static MIC_RING_OVERFLOWS: AtomicU32 = AtomicU32::new(0);

/// Zero-sized type wrapping the stats logger task. `Stats::spawn()`
/// drains the public counters above on a 1 Hz cadence.
pub struct Stats;

impl Stats {
    /// Spawn the 1 Hz stats logger. Logs only when something is non-zero
    /// so the serial line stays quiet when audio is healthy.
    pub fn spawn() -> std::io::Result<thread::JoinHandle<()>> {
        thread::Builder::new()
            .name("stats".into())
            .stack_size(2 * 1024)
            .spawn(|| Self::event_loop())
    }

    fn event_loop() {
        let mut tick = 0u32;
        loop {
            thread::sleep(Duration::from_secs(1));
            tick = tick.wrapping_add(1);

            // Every 5 s log free heap so we can spot pressure drift —
            // SCO setup and BT pairing both grab heap transiently.
            if tick % 5 == 0 {
                let free = unsafe { esp_idf_svc::sys::esp_get_free_heap_size() };
                let min_free = unsafe { esp_idf_svc::sys::esp_get_minimum_free_heap_size() };
                info!("heap: free={free} min_free={min_free}");
            }

            let dropped = DROPPED_SENDS.swap(0, Ordering::Relaxed);
            let i2s_err = I2S_WRITE_ERRS.swap(0, Ordering::Relaxed);
            let dec_err = DECODE_ERRS.swap(0, Ordering::Relaxed);
            let decoded = FRAMES_DECODED_DELTA.swap(0, Ordering::Relaxed);
            let mic_clip = MIC_CLIP_COUNT.swap(0, Ordering::Relaxed);
            let mic_overflow = MIC_RING_OVERFLOWS.swap(0, Ordering::Relaxed);
            if dropped != 0 || i2s_err != 0 || dec_err != 0 {
                warn!(
                    "audio: dropped_sends={dropped} i2s_errs={i2s_err} decode_errs={dec_err} decoded={decoded}"
                );
            }
            if mic_clip != 0 || mic_overflow != 0 {
                warn!("mic: clip={mic_clip} ring_overflow={mic_overflow}");
            }
        }
    }
}
