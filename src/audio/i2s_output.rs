//! `I2SAudioOutput` — single-owner I2S TX task. Producers push [`PcmFrame`]
//! messages, each carrying its own format. When the format differs from
//! the one currently driving the DAC, the task reconfigures in place
//! (lazy first-init, runtime reconfig thereafter). Blocking `write_all`
//! lives here so producers stay decoupled from DMA backpressure.

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

use std::sync::atomic::Ordering;
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TrySendError};
use std::thread;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::gpio::AnyIOPin;
use esp_idf_svc::hal::i2s::config::{
    Config, DataBitWidth, SlotMode, StdClkConfig, StdConfig, StdGpioConfig, StdSlotConfig,
};
use esp_idf_svc::hal::i2s::{I2sDriver, I2sTx};
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::{info, warn};

use crate::audio::{I2SAudioOutputConfig, I2SAudioOutputFormat, PcmFrame};
use crate::stats;

/// In-flight PCM chunks the task can buffer. ~8 chunks of one decoded
/// frame each (~4 KB at 44.1 k stereo) ≈ 185 ms of audio in-flight; the
/// remaining slots cover format-change burst.
const CHANNEL_DEPTH: usize = 12;

/// Producer-side handle. Clone to share between multiple producers
/// (decoder, HFP SCO RX, etc.). All clones write into the same task.
#[derive(Clone)]
pub struct I2SAudioOutput {
    tx: SyncSender<PcmFrame>,
}

impl I2SAudioOutput {
    /// Spawn the I2SAudioOutput task. The task holds `config` (pins +
    /// peripheral) for its lifetime; the I2S driver itself is built
    /// lazily on the first [`PcmFrame`] received.
    ///
    /// Pinned to Core1 at high priority — DAC underruns produce audible
    /// glitches, so we keep this task on the opposite core from
    /// Bluedroid (Core0 by default) and above the default priority so
    /// it preempts background work.
    pub fn spawn(config: I2SAudioOutputConfig) -> std::io::Result<Self> {
        let (tx, rx) = sync_channel::<PcmFrame>(CHANNEL_DEPTH);
        ThreadSpawnConfiguration {
            name: Some(c"i2s_output"),
            stack_size: 4 * 1024,
            priority: 20,
            pin_to_core: Some(Core::Core1),
            ..Default::default()
        }
        .set()
        .map_err(std::io::Error::other)?;
        let _ = thread::Builder::new()
            .name("i2s_output".into())
            .spawn(move || Self::event_loop(config, rx))?;
        // Restore default cfg so threads spawned after us don't inherit
        // our priority / core affinity.
        let _ = ThreadSpawnConfiguration::default().set();
        Ok(Self { tx })
    }

    /// Non-blocking PCM push. Drops on channel-full so producers never
    /// stall the BT callback or decoder hot paths. The DAC's `auto_clear`
    /// DMA fills underrun gaps with silence rather than repeating buffers.
    pub fn send_pcm(
        &self,
        format: I2SAudioOutputFormat,
        samples: Vec<u8>,
    ) -> Result<(), TrySendError<PcmFrame>> {
        self.tx.try_send(PcmFrame { format, samples })
    }

    fn event_loop(config: I2SAudioOutputConfig, rx: Receiver<PcmFrame>) {
        let I2SAudioOutputConfig {
            i2s,
            bclk,
            ws,
            dout,
        } = config;
        let mut available = Some((i2s, bclk, ws, dout));
        let mut driver: Option<I2sDriver<'static, I2sTx>> = None;
        let mut current: Option<I2SAudioOutputFormat> = None;

        info!("entered");

        while let Ok(frame) = rx.recv() {
            if current != Some(frame.format) {
                if let Err(()) = Self::apply_format(&mut driver, &mut available, frame.format) {
                    continue;
                }
                current = Some(frame.format);
            }
            let Some(drv) = driver.as_mut() else {
                continue;
            };
            if let Err(e) = drv.write_all(&frame.samples, u32::MAX) {
                warn!("write_all: {e}");
                stats::I2S_WRITE_ERRS.fetch_add(1, Ordering::Relaxed);
            }
        }
        info!("channel closed, exiting");
    }

    /// Apply a new format. First call consumes the held peripherals to
    /// build the driver; subsequent calls reconfigure in place via the
    /// local esp-idf-hal patch (`tx_reconfigure_std`).
    fn apply_format(
        driver: &mut Option<I2sDriver<'static, I2sTx>>,
        available: &mut Option<(
            esp_idf_svc::hal::i2s::I2S0<'static>,
            esp_idf_svc::hal::gpio::Gpio13<'static>,
            esp_idf_svc::hal::gpio::Gpio12<'static>,
            esp_idf_svc::hal::gpio::Gpio15<'static>,
        )>,
        fmt: I2SAudioOutputFormat,
    ) -> Result<(), ()> {
        let std_config = Self::build_std_config(fmt);
        if let Some(drv) = driver.as_mut() {
            match drv.tx_reconfigure_std(&std_config) {
                Ok(()) => {
                    info!(
                        "i2s_output: reconfigured to {} Hz / {}-bit / {}ch",
                        fmt.sample_rate, fmt.bits, fmt.channels
                    );
                    Ok(())
                }
                Err(e) => {
                    warn!("tx_reconfigure_std: {e}");
                    Err(())
                }
            }
        } else {
            let Some((p_i2s, p_bclk, p_ws, p_dout)) = available.take() else {
                warn!("format change after peripherals consumed; ignoring");
                return Err(());
            };
            match I2sDriver::new_std_tx(p_i2s, &std_config, p_bclk, p_dout, None::<AnyIOPin>, p_ws)
            {
                Ok(mut drv) => match drv.tx_enable() {
                    Ok(()) => {
                        info!(
                            "i2s_output: I2S0 up at {} Hz / {}-bit / {}ch",
                            fmt.sample_rate, fmt.bits, fmt.channels
                        );
                        *driver = Some(drv);
                        Ok(())
                    }
                    Err(e) => {
                        warn!("tx_enable: {e}");
                        Err(())
                    }
                },
                Err(e) => {
                    warn!("new_std_tx: {e}");
                    Err(())
                }
            }
        }
    }

    /// Build an `StdConfig` from our format. auto_clear=true makes the
    /// DMA emit silence when the TX queue underflows instead of
    /// repeating the last buffer (critical for clean A2DP disconnect).
    fn build_std_config(fmt: I2SAudioOutputFormat) -> StdConfig {
        let channel_cfg = Config::new().auto_clear(true);
        let slot_mode = if fmt.channels >= 2 {
            SlotMode::Stereo
        } else {
            SlotMode::Mono
        };
        let bit_width = match fmt.bits {
            16 => DataBitWidth::Bits16,
            24 => DataBitWidth::Bits24,
            32 => DataBitWidth::Bits32,
            other => {
                warn!("bits={other} unsupported, defaulting to 16");
                DataBitWidth::Bits16
            }
        };
        StdConfig::new(
            channel_cfg,
            StdClkConfig::from_sample_rate_hz(fmt.sample_rate),
            StdSlotConfig::philips_slot_default(bit_width, slot_mode),
            StdGpioConfig::default(),
        )
    }
}
