//! Audio pipeline modules + shared types. Submodules:
//!
//! - [`decoder`] — esp_audio_codec decoder task (A2DP-encoded → PCM)
//! - [`i2s_output`] — single-writer I2S TX task feeding the DAC
//! - [`i2s_input`] — SPH0645 mic capture task feeding the HFP outgoing cb

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

use esp_idf_svc::hal::gpio::{Gpio12, Gpio13, Gpio15, Gpio22, Gpio25, Gpio26};
use esp_idf_svc::hal::i2s::{I2S0, I2S1};

pub mod decoder;
pub mod i2s_input;
pub mod i2s_output;

/// Codec choice + parameters parsed out of the AVDTP negotiation.
/// A2DP delivers raw codec frames (no in-band header for AAC), so
/// the decoder needs sample rate / channel count up front.
#[derive(Debug, Copy, Clone)]
pub enum CodecChoice {
    Aac { sample_rate: u32, channels: u8 },
    Sbc { sample_rate: u32, channels: u8 },
}

impl CodecChoice {
    pub fn sample_rate(&self) -> u32 {
        match self {
            Self::Aac { sample_rate, .. } | Self::Sbc { sample_rate, .. } => *sample_rate,
        }
    }

    pub fn channels(&self) -> u8 {
        match self {
            Self::Aac { channels, .. } | Self::Sbc { channels, .. } => *channels,
        }
    }
}

/// Construction-time wiring for I2SAudioOutput. Pins + peripheral are
/// claimed once at `spawn()` and never resent.
pub struct I2SAudioOutputConfig {
    pub i2s: I2S0<'static>,
    pub bclk: Gpio13<'static>,
    pub ws: Gpio12<'static>,
    pub dout: Gpio15<'static>,
}

/// Runtime format for I2SAudioOutput PCM. Travels with every [`PcmFrame`]
/// — when the format differs from what the task currently has configured,
/// I2S reconfigures in place. There is no separate "set format" event:
/// producers always declare the format for the samples they push, which
/// eliminates restore-on-disconnect coordination between producers.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct I2SAudioOutputFormat {
    pub sample_rate: u32,
    pub channels: u8,
    pub bits: u8,
}

/// A chunk of PCM bytes tagged with the format it was rendered at. The
/// only message variant `I2SAudioOutput` accepts. Each producer owns its
/// own current-format state and includes it on every push; the consumer
/// reconfigures the DAC when it sees a format that differs from the one
/// currently applied.
pub struct PcmFrame {
    pub format: I2SAudioOutputFormat,
    pub samples: Vec<u8>,
}

/// Construction-time wiring for the SPH0645 mic on I2S1.
///
/// SPH0645 produces 24-bit data left-justified in a 32-bit slot, mono on
/// the left channel (SEL tied to GND). Sample rate is fixed at 16 kHz to
/// match SCO mSBC. Pin assignments mirror the cpp project.
pub struct MicCaptureConfig {
    pub i2s: I2S1<'static>,
    pub bclk: Gpio26<'static>,
    pub ws: Gpio25<'static>,
    pub din: Gpio22<'static>,
}
