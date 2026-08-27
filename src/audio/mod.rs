//! Audio pipeline modules + shared types. Submodules:
//!
//! - [`decoder`] — esp_audio_codec decoder task (A2DP-encoded → PCM)
//! - [`i2s_output`] — single-writer I2S TX task feeding the DAC
//! - [`i2s_input`] — SPH0645 mic capture task feeding the HFP outgoing cb

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

pub mod decoder;

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

/// Runtime format for I2SAudioOutput PCM. Travels with every [`PcmFrame`]
/// — when the format differs from what the task currently has configured,
/// I2S reconfigures in place. There is no separate "set format" event:
/// producers always declare the format for the samples they push, which
/// eliminates restore-on-disconnect coordination between producers.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct AudioOutputFormat {
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
    pub format: AudioOutputFormat,
    pub samples: Vec<u8>,
}
