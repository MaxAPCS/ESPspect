//! A2DP sink (external-codec) — codec negotiation parser, and the
//! subscribe callback that forwards encoded media frames to the decoder
//! task. GAP (device name + discoverable) lives in [`super::gap`].

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

use std::sync::Arc;

use esp_idf_svc::bt::a2dp::{A2dpEvent, Codec, ConnectionStatus, EspA2dp, Sink};
use esp_idf_svc::bt::{BdAddr, BtClassic, BtDriver};
use esp_idf_svc::sys::EspError;
use log::{info, warn};

use crate::audio::decoder::DecoderTask;
use crate::audio::CodecChoice;

/// Long-lived session for the A2DP-sink handle. Held by `main` for the
/// program's lifetime; dropping unwinds the A2DP host.
pub struct A2dpSession {
    a2dp: EspA2dp<'static, BtClassic, Arc<BtDriver<'static, BtClassic>>, Sink>,
}

impl A2dpSession {
    /// Bring up the A2DP external-codec sink, wire the cb to forward
    /// encoded frames into `decoder`, and register the supported codec
    /// endpoints.
    pub fn spawn(
        bt: Arc<BtDriver<'static, BtClassic>>,
        decoder: DecoderTask,
    ) -> anyhow::Result<Self> {
        let a2dp = EspA2dp::<'static, _, _, Sink>::new_external_codec(bt)?;
        info!("external-codec sink up");

        let cb_decoder = decoder.clone();
        a2dp.subscribe(move |event| {
            Self::handle_event(event, &cb_decoder);
            0
        })?;

        // AAC at seid 0 (iPhone prefers AAC when both are offered); SBC
        // at seid 1 as the mandatory fallback for non-Apple sources.
        for (seid, codec) in [
            (0, Codec::aac_default()),
            (1, Codec::aac_default()),
            (2, Codec::sbc_default()),
        ] {
            info!("register {} at seid {}", codec.name(), seid);
            a2dp.register_sink_endpoint(seid, &codec)?;
        }

        Ok(Self { a2dp })
    }

    /// Initiate an A2DP-sink connection back to a (presumably bonded)
    /// peer. Used by the boot auto-reconnect path; safe to call with no
    /// peer in range — the underlying stack just emits a fail-to-connect
    /// event and iOS-initiated reconnect picks up later.
    pub fn connect(&self, addr: &BdAddr) -> Result<(), EspError> {
        self.a2dp.connect_sink(addr)
    }

    fn handle_event(event: A2dpEvent, decoder: &DecoderTask) {
        // Consume `event` so the SinkAudioData arm can move the owned
        // A2dpAudioBuf into the channel without a copy.
        match event {
            A2dpEvent::AudioCodecConfigured { bd_addr, codec } => {
                // Pull bitrate out of the AAC cie bytes ourselves —
                // Codec::bitrate() in the fork is misnamed (sample rate).
                let bitrate_info = match &codec {
                    Codec::Mpeg2_4(data) => {
                        let (vbr, br) = Self::parse_aac_bitrate(data);
                        Some((vbr, br))
                    }
                    _ => None,
                };
                info!(
                    "codec negotiated with {bd_addr:?}: {} sr={:?} stereo={:?} bitrate={bitrate_info:?}",
                    codec.name(),
                    codec.bitrate(),
                    codec.stereo(),
                );
                // `Codec::bitrate()` returns sample rate (Hz). Parse into
                // something the decoder can hand to esp_audio_codec.
                let stereo = codec.stereo().unwrap_or(true);
                let channels: u8 = if stereo { 2 } else { 1 };
                let choice = match codec {
                    Codec::Mpeg2_4(_) => codec.bitrate().map(|sr| CodecChoice::Aac {
                        sample_rate: sr,
                        channels,
                    }),
                    Codec::Sbc(_) => codec.bitrate().map(|sr| CodecChoice::Sbc {
                        sample_rate: sr,
                        channels,
                    }),
                    _ => None,
                };
                if let Some(c) = choice {
                    decoder.switch_codec(c);
                } else {
                    warn!("unsupported codec negotiated; no decoder created");
                }
            }
            A2dpEvent::SinkAudioData(buf) => {
                // Move the owned A2dpAudioBuf into the decoder.
                // send_frame is non-blocking; drops on full so the BT
                // host task never wedges waiting on the decoder.
                decoder.send_frame(buf);
            }
            A2dpEvent::SinkEndpointRegistered { seid, state } => {
                info!("SEP {seid} registered: {state:?}");
            }
            A2dpEvent::ConnectionState {
                bd_addr, status, ..
            } => {
                info!("connection {bd_addr:?} -> {status:?}");
                if status == ConnectionStatus::Disconnected {
                    decoder.reset();
                }
            }
            A2dpEvent::AudioState { bd_addr, status } => {
                info!("audio {bd_addr:?} -> {status:?}");
            }
            other => info!("{other:?}"),
        }
    }

    /// Decode the VBR flag + max bitrate from the 6-byte AAC capability
    /// payload (esp_a2d_cie_m24_t wire form). Bytes 3..6 are:
    ///   b3.7        : VBR flag
    ///   b3.6..b5.0  : 23-bit max bitrate (bits/sec)
    fn parse_aac_bitrate(data: &[u8; 6]) -> (bool, u32) {
        let vbr = (data[3] & 0x80) != 0;
        let bitrate =
            ((u32::from(data[3]) & 0x7F) << 16) | (u32::from(data[4]) << 8) | u32::from(data[5]);
        (vbr, bitrate)
    }
}
