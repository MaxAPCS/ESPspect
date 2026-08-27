//! HFP HF — SCO send/receive over HCI audio path. Subscribes to events
//! from `EspHfpc`, routes incoming SCO PCM to the I2S DAC (mono → stereo
//! duplication), and fills outgoing SCO frames from the mic ring.
//!
//! Format-on-data principle: every PCM push to `I2SAudioOutput` carries
//! its own format; SCO disconnect doesn't have to "restore" any A2DP
//! format. When A2DP resumes streaming, its next decoded frame will
//! reconfigure the DAC by virtue of carrying its own format.

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

use std::sync::Arc;

use esp_idf_svc::bt::hfp::client::{AudioStatus, EspHfpc, HfpcEvent, Source};
use esp_idf_svc::bt::{BdAddr, BtClassic, BtDriver};
use esp_idf_svc::sys::EspError;
use log::{info, warn};

use crate::audio::i2s_input::Mic;
use crate::audio::i2s_output::I2SAudioOutput;
use crate::audio::I2SAudioOutputFormat;

type Hfpc = EspHfpc<'static, BtClassic, Arc<BtDriver<'static, BtClassic>>>;

/// SCO mSBC narrowband format. We always advertise 2 channels because
/// the DAC is wired stereo; mono samples get duplicated below before
/// being pushed.
const SCO_FORMAT: I2SAudioOutputFormat = I2SAudioOutputFormat {
    sample_rate: 16_000,
    channels: 2,
    bits: 16,
};

/// Mono→stereo duplication factor — each mono sample is written into
/// both stereo slots.
const STEREO_DUP_FACTOR: usize = 2;

/// HFP session guard. Held by `main()` for the program lifetime; drop
/// unwinds the HFP stack.
pub struct HfpSession {
    inner: Arc<Inner>,
}

/// Cloneable voice-control handle, used by the console to trigger Siri.
#[derive(Clone)]
pub struct HfpVoiceHandle {
    inner: Arc<Inner>,
}

struct Inner {
    hfpc: Hfpc,
}

impl HfpSession {
    /// Bring up the HFP HF client and subscribe its event handler.
    pub fn spawn(
        bt: Arc<BtDriver<'static, BtClassic>>,
        audio_out: I2SAudioOutput,
        mic: Mic,
    ) -> anyhow::Result<Self> {
        // Resampling source advertised to the controller. mSBC at 16 k
        // mono matches what we feed via SendData and expect via RecvData
        // — controller handles the on-wire codec.
        let source = Source {
            sample_rate_hz: 16_000,
            bits_per_sample: 16,
            stereo: false,
        };
        let hfpc = EspHfpc::new(bt, Some(source))?;
        let inner = Arc::new(Inner { hfpc });

        let cb_inner = Arc::clone(&inner);
        inner
            .hfpc
            .subscribe(move |event| Self::handle_event(&cb_inner, &audio_out, &mic, event))?;

        info!("client up (HCI audio path, mSBC narrowband)");
        Ok(Self { inner })
    }

    /// Construct a cloneable voice-control handle for stdin / console.
    pub fn voice_handle(&self) -> HfpVoiceHandle {
        HfpVoiceHandle {
            inner: Arc::clone(&self.inner),
        }
    }

    /// Initiate an HFP-HF connection to a (presumably bonded) peer. Used
    /// by the boot auto-reconnect path; fails silently if the peer is
    /// out of range — iOS-initiated reconnect handles that case once
    /// we're discoverable.
    pub fn connect(&self, addr: &BdAddr) -> Result<(), EspError> {
        self.inner.hfpc.connect(addr.raw())
    }

    /// Returns the byte count to report back to the HFP send-data
    /// callback. Only meaningful for [`HfpcEvent::SendData`]; all other
    /// arms return 0.
    fn handle_event(
        inner: &Arc<Inner>,
        audio_out: &I2SAudioOutput,
        mic: &Mic,
        event: HfpcEvent,
    ) -> usize {
        match event {
            HfpcEvent::ConnectionState {
                bd_addr, status, ..
            } => {
                info!("connection {bd_addr:?} -> {status:?}");
                // iOS Voice-Memo warmup primer (open SCO on SLC,
                // immediately tear down) is intentionally absent —
                // interacts badly with A2DP streaming heap pressure.
                // Will return alongside the ring-buffer refactor of the
                // PCM message path.
            }
            HfpcEvent::AudioState {
                bd_addr, status, ..
            } => {
                info!("audio {bd_addr:?} -> {status:?}");
                match status {
                    AudioStatus::Connected | AudioStatus::ConnectedMsbc => {
                        // Format-on-data: no SetFormat sent here. The
                        // first RecvData will carry SCO_FORMAT and
                        // i2s_output will reconfigure to 16 k stereo.
                        // We DO drain the mic so the AG doesn't start
                        // receiving stale samples.
                        mic.drain();
                    }
                    AudioStatus::Disconnected => {
                        // Mic drain is enough on the HFP side — A2DP's
                        // next decoded frame will reconfigure the DAC
                        // back to its own format by virtue of carrying
                        // it inline.
                        mic.drain();
                    }
                    AudioStatus::Connectng => {}
                }
            }
            HfpcEvent::VoiceRecognitionEnabled => info!("VR enabled"),
            HfpcEvent::VoiceRecognitionDisabled => info!("VR disabled"),
            HfpcEvent::RecvData(buf) => {
                // 16-bit mono PCM @ 16 kHz from the AG. Duplicate to
                // stereo because the DAC is wired for 2 channels.
                let mut stereo = Vec::with_capacity(buf.len() * STEREO_DUP_FACTOR);
                for s in buf.chunks_exact(2) {
                    stereo.extend_from_slice(s);
                    stereo.extend_from_slice(s);
                }
                if let Err(e) = audio_out.send_pcm(SCO_FORMAT, stereo) {
                    warn!("SCO RX → i2s_output full: {e}");
                }
                // Kick Bluedroid to pull our next outgoing frame. The
                // TX path is user-pull-driven (bta_hf_client_sco.c:855),
                // not on an internal timer — without this call
                // send_data never fires and the AG hears silence.
                // Matches cpp HfpClient.cpp:160.
                inner.hfpc.request_outgoing_data_ready();
            }
            HfpcEvent::SendData(buf) => {
                // AG wants `buf.len()` bytes of 16-bit mono PCM.
                // mic.read is all-or-nothing; on underrun return 0 and
                // let the BT stack handle the gap. The closure's return
                // value IS the bytes-filled count — see
                // send_data_handler in esp-idf-svc/hfp.rs.
                return mic.read(buf);
            }
            // Lots of low-priority status events; log at info.
            other => info!("{other:?}"),
        }
        0
    }
}

impl HfpVoiceHandle {
    pub fn start_voice_recognition(&self) {
        if let Err(e) = self.inner.hfpc.start_voice_recognition() {
            warn!("start_voice_recognition: {e}");
        } else {
            info!("voice recognition start (Siri)");
        }
    }

    pub fn stop_voice_recognition(&self) {
        if let Err(e) = self.inner.hfpc.stop_voice_recognition() {
            warn!("stop_voice_recognition: {e}");
        } else {
            info!("voice recognition stop");
        }
    }
}
