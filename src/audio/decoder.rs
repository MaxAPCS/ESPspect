//! Decoder task — owns the `esp_audio_codec::Decoder` and runs the
//! decode loop. Receives [`DecoderMsg`] from a producer (today: the BT
//! A2DP callback) and pushes decoded PCM to an [`I2SAudioOutput`] handle.
//! Each emitted [`PcmFrame`] carries the format the codec produced, so
//! `i2s_output` can reconfigure the DAC on session change without any
//! side-channel coordination.

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

use std::sync::atomic::Ordering;
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TrySendError};
use std::thread;

use esp_audio_codec::decoder::aac::AacConfig;
use esp_audio_codec::decoder::sbc::{SbcConfig, SbcMode};
use esp_audio_codec::decoder::{Decoder, DecoderResult, FrameRecovery};
use esp_idf_svc::bt::a2dp::A2dpAudioBuf;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::{info, warn};

use crate::audio::i2s_output::I2SAudioOutput;
use crate::audio::{CodecChoice, I2SAudioOutputFormat};
use crate::stats;

/// Output buffer for the decoder. AAC LC at 44.1 k stereo decodes to
/// 1024 samples = 4 KiB per frame; SBC emits 512-byte frames. 4 KiB is
/// the working-set ceiling for our codecs — the `BuffNotEnough` arm of
/// the inner loop grows this if a future codec needs more.
const PCM_OUT_BUF: usize = 4 * 1024;

/// Bounded channel between the BT callback and the decoder. Each slot
/// wraps an `A2dpAudioBuf` that points into the BT controller's audio
/// pool, so this bound is sized by the BT pool — NOT by Rust heap. 128
/// slots exhausted the pool in ~1.5 s and crashed the BT stack; 32 keeps
/// pressure bounded while still absorbing typical bursts.
const FRAME_CHANNEL_DEPTH: usize = 32;

/// Initial `decode_buffer` capacity — enough for one A2DP media packet
/// (~952 bytes from iPhone-SBC, ~600-900 from AAC at typical bitrates).
const DECODE_BUFFER_INIT_CAP: usize = 4 * 1024;

/// Messages accepted by the decoder. Codec changes flow through the
/// same channel as frames so the worker observes them in order with
/// respect to audio data. `A2dpAudioBuf` owns its underlying C buffer
/// and frees it on drop, so moving it across the channel is zero-copy.
pub enum DecoderMsg {
    SwitchCodec(CodecChoice),
    Frame(A2dpAudioBuf),
    /// Tear down per-session state on A2DP disconnect: drop the
    /// `Decoder`, clear `decode_buffer`, and drain any queued `Frame`s
    /// (each one pins a slot in the Bluedroid audio pool).
    Reset,
}

/// Producer handle for the decoder task. `Clone` so the BT callback (and
/// any future producer) can hold its own clone; all clones write into the
/// same task. Channel-access is via typed methods, not a raw `tx` field.
#[derive(Clone)]
pub struct DecoderTask {
    tx: SyncSender<DecoderMsg>,
}

impl DecoderTask {
    /// Spawn the decoder task. The task lives until the channel closes
    /// (i.e. when the last `DecoderTask` clone is dropped).
    ///
    /// Pinned to Core1 just below `i2s_output`'s priority — produces the
    /// PCM that the DAC task consumes, and decode time (especially for
    /// AAC) competes for CPU. Keeping it off the Bluedroid core (Core0)
    /// and ahead of background work prevents audible underruns when the
    /// host task is busy.
    pub fn spawn(audio_out: I2SAudioOutput) -> std::io::Result<Self> {
        let (tx, rx) = sync_channel::<DecoderMsg>(FRAME_CHANNEL_DEPTH);
        ThreadSpawnConfiguration {
            name: Some(c"audio_decoder"),
            stack_size: 8 * 1024,
            priority: 19,
            pin_to_core: Some(Core::Core1),
            ..Default::default()
        }
        .set()
        .map_err(std::io::Error::other)?;
        let _ = thread::Builder::new()
            .name("audio_decoder".into())
            .spawn(move || Self::event_loop(rx, audio_out))?;
        let _ = ThreadSpawnConfiguration::default().set();
        Ok(Self { tx })
    }

    /// Push an encoded A2DP media frame. Non-blocking: drops on
    /// channel-full and increments `stats::DROPPED_SENDS` so the BT
    /// callback never stalls the host task.
    pub fn send_frame(&self, frame: A2dpAudioBuf) {
        if let Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) =
            self.tx.try_send(DecoderMsg::Frame(frame))
        {
            stats::DROPPED_SENDS.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Hand a new codec choice to the decoder. Blocking; only happens
    /// once per AVDTP negotiation, never at audio rate.
    pub fn switch_codec(&self, choice: CodecChoice) {
        if let Err(e) = self.tx.send(DecoderMsg::SwitchCodec(choice)) {
            warn!("switch_codec send failed: {e}");
        }
    }

    /// Tear down per-session decoder state on A2DP disconnect.
    pub fn reset(&self) {
        if let Err(e) = self.tx.send(DecoderMsg::Reset) {
            warn!("reset send failed: {e}");
        }
    }

    fn event_loop(rx: Receiver<DecoderMsg>, audio_out: I2SAudioOutput) {
        let mut decoder: Option<Decoder> = None;
        let mut out_buf = vec![0u8; PCM_OUT_BUF];
        let mut current_format: Option<I2SAudioOutputFormat> = None;
        let mut frames_decoded: u64 = 0;
        let mut pcm_bytes: u64 = 0;
        // Persistent input buffer: each BT packet's bytes are appended
        // here and the decoder's process() consumes whatever it can.
        // Unconsumed bytes stay across BT packets — e.g. a half frame
        // trailing at the end of a packet rejoins the next packet's
        // bytes seamlessly.
        let mut decode_buffer: Vec<u8> = Vec::with_capacity(DECODE_BUFFER_INIT_CAP);

        info!("entered");

        while let Ok(msg) = rx.recv() {
            match msg {
                DecoderMsg::Reset => {
                    // A2DP disconnected — drop decoder + per-session
                    // working memory, then drain any encoded frames
                    // still queued (each pins a slot in the Bluedroid
                    // audio pool until its A2dpAudioBuf is dropped).
                    decoder = None;
                    current_format = None;
                    decode_buffer.clear();
                    decode_buffer.shrink_to_fit();
                    frames_decoded = 0;
                    pcm_bytes = 0;
                    let mut drained = 0u32;
                    while let Ok(stale) = rx.try_recv() {
                        drop(stale);
                        drained += 1;
                    }
                    info!("reset (drained {drained} stale msgs)");
                }
                DecoderMsg::SwitchCodec(choice) => {
                    // Cache the format these decoded samples will carry.
                    // No separate "set format" message — the format
                    // travels with every PcmFrame, so a session resume
                    // after SCO ends just works: the next decoded frame
                    // reconfigures the DAC.
                    current_format = Some(I2SAudioOutputFormat {
                        sample_rate: choice.sample_rate(),
                        channels: choice.channels(),
                        bits: 16,
                    });

                    let result = match choice {
                        CodecChoice::Aac {
                            sample_rate,
                            channels,
                        } => Decoder::aac(AacConfig {
                            sample_rate,
                            channels,
                            bits_per_sample: 16,
                            // A2DP delivers raw AAC frames; no ADTS header.
                            no_adts_header: true,
                            // HE-AAC SBR/PS isn't part of A2DP-AAC-LC.
                            aac_plus_enable: false,
                        }),
                        CodecChoice::Sbc { channels, .. } => Decoder::sbc(SbcConfig {
                            mode: SbcMode::Std,
                            channels,
                            // PLC is effective only for MSBC + RECOVERY_PLC;
                            // for STD-mode A2DP it's a no-op at best.
                            enable_plc: false,
                        }),
                    };
                    match result {
                        Ok(d) => {
                            decoder = Some(d);
                            frames_decoded = 0;
                            pcm_bytes = 0;
                            // Drop bytes from a previous codec's stream —
                            // garbage to the new decoder.
                            decode_buffer.clear();
                            info!("ready for {:?}", choice);
                        }
                        Err(e) => {
                            warn!("open failed for {choice:?}: {e:?}");
                            decoder = None;
                            current_format = None;
                        }
                    }
                }
                DecoderMsg::Frame(frame) => {
                    let Some(dec) = decoder.as_mut() else {
                        continue; // codec hasn't been negotiated yet — frame
                                  // dropped here → A2dpAudioBuf::drop frees it
                    };
                    let Some(format) = current_format else {
                        continue; // shouldn't happen: SwitchCodec sets both
                    };

                    // Append this BT packet's encoded bytes; the inner
                    // loop consumes what it can, the rest stays for next
                    // packet.
                    decode_buffer.extend_from_slice(frame.data());

                    // Drive the decoder until either it asks for more
                    // input (DataLack) or it stops consuming (consumed=0
                    // — could be a buffered silence emit or a partial
                    // frame at tail). Matches Espressif's
                    // esp_audio_codec test_sbc.c.
                    loop {
                        if decode_buffer.is_empty() {
                            break;
                        }
                        match dec.process(&decode_buffer, &mut out_buf, FrameRecovery::Normal) {
                            DecoderResult::Ok { consumed, decoded } => {
                                if decoded > 0 {
                                    frames_decoded += 1;
                                    pcm_bytes += decoded as u64;

                                    if frames_decoded == 1 {
                                        match dec.info() {
                                            Ok(info) => info!("info {info:?}"),
                                            Err(e) => warn!("info() failed: {e:?}"),
                                        }
                                    }

                                    // Peak-amplitude probe (max |sample|
                                    // since the last 1 Hz dump).
                                    {
                                        let pcm = &out_buf[..decoded];
                                        let mut peak: u16 = 0;
                                        for s in pcm.chunks_exact(2) {
                                            let v = i16::from_le_bytes([s[0], s[1]]).unsigned_abs();
                                            if v > peak {
                                                peak = v;
                                            }
                                        }
                                        let prev = stats::PCM_PEAK_ABS.load(Ordering::Relaxed);
                                        if u32::from(peak) > prev {
                                            stats::PCM_PEAK_ABS
                                                .store(u32::from(peak), Ordering::Relaxed);
                                        }
                                    }

                                    // Hand PCM off to i2s_output bundled
                                    // with its format. Non-blocking: when
                                    // the queue fills, the frame drops
                                    // here and the DAC reads silence on
                                    // its next DMA cycle (auto_clear=true).
                                    // We prefer dropping audio over
                                    // stalling the decoder.
                                    let samples = out_buf[..decoded].to_vec();
                                    if let Err(e) = audio_out.send_pcm(format, samples) {
                                        warn!("i2s_output full: {e}");
                                    }
                                    stats::FRAMES_DECODED_DELTA.fetch_add(1, Ordering::Relaxed);

                                    if frames_decoded.is_power_of_two() {
                                        let peak = stats::PCM_PEAK_ABS.swap(0, Ordering::Relaxed);
                                        info!(
                                            "decoder: frame #{} consumed={} decoded={} total_pcm={} peak={} buf_remain={}",
                                            frames_decoded,
                                            consumed,
                                            decoded,
                                            pcm_bytes,
                                            peak,
                                            decode_buffer.len().saturating_sub(consumed),
                                        );
                                    }
                                }
                                decode_buffer.drain(..consumed);
                                if consumed == 0 {
                                    // `consumed=0 decoded>0` is the
                                    // decoder emitting silence-padding
                                    // from its internal state; any real
                                    // PCM is already written. Break to
                                    // recv more BT bytes.
                                    break;
                                }
                            }
                            DecoderResult::DataLack { .. } => {
                                break;
                            }
                            DecoderResult::BuffNotEnough { consumed, needed } => {
                                warn!("buffnotenough: consumed={consumed} needed={needed}");
                                // Do NOT drain — retry same input after grow.
                                let new_len = needed.max(out_buf.len() * 2);
                                out_buf.resize(new_len, 0);
                                warn!("out_buf grown to {new_len} bytes");
                            }
                            DecoderResult::Continue { consumed } => {
                                decode_buffer.drain(..consumed);
                                if consumed == 0 {
                                    break;
                                }
                            }
                            DecoderResult::Failed(e) => {
                                warn!("failed: {:?}", e);
                                stats::DECODE_ERRS.fetch_add(1, Ordering::Relaxed);
                                if frames_decoded < 16 {
                                    warn!("process error: {e:?}");
                                }
                                decode_buffer.clear();
                                break;
                            }
                        }
                    }
                }
            }
        }
        info!("channel closed, exiting");
    }
}
