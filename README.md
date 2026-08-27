# esp32-bt-speaker

A BT-classic Bluetooth-speaker example for the ESP32 in Rust, built on
[`esp-idf-svc`](https://github.com/billylindeman/esp-idf-svc/tree/billy/a2dp-external-codec-api)
(fork, `billy/a2dp-external-codec-api` branch) with the external-codec
A2DP sink API. Your phone pairs as the audio source,
audio plays back via an I2S DAC, AVRCP transport controls work
end-to-end, and HFP routes call / voice-assistant audio through an
on-board microphone and the same DAC.

## What it demonstrates

- **External-codec A2DP sink.** AAC and SBC frames arrive raw from the
  controller; decoding happens on the host via
  [`esp-audio-codec-rs`](https://github.com/billylindeman/esp-audio-codec-rs/tree/billy/patch-for-sbc-bug)
  (fork, `billy/patch-for-sbc-bug` branch — Rust bindings around
  Espressif's `esp_audio_codec`).
  Requires `CONFIG_BT_A2DP_USE_EXTERNAL_CODEC=y` +
  `CONFIG_BT_A2DP_CODEC_AAC_ENABLED=y` (in `sdkconfig.defaults`).
- **Format-on-data PCM routing.** Every PCM frame carries its own
  format (sample rate, channels, bits). The DAC writer reconfigures
  inline when the format changes — there is no separate `SetFormat`
  event and no coordination needed between the A2DP and HFP
  producers.
- **HFP HF over the HCI audio path.** mSBC narrowband SCO RX → DAC,
  mic ring → SCO TX. Outgoing audio is driven by a pull callback from
  Bluedroid (`request_outgoing_data_ready`), matching the C++
  reference flow.
- **Boot reconnect via NVS pair cache.** After the first successful
  bond, the peer's BD_ADDR is persisted; the next boot fires
  A2DP+HFP connect at that address before going discoverable.
- **Hot-reboot recovery.** If the ESP reboots while the phone still
  holds the ACL link live, Bluedroid surfaces
  `BtStatus::HciConnectionExists`. There is no clean in-process
  unwedge for that state, so the GAP handler panics; ESP-IDF
  auto-restarts and the next cold boot path takes ~4 s.

## Hardware

The defaults match a PCM5102A DAC on I2S0 and an SPH0645 mic on I2S1.

| Function    | Peripheral | Pin |
| ----------- | ---------- | --- |
| DAC BCK     | I2S0       | 13  |
| DAC WS      | I2S0       | 12  |
| DAC DOUT    | I2S0       | 15  |
| Mic BCK     | I2S1       | 26  |
| Mic WS      | I2S1       | 25  |
| Mic DIN     | I2S1       | 22  |

PSRAM is required (`CONFIG_SPIRAM=y` in `sdkconfig.defaults`); large
allocations for the decoder and BT buffers route through external RAM.

## Build & flash

This crate targets `esp-idf-svc` and tracks ESP-IDF master (v6.1-dev
is the line that carries the external-codec A2DP API). `embuild`
resolves ESP-IDF automatically on first build via the
`[package.metadata.esp-idf-sys]` block in `Cargo.toml`.

```sh
# nightly is pinned in rust-toolchain.toml
cargo build --release

# requires `cargo install espflash`
espflash flash --monitor target/xtensa-esp32-espidf/release/esp32-bt-speaker
```

The first build pulls ESP-IDF + the `esp_audio_codec` component and
takes a while. Subsequent builds are incremental.

## Trying it

1. Power on and watch the boot log. The device advertises over GAP
   as `esp-bt-speaker`.
2. Pair from your phone's Bluetooth settings, then play audio. AAC
   negotiates by default on iPhone; SBC is the fallback for
   sources that don't support it.
3. Use the serial console for transport control:

   | Key | Action |
   | --- | ------ |
   | `p` | Play / pause |
   | `n` | Next track |
   | `b` | Previous track |
   | `s` | Start voice recognition (Siri / Assistant) |
   | `S` | Stop voice recognition |

4. Power-cycle the ESP. The pair cache replays A2DP + HFP connect to
   the last bonded peer; audio is back within ~4 s on a cold boot.

## Known caveats

- **Siri on a locked phone is restricted.** When Siri is triggered
  over HFP rather than over CarPlay, Apple blocks nav and messaging
  actions until the phone is unlocked. Plan your UX around it.
- **Hot reboot is recovered via panic.** Roughly 1 in 10 hot
  reboots still needs a second auto-restart (the eager profile
  connect races the controller's ACL bookkeeping in a way that
  doesn't always surface `HciConnectionExists`). The 1-second delay
  before the panic in `bt/gap.rs` gives the controller a moment to
  resolve the collision naturally.

## Architecture at a glance

```text
   BT host cb ──► DecoderMsg chan (32) ──► decoder task ──► I2SAudioOutput
     │                                                          ▲   │
     │                                                          │   ▼
     ▼                                  HFP RecvData ───────────┘  DAC
   AVRCP cb                              │
     │                                   ▼
     ▼                          PcmFrame chan (12)
   metadata logs                          ▲
                                          │
                                  HFP SendData ◄── mic ring ◄── mic task
```

Source layout:

```
src/
├── main.rs              app entry, peripherals, profile orchestration
├── console.rs           stdin REPL → AVRCP / HFP voice
├── stats.rs             pipeline counters
├── audio/
│   ├── decoder.rs       A2DP frame → PCM (esp_audio_codec)
│   ├── i2s_input.rs     SPH0645 capture, DC blocker, gain, ring
│   └── i2s_output.rs    DAC writer, format-on-data reconfig
└── bt/
    ├── mod.rs           shared BtDriver init
    ├── gap.rs           device name, discoverable, ACL events, bond persist
    ├── a2dp.rs          external-codec sink + codec parse
    ├── avrcp.rs         CT passthrough + metadata
    ├── hfp.rs           HFP HF + SCO RX/TX + voice recognition
    └── pair_cache.rs    NVS pair persistence
```

## Upstream

`Cargo.toml` pins three forks that carry the patches this example
relies on. They are intended to be upstreamable:

- [`billylindeman/esp-idf-svc @ billy/a2dp-external-codec-api`](https://github.com/billylindeman/esp-idf-svc/tree/billy/a2dp-external-codec-api)
  — Rust bindings for the external-codec A2DP API.
- [`billylindeman/esp-idf-hal @ billy/i2s-reconfigure-support`](https://github.com/billylindeman/esp-idf-hal/tree/billy/i2s-reconfigure-support)
  — `I2sDriver::tx_reconfigure_std` / `rx_reconfigure_std` so
  format-on-data PCM routing can reconfig the DAC without tearing
  it down.
- [`billylindeman/esp-audio-codec-rs @ billy/patch-for-sbc-bug`](https://github.com/billylindeman/esp-audio-codec-rs/tree/billy/patch-for-sbc-bug)
  — Rust bindings for Espressif's `esp_audio_codec` with an
  FFI-boundary workaround for an upstream bug in
  `esp_sbc_dec_decode` (writes remaining-bytes into
  `raw->consumed` instead of bytes-consumed; visible only on
  multi-frame A2DP-SBC inputs).
  Tracking: [esp-adf-libs#75](https://github.com/espressif/esp-adf-libs/issues/75).

`esp-idf-sys` and `embuild` track upstream `master` unmodified.

## License

MIT — copy and adapt freely.
