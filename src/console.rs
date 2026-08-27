//! Stdin REPL — single-char commands wired to AVRCP transport control
//! and HFP voice-recognition. Polls stdin one byte at a time, sleeping
//! when the ROM UART driver reports no data — same pattern as the cpp
//! project's `console_task` (which calls `getchar()` and treats EOF as
//! "wait and retry"). Works under both `idf.py monitor` and
//! `espflash monitor` without any VFS reconfiguration.

#![cfg(all(esp32, esp_idf_bt_a2dp_use_external_codec))]

use std::io::{ErrorKind, Read};
use std::thread;
use std::time::Duration;

use log::{info, warn};

use crate::bt::avrcp::AvrcpController;
use crate::bt::hfp::HfpVoiceHandle;

/// Polling interval when stdin reports no data.
const POLL_IDLE_MS: u64 = 50;
/// Slow-poll interval when stdin returns a hard error.
const POLL_ERR_MS: u64 = 500;

/// Zero-sized handle for the console task. `Console::spawn` consumes the
/// AVRCP and HFP voice handles and runs an infinite stdin loop.
pub struct Console;

impl Console {
    pub fn spawn(
        avrcp: AvrcpController,
        hfp_voice: HfpVoiceHandle,
    ) -> std::io::Result<thread::JoinHandle<()>> {
        thread::Builder::new()
            .name("console".into())
            .stack_size(4 * 1024)
            .spawn(move || Self::event_loop(avrcp, hfp_voice))
    }

    fn event_loop(avrcp: AvrcpController, hfp_voice: HfpVoiceHandle) {
        info!("ready — p=play/pause  n=next  b=back  s=siri-start  S=siri-stop");
        let mut stdin = std::io::stdin();
        let mut buf = [0u8; 1];
        loop {
            match stdin.read(&mut buf) {
                // ROM UART driver returns 0 bytes when the RX FIFO is
                // empty; treat as "no key pressed", sleep, try again.
                Ok(0) => thread::sleep(Duration::from_millis(POLL_IDLE_MS)),
                Ok(_) => {
                    let c = buf[0];
                    if c == b'\n' || c == b'\r' {
                        continue;
                    }
                    Self::handle_key(c as char, &avrcp, &hfp_voice);
                }
                // Some std implementations surface "no data" as
                // WouldBlock instead of Ok(0) — handle both.
                Err(e) if e.kind() == ErrorKind::WouldBlock => {
                    thread::sleep(Duration::from_millis(POLL_IDLE_MS));
                }
                Err(e) => {
                    warn!("stdin: {e}");
                    thread::sleep(Duration::from_millis(POLL_ERR_MS));
                }
            }
        }
    }

    fn handle_key(c: char, avrcp: &AvrcpController, hfp_voice: &HfpVoiceHandle) {
        match c {
            'p' | 'P' => avrcp.play_pause(),
            'n' | 'N' => avrcp.next_track(),
            'b' | 'B' => avrcp.prev_track(),
            's' => hfp_voice.start_voice_recognition(),
            'S' | 'e' | 'E' => hfp_voice.stop_voice_recognition(),
            _ => info!("key={c:?} (unmapped)"),
        }
    }
}
