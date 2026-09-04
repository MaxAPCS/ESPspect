use std::{mem, thread};

use esp_idf_svc::hal::{cpu::Core, delay::FreeRtos, gpio, task::thread::ThreadSpawnConfiguration};
use microfft::{Complex32, real::rfft_128};
use rtrb::{Consumer, Producer, RingBuffer, chunks::ChunkError};

use crate::LEDOutput;
include!(concat!(env!("OUT_DIR"), "/hann_window.rs"));

const WINDOW_SIZE: usize = 128;

pub struct FFTAnalysis {
    tx: Producer<u8>,
}

impl FFTAnalysis {
    pub fn spawn<'a, T: LEDOutput<'a>>(
        output_pin: impl gpio::OutputPin + 'static,
    ) -> anyhow::Result<Self> {
        let (tx, rx) = RingBuffer::new(4 * WINDOW_SIZE);
        ThreadSpawnConfiguration {
            name: Some(c"fft_analysis"),
            priority: 20,
            pin_to_core: Some(Core::Core1),
            ..Default::default()
        }
        .set()?;
        thread::Builder::new()
            .stack_size(
                const {
                    let raw = 8 * 1024 // wiggle room
                        + size_of::<[u8; WINDOW_SIZE]>() // window
                        + size_of::<[f32; WINDOW_SIZE]>() // window_f
                        + size_of::<[u8; 3 * 20]>(); // send_amplitudes buffer
                    (raw as f32 / 1024.).ceil() as usize * 1024
                },
            )
            .spawn(move || Self::event_loop(rx, T::init(output_pin).unwrap()))?;

        ThreadSpawnConfiguration::default().set()?;
        Ok(Self { tx })
    }

    pub fn send_pcm(&mut self, buffer: &[u8]) -> Result<(), ChunkError> {
        self.tx.push_entire_slice(buffer)
    }

    #[inline]
    fn event_loop<'a, T: LEDOutput<'a>>(mut rx: Consumer<u8>, mut output: T) {
        loop {
            let mut window = [mem::MaybeUninit::uninit(); WINDOW_SIZE];
            let Ok(window) = rx.pop_entire_slice_uninit(&mut window) else {
                FreeRtos::delay_ms(1);
                continue;
            };

            let mut window_f: [f32; WINDOW_SIZE] =
                std::array::from_fn(|i| (window[i] as f32 - 128.) * HANN_PREDIV[i]);

            let spectrum = Self::reinterpret_complex(rfft_128(&mut window_f));
            spectrum[1] = 0.; // clear packed nyquist freq
            for i in 0..WINDOW_SIZE / 2 {
                spectrum[i] =
                    spectrum[2 * i] * spectrum[2 * i] + spectrum[2 * i + 1] * spectrum[2 * i + 1]
            }

            output
                .send_amplitudes(Self::reinterpret_truncate(spectrum))
                .ok();
        }
    }

    #[inline]
    fn reinterpret_complex(x: &mut [Complex32; WINDOW_SIZE / 2]) -> &mut [f32; WINDOW_SIZE] {
        unsafe { &mut *(x as *mut _ as *mut _) }
    }

    #[inline]
    fn reinterpret_truncate(x: &mut [f32; WINDOW_SIZE]) -> &mut [f32; WINDOW_SIZE / 2] {
        unsafe { &mut *(x as *mut _ as *mut _) }
    }
}
