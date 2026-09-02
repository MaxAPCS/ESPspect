use std::thread;

use esp_idf_svc::hal::{
    cpu::Core, delay::FreeRtos, gpio, spi, sys::EspError, task::thread::ThreadSpawnConfiguration,
    units::Hertz,
};
use rtrb::{Consumer, Producer, RingBuffer};

gamma_table_macros::gamma_table! {
    name: GAMMA,
    entry_type: u8,
    gamma: 2.2,
    size: 256
}

const LED_COUNT: usize = 20;
static RESET_BUFFER: [u8; 260] = [0; _]; // 240 bytes / 6.4Mb/s = 300us

pub struct LEDOutput {
    tx: Producer<[RGB8; LED_COUNT]>,
}
impl LEDOutput {
    pub fn spawn<SPI: spi::SpiAnyPins + 'static>(
        spi: SPI,
        sdo: impl gpio::OutputPin + 'static,
    ) -> anyhow::Result<Self> {
        let (tx, rx) = RingBuffer::new(4);
        ThreadSpawnConfiguration {
            name: Some(c"led_output"),
            priority: 10,
            pin_to_core: Some(Core::Core1),
            ..Default::default()
        }
        .set()?;
        thread::Builder::new()
            .stack_size(
                const {
                    let raw = 4 * 1024 + size_of::<[RGB8; LED_COUNT]>();
                    (raw as f32 / 1024.).ceil() as usize * 1024
                },
            )
            .spawn(move || Self::event_loop(rx, spi, sdo))?;
        ThreadSpawnConfiguration::default().set()?;
        Ok(Self { tx })
    }

    pub fn send_amplitudes(&mut self, buf: &mut [f32]) -> Result<(), ()> {
        self.tx
            .push(std::array::from_fn(|i| {
                let x = buf[i].sqrt();
                RGB8::uncorrected_rgb(
                    (x - 255.).clamp(0., 255.) as usize,
                    x.clamp(0., 255.) as usize,
                    0,
                )
            }))
            .map_err(|_| {})
    }

    #[inline]
    fn event_loop<SPI: spi::SpiAnyPins + 'static>(
        mut rx: Consumer<[RGB8; LED_COUNT]>,
        spi: SPI,
        sdo: impl gpio::OutputPin + 'static,
    ) -> Result<(), EspError> {
        let driver = spi::SpiDriver::new_without_sclk(
            spi,
            sdo,
            None::<gpio::AnyInputPin>,
            &spi::config::DriverConfig {
                // dma: spi::Dma::Auto(size_of_val(&RESET_BUFFER) + size_of::<[u8; 24 * LED_COUNT]>()), TODO https://github.com/georgik/esp-display-interface-spi-dma/blob/main/src/display_interface_spi_dma.rs
                ..Default::default()
            },
        )?;
        let mut busdriver = spi::SpiBusDriver::new(
            driver,
            &spi::config::Config {
                baudrate: Hertz(6_400_000),
                data_mode: spi::config::MODE_0,
                write_only: true,
                duplex: spi::config::Duplex::Full,
                bit_order: spi::config::BitOrder::MsbFirst,
                polling: true,
                ..Default::default()
            },
        )?;
        loop {
            FreeRtos::delay_ms(1000);
            let Ok(leds) = rx.pop() else {
                continue;
            };
            for item in leds {
                busdriver.write(&item.to_ws_bytes())?;
            }
            busdriver.write(&RESET_BUFFER)?;
        }
    }
}

/// 0xRRGGBB00
#[repr(C)]
struct RGB8(u32);

impl RGB8 {
    #[inline]
    fn uncorrected_rgb(r: usize, g: usize, b: usize) -> Self {
        Self((GAMMA[g] as u32) << 24 | (GAMMA[r] as u32) << 16 | (GAMMA[b] as u32) << 8)
    }

    #[inline]
    const fn to_ws_bytes(mut self) -> [u8; 24] {
        static PATTERNS: [u8; 2] = [0b11000000, 0b11111100]; // 313ns, 938ns

        let mut out = [0; _];
        let mut i = out.len();
        while i > 0 {
            i -= 1;
            let msb = (self.0 >> 31) & 1;
            self.0 <<= 1;
            out[i] = PATTERNS[msb as usize];
        }

        out
    }
}
