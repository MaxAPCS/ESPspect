use std::{mem, sync::LazyLock, time::Duration};

use esp_idf_svc::{
    hal::{
        gpio,
        rmt::{PinState, Symbol, TxChannelDriver, config, encoder::simple_encoder},
        units::Hertz,
    },
    sys::EspError,
};

use crate::LEDOutput;

const NUM_ENCODERS: usize = 1;
pub struct RMTOutput<'a> {
    driver: TxChannelDriver<'a>,
    encoders: [simple_encoder::SimpleEncoder<WSEncoder>; NUM_ENCODERS],
}

impl<'a> LEDOutput<'a> for RMTOutput<'a> {
    fn init(pin: impl gpio::OutputPin + 'a) -> Result<Self, EspError> {
        Ok(Self {
            driver: TxChannelDriver::new(
                pin,
                &config::TxChannelConfig {
                    resolution: RMTRESOULTION,
                    memory_access: config::MemoryAccess::Indirect {
                        memory_block_symbols: 24 * 20,
                    },
                    transaction_queue_depth: 4,
                    ..Default::default()
                },
            )?,
            encoders: {
                let mut out = [const { mem::MaybeUninit::uninit() }; NUM_ENCODERS];
                for x in &mut out {
                    x.write(WSEncoder::create()?);
                }
                unsafe { mem::transmute(out) }
            },
        })
    }

    fn send_amplitudes(&mut self, buf: &mut [f32]) -> Result<(), EspError> {
        self.driver.queue(&mut self.encoders).push(
            &std::array::from_fn::<_, 20, _>(|i| {
                let x = buf[i].sqrt();
                RGB8::uncorrected_rgb(
                    0, // (x - 255.).clamp(0., 255.) as usize
                    x.clamp(0., 255.) as usize,
                    0,
                )
            }),
            &config::TransmitConfig {
                queue_non_blocking: false,
                ..Default::default()
            },
        )
    }
}

const RMTRESOULTION: Hertz = Hertz(6_250_000); // 160ns
static WS_BITS: LazyLock<[Symbol; 2]> = LazyLock::new(|| {
    [
        Symbol::new_with(
            RMTRESOULTION,
            PinState::High,
            Duration::from_nanos(320),
            PinState::Low,
            Duration::from_nanos(960),
        )
        .unwrap(),
        Symbol::new_with(
            RMTRESOULTION,
            PinState::High,
            Duration::from_nanos(960),
            PinState::Low,
            Duration::from_nanos(320),
        )
        .unwrap(),
    ]
});
static WS_RESET: LazyLock<[Symbol; 1]> = LazyLock::new(|| {
    [Symbol::new_half_split(
        RMTRESOULTION,
        PinState::Low,
        PinState::Low,
        Duration::from_micros(280),
    )
    .unwrap()]
});

struct WSEncoder {
    input_position: usize,
}

impl simple_encoder::EncoderCallback for WSEncoder {
    type Item = RGB8;

    fn encode(
        &mut self,
        input_data: &[Self::Item],
        buffer: &mut simple_encoder::SymbolBuffer<'_>,
    ) -> Result<(), simple_encoder::NotEnoughSpace> {
        if buffer.position() == 0 {
            self.input_position = 0;
            buffer.write_all(&*WS_RESET)?;
        }

        for next_color in &input_data[self.input_position..] {
            buffer.write_all(&next_color.to_ws())?;
            self.input_position += 1;
        }

        Ok(())
    }
}

impl WSEncoder {
    fn create() -> Result<simple_encoder::SimpleEncoder<Self>, EspError> {
        simple_encoder::SimpleEncoder::with_config(
            Self { input_position: 0 },
            &simple_encoder::SimpleEncoderConfig {
                min_chunk_size: 24,
                ..Default::default()
            },
        )
    }
}

gamma_table_macros::gamma_table! {
    name: GAMMA,
    entry_type: u8,
    gamma: 2.2,
    size: 256
}

#[derive(Clone)]
struct RGB8 {
    red: u8,
    green: u8,
    blue: u8,
}

impl RGB8 {
    #[inline]
    fn uncorrected_rgb(r: usize, g: usize, b: usize) -> Self {
        Self {
            red: GAMMA[r],
            green: GAMMA[g],
            blue: GAMMA[b],
        }
    }

    #[inline]
    fn to_ws(&self) -> [Symbol; 24] {
        let packed = (self.red as usize) << 16 | (self.green as usize) << 8 | (self.blue as usize);
        std::array::from_fn(|i| WS_BITS[packed >> (23 - i) & 1])
    }
}
