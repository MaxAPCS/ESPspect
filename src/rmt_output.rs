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
                    resolution: RMT_FREQ,
                    memory_access: config::MemoryAccess::Indirect {
                        memory_block_symbols: 512, // max 512
                    },
                    transaction_queue_depth: 2,
                    interrupt_priority: 3, // max 3
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
            &std::array::from_fn::<_, 21, _>(|i| {
                if i == 0 {
                    return WSItem::Reset;
                }
                let x = buf[i] * 300.;
                WSItem::RGB(RGB8::uncorrected_rgb(
                    (x - 255.).clamp(0., 255.) as usize,
                    x.clamp(0., 255.) as usize,
                    0,
                ))
            }),
            &config::TransmitConfig {
                queue_non_blocking: false,
                ..Default::default()
            },
        )
    }
}

const RMT_FREQ: Hertz = Hertz(20_000_000);
static WS_BITS: LazyLock<[Symbol; 2]> = LazyLock::new(|| {
    [
        Symbol::new_with(
            RMT_FREQ,
            PinState::High,
            Duration::from_nanos(300),
            PinState::Low,
            Duration::from_nanos(950),
        )
        .unwrap(),
        Symbol::new_with(
            RMT_FREQ,
            PinState::High,
            Duration::from_nanos(950), //600ns
            PinState::Low,
            Duration::from_nanos(300), //650ns
        )
        .unwrap(),
    ]
});
static WS_RESET: LazyLock<[Symbol; 1]> = LazyLock::new(|| {
    [Symbol::new_half_split(
        RMT_FREQ,
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
    type Item = WSItem;

    fn encode(
        &mut self,
        input_data: &[Self::Item],
        buffer: &mut simple_encoder::SymbolBuffer<'_>,
    ) -> Result<(), simple_encoder::NotEnoughSpace> {
        if buffer.position() == 0 {
            self.input_position = 0;
        }

        for next_color in &input_data[self.input_position..] {
            match next_color {
                WSItem::Reset => buffer.write_all(&*WS_RESET)?,
                WSItem::RGB(rgb8) => buffer.write_all(&rgb8.to_ws())?,
            }
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

#[derive(Clone)]
enum WSItem {
    Reset,
    RGB(RGB8),
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
        let packed = (self.blue as usize) << 16 | (self.red as usize) << 8 | (self.green as usize);
        std::array::from_fn(|i| WS_BITS[packed >> (23 - i) & 1])
    }
}
