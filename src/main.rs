use std::sync::Arc;

use esp_idf_svc::{
    hal::{delay::FreeRtos, gpio, peripherals::Peripherals},
    log::EspLogger,
    nvs::EspDefaultNvsPartition,
};

mod bluetooth;
mod fft_analysis;
mod pair_cache;
mod rmt_output;
use crate::fft_analysis::FFTAnalysis;
use crate::pair_cache::PairCache;
use crate::{bluetooth::Bluetooth, rmt_output::RMTOutput};

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let mut fft_analysis = FFTAnalysis::spawn::<RMTOutput>(peripherals.pins.gpio19)?;
    let pair_cache = Arc::new(PairCache::open(nvs.clone())?);

    // iPhone caches SDP record from FIRST connection;
    // profiles not registered before GAP won't appear in cache;
    // peer will refuse to open channel later.
    let bluetooth = Bluetooth::init(peripherals.modem, nvs)?;
    let a2dp = bluetooth.start_a2dp(move |buf| {
        fft_analysis.send_pcm(buf).ok();
    })?;
    bluetooth.start_gap("espspect", pair_cache.clone())?;

    // Boot reconnect: fire A2DP connect directly. Cold path
    // hits power-on-to-music in ~4 s. Hot reboot races BTM's ACL
    // bookkeeping and surfaces HciConnectionExists; that wedge has
    // no clean in-process recovery, so the plan is to detect it in
    // GAP and panic — esp-idf auto-restarts and the next cold boot
    // brings us back in ~4 s.
    if let Some(addr) = pair_cache.read() {
        a2dp.connect_sink(&addr).ok();
    }

    loop {
        FreeRtos::delay_ms(10_000);
    }
}

pub trait LEDOutput<'a> {
    fn init(pin: impl gpio::OutputPin + 'a) -> Result<Self, esp_idf_svc::sys::EspError>
    where
        Self: Sized;
    fn send_amplitudes(&mut self, buf: &mut [f32]) -> Result<(), esp_idf_svc::sys::EspError>;
}
