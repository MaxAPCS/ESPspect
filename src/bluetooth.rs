use std::sync::Arc;

use esp_idf_svc::{
    bt::{BtClassic, BtDriver, BtStatus, a2dp, gap, reduce_bt_memory},
    hal::modem::Modem,
    nvs::EspNvsPartition,
    sys::EspError,
};

use crate::pair_cache::PairCache;

const DOWNMIX_BUF_SIZE: usize = 2560; // SBC Stereo 44.1k = 1280

pub struct Bluetooth<'a> {
    driver: Arc<BtDriver<'a, BtClassic>>,
}

impl<'a> Bluetooth<'a> {
    pub fn init(
        mut modem: Modem<'a>,
        nvs: EspNvsPartition<esp_idf_svc::nvs::NvsDefault>,
    ) -> Result<Self, EspError> {
        reduce_bt_memory(unsafe { modem.reborrow() })?;
        Ok(Self {
            driver: Arc::new(BtDriver::<BtClassic>::new(modem, Some(nvs))?),
        })
    }

    pub fn start_a2dp(
        &self,
        mut callback: impl FnMut(&[u8]) + Send + 'static,
    ) -> Result<a2dp::EspA2dp<'a, BtClassic, Arc<BtDriver<'a, BtClassic>>, a2dp::Sink>, EspError>
    {
        let a2dp = a2dp::EspA2dp::new(self.driver.clone())?;
        let mut stereo = true;
        let mut downmix_buf: [u8; DOWNMIX_BUF_SIZE] = [0; _];
        a2dp.subscribe(move |event| match event {
            a2dp::A2dpEvent::AudioCodecConfigured { codec, .. } => {
                stereo = codec.stereo().unwrap_or_default();
                0
            }
            a2dp::A2dpEvent::SinkData(buf) => {
                if stereo {
                    let pairs = unsafe { buf.as_chunks_unchecked::<2>() };
                    debug_assert!(pairs.len() <= DOWNMIX_BUF_SIZE, "downmix buffer overflow");
                    for (m, &[l, r]) in Iterator::zip(downmix_buf.iter_mut(), pairs.into_iter()) {
                        *m = l / 2 + r / 2;
                    }
                    callback(&downmix_buf[..usize::min(DOWNMIX_BUF_SIZE, pairs.len())])
                } else {
                    callback(buf);
                };
                0
            }
            _ => 0,
        })?;
        Ok(a2dp)
    }

    pub fn start_gap(
        &self,
        device_name: &str,
        pair_cache: Arc<PairCache>,
    ) -> Result<gap::EspGap<'a, BtClassic, Arc<BtDriver<'a, BtClassic>>>, EspError> {
        let gap = gap::EspGap::new(self.driver.clone())?;
        gap.set_device_name(device_name)?;
        gap.set_scan_mode(true, gap::DiscoveryMode::Discoverable)?;
        gap.subscribe(move |event| {
            if let gap::GapEvent::AuthenticationCompleted {
                status: BtStatus::Success,
                bd_addr,
                ..
            } = event
            {
                pair_cache.write(&bd_addr)
            }
        })?;
        Ok(gap)
    }
}
