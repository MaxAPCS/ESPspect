use crate::audio::AudioOutputFormat;

pub struct FFTOutput {}
impl FFTOutput {
    fn send_pcm(format: AudioOutputFormat, samples: Vec<u8>) {
        let fft = microfft::real::rfft_1024(input);
    }
}
