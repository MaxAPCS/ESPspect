use std::env;
use std::fs;
use std::path::Path;

fn main() {
    embuild::espidf::sysenv::output();

    // Read the constant out of your source file.
    let src = fs::read_to_string("src/fft_analysis.rs").expect("src/fft_analysis.rs missing");
    let n = src
        .lines()
        .find_map(|line| {
            let line = line.trim();
            line.strip_prefix("const WINDOW_SIZE: usize = ")?
                .trim_end_matches(';')
                .parse::<usize>()
                .ok()
        })
        .expect("WINDOW_SIZE missing");

    // Compute the window using whatever size we found
    let mut table = vec![0.; n];
    let n_f = n as f32;
    for i in 0..n {
        // pre-divide the u8 [0, 255] -> f32 [-1, 1] conversion factor (128)
        table[i] = (1. - (2. * std::f32::consts::PI * i as f32 / n_f).cos()) / 256.
    }

    let table_str = table
        .into_iter()
        .map(|v| format!("{v:e}"))
        .collect::<Vec<_>>()
        .join(", ");

    let out = format!("pub static HANN_PREDIV: [f32; {n}] = [{table_str}];\n");

    let out_dir = env::var("OUT_DIR").unwrap();
    let dest = Path::new(&out_dir).join("hann_window.rs");
    fs::write(&dest, out).expect("failed to write hann_window.rs");

    // Re-run when build.rs OR the source file changes
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/fft_analysis.rs");
}
