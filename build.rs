#![feature(float_algebraic)]

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
    let mut sum = 0.;
    for i in 0..n {
        let val = 1f32
            .algebraic_sub(f32::cos(
                std::f32::consts::PI
                    .algebraic_mul((2 * i) as f32)
                    .algebraic_div(n as f32),
            ))
            .algebraic_div(2.); // (1 - cos((pi * 2 * i) / n))/2
        table[i] = val;
        sum += val;
    }

    let table_str = table
        .into_iter()
        .map(|v| {
            // (2v / 128) / sum
            v.algebraic_mul(2.)
                .algebraic_div(128.)
                .algebraic_div(sum)
                .to_bits()
                .to_string()
        })
        .collect::<Vec<_>>()
        .join(", ");

    let out = format!("pub static HANN_PREDIV: [u32; {n}] = [{table_str}];\n");

    let out_dir = env::var("OUT_DIR").unwrap();
    let dest = Path::new(&out_dir).join("hann_window.rs");
    fs::write(&dest, out).expect("failed to write hann_window.rs");

    // Re-run when build.rs OR the source file changes
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/fft_analysis.rs");
}
