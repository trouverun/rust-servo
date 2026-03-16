use std::env;

fn main() {
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    let profile = env::var("PROFILE").unwrap();

    match profile.as_str() {
        "release" => {
            // In release, we only want errors (silences info/debug/trace)
            // This reduces binary size and interrupt latency
            println!("cargo:rustc-env=DEFMT_LOG=error");
        }
        _ => {
            // In debug (default), we want everything
            println!("cargo:rustc-env=DEFMT_LOG=trace");
        }
    }
    
    // Ensure build.rs re-runs if the environment changes (good practice)
    println!("cargo:rerun-if-changed=build.rs");
}
