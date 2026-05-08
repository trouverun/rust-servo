#[allow(unused_comparisons, unreachable_patterns, unexpected_cfgs, clippy::all)]
pub mod messages {
    include!(concat!(env!("OUT_DIR"), "/messages.rs"));
}
pub mod transport;
