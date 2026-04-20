//! RM32 host-side test harness.
//!
//! Implements the same stdin/stdout line protocol as the C am32_harness,
//! allowing the Python blackbox tests to verify the Rust implementation
//! produces identical output to the C reference.
//!
//! Protocol:
//!   config key=value        Set state
//!   tick [key=value ...]    Advance one tick
//!   ticks N [key=value ...] Advance N ticks
//!   state                   Print current state
//!   reset                   Reset all state
//!   quit                    Exit

fn main() {
    eprintln!("rm32_harness: not yet implemented");
    eprintln!("This will implement the same protocol as am32_harness");
    eprintln!("so blackbox test vectors can run against both C and Rust.");
    std::process::exit(1);
}
