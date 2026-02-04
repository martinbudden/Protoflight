use std::time::Instant;

/// Example function to be timed
fn heavy_computation() {
    let mut sum = 0u64;
    for i in 0..10_000_000 {
        sum = sum.wrapping_add(i);
    }
    println!("Sum: {}", sum);
}

fn main() {
    // Only compile this block in debug builds
    #[cfg(debug_assertions)]
    {
        let start = Instant::now();
        heavy_computation();
        let duration = start.elapsed();
        println!("(Debug) Execution time: {:?}", duration);
    }

    // In release builds, just run without timing
    #[cfg(not(debug_assertions))]
    {
        heavy_computation();
    }
}
[features]
timing = []

cargo run --features timing

Instant is monotonic and ideal for measuring elapsed time.
Conditional compilation ensures no performance penalty in builds where timing is disabled.
You can target:
#[cfg(debug_assertions)] → debug builds only.
#[cfg(not(debug_assertions))] → release builds only.
#[cfg(feature = "timing")] → custom feature flag.


Got it ✅ — you want a no_std Rust timing utility that works across multiple architectures (x86, ARM, RISC‑V, ESP32), with build‑variant‑specific timing.

In no_std we can’t use std::time::Instant, so we must rely on architecture‑specific cycle counters or hardware timers.

Approach
#[cfg(...)] selects the correct timing method for each architecture.
Use core::arch intrinsics where available.
For ESP32 (Xtensa), use the xtensa_lx or esp-idf HAL cycle counter.
Works in no_std environments (embedded, bare‑metal).
Still supports conditional compilation for enabling/disabling timing.

#![no_std]
#![no_main]

use core::panic::PanicInfo;

/// Architecture-specific cycle counter
#[inline(always)]
fn get_cycles() -> u64 {
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    unsafe {
        use core::arch::x86_64::_rdtsc;
        return _rdtsc() as u64;
    }

    #[cfg(target_arch = "arm")]
    unsafe {
        // ARM Performance Monitor Cycle Counter
        let r: u32;
        core::arch::asm!("mrc p15, 0, {0}, c9, c13, 0", out(reg) r);
        return r as u64;
    }

    #[cfg(target_arch = "aarch64")]
    unsafe {
        let r: u64;
        core::arch::asm!("mrs {0}, cntvct_el0", out(reg) r);
        return r;
    }

    #[cfg(target_arch = "riscv32")]
    unsafe {
        let r: u64;
        core::arch::asm!("rdcycle {0}", out(reg) r);
        return r;
    }

    #[cfg(target_arch = "riscv64")]
    unsafe {
        let r: u64;
        core::arch::asm!("rdcycle {0}", out(reg) r);
        return r;
    }

    #[cfg(target_arch = "xtensa")]
    unsafe {
        extern "C" {
            fn xthal_get_ccount() -> u32;
        }
        return xthal_get_ccount() as u64;
    }

    #[allow(unreachable_code)]
    0
}

/// Example workload
fn heavy_computation() {
    let mut sum: u64 = 0;
    for i in 0..1_000_000 {
        sum = sum.wrapping_add(i);
    }
    unsafe {
        // Replace with your board's output method
        dummy_output(sum);
    }
}

/// Dummy output function (replace with UART/LED/etc.)
unsafe fn dummy_output(_val: u64) {
    // No-op in this example
}

/// Entry point
#[no_mangle]
pub extern "C" fn main() -> ! {
    #[cfg(feature = "timing")]
    {
        let start = get_cycles();
        heavy_computation();
        let end = get_cycles();
        let elapsed = end.wrapping_sub(start);

        unsafe { dummy_output(elapsed) };
    }

    #[cfg(not(feature = "timing"))]
    {
        heavy_computation();
    }

    loop {}
}

/// Panic handler for no_std
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

# Debug build for x86_64 with timing
cargo build --target x86_64-unknown-none --features timing

# ARM Cortex-M (example: thumbv7em)
cargo build --target thumbv7em-none-eabihf --features timing

# RISC-V 32-bit
cargo build --target riscv32imac-unknown-none-elf --features timing

# ESP32 (Xtensa)
cargo build --target xtensa-esp32-none-elf --features timing

Key Notes
Cycle counters are architecture‑specific and may require enabling in hardware (e.g., ARM PMU).
On ESP32, you need the Xtensa HAL or esp-idf bindings to access xthal_get_ccount().
This is no_std safe — no heap, no OS dependencies.
The #[cfg(feature = "timing")] flag ensures zero overhead when timing is disabled.

#[cfg(feature = "timing")] → enables timing only when requested.
Cycle counters are hardware-specific:
x86/x86_64 → RDTSC
ARM → PMU register
AArch64 → cntvct_el0
RISC‑V → rdcycle
ESP32 → xthal_get_ccount
no_std safe — no heap, no OS calls.
Replace dummy_output() with your board’s UART, logging, or LED output.
If you want, I can now turn this into a macro so you can wrap any function call like:

Rust

Copy code
time_call!(heavy_computation());
and it will automatically measure cycles only for the architectures and build variants you specify.

Do you want me to prepare that macro version next?

#![no_std]
#![no_main]

use core::panic::PanicInfo;

/// Architecture-specific cycle counter
#[inline(always)]
fn get_cycles() -> u64 {
    // x86 / x86_64
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    unsafe {
        use core::arch::x86_64::_rdtsc;
        return _rdtsc() as u64;
    }

    // ARM 32-bit
    #[cfg(target_arch = "arm")]
    unsafe {
        let r: u32;
        core::arch::asm!("mrc p15, 0, {0}, c9, c13, 0", out(reg) r);
        return r as u64;
    }

    // ARM 64-bit
    #[cfg(target_arch = "aarch64")]
    unsafe {
        let r: u64;
        core::arch::asm!("mrs {0}, cntvct_el0", out(reg) r);
        return r;
    }

    // RISC-V 32/64
    #[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
    unsafe {
        let r: u64;
        core::arch::asm!("rdcycle {0}", out(reg) r);
        return r;
    }

    // ESP32 (Xtensa)
    #[cfg(target_arch = "xtensa")]
    unsafe {
        extern "C" {
            fn xthal_get_ccount() -> u32;
        }
        return xthal_get_ccount() as u64;
    }

    // Fallback
    #[allow(unreachable_code)]
    0
}

/// Macro to time a function call in CPU cycles
#[macro_export]
macro_rules! time_call {
    ($expr:expr) => {{
        #[cfg(feature = "timing")]
        {
            let start = get_cycles();
            let result = $expr;
            let end = get_cycles();
            let elapsed = end.wrapping_sub(start);
            unsafe { dummy_output(elapsed) };
            result
        }
        #[cfg(not(feature = "timing"))]
        {
            $expr
        }
    }};
}

/// Example workload
fn heavy_computation() -> u64 {
    let mut sum: u64 = 0;
    for i in 0..1_000_000 {
        sum = sum.wrapping_add(i);
    }
    sum
}

/// Dummy output (replace with UART, LED, etc.)
unsafe fn dummy_output(val: u64) {
    // In real embedded code, send `val` over UART or log it
    let _ = val; // suppress unused warning
}

/// Entry point
#[no_mangle]
pub extern "C" fn main() -> ! {
    // Call with timing (only active if `--features timing` is set)
    let result = time_call!(heavy_computation());

    unsafe { dummy_output(result) };

    loop {}
}

/// Panic handler for no_std
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
