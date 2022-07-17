#![no_std]

/// Handles a panic
#[cfg_attr(not(test), panic_handler)]
fn panic_handler(_panic: &core::panic::PanicInfo) -> ! {
    loop { /* Loop forever */ }
}

/// The main entry function
#[no_mangle]
pub extern "C" fn rust_entry() {
    todo!()
}
