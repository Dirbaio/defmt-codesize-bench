//! [`defmt`](https://github.com/knurling-rs/defmt) global logger over RTT.
//!
//! NOTE when using this crate it's not possible to use (link to) the `rtt-target` crate
//!
//! To use this crate, link to it by importing it somewhere in your project.
//!
//! ```
//! // src/main.rs or src/bin/my-app.rs
//! use defmt_rtt as _;
//! ```

use core::{
    mem::MaybeUninit,
    ptr,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

use cortex_m::{interrupt, register};
use rzcobs::Write;

// TODO make configurable
// NOTE use a power of 2 for best performance
const SIZE: usize = 128 * 1024;

#[defmt::global_logger]
struct Logger;

static TAKEN: AtomicUsize = AtomicUsize::new(0);
static INTERRUPTS_ACTIVE: AtomicBool = AtomicBool::new(false);
static BYTES_WRITTEN: AtomicUsize = AtomicUsize::new(0);

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        let primask = register::primask::read();
        interrupt::disable();
        let taken = TAKEN.load(Ordering::Relaxed);
        TAKEN.store(taken + 1, Ordering::Relaxed);
        if taken == 0 {
            INTERRUPTS_ACTIVE.store(primask.is_active(), Ordering::Relaxed);
            unsafe {
                RCOBS_ENCODER
                    .as_mut_ptr()
                    .write(rzcobs::Encoder::new(Writer));
            }
        }
    }

    unsafe fn release() {
        let taken = TAKEN.load(Ordering::Relaxed);
        TAKEN.store(taken - 1, Ordering::Relaxed);
        if taken == 1 {
            let e = &mut *RCOBS_ENCODER.as_mut_ptr();
            e.end().unwrap();
            Writer.write(0).unwrap();

            if INTERRUPTS_ACTIVE.load(Ordering::Relaxed) {
                // re-enable interrupts
                interrupt::enable()
            }
        }
    }

    unsafe fn write(bytes: &[u8]) {
        if TAKEN.load(Ordering::Relaxed) == 1 {
            let e = &mut *RCOBS_ENCODER.as_mut_ptr();
            for &b in bytes {
                e.write(b).unwrap();
            }
        }
    }
}
static mut RCOBS_ENCODER: MaybeUninit<rzcobs::Encoder<Writer>> = MaybeUninit::uninit();

struct Writer;
impl rzcobs::Write for Writer {
    type Error = core::convert::Infallible;
    fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
        BYTES_WRITTEN.fetch_add(1, Ordering::Relaxed);
        unsafe { write_byte(byte) };
        Ok(())
    }
}

pub fn bytes_written() -> usize {
    BYTES_WRITTEN.load(Ordering::Relaxed)
}

#[repr(C)]
struct Header {
    id: [u8; 16],
    max_up_channels: usize,
    max_down_channels: usize,
    up_channel: Channel,
}

#[repr(C)]
struct Channel {
    name: *const u8,
    buffer: *mut u8,
    size: usize,
    write: AtomicUsize,
    read: AtomicUsize,
    flags: AtomicUsize,
}

unsafe fn write_byte(b: u8) {
    let c = &_SEGGER_RTT.up_channel;

    let mut w = c.write.load(Ordering::Acquire);
    BUFFER[w] = b;
    w += 1;
    if w >= SIZE {
        w = 0;
    }
    c.write.store(w, Ordering::Release);
}

const BLOCK_IF_FULL: usize = 2;
const NOBLOCK_TRIM: usize = 1;

// NOTE the `rtt-target` API is too permissive. It allows writing arbitrary data to any
// channel (`set_print_channel` + `rprint*`) and that can corrupt defmt log frames.
// So we declare the RTT control block here and make it impossible to use `rtt-target` together
// with this crate.
#[no_mangle]
static mut _SEGGER_RTT: Header = Header {
    id: *b"SEGGER RTT\0\0\0\0\0\0",
    max_up_channels: 1,
    max_down_channels: 0,
    up_channel: Channel {
        name: NAME as *const _ as *const u8,
        buffer: unsafe { &mut BUFFER as *mut _ as *mut u8 },
        size: SIZE,
        write: AtomicUsize::new(0),
        read: AtomicUsize::new(0),
        flags: AtomicUsize::new(NOBLOCK_TRIM),
    },
};

#[cfg_attr(target_os = "macos", link_section = ".uninit,defmt-rtt.BUFFER")]
#[cfg_attr(not(target_os = "macos"), link_section = ".uninit.defmt-rtt.BUFFER")]
static mut BUFFER: [u8; SIZE] = [0; SIZE];

static NAME: &[u8] = b"defmt\0";
