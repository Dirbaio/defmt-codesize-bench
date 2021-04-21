#![no_std]
#![no_main]

#[path = "../rtt.rs"]
mod rtt;

use core::cell::Cell;
use core::marker::PhantomData;
use core::str;
use core::sync::atomic::{AtomicUsize, Ordering};
use defmt::*;
use defmt::{debug, error, info};
use heapless::consts::*;
use heapless::{spsc::Queue, Vec};
use panic_probe as _;
use smoltcp::iface::{InterfaceBuilder, NeighborCache};
use smoltcp::phy::Medium;
use smoltcp::phy::{self, Device, DeviceCapabilities};
use smoltcp::socket::{SocketSet, TcpSocket, TcpSocketBuffer};
use smoltcp::time::{Duration, Instant};
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr};
use smoltcp::Result;

#[link_section = ".vector_table.interrupts"]
#[no_mangle]
pub static __INTERRUPTS: [u32; 1] = [0];

defmt::timestamp! {"{=u64}", {
        static COUNT: AtomicUsize = AtomicUsize::new(0);
        // NOTE(no-CAS) `timestamps` runs with interrupts disabled
        let n = COUNT.load(Ordering::Relaxed);
        COUNT.store(n + 1, Ordering::Relaxed);
        n as u64
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    run_test();

    let bytes = rtt::bytes_written();
    info!("Bytes: {=usize}", bytes);

    loop {}
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Clock(Cell<Instant>);

impl Clock {
    pub fn new() -> Clock {
        Clock(Cell::new(Instant::from_millis(0)))
    }

    pub fn advance(&self, duration: Duration) {
        self.0.set(self.0.get() + duration)
    }

    pub fn elapsed(&self) -> Instant {
        self.0.get()
    }
}

type Mtu = U2048;
type QueueSize = U8;

/// A loopback device.
#[derive(Debug)]
pub struct Loopback {
    queue: Queue<Vec<u8, Mtu>, QueueSize>,
    medium: Medium,
}

#[allow(clippy::new_without_default)]
impl Loopback {
    /// Creates a loopback device.
    ///
    /// Every packet transmitted through this device will be received through it
    /// in FIFO order.
    pub fn new(medium: Medium) -> Loopback {
        Loopback {
            queue: Queue::new(),
            medium,
        }
    }
}

impl<'a> Device<'a> for Loopback {
    type RxToken = RxToken;
    type TxToken = TxToken<'a>;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 2000;
        caps.medium = self.medium;
        caps
    }

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        self.queue.dequeue().map(move |buffer| {
            let rx = RxToken { buffer };
            let tx = TxToken {
                queue: &mut self.queue,
            };
            (rx, tx)
        })
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(TxToken {
            queue: &mut self.queue,
        })
    }
}

#[doc(hidden)]
pub struct RxToken {
    buffer: Vec<u8, Mtu>,
}

impl phy::RxToken for RxToken {
    fn consume<R, F>(mut self, _timestamp: Instant, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        f(&mut self.buffer)
    }
}

#[doc(hidden)]
pub struct TxToken<'a> {
    queue: &'a mut Queue<Vec<u8, Mtu>, QueueSize>,
}

impl<'a> phy::TxToken for TxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        let mut buffer = Vec::new();
        buffer.resize(len, 0);
        let result = f(&mut buffer);
        self.queue.enqueue(buffer);
        result
    }
}

fn run_test() {
    let clock = Clock::new();
    let device = Loopback::new(Medium::Ethernet);

    #[cfg(feature = "std")]
    let device = {
        let clock = clock.clone();
        utils::setup_logging_with_clock("", move || clock.elapsed());

        let (mut opts, mut free) = utils::create_options();
        utils::add_middleware_options(&mut opts, &mut free);

        let mut matches = utils::parse_options(&opts, free);
        utils::parse_middleware_options(&mut matches, device, /*loopback=*/ true)
    };

    let mut neighbor_cache_entries = [None; 8];
    let mut neighbor_cache = NeighborCache::new(&mut neighbor_cache_entries[..]);

    let mut ip_addrs = [IpCidr::new(IpAddress::v4(127, 0, 0, 1), 8)];
    let mut iface = InterfaceBuilder::new(device)
        .ethernet_addr(EthernetAddress::default())
        .neighbor_cache(neighbor_cache)
        .ip_addrs(&mut ip_addrs[..])
        .finalize();

    let server_socket = {
        // It is not strictly necessary to use a `static mut` and unsafe code here, but
        // on embedded systems that smoltcp targets it is far better to allocate the data
        // statically to verify that it fits into RAM rather than get undefined behavior
        // when stack overflows.
        static mut TCP_SERVER_RX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_SERVER_TX_DATA: [u8; 1024] = [0; 1024];
        let tcp_rx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_SERVER_RX_DATA[..] });
        let tcp_tx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_SERVER_TX_DATA[..] });
        TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer)
    };

    let client_socket = {
        static mut TCP_CLIENT_RX_DATA: [u8; 1024] = [0; 1024];
        static mut TCP_CLIENT_TX_DATA: [u8; 1024] = [0; 1024];
        let tcp_rx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_CLIENT_RX_DATA[..] });
        let tcp_tx_buffer = TcpSocketBuffer::new(unsafe { &mut TCP_CLIENT_TX_DATA[..] });
        TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer)
    };

    let mut socket_set_entries: [_; 2] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let server_handle = socket_set.add(server_socket);
    let client_handle = socket_set.add(client_socket);

    let mut did_listen = false;
    let mut did_connect = false;
    let mut done = false;
    while !done {
        match iface.poll(&mut socket_set, clock.elapsed()) {
            Ok(_) => {}
            Err(e) => {
                debug!("poll error: {}", e);
            }
        }

        {
            let mut socket = socket_set.get::<TcpSocket>(server_handle);
            if !socket.is_active() && !socket.is_listening() {
                if !did_listen {
                    debug!("listening");
                    socket.listen(1234).unwrap();
                    did_listen = true;
                }
            }

            if socket.can_recv() {
                debug!(
                    "got {:?}",
                    socket.recv(|buffer| { (buffer.len(), str::from_utf8(buffer).unwrap()) })
                );
                socket.close();
                done = true;
            }
        }

        {
            let mut socket = socket_set.get::<TcpSocket>(client_handle);
            if !socket.is_open() {
                if !did_connect {
                    debug!("connecting");
                    socket
                        .connect(
                            (IpAddress::v4(127, 0, 0, 1), 1234),
                            (IpAddress::Unspecified, 65000),
                        )
                        .unwrap();
                    did_connect = true;
                }
            }

            if socket.can_send() {
                debug!("sending");
                socket.send_slice(&[0; 10]).unwrap();
                socket.close();
            }
        }

        match iface.poll_delay(&socket_set, clock.elapsed()) {
            Some(Duration { millis: 0 }) => debug!("resuming"),
            Some(delay) => {
                debug!("sleeping for {} ms", delay);
                clock.advance(delay)
            }
            None => clock.advance(Duration::from_millis(1)),
        }
    }

    info!("done")
}
