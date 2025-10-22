use std::{
    error::Error,
    io::{self, Write},
    time::Duration,
};

use rppal::uart::Uart;

const BAUD_RATE: u32 = 115200;

fn main() -> Result<(), Box<dyn Error>> {
    let mut uart = Uart::with_path("/dev/serial0", BAUD_RATE, rppal::uart::Parity::None, 8, 1)?;

    uart.set_read_mode(1, Duration::from_millis(100))?;

    let mut buffer = [0u8; 256];

    loop {
        match uart.read(&mut buffer) {
            Ok(length) => {
                if length == 0 {
                    continue;
                }

                let data = String::from_utf8_lossy(&buffer[..length]);

                if data.is_empty() {
                    println!("{:?}", &buffer[..length]);
                } else {
                    print!("{data}");
                    io::stdout().flush()?; // print is line buffered so this prints it immediatly 
                }
            }
            Err(e) => {
                eprintln!("UART error: {e}");
            }
        }
    }
}
