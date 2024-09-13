use core::convert::Infallible;

use embassy_executor::task;
use embedded_io::Write;
use menu::{self, Item, ItemType, Menu, Parameter, Runner};

#[derive(Default)]
struct Context {
    _inner: u32,
}

struct Output();

impl embedded_io::ErrorType for Output {
    type Error = embedded_io::ErrorKind;
}

impl Write for Output {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let result = esp_fast_serial::write_to_usb_serial_buffer(buf);
        if result.is_err() {
            return Err(embedded_io::ErrorKind::Other);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn select_foo(
    _menu: &Menu<Output, Context>,
    item: &Item<Output, Context>,
    args: &[&str],
    interface: &mut Output,
    _context: &mut Context,
) {
    defmt::info!("select_foo");
}

const ROOT_MENU: Menu<Output, Context> = Menu {
    label: "root",
    items: &[&Item {
        item_type: ItemType::Callback {
            function: select_foo,
            parameters: &[
                Parameter::Mandatory {
                    parameter_name: "a",
                    help: Some("This is the help text for 'a'"),
                },
                Parameter::Optional {
                    parameter_name: "b",
                    help: None,
                },
                Parameter::Named {
                    parameter_name: "verbose",
                    help: None,
                },
                Parameter::NamedValue {
                    parameter_name: "level",
                    argument_name: "INT",
                    help: Some("Set the level of the dangle"),
                },
            ],
        },
        command: "foo",
        help: Some(
            "Makes a foo appear.

This is some extensive help text.

It contains multiple paragraphs and should be preceeded by the parameter list.
",
        ),
    }],
    entry: None,
    exit: None,
};

#[task]
pub async fn console() {
    let serial_in = esp_fast_serial::reader_take();

    let mut buffer = [0u8; 256];
    let mut context = Context::default();
    let mut r = Runner::new(ROOT_MENU, &mut buffer, Output(), &mut context);

    loop {
        // extend command buffer with new bytes from serial
        let mut cmd_buf = [0u8; 64];
        let len = serial_in.read(&mut cmd_buf).await;
        if len == 0 {
            continue;
        }

        for b in cmd_buf[..len].iter() {
            r.input_byte(*b, &mut context);
        }
    }
}
