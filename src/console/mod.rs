use core::convert::Infallible;

use crate::configuration::ConfigurationStore;
use alloc::sync::Arc;
use embassy_executor::task;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

mod apps;

const MAX_BUFFER_SIZE: usize = 128;

#[derive(Debug, defmt::Format, PartialEq, Eq)]
pub enum Token<'a> {
    /// A string
    String(&'a str),
    /// Quoted string
    Quoted(&'a str),
}

pub fn parse_arguments<'a>(command: &'a str) -> heapless::Vec<Token<'a>, 16> {
    let mut tokens = heapless::Vec::<Token<'a>, 16>::new();

    // Split the command into tokens of whitespace, quotes, and other characters
    let mut subtokens = heapless::Vec::<&str, 32>::new();
    {
        let mut last = 0;
        for (index, matched) in command.match_indices(|c: char| c == ' ' || c == '\"') {
            if last != index {
                let _ = subtokens.push(&command[last..index]);
            }
            let _ = subtokens.push(matched);
            last = index + matched.len();
        }
        if last < command.len() {
            let _ = subtokens.push(&command[last..]);
        }
    }

    let mut last_quote_pos: Option<usize> = None;
    let mut current_pos: usize = 0;

    for token in subtokens {
        current_pos += token.len();
        if last_quote_pos.is_some() {
            if token == "\"" {
                // This is the closing quote
                let token = &command[last_quote_pos.unwrap()..current_pos - 1];

                let _ = tokens.push(Token::Quoted(token));
                last_quote_pos = None;
            }
        } else {
            if token == "\"" {
                last_quote_pos = Some(current_pos);
                continue;
            }
            if token != " " {
                let _ = tokens.push(Token::String(token));
            }
        }
    }

    tokens
}

#[task]
pub async fn console(config_store: Arc<Mutex<CriticalSectionRawMutex, ConfigurationStore>>) {
    let serial_in = esp_fast_serial::reader_take();

    const PROMPT: &[u8] = b"(ml)> ";

    loop {
        let mut command_buffer = heapless::Vec::<u8, MAX_BUFFER_SIZE>::new();

        let echo_back = |buf: &[u8], new_line: bool| {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"\x1b[2K\r");
            let _ = esp_fast_serial::write_to_usb_serial_buffer(PROMPT);
            let _ = esp_fast_serial::write_to_usb_serial_buffer(buf);
            if new_line {
                let _ = esp_fast_serial::write_to_usb_serial_buffer(b"\n\r");
                let _ = esp_fast_serial::write_to_usb_serial_buffer(PROMPT);
            }
        };

        loop {
            let mut buf = [0u8; 1];
            let read = serial_in.read(&mut buf).await;
            if read == 0 {
                continue;
            }

            if buf[0] != b'\n' && buf[0] != b'\r' {
                let not_full = command_buffer.push(buf[0]);
                if not_full.is_err() {
                    command_buffer.clear();
                }

                echo_back(command_buffer.as_slice(), false);
            } else {
                echo_back(command_buffer.as_slice(), true);

                break;
            }
        }

        let command = core::str::from_utf8(&command_buffer).unwrap_or_default();

        defmt::debug!("Command: {}", command);

        let tokens = parse_arguments(command);

        defmt::debug!("Tokens: {:?}", tokens);

        if tokens.len() == 0 {
            continue;
        }

        if let Token::String(app) = tokens[0] {
            match app {
                "conf" => {
                    let _ = apps::conf(config_store.clone(), &tokens).await;
                }
                _ => {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Unknown command\n");
                }
            }
        }
    }
}