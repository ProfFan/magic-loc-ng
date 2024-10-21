use crate::configuration::ConfigurationStore;

use super::Token;
use alloc::sync::Arc;
use bstr::ByteSlice;
use core::any::TypeId;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};

/// Configuration Set and Get Application
///
/// conf set <key> <value>
/// conf get <key>
pub async fn conf<'a>(
    config_store: Arc<Mutex<CriticalSectionRawMutex, ConfigurationStore>>,
    args: &[Token<'a>],
) -> Result<(), ()> {
    if args.len() < 2 {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Invalid number of arguments\n");
        return Err(());
    }

    if args[1] == Token::String("list") {
        let registry = &config_store.lock().await.registry;
        for (key, value) in registry.iter() {
            let out = alloc::format!("{:?} = {:?}\n", key.as_bstr(), value.name);
            let _ = esp_fast_serial::write_to_usb_serial_buffer(out.as_bytes());
        }

        return Ok(());
    }

    if args.len() < 3 {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Invalid number of arguments\n");
        return Err(());
    }

    let key = if let Token::String(key) = args[2] {
        // Check if key is valid ASCII
        let key_bytes = key.as_bytes();
        let mut key_array = [0u8; 8];
        if key_bytes.len() != 8 {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Key must be 8 characters long\n");
            return Err(());
        }
        key_array.copy_from_slice(key_bytes);

        for c in key_bytes {
            if !c.is_ascii() {
                let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Key must be valid ASCII\n");
                return Err(());
            }
        }

        key_array
    } else {
        return Err(());
    };

    if args[1] == Token::String("set") {
        if args.len() < 4 {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Invalid number of arguments\n");
            return Err(());
        }

        let value = if let Token::String(value) = args[3] {
            value
        } else {
            return Err(());
        };

        let type_id = config_store.lock().await.registry.get(&key).cloned();

        let type_id = if let Some(type_id) = type_id {
            type_id
        } else {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Config type not registered\n");
            return Err(());
        };

        if type_id.type_id == TypeId::of::<u8>() {
            let value = value.parse::<u8>().unwrap();
            let result = config_store.lock().await.set::<u8>(&key, value).await;
            match result {
                Ok(_) => {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Config set\n");
                }
                Err(_) => {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Configuration error\n");
                    return Err(());
                }
            }
        }

        if type_id.type_id == TypeId::of::<u32>() {
            let value = value.parse::<u32>().unwrap();
            let result = config_store.lock().await.set::<u32>(&key, value).await;
            match result {
                Ok(_) => {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Config set\n");
                }
                Err(_) => {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Configuration error\n");
                    return Err(());
                }
            }
        }

        if type_id.type_id == TypeId::of::<u64>() {
            let value = value.parse::<u64>().unwrap();
            let result = config_store.lock().await.set::<u64>(&key, value).await;
            match result {
                Ok(_) => {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Config set\n");
                }
                Err(_) => {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Configuration error\n");
                    return Err(());
                }
            }
        }

        // IP Address
        if type_id.type_id == TypeId::of::<[u8; 4]>() {
            let ipv4_addr = value.parse::<smoltcp::wire::Ipv4Address>().unwrap();
            let result = config_store
                .lock()
                .await
                .set::<[u8; 4]>(&key, ipv4_addr.0)
                .await;
            match result {
                Ok(_) => {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Config set\n");
                }
                Err(_) => {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Configuration error\n");
                    return Err(());
                }
            }
        }
    }

    if args[1] == Token::String("get") {
        let mut buffer = [0u8; 128];

        let entry = config_store.lock().await.registry.get(&key).cloned();

        if entry.is_none() {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Config type not registered\n");
            return Err(());
        }

        if entry.unwrap().type_id == TypeId::of::<u8>() {
            let value = config_store.lock().await.get::<u8>(&mut buffer, &key).await;

            match value {
                Ok(value) => match value {
                    Some(value) => {
                        let s = alloc::format!("{:x}\n", value);
                        let _ = esp_fast_serial::write_to_usb_serial_buffer(s.as_bytes());
                    }
                    None => {
                        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Value not found\n");
                    }
                },
                Err(_) => {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Configuration error\n");
                }
            }
        }

        if entry.unwrap().type_id == TypeId::of::<u32>() {
            let value = config_store
                .lock()
                .await
                .get::<u32>(&mut buffer, &key)
                .await;

            match value {
                Ok(value) => match value {
                    Some(value) => {
                        let s = alloc::format!("{}\n", value);
                        let _ = esp_fast_serial::write_to_usb_serial_buffer(s.as_bytes());
                    }
                    None => {
                        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Value not found\n");
                    }
                },
                Err(_) => {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Configuration error\n");
                }
            }
        }

        // IP Address
        if entry.unwrap().type_id == TypeId::of::<[u8; 4]>() {
            let value = config_store
                .lock()
                .await
                .get::<[u8; 4]>(&mut buffer, &key)
                .await;

            match value {
                Ok(value) => match value {
                    Some(value) => {
                        let s = alloc::format!("{:?}\n", value);
                        let _ = esp_fast_serial::write_to_usb_serial_buffer(s.as_bytes());
                    }
                    None => {
                        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Value not found\n");
                    }
                },
                Err(_) => {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Configuration error\n");
                }
            }
        }
    }

    Ok(())
}
