// BMS Configuration app

use crate::console::*;

pub async fn battery_config<'a>(args: &[Token<'a>]) -> Result<(), ()> {
    let mut bms_i2c = crate::BMS_I2C.get().await.lock().await;

    if let Some(action) = args.get(1) {
        match action.as_str() {
            "status" => {
                let mut reg0b = [0u8; 1];
                bms_i2c.write_read(0x6B, &[0x0b], &mut reg0b).await.ok();
                defmt::info!("BMS Status: {:#x}", reg0b);
            }

            "keepon" => {
                let mut reg07 = [0u8; 1];

                // Register 0x07, 1 byte
                bms_i2c.write_read(0x6B, &[0x07], &mut reg07).await.ok();
                bms_i2c
                    .write(0x6B, &[0x07, reg07[0] & !0b00100000u8])
                    // .write(0x6B, &[0x07, reg07[0] & 0b11011111u8])
                    .await
                    .ok();
            }

            _ => {
                return Err(());
            }
        }
    }

    Ok(())
}
