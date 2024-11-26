// BMS Configuration app

use crate::console::*;

use arbitrary_int::{u2, u3, u4, u6};
use bitbybit::bitfield;

/// BMS Charge Control Register
/// Address: 0x01
#[bitfield(u8)]
struct BMSChargeControl {
    #[bit(7, rw)]
    pfm_dis: bool,
    #[bit(6, rw)]
    wd_rst: bool,
    #[bit(5, rw)]
    otg_config: bool,
    #[bit(4, rw)]
    chg_config: bool,
    #[bits(1..=3, rw)]
    sys_min: u3,
    #[bit(0, rw)]
    min_bat_sel: bool,
}

impl defmt::Format for BMSChargeControl {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "BMSChargeControl: {{ pfm_dis: {}, wd_rst: {}, otg_config: {}, chg_config: {}, sys_min: {}, min_bat_sel: {} }}", self.pfm_dis(), self.wd_rst(), self.otg_config(), self.chg_config(), self.sys_min(), self.min_bat_sel())
    }
}

/// BMS Limits Register
/// Address: 0x02
#[bitfield(u8)]
struct BMSLimits {
    #[bit(7, rw)]
    boost_lim: bool,
    #[bit(6, rw)]
    q1_fullon: bool,
    #[bits(0..=5, rw)]
    ichg: u6,
}

impl defmt::Format for BMSLimits {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "BMSLimits: {{ boost_lim: {}, q1_fullon: {}, ichg: {} }}",
            self.boost_lim(),
            self.q1_fullon(),
            self.ichg()
        )
    }
}

/// BMS Watchdog Register
/// Address: 0x05
#[bitfield(u8)]
struct BMSWatchdog {
    #[bit(7, rw)]
    en_term: bool,
    #[bit(6, r)]
    reserved: bool,
    #[bits(4..=5, rw)]
    watchdog: u2,
    #[bit(3, rw)]
    en_timer: bool,
    #[bit(2, rw)]
    chg_timer: bool,
    #[bit(1, rw)]
    treg: bool,
    #[bit(0, rw)]
    jeita_iset: bool,
}

impl defmt::Format for BMSWatchdog {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt,
            "BMSWatchdog: {{ en_term: {}, watchdog: {}, en_timer: {}, chg_timer: {}, treg: {}, jeita_iset: {} }}",
            self.en_term(),
            self.watchdog(),
            self.en_timer(),
            self.chg_timer(),
            self.treg(),
            self.jeita_iset()
        )
    }
}

/// BMS Control Register
/// Address: 0x07
#[bitfield(u8)]
struct BMSControl {
    /// Input Current Limit Detection Enable
    #[bit(7, rw)]
    iindet_en: bool,
    /// Enable Half Clock Rate Safety Timer
    #[bit(6, rw)]
    tmr2x_en: bool,
    /// Battery FET Disable
    #[bit(5, rw)]
    batfet_dis: bool,
    /// JEITA Charging Voltage Set
    #[bit(4, rw)]
    jeita_vset: bool,
    /// Battery FET Turn-off Delay
    #[bit(3, rw)]
    batfet_dly: bool,
    /// Battery FET Reset Enable
    #[bit(2, rw)]
    batfet_rst_en: bool,
    /// Dynamic VINDPM Tracking
    #[bits(0..=1, rw)]
    vdpm_bat_track: u2,
}

impl defmt::Format for BMSControl {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "BMSControl: {{ iindet_en: {}, tmr2x_en: {}, batfet_dis: {}, jeita_vset: {}, batfet_dly: {}, batfet_rst_en: {}, vdpm_bat_track: {} }}", self.iindet_en(), self.tmr2x_en(), self.batfet_dis(), self.jeita_vset(), self.batfet_dly(), self.batfet_rst_en(), self.vdpm_bat_track())
    }
}

/// BMS Status Register
/// Address: 0x08
#[bitfield(u8)]
struct BMSStatus {
    #[bits(5..=7, r)]
    vbus_stat: u3,
    #[bits(3..=4, r)]
    chrg_stat: u2,
    #[bit(2, r)]
    pg_stat: bool,
    #[bit(1, r)]
    therm_stat: bool,
    #[bit(0, r)]
    vsys_stat: bool,
}

impl defmt::Format for BMSStatus {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "BMSStatus: {{ vbus: {}, chrg: {}, pg: {}, therm: {}, vsys: {} }}",
            self.vbus_stat(),
            self.chrg_stat(),
            self.pg_stat(),
            self.therm_stat(),
            self.vsys_stat()
        )
    }
}

/// BMS Faults Register
/// Address: 0x09
#[bitfield(u8)]
struct BMSFaults {
    #[bit(7, r)]
    watchdog_fault: bool,
    #[bit(6, r)]
    boost_fault: bool,
    #[bits(4..=5, r)]
    chrg_fault: u2,
    #[bit(3, r)]
    bat_fault: bool,
    #[bits(0..=2, r)]
    ntc_fault: u3,
}

impl defmt::Format for BMSFaults {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "BMSFaults: {{ watchdog_fault: {}, boost_fault: {}, chrg_fault: {}, bat_fault: {}, ntc_fault: {} }}", self.watchdog_fault(), self.boost_fault(), self.chrg_fault(), self.bat_fault(), self.ntc_fault())
    }
}

#[bitfield(u8)]
struct BMSInput {
    #[bit(7, r)]
    vbus_gd: bool,
    #[bit(6, r)]
    vindpm_stat: bool,
    #[bit(5, r)]
    iindpm_stat: bool,
    #[bit(4, r)]
    reserved: bool,
    #[bit(3, r)]
    topoff_active: bool,
    #[bit(2, r)]
    acov_stat: bool,
    #[bit(1, rw)]
    vindpm_int_mask: bool,
    #[bit(0, rw)]
    iindpm_int_mask: bool,
}

impl defmt::Format for BMSInput {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "BMSInput: {{ vbus_gd: {}, vindpm_stat: {}, iindpm_stat: {}, topoff_active: {}, acov_stat: {}, vindpm_int_mask: {}, iindpm_int_mask: {} }}", self.vbus_gd(), self.vindpm_stat(), self.iindpm_stat(), self.topoff_active(), self.acov_stat(), self.vindpm_int_mask(), self.iindpm_int_mask())
    }
}

#[bitfield(u8)]
struct BMSSystem {
    #[bit(7, rw)]
    reg_rst: bool,
    #[bits(3..=6, r)]
    pn: u4,
    #[bit(2, r)]
    sgmpart: bool,
    #[bits(0..=1, r)]
    dev_rev: u2,
}

impl defmt::Format for BMSSystem {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "BMSSystem: {{ reg_rst: {}, pn: {}, sgmpart: {}, dev_rev: {} }}",
            self.reg_rst(),
            self.pn(),
            self.sgmpart(),
            self.dev_rev()
        )
    }
}

pub async fn battery_config<'a>(args: &[Token<'a>]) -> Result<(), ()> {
    let mut bms_i2c = crate::BMS_I2C.get().await.lock().await;

    if let Some(action) = args.get(1) {
        match action.as_str() {
            "status" => {
                let mut reg08 = [0u8; 1];
                bms_i2c.write_read(0x6B, &[0x08], &mut reg08).await.ok();
                defmt::info!("BMS Status: {:#x}", BMSStatus::new_with_raw_value(reg08[0]));
            }

            "limits" => {
                let mut reg02 = [0u8; 1];
                bms_i2c.write_read(0x6B, &[0x02], &mut reg02).await.ok();
                defmt::info!("BMS Limits: {:#x}", BMSLimits::new_with_raw_value(reg02[0]));

                if let Some(action) = args.get(2) {
                    match action.as_str() {
                        "480ma" => {
                            let mut val = BMSLimits::new_with_raw_value(reg02[0]);
                            val = val.with_ichg(u6::new(0b1000));
                            bms_i2c.write(0x6B, &[0x02, val.raw_value()]).await.ok();
                        }
                        _ => {}
                    }
                }
            }

            "watchdog" => {
                let mut reg05 = [0u8; 1];
                bms_i2c.write_read(0x6B, &[0x05], &mut reg05).await.ok();
                defmt::info!(
                    "BMS Watchdog: {:#x}",
                    BMSWatchdog::new_with_raw_value(reg05[0])
                );

                if let Some(action) = args.get(2) {
                    match action.as_str() {
                        "disable" => {
                            // Reset watchdog timer
                            let mut reg01 = [0u8; 1];
                            bms_i2c.write_read(0x6B, &[0x01], &mut reg01).await.ok();
                            let mut val = BMSChargeControl::new_with_raw_value(reg01[0]);
                            val = val.with_wd_rst(true);
                            bms_i2c.write(0x6B, &[0x01, val.raw_value()]).await.ok();

                            let mut val = BMSWatchdog::new_with_raw_value(reg05[0]);
                            val = val.with_watchdog(u2::new(0));
                            bms_i2c.write(0x6B, &[0x05, val.raw_value()]).await.ok();
                        }
                        _ => {}
                    }
                }
            }

            "faults" => {
                let mut reg09 = [0u8; 1];
                bms_i2c.write_read(0x6B, &[0x09], &mut reg09).await.ok();
                defmt::info!("BMS Faults: {:#x}", BMSFaults::new_with_raw_value(reg09[0]));
            }

            "chgctrl" => {
                let mut reg01 = [0u8; 1];
                bms_i2c.write_read(0x6B, &[0x01], &mut reg01).await.ok();
                defmt::info!(
                    "BMS Charge Control: {:#x}",
                    BMSChargeControl::new_with_raw_value(reg01[0])
                );

                if let Some(action) = args.get(2) {
                    match action.as_str() {
                        "restart" => {
                            let mut val = BMSChargeControl::new_with_raw_value(reg01[0]);
                            val = val.with_chg_config(false);
                            bms_i2c.write(0x6B, &[0x01, val.raw_value()]).await.ok();

                            val = val.with_chg_config(true);
                            bms_i2c.write(0x6B, &[0x01, val.raw_value()]).await.ok();

                            defmt::info!("BMS Charge Control Restarted: {:#x}", val);
                        }
                        _ => {}
                    }
                }
            }

            "control" => {
                let mut reg07 = [0u8; 1];
                bms_i2c.write_read(0x6B, &[0x07], &mut reg07).await.ok();
                defmt::info!(
                    "BMS Control: {:#x}",
                    BMSControl::new_with_raw_value(reg07[0])
                );
            }

            "input" => {
                let mut reg0a = [0u8; 1];
                bms_i2c.write_read(0x6B, &[0x0A], &mut reg0a).await.ok();
                defmt::info!("BMS Input: {:#x}", BMSInput::new_with_raw_value(reg0a[0]));
            }

            "devinfo" => {
                let mut reg0b = [0u8; 1];
                bms_i2c.write_read(0x6B, &[0x0B], &mut reg0b).await.ok();
                defmt::info!(
                    "BMS Device Info: {:#x}",
                    BMSSystem::new_with_raw_value(reg0b[0])
                );
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

            "poweroff" => {
                let mut reg07 = [0u8; 1];

                // Register 0x07, 1 byte
                bms_i2c.write_read(0x6B, &[0x07], &mut reg07).await.ok();
                bms_i2c
                    .write(0x6B, &[0x07, reg07[0] | 0b00100000u8])
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
