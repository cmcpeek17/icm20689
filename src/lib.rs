/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_hal::spi as hal_spi;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

//mod interface;
//pub use interface::{SensorInterface, SpiInterface};

/// Errors in this crate
#[derive(Debug)]
pub enum Error<SpiE> {
    // Spi Error
    Comm(SpiE),

    /// Unrecognized chip ID
    UnknownChipId,
    /// Sensor not responding
    Unresponsive,
}

pub trait ICM20689Interface
{
    type Error;

    fn check_identity(
        &mut self,
        delay_source: &mut impl DelayNs,
    ) -> Result<bool, Self::Error>;

    fn soft_reset(
        &mut self,
        delay_source: &mut impl DelayNs,
    ) -> Result<(), Self::Error>;

    fn setup(&mut self, delay_source: &mut impl DelayNs) -> Result<(), Self::Error>;

    fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Self::Error>;
    fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Self::Error>;
    fn get_raw_accel(&mut self) -> Result<[i16; 3], Self::Error>;
    fn get_raw_gyro(&mut self) -> Result<[i16; 3], Self::Error>;
    fn get_scaled_accel(&mut self) -> Result<[f32; 3], Self::Error>;
    fn get_scaled_gyro(&mut self) -> Result<[f32; 3], Self::Error>;
}

pub struct ICM20689<Spi, SpiE>
where
    Spi: hal_spi::SpiDevice::<u8, Error = SpiE>,
    SpiE: hal_spi::Error
{
    pub(crate) spi_dev: Spi,
    pub(crate) gyro_scale: f32,
    pub(crate) accel_scale: f32,
}

impl<Spi, SpiE> ICM20689<Spi, SpiE>
where
    Spi: hal_spi::SpiDevice::<u8, Error = SpiE>,
    SpiE: hal_spi::Error
{
    const DIR_READ: u8 = 0x80; // same as 1<<7

    pub fn new_with_interface(spi_dev: Spi) -> Self {
        Self {
            spi_dev: spi_dev,
            gyro_scale: 0.0,
            accel_scale: 0.0,
        }
    }

    fn read_block(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), SpiE> {
        buffer[0] = reg | Self::DIR_READ;
        self.spi_dev.read(buffer)?;
        Ok(())
    }

    fn read_vec3_i16(&mut self, reg: u8) -> Result<[i16; 3], SpiE> {
        let mut block: [u8; 7] = [0; 7];
        self.read_block(reg, &mut block)?;

        Ok([
            (block[1] as i16) << 8 | (block[2] as i16),
            (block[3] as i16) << 8 | (block[4] as i16),
            (block[5] as i16) << 8 | (block[6] as i16),
        ])
    }

    fn register_write(&mut self, reg: u8, val: u8) -> Result<(), SpiE> {
        let block: [u8; 2] = [reg, val];
        self.spi_dev.write(&block)?;
        Ok(())
    }

    fn register_read(&mut self, reg: u8) -> Result<u8, SpiE> {
        let mut block: [u8; 2] = [reg | Self::DIR_READ, 0u8];
        self.spi_dev.transfer_in_place( &mut block)?;

        #[cfg(feature = "rttdebug")]
        rprintln!("read reg 0x{:x} {:x?} ", reg, block[1]);

        Ok(block[1])
    }

}

impl<Spi, SpiE> ICM20689Interface for ICM20689<Spi, SpiE>
where
    Spi: hal_spi::SpiDevice::<u8, Error = SpiE>,
    SpiE: hal_spi::Error
{
    type Error = Error<SpiE>;

    /// Read the sensor identifier and return true if they match a supported value
    fn check_identity(
        &mut self,
        delay_source: &mut impl DelayNs,
    ) -> Result<bool, Self::Error> {
        for _ in 0..5 {
            let chip_id = self.register_read(REG_WHO_AM_I).map_err(|e| Error::Comm(e))?;
            match chip_id {
                ICM20602_WAI | ICM20608_WAI | ICM20689_WAI => {
                    #[cfg(feature = "rttdebug")]
                    rprintln!("found device: 0x{:0x}  ", chip_id);
                    return Ok(true);
                }
                _ => {
                    #[cfg(feature = "rttdebug")]
                    rprintln!("bogus whoami: 0x{:0x}  ", chip_id);
                }
            }

            delay_source.delay_ms(10);
        }

        Ok(false)
    }

    /// Perform a soft reset on the sensor
    fn soft_reset(
        &mut self,
        delay_source: &mut impl DelayNs,
    ) -> Result<(), Self::Error> {
        /// disable I2C interface if we're using SPI
        const I2C_IF_DIS: u8 = 1 << 4;

        /// reset the device
        const PWR_DEVICE_RESET: u8 = 1 << 7; // 0x80 : 0b10000000;

        /// Auto-select between internal relaxation oscillator and
        /// gyroscope MEMS oscillator to use the best available source
        const CLKSEL_AUTO: u8 = 0x01;
        const SENSOR_ENABLE_ALL: u8 = 0x00;

        // self.dev.write(Register::PWR_MGMT_1, 0x80)?;
        // // get stable time source;
        // // Auto select clock source to be PLL gyroscope reference if ready
        // // else use the internal oscillator, bits 2:0 = 001
        // self.dev.write(Register::PWR_MGMT_1, 0x01)?;
        // // Enable all sensors
        // self.dev.write(Register::PWR_MGMT_2, 0x00)?;
        // delay.delay_ms(200);

        //reset can take up to 100 ms?
        self.register_write(REG_PWR_MGMT_1, PWR_DEVICE_RESET).map_err(|e| Error::Comm(e))?;

        delay_source.delay_ms(110);

        let mut reset_success = false;
        for _ in 0..10 {
            //The reset bit automatically clears to 0 once the reset is done.
            if let Ok(reg_val) = self.register_read(REG_PWR_MGMT_1) {
                if reg_val & PWR_DEVICE_RESET == 0 {
                    reset_success = true;
                    break;
                }
            }
            delay_source.delay_ms(10);
        }
        if !reset_success {
            #[cfg(feature = "rttdebug")]
            rprintln!("couldn't read REG_PWR_MGMT_1");
            return Err(Error::Unresponsive);
        }

        self.register_write(REG_USER_CTRL, I2C_IF_DIS).map_err(|e| Error::Comm(e))?;

        //setup the automatic clock selection
        self.register_write(REG_PWR_MGMT_1, CLKSEL_AUTO).map_err(|e| Error::Comm(e))?;
        //enable accel and gyro
        self.register_write(REG_PWR_MGMT_2, SENSOR_ENABLE_ALL).map_err(|e| Error::Comm(e))?;

        delay_source.delay_ms(200);

        Ok(())
    }

    /// give the sensor interface a chance to set up
    fn setup(&mut self, delay_source: &mut impl DelayNs) -> Result<(), Self::Error> {
        // const DLPF_CFG_1: u8 = 0x01;
        //const SIG_COND_RST: u8 = 1 << 0;
        const FIFO_RST: u8 = 1 << 2;
        const DMP_RST: u8 = 1 << 3;

        // note that id check before reset will fail
        self.soft_reset(delay_source)?;
        let supported = self.check_identity(delay_source)?;
        if !supported {
            return Err(Error::UnknownChipId);
        }

        //TODO Configure the Digital Low Pass Filter (DLPF)
        // self.si.register_write(Self::REG_CONFIG, DLPF_CFG_1)?;
        // //set the sample frequency
        // self.si.register_write(Self::REG_SMPLRT_DIV, 0x01)?;

        // disable interrupt pin
        self.register_write(REG_INT_ENABLE, 0x00).map_err(|e| Error::Comm(e))?;

        // disable FIFO
        //self.si.register_write(REG_FIFO_EN, 0x00)?;

        //enable FIFO for gyro and accel only:
        self.register_write(REG_FIFO_EN, 0x7C).map_err(|e| Error::Comm(e))?;

        //TODO what about SIG_COND_RST  ?
        let ctrl_flags = FIFO_RST | DMP_RST;
        self.register_write(REG_USER_CTRL, ctrl_flags).map_err(|e| Error::Comm(e))?;

        //configure some default ranges
        self.set_accel_range(AccelRange::default())?;
        self.set_gyro_range(GyroRange::default())?;

        Ok(())
    }

    /// Set the full scale range of the accelerometer
    fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Self::Error> {
        self.accel_scale = range.scale();
        self.register_write(REG_ACCEL_CONFIG, (range as u8) << 3).map_err(|e| Error::Comm(e))
    }

    /// Set the full scale range of the gyroscope
    fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Self::Error> {
        self.gyro_scale = range.scale();
        self.register_write(REG_GYRO_CONFIG, (range as u8) << 2).map_err(|e| Error::Comm(e))
    }

    fn get_raw_accel(&mut self) -> Result<[i16; 3], Self::Error> {
        self.read_vec3_i16(REG_ACCEL_START).map_err(|e| Error::Comm(e))
    }

    fn get_raw_gyro(&mut self) -> Result<[i16; 3], Self::Error> {
        self.read_vec3_i16(REG_GYRO_START).map_err(|e| Error::Comm(e))
    }

    fn get_scaled_accel(&mut self) -> Result<[f32; 3], Self::Error> {
        let raw_accel = self.get_raw_accel()?;
        Ok([
            self.accel_scale * (raw_accel[0] as f32),
            self.accel_scale * (raw_accel[1] as f32),
            self.accel_scale * (raw_accel[2] as f32),
        ])
    }

    fn get_scaled_gyro(&mut self) -> Result<[f32; 3], Self::Error> {
        let raw_gyro = self.get_raw_gyro()?;
        Ok([
            self.gyro_scale * (raw_gyro[0] as f32),
            self.gyro_scale * (raw_gyro[1] as f32),
            self.gyro_scale * (raw_gyro[2] as f32),
        ])
    }

}

/// Common registers
///
const REG_USER_CTRL: u8 = 0x6A;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_PWR_MGMT_2: u8 = 0x6C;

// const REG_CONFIG: u8 = 0x1A;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;

const REG_FIFO_EN: u8 = 0x23;
const REG_INT_ENABLE: u8 = 0x38;
// const REG_SMPLRT_DIV: u8 = 0x19;

const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_ACCEL_START: u8 = REG_ACCEL_XOUT_H;

const REG_GYRO_XOUT_H: u8 = 0x43;
const REG_GYRO_START: u8 = REG_GYRO_XOUT_H;

const REG_WHO_AM_I: u8 = 0x75;

/// Device IDs for various supported devices
const ICM20602_WAI: u8 = 0x12;
const ICM20608_WAI: u8 = 0xAF;
const ICM20689_WAI: u8 = 0x98;

#[repr(u8)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Default)]
/// The gyroscope has a programmable full-scale range of ±250, ±500, ±1000, or ±2000 degrees/sec.
pub enum GyroRange {
    /// ±250
    Range_250dps = 0b00,
    /// ±500
    Range_500dps = 0b01,
    /// ±1000
    Range_1000dps = 0b10,
    /// ±2000
    #[default]
    Range_2000dps = 0b11,
}

//Gyro Full Scale Select: 00 = ±250dps
// 01= ±500dps
// 10 = ±1000dps
// 11 = ±2000dps

impl GyroRange {
    /// convert degrees into radians
    const RADIANS_PER_DEGREE: f32 = core::f32::consts::PI / 180.0;

    /// Gyro range in radians per second per bit
    pub(crate) fn scale(&self) -> f32 {
        Self::RADIANS_PER_DEGREE * self.resolution()
    }

    /// Gyro resolution in degrees per second per bit
    /// Note that the ranges are ± which splits the raw i16 resolution between + and -
    pub(crate) fn resolution(&self) -> f32 {
        match self {
            GyroRange::Range_250dps => 250.0 / 32768.0,
            GyroRange::Range_500dps => 500.0 / 32768.0,
            GyroRange::Range_1000dps => 1000.0 / 32768.0,
            GyroRange::Range_2000dps => 2000.0 / 32768.0,
        }
    }
}

#[repr(u8)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Default)]
/// The accelerometer has a user-programmable accelerometer full-scale range
/// of ±2g, ±4g, ±8g, and ±16g.
/// (g is gravitational acceleration: 9.82 m/s^2)
/// The numeric values of these enums correspond to AFS_SEL
pub enum AccelRange {
    /// ±2g
    Range_2g = 0b00,
    /// ±4g
    Range_4g = 0b01,
    /// ±8g
    #[default]
    Range_8g = 0b10,
    /// ±16g
    Range_16g = 0b11,
}

impl AccelRange {
    /// Earth gravitational acceleration (G) standard, in meters per second squared
    const EARTH_GRAVITY_ACCEL: f32 = 9.807;

    /// accelerometer scale in meters per second squared per bit
    pub(crate) fn scale(&self) -> f32 {
        Self::EARTH_GRAVITY_ACCEL * self.resolution()
    }

    /// Accelerometer resolution in G / bit
    /// Note that the ranges are ± which splits the raw i16 resolution between + and -
    pub(crate) fn resolution(&self) -> f32 {
        match self {
            Self::Range_2g => 2.0 / 32768.0,
            Self::Range_4g => 4.0 / 32768.0,
            Self::Range_8g => 8.0 / 32768.0,
            Self::Range_16g => 16.0 / 32768.0,
        }
    }
}
