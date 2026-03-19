use embedded_hal::blocking::i2c;

// Wrapper for I2C
pub struct I2cInterface<I2C> {
    pub i2c: I2C,
    pub address: u8,
}

// Wrapper for SPI
pub struct SpiInterface<SPI, CS> {
    pub spi: SPI,
    pub cs: CS,
}



#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector3df32 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

// Define what any connection (I2C or SPI) must do for THIS chip
pub trait ImuBus {
    type Error;
    fn read_register(&mut self, reg: u8) -> Result<u8, Self::Error>;
    fn read_registers(&mut self, reg: u8, data: &mut [u8]) -> Result<(), Self::Error>;
}

// Shared data members
pub struct ImuState {
    pub acc_offset: Vector3df32,
    pub gyro_offset: Vector3df32,
    pub scale_factor: f32,
    pub gyro_resolution_rps : f32,
    pub gyro_resolution_dps : f32,
    pub acc_resolution : f32,
    pub gyro_sample_rate_hz : u32,
    pub acc_sample_rate_hz :u32,
    pub gyro_id_msp : u16,
    pub acc_id_msp : u16,
    pub uint8_t _axis_order : u8,
    pub uint8_t _flags : u8, // lags for describing IMU characteristics
}

// The Main Trait uses that Bus as an associated type
pub trait Imu {
    type Bus: ImuBus;
    
    fn bus(&mut self) -> &mut Self::Bus;
    fn state(&self) -> &ImuState;

    fn read_gyro_rps(&mut self) -> Vector3df32;
    fn read_acc(&mut self) -> Vector3df32;
    fn read_acc_gyro_rps(&mut self) -> (Vector3df32, Vector3df32);
    fn gyro_offset(&self) -> Vector3df32;
    fn set_gyro_offset(&self, gyro_offset : Vector3df32);
    fn acc_offset(&self) -> Vector3df32;
    fn set_acc_offset(&self, acc_offset : Vector3df32);
}

/*
    fn gyro_offset(&self) -> Vector3df32 {
        self.gyro_offset
    }
    fn set_gyro_offset(&self, gyro_offset : Vector3df32) {
        self.gyro_offset = gyro_offset;
    }
    fn acc_offset(&self) -> Vector3df32 {
        self.acc_offset
    }
    fn set_acc_offset(&self, acc_offset : Vector3df32) {
        self.acc_offset = acc_offset;
    }
*/
// 3. Your generic struct
pub struct Mpu6050<B: ImuBus> {
    pub bus: B,
    pub state: ImuState,
}

impl<B: ImuBus> Imu for Mpu6050<B> {
    type Bus = B;
    fn bus(&mut self) -> &mut Self::Bus { &mut self.bus }
    fn state(&self) -> &ImuState { &self.state }
    fn read_accel_x(&mut self) -> f32 {
        const REG_ACCEL_X_OUT_H = 0x3B;
        let mut bytes = [0u8; 2];
        self.bus().read_registers(REG_ACCEL_X_OUT_H, &mut bytes).ok();
        // MPU6050 is Big Endian
        let raw = i16::from_be_bytes(buffer); 
        // Convert to Gs using the shared state's scale factor
        (raw as f32) * self.state.accel_scale
    }
}

impl<I2C, E> ImuBus for I2cInterface<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
{
    type Error = E;

    fn read_register(&mut self, reg: u8) -> Result<u8, Self::Error> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buffer)?;
        Ok(buffer[0])
    }

    fn read_registers(&mut self, reg: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[reg], data)
    }
}

// The Generic Struct
pub struct Mpu6050<B: ImuBus> {
    bus: B,
    state: ImuState,
}

// A helper to make a new I2C-based MPU6050
impl<I2C> Mpu6050<I2cInterface<I2C>> {
    pub fn new_i2c(i2c: I2C, address: u8) -> Self {
        Self {
            bus: I2cInterface { i2c, address },
            state: ImuState::default(),
        }
    }
}
