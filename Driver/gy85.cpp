#include "gy85.h"
#include <Math.h>

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define address   0x29
u8 gbuf[16];

void Gy85::init()
{
	Wire.begin();
}
u16 Gy85::bswap(u8 b[]) {
  // Big Endian unsigned short to little endian unsigned short
  u16 val = ((b[0] << 8) & b[1]);
  return val;
}
u16 Gy85::makeuint16(int lsb, int msb) {
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}
void Gy85::write_byte_data(u8 data) {
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
}
void Gy85::write_byte_data_at(u8 reg, u8 data) {
  // write data word at address and register
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}
void Gy85::write_word_data_at(u8 reg, u16 data) {
  // write data word at address and register
  u8 b0 = (data &0xFF);
  u8 b1 = ((data >> 8) && 0xFF);
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(b0);
  Wire.write(b1);
  Wire.endTransmission();
}

u8 Gy85::read_byte_data() {
  Wire.requestFrom(address, 1);
  while (Wire.available() < 1) delay(1);
  u8 b = Wire.read();
  return b;
}

u8 Gy85::read_byte_data_at(u8 reg) {
  
  write_byte_data(reg);
  Wire.requestFrom(address, 1);
  while (Wire.available() < 1) delay(1);
  u8 b = Wire.read();
  return b;
}

u16 Gy85::read_word_data_at(u8 reg) {
  write_byte_data(reg);
  Wire.requestFrom(address, 2);
  while (Wire.available() < 2) delay(1);
  gbuf[0] = Wire.read();
  gbuf[1] = Wire.read();
  return bswap(gbuf); 
}

void Gy85::read_block_data_at(u8 reg, int sz) {
  int i = 0;
  write_byte_data(reg);
  Wire.requestFrom(address, sz);
  for (i=0; i<sz; i++) {
    while (Wire.available() < 1) delay(1);
    gbuf[i] = Wire.read();
  }
}

u16 Gy85::VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
  // Converts the encoded VCSEL period register value into the real
  // period in PLL clocks
  u16 vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
  return vcsel_period_pclks;
}

void Gy85::Mesure_Distance() {

  u8 val1 =  read_byte_data_at(VL53L0X_REG_IDENTIFICATION_REVISION_ID);
//  Serial.print("Revision ID: "); Serial.println(val1);
  val1 = read_byte_data_at(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
//  Serial.print("Device ID: "); Serial.println(val1);
  val1 = read_byte_data_at(VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD);
//  Serial.print("PRE_RANGE_CONFIG_VCSEL_PERIOD="); Serial.println(val1); 
//  Serial.print(" decode: "); Serial.println(VL53L0X_decode_vcsel_period(val1));
  val1 = read_byte_data_at(VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD);
//  Serial.print("FINAL_RANGE_CONFIG_VCSEL_PERIOD="); Serial.println(val1);
//  Serial.print(" decode: "); Serial.println(VL53L0X_decode_vcsel_period(val1));
  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);
  u8  val = 0;
  int cnt = 0;
  while (cnt < 100) { // 1 second waiting time max
    delay(10);
    val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
    if (val & 0x01) break;
    cnt++;
  }
 // if (val & 0x01) //Serial.println("ready"); else Serial.println("not ready");
  
   read_block_data_at(0x14, 12);
   acnt = makeuint16(gbuf[7], gbuf[6]);
   scnt = makeuint16(gbuf[9], gbuf[8]);
   dist = makeuint16(gbuf[11], gbuf[10]);
   DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);

//  Serial.print("ambient count: "); Serial.println(acnt);
//  Serial.print("signal count: ");  Serial.println(scnt);
//  Serial.print("distance ");       Serial.println(dist);
//  Serial.print("status: ");        Serial.println(DeviceRangeStatusInternal);
}


void Gy85::send_value(int dev_addr, uint8_t value)
{
	Wire.beginTransmission(dev_addr);
	Wire.write(value);
	Wire.endTransmission();
}

uint8_t Gy85::check_id(int dev_addr, uint8_t reg_addr)
{
	Wire.beginTransmission(dev_addr);
	Wire.write(reg_addr);
	Wire.endTransmission();
	Wire.requestFrom(dev_addr, 1);
	return Wire.read();
}

void Gy85::write_to_register(int dev_addr, uint8_t reg_addr, uint8_t reg_value)
{
	Wire.beginTransmission(dev_addr);
	Wire.write(reg_addr);
	Wire.write(reg_value);
	Wire.endTransmission();
}

bool Gy85::check_gyroscope()
{
	if((check_id(ITG3205_GYRO_ADDRESS, ITG3205_WHO_AM_I) & 0x7E) == ITG3205_GYRO_ADDRESS){
		write_to_register(ITG3205_GYRO_ADDRESS, ITG3205_PWR_MGM, ITG3205_RESET);              // 3E      80
		write_to_register(ITG3205_GYRO_ADDRESS, ITG3205_DLPF_FS, 0x1B);                       // 16     1B
		write_to_register(ITG3205_GYRO_ADDRESS, 0x15, 0x13);                                  // 0x15  0x13
		write_to_register(ITG3205_GYRO_ADDRESS, ITG3205_PWR_MGM, 0x03);                       // 3E     03
		return true;
	} else {
		return false;
	}

}

void Gy85::measure_gyroscope()
{
	uint8_t gyro_read = 0;
	send_value(ITG3205_GYRO_ADDRESS, 0x1D);
	Wire.requestFrom(ITG3205_GYRO_ADDRESS, 6);
	while(Wire.available()){
		gyro_buffer[gyro_read] = Wire.read();
		gyro_read++;
	}
	raw_rotation.x = (float)(GYRO_X_INVERT*(int16_t)(((int)gyro_buffer[2*GYRO_X_AXIS] <<8) | gyro_buffer[2*GYRO_X_AXIS+1])) * ITG3205_SCALE;  //rad/s
	raw_rotation.y = (float)(GYRO_Y_INVERT*(int16_t)(((int)gyro_buffer[2*GYRO_Y_AXIS] <<8) | gyro_buffer[2*GYRO_Y_AXIS+1])) * ITG3205_SCALE;
	raw_rotation.z = (float)(GYRO_Z_INVERT*(int16_t)(((int)gyro_buffer[2*GYRO_Z_AXIS] <<8) | gyro_buffer[2*GYRO_Z_AXIS+1])) * ITG3205_SCALE;
}

bool Gy85::check_accelerometer()
{
	if (check_id(ADXL345_ACCELEROMETER_ADDRESS, ADXL345_DEVID) == ADXL345_DEVICE_ID){
		write_to_register(ADXL345_ACCELEROMETER_ADDRESS,ADXL345_POWER_CTL,0x08);  //D3, enables measuring                                  2D 08
		write_to_register(ADXL345_ACCELEROMETER_ADDRESS,ADXL345_DATA_FORMAT,0x09); //D3 and D0, enables FULL_RES and +/-4g     31 09
		write_to_register(ADXL345_ACCELEROMETER_ADDRESS,ADXL345_BW_RATE,0x09); //Set the bw to 0 Hz                                             2C 09
		return true;
	}
	else
		return false;
}

void Gy85::measure_acceleration()
{
	uint8_t acc_reads = 0;
	send_value(ADXL345_ACCELEROMETER_ADDRESS, ADXL345_DATAX0);
	Wire.requestFrom(ADXL345_ACCELEROMETER_ADDRESS, 6);
	while(Wire.available()){
		acc_buffer[acc_reads] = Wire.read();
		acc_reads++;
	}

	raw_acceleration.x =  ((float)ACC_X_INVERT*(int16_t)((int)acc_buffer[2*ACC_X_AXIS+1]<<8 | acc_buffer[2*ACC_X_AXIS]) / ADXL345_SCALE);
	raw_acceleration.y =  ((float)ACC_Y_INVERT*(int16_t)((int)acc_buffer[2*ACC_Y_AXIS+1]<<8 | acc_buffer[2*ACC_Y_AXIS]) / ADXL345_SCALE);
	raw_acceleration.z =  ((float)ACC_Z_INVERT*(int16_t)((int)acc_buffer[2*ACC_Z_AXIS+1]<<8 | acc_buffer[2*ACC_Z_AXIS]) / ADXL345_SCALE);

}

bool Gy85::check_magnetometer()
{
	write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_B,HMC5883L_MAG_GAIN);  //Sets the gain
	write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_A,0x18); //75Hz output
	write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x01); //Single-Measurement Mode
	return true;;
}

int x=0;
int y=0;
int z=0;

void Gy85::measure_magnetometer()
{
	uint8_t mag_reads = 0;
	char str[20]= {0};
	send_value(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_DATAX0);
	Wire.requestFrom(HMC5883L_MAG_ADDRESS,6);
	while(Wire.available()){
		mag_buffer[mag_reads] = Wire.read();
		mag_reads++;
	}
	raw_magnetic_field.x =  (float)(MAG_X_INVERT * ((int16_t)((int)mag_buffer[2*MAG_X_AXIS] << 8) | (mag_buffer[2*MAG_X_AXIS+1]))) * HMC5883L_MAG_SCALE;
	raw_magnetic_field.y =  (float)(MAG_Y_INVERT * ((int16_t)((int)mag_buffer[2*MAG_Y_AXIS] << 8) | (mag_buffer[2*MAG_Y_AXIS+1]))) * HMC5883L_MAG_SCALE;
	raw_magnetic_field.z =  (float)(MAG_Z_INVERT * ((int16_t)((int)mag_buffer[2*MAG_Z_AXIS] << 8) | (mag_buffer[2*MAG_Z_AXIS+1]))) * HMC5883L_MAG_SCALE;
	if(raw_magnetic_field.x == 0.0 )
		raw_magnetic_field.x = -28514.00;
	if(raw_magnetic_field.y == 0.0)
        raw_magnetic_field.y = -28302.00;
	if(raw_magnetic_field.z ==0.0)
		raw_magnetic_field.z = -28412.00;
	write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x01);
}

