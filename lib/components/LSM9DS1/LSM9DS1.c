#include <stdlib.h>
#include <math.h>
#include "LSM9DS1.h"
#include "driver/i2c.h"
esp_err_t lsm_accel_read_reg(i2c_port_t i2c_num,  uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM9DS1_ADDRESS_ACCELGYRO <<1), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM9DS1_ADDRESS_ACCELGYRO <<1)|1, ACK_CHECK_EN);
    i2c_master_read(cmd, data, len,  I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t lsm_magnetometer_read_reg(i2c_port_t i2c_num,  uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM9DS1_ADDRESS_MAG <<1), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM9DS1_ADDRESS_MAG <<1)|1, ACK_CHECK_EN);
    i2c_master_read(cmd, data, len,  I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t lsm_accel_write_reg(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( LSM9DS1_ADDRESS_ACCELGYRO << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t lsm_mag_write_reg(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( LSM9DS1_ADDRESS_MAG << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
void readAccel() {
  // Read the accelerometer
  int buffer[6];
  for (uint8_t i=0; i<6; i++){
      int value= 0;
  lsm_accel_read_reg(I2C_BUS, LSM9DS1_REGISTER_OUT_X_L_XL, value, 1);
  buffer[i] = value;
  }
  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];
  
  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;
  accelData.x = xhi;
  accelData.y = yhi;
  accelData.z = zhi;
   accelData.x = accelData.x * _accel_mg_lsb;
  accelData.x /= 1000;
  accelData.x *= SENSORS_GRAVITY_STANDARD;
  accelData.y = accelData.y * _accel_mg_lsb;
  accelData.y /= 1000;
  accelData.y *= SENSORS_GRAVITY_STANDARD;
  accelData.z = accelData.z * _accel_mg_lsb;
  accelData.z /= 1000;
  accelData.z *= SENSORS_GRAVITY_STANDARD;
}

void readMag() {
  // Read the magnetometer
  int buffer[6];
  for (uint8_t i=0; i<6; i++){
      int value = 0;
  lsm_magnetometer_read_reg(I2C_BUS, LSM9DS1_REGISTER_OUT_X_L_M,value ,1);
  buffer[i] = value;
  }
  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];
  
  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;
  magData.x = xhi;
  magData.y = yhi;
  magData.z = zhi;
magData.x  = magData.x * _mag_mgauss_lsb;
  magData.x  /= 1000;
  magData.y = magData.y * _mag_mgauss_lsb;
  magData.y /= 1000;
  magData.z = magData.z * _mag_mgauss_lsb;
  magData.z /= 1000;
}

void readGyro() {
  // Read gyro
  int buffer[6];
 for (uint8_t i=0; i<6; i++) {
lsm_accel_read_reg(I2C_BUS, 0x80 | LSM9DS1_REGISTER_OUT_X_L_G, buffer[i], 1);
 }
  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];
  uint8_t ylo = buffer[2];
  int16_t yhi = buffer[3];
  uint8_t zlo = buffer[4];
  int16_t zhi = buffer[5];
  
  // Shift values to create properly formed integer (low byte first)
  xhi <<= 8; xhi |= xlo;
  yhi <<= 8; yhi |= ylo;
  zhi <<= 8; zhi |= zlo;
  
  gyroData.x = xhi;
  gyroData.y = yhi;
  gyroData.z = zhi;
  gyroData.x = gyroData.x * _gyro_dps_digit;
  gyroData.y = gyroData.y * _gyro_dps_digit;
gyroData.z = gyroData.z * _gyro_dps_digit;
}

void readTemp() {
  // Read temp sensor
  int buffer[2];
   for (int i=0; i<2; i++) {
    int r =0;
    lsm_accel_read_reg(I2C_BUS, 0x80 | LSM9DS1_REGISTER_TEMP_OUT_L, r, 1);
    buffer[i] = r;
   }
  uint8_t xlo = buffer[0];
  int16_t xhi = buffer[1];

  xhi <<= 8; xhi |= xlo;
  
  // Shift values to create properly formed integer (low byte first)
  temperature = xhi;
}
void setupAccel ( lsm9ds1AccelRange_t range )
{
  uint8_t reg;
  lsm_accel_read_reg(I2C_BUS, LSM9DS1_REGISTER_CTRL_REG6_XL,&reg,1);
  reg &= ~(0b00011000);
  reg |= range;
  lsm_accel_write_reg(I2C_BUS,LSM9DS1_REGISTER_CTRL_REG6_XL,reg,1);  
  switch (range)
  {
    case LSM9DS1_ACCELRANGE_2G:
      _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_2G;
      break;
    case LSM9DS1_ACCELRANGE_4G:
      _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_4G;
      break;
    case LSM9DS1_ACCELRANGE_8G:
      _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_8G;
      break;    
    case LSM9DS1_ACCELRANGE_16G:
      _accel_mg_lsb =LSM9DS1_ACCEL_MG_LSB_16G;
      break;
  }
}
void setupMag ( lsm9ds1MagGain_t gain )
{
  uint8_t reg;
  lsm_magnetometer_read_reg(I2C_BUS, LSM9DS1_REGISTER_CTRL_REG2_M,&reg,1);
  reg &= ~(0b00011000);
  reg |= gain;
  lsm_mag_write_reg(I2C_BUS,LSM9DS1_REGISTER_CTRL_REG2_M,reg,1);
  switch(gain)
  {
    case LSM9DS1_MAGGAIN_4GAUSS:
      _mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_4GAUSS;
      break;
    case LSM9DS1_MAGGAIN_8GAUSS:
      _mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_8GAUSS;
      break;
    case LSM9DS1_MAGGAIN_12GAUSS:
      _mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_12GAUSS;
      break;
    case LSM9DS1_MAGGAIN_16GAUSS:
      _mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_16GAUSS;
      break;
  }
}
void setupGyro ( lsm9ds1GyroScale_t scale )
{
  uint8_t reg;
  lsm_accel_read_reg(I2C_BUS, LSM9DS1_REGISTER_CTRL_REG1_G,&reg,1);
  reg &= ~(0b00011000);
  reg |= scale;
lsm_accel_write_reg(I2C_BUS,LSM9DS1_REGISTER_CTRL_REG1_G,reg,1);  
switch(scale)
  {
    case LSM9DS1_GYROSCALE_245DPS:
      _gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_245DPS;
      break;
    case LSM9DS1_GYROSCALE_500DPS:
      _gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_500DPS;
      break;
    case LSM9DS1_GYROSCALE_2000DPS:
      _gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_2000DPS;
      break;
  }
}
void lsmbegin(){
//      // soft reset & reboot accel/gyro
//   write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG8, 0x05);
//   // soft reset & reboot magnetometer
//   write8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG2_M, 0x0C);
    lsm_accel_write_reg(I2C_BUS,LSM9DS1_REGISTER_CTRL_REG8,0x05,1);  
    lsm_mag_write_reg(I2C_BUS,LSM9DS1_REGISTER_CTRL_REG2_M,0x0C,1);


    lsm_accel_write_reg(I2C_BUS,LSM9DS1_REGISTER_CTRL_REG1_G,0xC0,1);  
    lsm_accel_write_reg(I2C_BUS,LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38,1);  
    lsm_accel_write_reg(I2C_BUS,LSM9DS1_REGISTER_CTRL_REG6_XL,0xC0,1);  
    lsm_mag_write_reg(I2C_BUS,LSM9DS1_REGISTER_CTRL_REG3_M,0x00,1);
    // write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG1_G, 0xC0); // on XYZ

  // Enable the accelerometer continous
//   write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38); // enable X Y and Z axis
//   write8(XGTYPE, LSM9DS1_REGISTER_CTRL_REG6_XL, 0xC0); // 1 KHz out data rate, BW set by ODR, 408Hz anti-aliasing
  // enable mag continuous
  //write8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG1_M, 0xFC); // high perf XY, 80 Hz ODR
//   write8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG3_M, 0x00); // continuous mode
  //write8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG4_M, 0x0C); // high perf Z mode
  // Set default ranges for the various sensors  
  setupAccel(LSM9DS1_ACCELRANGE_2G);
  setupMag(LSM9DS1_MAGGAIN_4GAUSS);
  setupGyro(LSM9DS1_GYROSCALE_245DPS);
}
