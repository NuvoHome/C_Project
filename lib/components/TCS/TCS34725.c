#include <stdlib.h>
#include <math.h>
#include "TCS34725.h"
#include "driver/i2c.h"
esp_err_t tcs_read_reg(i2c_port_t i2c_num,  uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCS34725_ADDRESS <<1), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, TCS34725_COMMAND_BIT | reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCS34725_ADDRESS <<1)|1, ACK_CHECK_EN);
    i2c_master_read(cmd, data, len,  I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
uint8_t tcs_read(uint8_t reg){
uint16_t x; uint16_t t;
tcs_read_reg(I2C_BUS, reg, &x, 2);
tcs_read_reg(I2C_BUS, reg, &t, 2);
  x <<= 8;
  x |= t;
  return x;
}
void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  // if (!_tcs34725Initialised) begin();
  //TRIAL 
  *c = tcs_read(TCS34725_CDATAL);
  *r = tcs_read(TCS34725_RDATAL);
  *g = tcs_read(TCS34725_GDATAL);
  *b = tcs_read(TCS34725_BDATAL);
  tcs_read_reg(I2C_BUS,TCS34725_CDATAL,*c,2);
  tcs_read_reg(I2C_BUS,TCS34725_RDATAL,*r,2);
  tcs_read_reg(I2C_BUS,TCS34725_GDATAL,*g,2);
  tcs_read_reg(I2C_BUS,TCS34725_BDATAL,*b,2);
}
uint16_t calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}
uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}
