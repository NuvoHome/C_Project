#include <stdlib.h>
#include <math.h>
#include "TCS34725.h"
#include "driver/i2c.h"


float powf(const float x, const float y)
{
  return (float)(pow((double)x, (double)y));
}

void write8 (uint8_t reg, uint32_t value)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, TCS34725_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, TCS34725_COMMAND_BIT | reg, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value & 0xFF, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;  
}
uint32_t tcs_get_Raw_Data(uint16_t* r, uint16_t* g,uint16_t* b,uint16_t* c)
{
   int ret;
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, TCS34725_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
   i2c_master_write_byte(cmd, TCS34725_COMMAND_BIT, ACK_CHECK_EN);
   i2c_master_stop(cmd);
   ret = i2c_master_cmd_begin(TCS34725_RDATAL, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
   if (ret != ESP_OK) {
       return ret;
   }
   vTaskDelay(30 / portTICK_RATE_MS);
   cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, TCS34725_RDATAL << 1 | READ_BIT, ACK_CHECK_EN);
  //  *r = i2c_master_read_byte(cmd, TCS34725_RDATAL , ACK_VAL);

   i2c_master_stop(cmd);
   ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
   i2c_cmd_link_delete(cmd);
     switch (_tcs34725IntegrationTime)
  {
    case TCS34725_INTEGRATIONTIME_2_4MS:
      ets_delay_us(3);
      break;
    case TCS34725_INTEGRATIONTIME_24MS:
      ets_delay_us(24);
      break;
    case TCS34725_INTEGRATIONTIME_50MS:
      ets_delay_us(50);
      break;
    case TCS34725_INTEGRATIONTIME_101MS:
      ets_delay_us(101);
      break;
    case TCS34725_INTEGRATIONTIME_154MS:
      ets_delay_us(154);
      break;
    case TCS34725_INTEGRATIONTIME_700MS:
      ets_delay_us(700);
      break;
  }
   return ret;

}
uint8_t read8(uint8_t reg)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_read_byte(cmd, reg, ACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}
// uint16_t read16(uint8_t reg)
// {
//     uint16_t x; uint16_t t;


// }
void enable(void)
{
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
//   delay(3);
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);  
}
// void disable(void)
// {
//   /* Turn the device off to save power */
//   uint8_t reg = 0;
//   reg = read8(TCS34725_ENABLE);
//   write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
// }
void setGain(tcs34725Gain_t gain)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_CONTROL, gain);

  /* Update value placeholders */
  _tcs34725Gain = gain;
}
void setIntegrationTime(tcs34725IntegrationTime_t it)
{
  if (!_tcs34725Initialised) begin();

  /* Update the timing register */
  write8(TCS34725_ATIME, it);

  /* Update value placeholders */
  _tcs34725IntegrationTime = it;
}

bool begin(void)
 
 {
    //  Wire.begin();
  
  /* Make sure we're actually connected */

  uint8_t x = read8(TCS34725_ID);
  if (x != 0x44)
  {
    printf(x);
    return false;
  }
  // _tcs34725Initialised = true;

  // /* Set default integration time and gain */
  // setIntegrationTime(_tcs34725IntegrationTime);
  // setGain(_tcs34725Gain);

  // /* Note: by default, the device is in power down mode on bootup */
  // enable();

  return true;
  }
//   void getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
// {

//   *c = read16(TCS34725_CDATAL);
//   *r = read16(TCS34725_RDATAL);
//   *g = read16(TCS34725_GDATAL);
//   *b = read16(TCS34725_BDATAL);
  
//   /* Set a delay for the integration time */
//   switch (_tcs34725IntegrationTime)
//   {
//     case TCS34725_INTEGRATIONTIME_2_4MS:
//       delay(3);
//       break;
//     case TCS34725_INTEGRATIONTIME_24MS:
//       delay(24);
//       break;
//     case TCS34725_INTEGRATIONTIME_50MS:
//       delay(50);
//       break;
//     case TCS34725_INTEGRATIONTIME_101MS:
//       delay(101);
//       break;
//     case TCS34725_INTEGRATIONTIME_154MS:
//       delay(154);
//       break;
//     case TCS34725_INTEGRATIONTIME_700MS:
//       delay(700);
//       break;
//   }
// }
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