
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
    enum
    {
        AMG88xx_PCTL = 0x00,
		AMG88xx_RST = 0x01,
		AMG88xx_FPSC = 0x02,
		AMG88xx_INTC = 0x03,
		AMG88xx_STAT = 0x04,
		AMG88xx_SCLR = 0x05,
		//0x06 reserved
		AMG88xx_AVE = 0x07,
		AMG88xx_INTHL = 0x08,
		AMG88xx_INTHH = 0x09,
		AMG88xx_INTLL = 0x0A,
		AMG88xx_INTLH = 0x0B,
		AMG88xx_IHYSL = 0x0C,
		AMG88xx_IHYSH = 0x0D,
		AMG88xx_TTHL = 0x0E,
		AMG88xx_TTHH = 0x0F,
		AMG88xx_INT_OFFSET = 0x010,
		AMG88xx_PIXEL_OFFSET = 0x80
    };
	
	enum power_modes{
		AMG88xx_NORMAL_MODE = 0x00,
		AMG88xx_SLEEP_MODE = 0x01,
		AMG88xx_STAND_BY_60 = 0x20,
		AMG88xx_STAND_BY_10 = 0x21
	};
	
	enum sw_resets {
		AMG88xx_FLAG_RESET = 0x30,
		AMG88xx_INITIAL_RESET = 0x3F
	};
	
	enum frame_rates {
		AMG88xx_FPS_10 = 0x00,
		AMG88xx_FPS_1 = 0x01
	};
	
	enum int_enables{
		AMG88xx_INT_DISABLED = 0x00,
		AMG88xx_INT_ENABLED = 0x01
	};
	
	enum int_modes {
		AMG88xx_DIFFERENCE = 0x00,
		AMG88xx_ABSOLUTE_VALUE = 0x01
	};
// //I2C Address
// #define AMG88xx_ADDRESS = 0x68;
// //REGISTERS
// int AMG88xx_PCTL = 0x00;
// int AMG88xx_RST = 0x01;
// int	AMG88xx_FPSC = 0x02;
// int	AMG88xx_INTC = 0x03;
// int	AMG88xx_STAT = 0x04;
// int	AMG88xx_SCLR = 0x05;
// int	AMG88xx_AVE = 0x07;
// int	AMG88xx_INTHL = 0x08;
// int AMG88xx_INTHH = 0x09;
// int	AMG88xx_INTLL = 0x0A;
// int	AMG88xx_INTLH = 0x0B;
// int	AMG88xx_IHYSL = 0x0C;
// int	AMG88xx_IHYSH = 0x0D;
// int	AMG88xx_TTHL = 0x0E;
// int	AMG88xx_TTHH = 0x0F;
// int	AMG88xx_INT_OFFSET = 0x010;
// int	AMG88xx_PIXEL_OFFSET = 0x80;
// //Modes
// int AMG88xx_NORMAL_MODE = 0x00;
// int	AMG88xx_SLEEP_MODE = 0x01;
// int AMG88xx_STAND_BY_60 = 0x20;
// int AMG88xx_STAND_BY_10 = 0x21;
// //RESETS
// int AMG88xx_FLAG_RESET = 0x30;
// int AMG88xx_INITIAL_RESET = 0x3F;
// //FRAME RATE
// int AMG88xx_FPS_10 = 0x00;
// int AMG88xx_FPS_1 = 0x01;
// //ENABLES
// int AMG88xx_INT_DISABLED = 0x00;
// int AMG88xx_INT_ENABLED = 0x01;
// //INT MODES
// int AMG88xx_DIFFERENCE = 0x00;
// int AMG88xx_ABSOLUTE_VALUE = 0x01;
#define AMG88xx_ADDRESS  0x68
// //PIXELS
#define AMG88xx_PIXEL_ARRAY_SIZE 64
#define AMG88xx_PIXEL_TEMP_CONVERSION .25
#define AMG88xx_THERMISTOR_CONVERSION .0625
//I2C
#define WRITE_BIT                 (I2C_MASTER_WRITE) /*!< I2C master write */
#define READ_BIT                  (I2C_MASTER_READ)  /*!< I2C master read */
#define ACK_CHECK_EN              (0x1)              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS             (0x0)              /*!< I2C master will not check ack from slave */
#define ACK_VAL                   (0x0)            /*!< I2C ack value */
#define NACK_VAL                   (0x1)
#define I2C_BUS                   (0)
#ifndef AMG8833_H
#define AMG8833_H
#ifdef __cplusplus
extern "C"
{
#endif
bool amgbegin();
esp_err_t amg_write_reg(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_wr, size_t size);
esp_err_t amg_read_reg(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, uint16_t len);
esp_err_t amg_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size);
esp_err_t amg_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size);
uint8_t min(uint8_t a, uint8_t b);
float signedMag12ToFloat(uint16_t val);
void readPixels(float *buf, uint8_t size);
void amg_i2c_init(int bus);
#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif
