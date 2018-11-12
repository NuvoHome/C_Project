#include <stdlib.h>
#include <math.h>
#include "AMG8833.h"
#include "driver/i2c.h"
void amg_i2c_init(int bus)
{  
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 15;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = 4;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    esp_err_t res = i2c_param_config(bus, &conf);
    printf("Driver param setup : %d\n",res);
   res = i2c_driver_install(bus, I2C_MODE_MASTER, 0, 0, 0);
    printf("Driver installed   : %d\n",res);
}
esp_err_t amg_write_reg(i2c_port_t i2c_num, uint8_t i2c_reg, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( AMG88xx_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t amg_read_reg(i2c_port_t i2c_num,  uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AMG88xx_ADDRESS <<1), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AMG88xx_ADDRESS <<1)|1, ACK_CHECK_EN);
    i2c_master_read(cmd, data, len,  I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
bool amgbegin(){
    amg_i2c_init(I2C_BUS);
    amg_write_reg(I2C_BUS, AMG88xx_PCTL, AMG88xx_NORMAL_MODE, 1);
    amg_write_reg(I2C_BUS, AMG88xx_RST, AMG88xx_INITIAL_RESET, 1);
    amg_write_reg(I2C_BUS, AMG88xx_FPSC, AMG88xx_FPS_10, 1);
    amg_write_reg(I2C_BUS, AMG88xx_INTC, 0, 1);
    return true;
}
esp_err_t amg_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AMG88xx_ADDRESS << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t amg_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AMG88xx_ADDRESS << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
uint8_t min(uint8_t a, uint8_t b){
    if( a > b){
        return b;
    }
    else if(b > a ){
        return a;
    }
    else if( a == b){
        return a;
    }
    else{
        return -1;
    }
}

void amgread(uint8_t reg, uint8_t *buf, uint8_t num){
    int AMG_I2C_CHUNKSIZE = 16;
    uint8_t value;
	uint8_t pos = 0;
	
	//on arduino we need to read in AMG_I2C_CHUNKSIZE byte chunks
	while(pos < num){
		uint8_t read_now = min((uint8_t)AMG_I2C_CHUNKSIZE, (uint8_t)(num - pos));
        // amg_write_slave(I2C_BUS, reg + pos, 1);
    		for(int i=0; i<read_now; i++){
            amg_read_slave(I2C_BUS, buf[pos], 1);
            pos++;
            }
    }
}
float signedMag12ToFloat(uint16_t val)
{
	//take first 11 bits as absolute val
	uint16_t absVal = (val & 0x7FF);
	
	return (val & 0x8000) ? 0 - (float)absVal : (float)absVal ;
}
void readPixels(float *buf, uint8_t size){
    uint16_t recast;
	float converted;
	uint8_t bytesToRead = min((uint8_t)(size << 1), (uint8_t)(AMG88xx_PIXEL_ARRAY_SIZE << 1));
    uint8_t rawArray[bytesToRead];
    amg_read_reg(I2C_BUS, AMG88xx_PIXEL_OFFSET, rawArray, bytesToRead);
	for(int i=0; i<size; i++){
		uint8_t pos = i << 1;
		recast = ((uint16_t)rawArray[pos + 1] << 8) | ((uint16_t)rawArray[pos]);
		converted = signedMag12ToFloat(recast) * AMG88xx_PIXEL_TEMP_CONVERSION;
		buf[i] = converted;
	}
}
