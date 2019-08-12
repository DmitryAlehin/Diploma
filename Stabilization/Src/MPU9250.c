#include "MPU9250.h"
 
 //функция записи байта
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
   uint8_t data_write[2];
   data_write[0] = subAddress;
   data_write[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, address, data_write, 2, 1000);
}

//функция чтения байта
uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data[1];   
    uint8_t data_write[1];
    data_write[0] = subAddress;
		HAL_I2C_Master_Transmit(&hi2c1, address, data_write, 1, 1000);
		HAL_I2C_Master_Receive(&hi2c1, address, data, 1, 1000);
    return data[0]; 
}

//функция чтения нескольких байт
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{     
    uint8_t data[14];
    uint8_t data_write[1];
    data_write[0] = subAddress;
		HAL_I2C_Master_Transmit(&hi2c1, address, data_write, 1, 1000);
		HAL_I2C_Master_Receive(&hi2c1, address, data, count, 1000);
    for(int ii = 0; ii < count; ii++) {
     dest[ii] = data[ii];
    }
} 
//функция инициализации MPU9250
void initMPU9250(MPU9250_variables_HandleTypedef *dev)
{ 	
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
  HAL_Delay(1);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
  HAL_Delay(1);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  uint8_t c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
  c = c & ~0x18;
  c = c | AFS_16G << 3;
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c);
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
  c = c & ~0x0F;
  c = c | 0x03;
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); 
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);	 
  uint8_t rawData[3];
	//инициализация магнитометра
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
  HAL_Delay(1);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F);
  HAL_Delay(1);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);
	dev->magCalibrationX = (float)(rawData[0] - 128)/256.0f + 1.0f;
	dev->magCalibrationY = (float)(rawData[1] - 128)/256.0f + 1.0f;
	dev->magCalibrationZ = (float)(rawData[2] - 128)/256.0f + 1.0f; 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); 
	HAL_Delay(1);  
  writeByte(AK8963_ADDRESS, AK8963_CNTL, MFS_16BITS << 4 | 0x06);
  HAL_Delay(1);
}

//функция калибровки MPU9250
void calibrateMPU9250(MPU9250_variables_HandleTypedef *dev)
{  	
  uint8_t data[12];
  uint16_t ii, packet_count, fifo_count;
  int32_t accel_bias[3] = {0, 0, 0};
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); 
  HAL_Delay(1); 
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00); 
  HAL_Delay(1);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00);
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);
  HAL_Delay(1);
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
  uint16_t  accelsensitivity = 16384;
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);
  HAL_Delay(1);
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;
  for (ii = 0; ii < packet_count; ii++) 
	{
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
            
	}
    accel_bias[0] /= (int32_t) packet_count;
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;    
		if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}
		else {accel_bias[2] += (int32_t) accelsensitivity;}
		int32_t accel_bias_reg[3] = {0, 0, 0};
		readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]);
		accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
		readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
		accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
		readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
		accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];  
		uint32_t mask = 1uL;
		uint8_t mask_bit[3] = {0, 0, 0};  
		for(ii = 0; ii < 3; ii++) {
			if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01;
		}  
		accel_bias_reg[0] -= (accel_bias[0]/8);
		accel_bias_reg[1] -= (accel_bias[1]/8);
		accel_bias_reg[2] -= (accel_bias[2]/8); 
		data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
		data[1] = (accel_bias_reg[0])      & 0xFF;
		data[1] = data[1] | mask_bit[0]; 
		data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
		data[3] = (accel_bias_reg[1])      & 0xFF;
		data[3] = data[3] | mask_bit[1]; 
		data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
		data[5] = (accel_bias_reg[2])      & 0xFF;
		data[5] = data[5] | mask_bit[2]; 
	  dev->accelBiasX = (float)accel_bias[0]/(float)accelsensitivity;
	  dev->accelBiasY = (float)accel_bias[1]/(float)accelsensitivity;
	  dev->accelBiasZ = (float)accel_bias[2]/(float)accelsensitivity;
}

void get_angles(MPU9250_variables_HandleTypedef *dev)
{	
	int16_t Ax, Ay, Az, Mx, My, Mz;
	float ax, ay, az, mx, my, mz;
	uint8_t rawData[6];	
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
	Ax = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
	Ay = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
	Az = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	ax = (float) Ax * 0.00048828125f -  dev->accelBiasX;
	ay = (float) Ay * 0.00048828125f -  dev->accelBiasY;   
	az = (float) Az * 0.00048828125f -  dev->accelBiasZ; 
	mx = (float) Mx * 1.4993895f * dev->magCalibrationX;
	my = (float) My * 1.4993895f * dev->magCalibrationY;  
	mz = (float) Mz * 1.4993895f * dev->magCalibrationZ;   
	float Pitch = dev->pitch;
	float Roll = dev->roll;
	float Yaw = dev->yaw;
	Pitch = (180/3.141592) * atan(ax / sqrt(pow(ay, 2) + pow(az,2)));
	Roll = (180/3.141592) * atan(ay / sqrt(ax*ax + az*az));
	float xh = mx * cos(Pitch) + mz * sin(Pitch);
  float yh = mx * sin(Roll) * sin(Pitch) + my * cos(Roll) - mz * sin(Roll) * cos(Pitch);
  float zh = -(mx) * cos(Roll) * sin(Pitch) + my * sin(Roll) + mz * cos(Roll) * cos(Pitch);
  Yaw = (180/3.141592) * atan2(yh, xh);
  if (yh < 0)    Yaw += 360;
	dev->pitch = Pitch;
	dev->roll = Roll;
	dev->yaw = Yaw;
}