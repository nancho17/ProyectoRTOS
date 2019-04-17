/* Copyright 2017 Bolder Flight Systems <brian.taylor@bolderflight.com>.
 * Copyright 2018, Sergio Renato De Jesus Melean <sergiordj@gmail.com>.
 * Copyright 2018, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
/* Date: 2018-07-06 */

/*==================[inclusions]=============================================*/

#include "adaptacion_mpu9250.h"   /* <= sAPI HMC5883L header */
//#include "sapi.h"
#include "sapi_i2c.h"           /* <= sAPI I2C header */

//#include "sapi_delay.h"         /* <= sAPI Delay header */

#include "FreeRTOS.h"
#include "task.h"
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static int8_t Ampu9250InitializeControlStructure( void );
static int8_t Ampu9250WriteRegister( uint8_t subAddress, uint8_t data );
static int8_t Ampu9250ReadRegisters( uint8_t subAddress, uint8_t count );
static int8_t Ampu9250WriteAK8963Register( uint8_t subAddress, uint8_t data );
static int8_t Ampu9250WhoAmI( void );
static int8_t Ampu9250WhoAmIAK8963( void );
static int8_t Ampu9250ReadAK8963Registers( uint8_t subAddress, uint8_t count );
static int8_t Ampu9250CalibrateGyro( void );
static int8_t Ampu9250SetGyroRange( MPU9250_GyroRange_t range );
static int8_t Ampu9250SetDlpfBandwidth( MPU9250_DlpfBandwidth_t bandwidth );
static int8_t Ampu9250SetSrd( uint8_t srd );

/*==================[internal data definition]===============================*/

//MPU control structure
static MPU9250_control_t Acontrol;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static int8_t Ampu9250InitializeControlStructure( void )
{
	Acontrol._tempScale = 333.87f;
	Acontrol._tempOffset = 21.0f;
	Acontrol._numSamples = 100;
	Acontrol._axs = 1.0f;
	Acontrol._ays = 1.0f;
	Acontrol._azs = 1.0f;
	Acontrol._maxCounts = 1000;
	Acontrol._deltaThresh = 0.3f;
	Acontrol._coeff = 8;
	Acontrol._hxs = 1.0f;
	Acontrol._hys = 1.0f;
	Acontrol._hzs = 1.0f;
	Acontrol.tX[0] = 0;
	Acontrol.tX[1] = 1;
	Acontrol.tX[2] = 0;
	Acontrol.tY[0] = 1;
	Acontrol.tY[1] = 0;
	Acontrol.tY[2] = 0;
	Acontrol.tZ[0] = 0;
	Acontrol.tZ[1] = 0;
	Acontrol.tZ[2] = -1;
}

static int8_t Ampu9250WriteRegister( uint8_t subAddress, uint8_t data )
{
	uint8_t transmitDataBuffer[2];
	transmitDataBuffer[0] = subAddress;
	transmitDataBuffer[1] = data;
	i2cWrite(I2C0, Acontrol.address, transmitDataBuffer, 2, TRUE);

	vTaskDelay( 10 / portTICK_RATE_MS );//delay(10);

	/* read back the register */
	Ampu9250ReadRegisters(subAddress,1);
	/* check the read back register against the written register */
	if(Acontrol._buffer[0] == data) {
      return 1;
	}
	else{
      return -1;
	}
}

static int8_t Ampu9250ReadRegisters( uint8_t subAddress, uint8_t count )
{
	if( i2cRead( I2C0,Acontrol.address,&subAddress,1,TRUE,Acontrol._buffer,count,TRUE) ){
		return 1;
	} else {
		return -1;
	}
}

static int8_t Ampu9250WriteAK8963Register( uint8_t subAddress, uint8_t data )
{
	// set slave 0 to the AK8963 and set for write
	if (Ampu9250WriteRegister( MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR) < 0) {
		return -1;
	}
	// set the register to the desired AK8963 sub address
	if (Ampu9250WriteRegister( MPU9250_I2C_SLV0_REG, subAddress) < 0) {
		return -2;
	}
	// store the data for write
	if (Ampu9250WriteRegister( MPU9250_I2C_SLV0_DO, data) < 0) {
		return -3;
	}
	// enable I2C and send 1 byte
	if (Ampu9250WriteRegister( MPU9250_I2C_SLV0_CTRL, MPU9250_I2C_SLV0_EN | (uint8_t)1) < 0) {
		return -4;
	}
	// read the register and confirm
	if (Ampu9250ReadAK8963Registers(subAddress,1) < 0) {
		return -5;
	}
	if(Acontrol._buffer[0] == data) {
		return 1;
	} else{
		return -6;
	}
}

static int8_t Ampu9250WhoAmI( void )
{
	// read the WHO AM I register
	if (Ampu9250ReadRegisters(MPU9250_WHO_AM_I,1) < 0) {
		return -1;
	}
	// return the register value
	return Acontrol._buffer[0];
}

static int8_t Ampu9250WhoAmIAK8963( void )
{
	// read the WHO AM I register
	if (Ampu9250ReadAK8963Registers(MPU9250_AK8963_WHO_AM_I,1) < 0) {
		return -1;
	}
	// return the register value
	return Acontrol._buffer[0];
}

static int8_t Ampu9250ReadAK8963Registers( uint8_t subAddress, uint8_t count )
{
	// set slave 0 to the AK8963 and set for read
	if (Ampu9250WriteRegister( MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ_FLAG) < 0) {
		return -1;
	}
	// set the register to the desired AK8963 sub address
	if (Ampu9250WriteRegister( MPU9250_I2C_SLV0_REG, subAddress) < 0) {
		return -2;
	}
	// enable I2C and request the bytes
	if (Ampu9250WriteRegister( MPU9250_I2C_SLV0_CTRL, MPU9250_I2C_SLV0_EN | count) < 0) {
		return -3;
	}
	vTaskDelay( 1 / portTICK_RATE_MS );//delay(1); // takes some time for these registers to fill
	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	Acontrol._status = Ampu9250ReadRegisters(MPU9250_EXT_SENS_DATA_00,count);
	return Acontrol._status;
}

static int8_t Ampu9250CalibrateGyro( void )
{
	// set the range, bandwidth, and srd
	if (Ampu9250SetGyroRange(MPU9250_GYRO_RANGE_250DPS) < 0) {
		return -1;
	}
	if (Ampu9250SetDlpfBandwidth(MPU9250_DLPF_BANDWIDTH_20HZ) < 0) {
		return -2;
	}
	if (Ampu9250SetSrd(19) < 0) {
		return -3;
	}

	// take samples and find bias
	Acontrol._gxbD = 0;
	Acontrol._gybD = 0;
	Acontrol._gzbD = 0;
	for (uint8_t i=0; i < Acontrol._numSamples; i++) {
		Ampu9250Read();
		Acontrol._gxbD += ((Ampu9250GetGyroX_rads() + Acontrol._gxb)/Acontrol._numSamples);
		Acontrol._gybD += ((Ampu9250GetGyroY_rads() + Acontrol._gyb)/Acontrol._numSamples);
		Acontrol._gzbD += ((Ampu9250GetGyroZ_rads() + Acontrol._gzb)/Acontrol._numSamples);
		vTaskDelay( 20 / portTICK_RATE_MS );//delay(20);
	}
	Acontrol._gxb = (float)Acontrol._gxbD;
	Acontrol._gyb = (float)Acontrol._gybD;
	Acontrol._gzb = (float)Acontrol._gzbD;

	// set the range, bandwidth, and srd back to what they were
	if (Ampu9250SetGyroRange(Acontrol._gyroRange) < 0) {
		return -4;
	}
	if (Ampu9250SetDlpfBandwidth(Acontrol._bandwidth) < 0) {
		return -5;
	}
	if (Ampu9250SetSrd(Acontrol._srd) < 0) {
		return -6;
	}
	return 1;
}

static int8_t Ampu9250SetGyroRange( MPU9250_GyroRange_t range )
{
	switch(range) {
		case MPU9250_GYRO_RANGE_250DPS: {
		  // setting the gyro range to 250DPS
		  if(Ampu9250WriteRegister(MPU9250_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_250DPS) < 0){
			return -1;
		  }
        // setting the gyro scale to 250DPS
		  Acontrol._gyroScale = 250.0f/32767.5f * MPU9250_D2R;
		  break;
		}
		case MPU9250_GYRO_RANGE_500DPS: {
		  // setting the gyro range to 500DPS
		  if(Ampu9250WriteRegister(MPU9250_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_500DPS) < 0){
			return -1;
		  }
        // setting the gyro scale to 500DPS
		  Acontrol._gyroScale = 500.0f/32767.5f * MPU9250_D2R;
		  break;
		}
		case MPU9250_GYRO_RANGE_1000DPS: {
		  // setting the gyro range to 1000DPS
		  if(Ampu9250WriteRegister(MPU9250_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_1000DPS) < 0){
			return -1;
		  }
        // setting the gyro scale to 1000DPS
		  Acontrol._gyroScale = 1000.0f/32767.5f * MPU9250_D2R;
		  break;
		}
		case MPU9250_GYRO_RANGE_2000DPS: {
		  // setting the gyro range to 2000DPS
		  if(Ampu9250WriteRegister(MPU9250_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_2000DPS) < 0){
			return -1;
		  }
        // setting the gyro scale to 2000DPS
		  Acontrol._gyroScale = 2000.0f/32767.5f * MPU9250_D2R;
		  break;
		}
	}
	Acontrol._gyroRange = range;
	return 1;
}

static int8_t Ampu9250SetDlpfBandwidth( MPU9250_DlpfBandwidth_t bandwidth )
{
	switch (bandwidth) {
		case MPU9250_DLPF_BANDWIDTH_184HZ: {
         // setting accel bandwidth to 184Hz
			if (Ampu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_184) < 0) {
				return -1;
			}
         // setting gyro bandwidth to 184Hz
			if (Ampu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_184) < 0) {
				return -2;
			}
			break;
		}
		case MPU9250_DLPF_BANDWIDTH_92HZ: {
         // setting accel bandwidth to 92Hz
			if (Ampu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_92) < 0) {
				return -1;
			}
         // setting gyro bandwidth to 92Hz
			if (Ampu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_92) < 0) {
				return -2;
			}
			break;
		}
		case MPU9250_DLPF_BANDWIDTH_41HZ: {
         // setting accel bandwidth to 41Hz
			if (Ampu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_41) < 0) {
				return -1;
			}
         // setting gyro bandwidth to 41Hz
			if (Ampu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_41) < 0) {
				return -2;
			}
			break;
		}
		case MPU9250_DLPF_BANDWIDTH_20HZ: {
         // setting accel bandwidth to 20Hz
			if (Ampu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_20) < 0) {
				return -1;
			}
         // setting gyro bandwidth to 20Hz
			if (Ampu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_20) < 0) {
				return -2;
			}
			break;
		}
		case MPU9250_DLPF_BANDWIDTH_10HZ: {
         // setting accel bandwidth to 10Hz
			if (Ampu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_10) < 0) {
				return -1;
			}
         // setting gyro bandwidth to 10Hz
			if (Ampu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_10) < 0) {
				return -2;
			}
			break;
		}
		case MPU9250_DLPF_BANDWIDTH_5HZ: {
         // setting accel bandwidth to 5Hz
			if (Ampu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_5) < 0) {
				return -1;
			}
         // setting gyro bandwidth to 5Hz
			if (Ampu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_5) < 0) {
				return -2;
			}
			break;
		}
	}
	Acontrol._bandwidth = bandwidth;
	return 1;
}

static int8_t Ampu9250SetSrd( uint8_t srd )
{
	/* setting the sample rate divider to 19 to facilitate setting up 
      magnetometer */
   // setting the sample rate divider
	if (Ampu9250WriteRegister(MPU9250_SMPDIV, 19) < 0) {
		return -1;
	}
	if (srd > 9) {
		// set AK8963 to Power Down
		if (Ampu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN) < 0) {
			return -2;
		}
		vTaskDelay( 100 / portTICK_RATE_MS );//delay(100); // long wait between AK8963 mode changes
		// set AK8963 to 16 bit resolution, 8 Hz update rate
		if (Ampu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_CNT_MEAS1) < 0) {
			return -3;
		}
		vTaskDelay( 100 / portTICK_RATE_MS );//delay(100); // long wait between AK8963 mode changes
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		Ampu9250ReadAK8963Registers(MPU9250_AK8963_HXL, 7);
	} else {
		// set AK8963 to Power Down
		if (Ampu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN) < 0) {
			return -2;
		}
		vTaskDelay( 100 / portTICK_RATE_MS );//delay(100); // long wait between AK8963 mode changes
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		if (Ampu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_CNT_MEAS2) < 0) {
			return -3;
		}
		vTaskDelay( 100 / portTICK_RATE_MS );//delay(100); // long wait between AK8963 mode changes
		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		Ampu9250ReadAK8963Registers(MPU9250_AK8963_HXL, 7);
	}
	/* setting the sample rate divider */
	if (Ampu9250WriteRegister(MPU9250_SMPDIV, srd) < 0) { // setting the sample rate divider
		return -4;
	}
	Acontrol._srd = srd;
	return 1;
}

/*==================[external functions definition]==========================*/

//Initialize MPU9250 (TODO: include SPI communication)
int8_t Ampu9250Init( MPU9250_address_t address )
{
	Ampu9250InitializeControlStructure();

	Acontrol.address = address;

	// using I2C for communication
	// starting the I2C bus
	i2cInit(I2C0, MPU9250_I2C_RATE);

	// select clock source to gyro
	if (Ampu9250WriteRegister(MPU9250_PWR_MGMNT_1, MPU9250_CLOCK_SEL_PLL) < 0) {
		return -1;
	}
	// enable I2C master mode
	if (Ampu9250WriteRegister(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN) < 0) {
		return -2;
	}
	// set the I2C bus speed to 400 kHz
	if (Ampu9250WriteRegister(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_CLK) < 0) {
		return -3;
	}
	// set AK8963 to Power Down
	Ampu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN);
	// reset the MPU9250
	Ampu9250WriteRegister(MPU9250_PWR_MGMNT_1, MPU9250_PWR_RESET);
	// wait for MPU-9250 to come back up
	vTaskDelay( 1 / portTICK_RATE_MS );//delay(1);
	// reset the AK8963
	Ampu9250WriteAK8963Register(MPU9250_AK8963_CNTL2, MPU9250_AK8963_RESET);
	// select clock source to gyro
	if (Ampu9250WriteRegister(MPU9250_PWR_MGMNT_1, MPU9250_CLOCK_SEL_PLL) < 0) {
		return -4;
	}
	// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
	if ((Ampu9250WhoAmI() != 113) && (Ampu9250WhoAmI() != 115)) {
		return -5;
	}
	// enable accelerometer and gyro
	if (Ampu9250WriteRegister(MPU9250_PWR_MGMNT_2, MPU9250_SEN_ENABLE) < 0) {
		return -6;
	}
	// setting accel range to 16G as default
	if (Ampu9250WriteRegister(MPU9250_ACCEL_CONFIG, MPU9250_ACCEL_FS_SEL_16G) < 0) {
		return -7;
	}
	Acontrol._accelScale = MPU9250_G * 16.0f / 32767.5f; // setting the accel scale to 16G
	Acontrol._accelRange = MPU9250_ACCEL_RANGE_16G;
	// setting the gyro range to 2000DPS as default
	if (Ampu9250WriteRegister(MPU9250_GYRO_CONFIG, MPU9250_GYRO_FS_SEL_2000DPS) < 0) {
		return -8;
	}
   // setting the gyro scale to 2000DPS
	Acontrol._gyroScale = 2000.0f / 32767.5f * MPU9250_D2R;
	Acontrol._gyroRange = MPU9250_GYRO_RANGE_2000DPS;
	// setting bandwidth to 184Hz as default
	if (Ampu9250WriteRegister(MPU9250_ACCEL_CONFIG2, MPU9250_ACCEL_DLPF_184) < 0) {
		return -9;
	}
   // setting gyro bandwidth to 184Hz
	if (Ampu9250WriteRegister(MPU9250_CONFIG, MPU9250_GYRO_DLPF_184) < 0) {
		return -10;
	}
	Acontrol._bandwidth = MPU9250_DLPF_BANDWIDTH_184HZ;
	// setting the sample rate divider to 0 as default
	if (Ampu9250WriteRegister(MPU9250_SMPDIV, 0x00) < 0) {
		return -11;
	}
	Acontrol._srd = 0;
	// enable I2C master mode
	if (Ampu9250WriteRegister(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN) < 0) {
		return -12;
	}
	// set the I2C bus speed to 400 kHz
	if (Ampu9250WriteRegister(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_CLK) < 0) {
		return -13;
	}
	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if (Ampu9250WhoAmIAK8963() != 72) {
		return -14;
	}
	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	if (Ampu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN) < 0) {
		return -15;
	}
	vTaskDelay( 100 / portTICK_RATE_MS );//delay(100); // long wait between AK8963 mode changes
	// set AK8963 to FUSE ROM access
	if (Ampu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_FUSE_ROM) < 0) {
		return -16;
	}
	vTaskDelay( 100 / portTICK_RATE_MS );//delay(100); // long wait between AK8963 mode changes
	// read the AK8963 ASA registers and compute magnetometer scale factors
	Ampu9250ReadAK8963Registers(MPU9250_AK8963_ASA, 3);
	Acontrol._magScaleX = ((((float) Acontrol._buffer[0]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f
			/ 32760.0f; // micro Tesla
	Acontrol._magScaleY = ((((float) Acontrol._buffer[1]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f
			/ 32760.0f; // micro Tesla
	Acontrol._magScaleZ = ((((float) Acontrol._buffer[2]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f
			/ 32760.0f; // micro Tesla
	// set AK8963 to Power Down
	if (Ampu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_PWR_DOWN) < 0) {
		return -17;
	}
	vTaskDelay( 100 / portTICK_RATE_MS );//delay(100); // long wait between AK8963 mode changes
	// set AK8963 to 16 bit resolution, 100 Hz update rate
	if (Ampu9250WriteAK8963Register(MPU9250_AK8963_CNTL1, MPU9250_AK8963_CNT_MEAS2) < 0) {
		return -18;
	}
	vTaskDelay( 100 / portTICK_RATE_MS );//delay(100); // long wait between AK8963 mode changes
	// select clock source to gyro
	if (Ampu9250WriteRegister(MPU9250_PWR_MGMNT_1, MPU9250_CLOCK_SEL_PLL) < 0) {
		return -19;
	}
	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	Ampu9250ReadAK8963Registers(MPU9250_AK8963_HXL, 7);
	// estimate gyro bias
	if (Ampu9250CalibrateGyro() < 0) {
		return -20;
	}
	// successful init, return 1
	return 1;
}

//Read sensor registers and store data at control structure
bool_t Ampu9250Read(void)
{
	// grab the data from the MPU9250
	if( !Ampu9250ReadRegisters(MPU9250_ACCEL_OUT, 21) ){
		return 0;
	}
	// combine into 16 bit values
	Acontrol._axcounts = (((int16_t)Acontrol._buffer[0]) << 8)  | Acontrol._buffer[1];
	Acontrol._aycounts = (((int16_t)Acontrol._buffer[2]) << 8)  | Acontrol._buffer[3];
	Acontrol._azcounts = (((int16_t)Acontrol._buffer[4]) << 8)  | Acontrol._buffer[5];
	Acontrol._tcounts  = (((int16_t)Acontrol._buffer[6]) << 8)  | Acontrol._buffer[7];
	Acontrol._gxcounts = (((int16_t)Acontrol._buffer[8]) << 8)  | Acontrol._buffer[9];
	Acontrol._gycounts = (((int16_t)Acontrol._buffer[10]) << 8) | Acontrol._buffer[11];
	Acontrol._gzcounts = (((int16_t)Acontrol._buffer[12]) << 8) | Acontrol._buffer[13];
	Acontrol._hxcounts = (((int16_t)Acontrol._buffer[15]) << 8) | Acontrol._buffer[14];
	Acontrol._hycounts = (((int16_t)Acontrol._buffer[17]) << 8) | Acontrol._buffer[16];
	Acontrol._hzcounts = (((int16_t)Acontrol._buffer[19]) << 8) | Acontrol._buffer[18];
	// transform and convert to float values
	Acontrol._ax = (((float)(Acontrol.tX[0]*Acontrol._axcounts + Acontrol.tX[1]*Acontrol._aycounts + Acontrol.tX[2]*Acontrol._azcounts) * Acontrol._accelScale) - Acontrol._axb)*Acontrol._axs;
	Acontrol._ay = (((float)(Acontrol.tY[0]*Acontrol._axcounts + Acontrol.tY[1]*Acontrol._aycounts + Acontrol.tY[2]*Acontrol._azcounts) * Acontrol._accelScale) - Acontrol._ayb)*Acontrol._ays;
	Acontrol._az = (((float)(Acontrol.tZ[0]*Acontrol._axcounts + Acontrol.tZ[1]*Acontrol._aycounts + Acontrol.tZ[2]*Acontrol._azcounts) * Acontrol._accelScale) - Acontrol._azb)*Acontrol._azs;
	Acontrol._gx = ((float) (Acontrol.tX[0]*Acontrol._gxcounts + Acontrol.tX[1]*Acontrol._gycounts + Acontrol.tX[2]*Acontrol._gzcounts) * Acontrol._gyroScale) -  Acontrol._gxb;
	Acontrol._gy = ((float) (Acontrol.tY[0]*Acontrol._gxcounts + Acontrol.tY[1]*Acontrol._gycounts + Acontrol.tY[2]*Acontrol._gzcounts) * Acontrol._gyroScale) -  Acontrol._gyb;
	Acontrol._gz = ((float) (Acontrol.tZ[0]*Acontrol._gxcounts + Acontrol.tZ[1]*Acontrol._gycounts + Acontrol.tZ[2]*Acontrol._gzcounts) * Acontrol._gyroScale) -  Acontrol._gzb;
	Acontrol._hx = (((float)(Acontrol._hxcounts) * Acontrol._magScaleX) - Acontrol._hxb)*Acontrol._hxs;
	Acontrol._hy = (((float)(Acontrol._hycounts) * Acontrol._magScaleY) - Acontrol._hyb)*Acontrol._hys;
	Acontrol._hz = (((float)(Acontrol._hzcounts) * Acontrol._magScaleZ) - Acontrol._hzb)*Acontrol._hzs;
	Acontrol._t = ((((float) Acontrol._tcounts)  - Acontrol._tempOffset)/ Acontrol._tempScale) + Acontrol._tempOffset;

	return 1;
}

// Returns the accelerometer measurement in the x direction, m/s/s
float Ampu9250GetAccelX_mss( void )
{
	return Acontrol._ax;
}

// Returns the accelerometer measurement in the y direction, m/s/s
float Ampu9250GetAccelY_mss( void )
{
	return Acontrol._ay;
}

// Returns the accelerometer measurement in the z direction, m/s/s
float Ampu9250GetAccelZ_mss( void )
{
	return Acontrol._az;
}

// Returns the gyroscope measurement in the x direction, rad/s
float Ampu9250GetGyroX_rads( void )
{
	return Acontrol._gx;
}

// Returns the gyroscope measurement in the y direction, rad/s
float Ampu9250GetGyroY_rads( void )
{
	return Acontrol._gy;
}

// Returns the gyroscope measurement in the z direction, rad/s
float Ampu9250GetGyroZ_rads( void )
{
	return Acontrol._gz;
}

// Returns the magnetometer measurement in the x direction, uT
float Ampu9250GetMagX_uT( void )
{
  return Acontrol._hx;
}

// Returns the magnetometer measurement in the y direction, uT
float Ampu9250GetMagY_uT( void )
{
  return Acontrol._hy;
}

// Returns the magnetometer measurement in the z direction, uT
float Ampu9250GetMagZ_uT( void )
{
  return Acontrol._hz;
}

// Returns the die temperature, C
float Ampu9250GetTemperature_C( void )
{	return Acontrol._t;

}

/*==================[end of file]============================================*/
