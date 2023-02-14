/*
 * QMCL5883.h
 *
 *  Created on: Jan 13, 2023
 *      Author: Alihan
 */

#ifndef SENSOR_QMCL5883_H_
#define SENSOR_QMCL5883_H_

#include "stm32f4xx_hal.h"
extern I2C_HandleTypeDef hi2c1;
#define NumPoints 3

#define R_Device_address   (0x1A)

#define R_Data_LSB_X		0X00
#define R_Data_MSB_X		0X01
#define R_Data_LSB_Y		0X02
#define R_Data_MSB_Y		0X03
#define R_Data_LSB_Z		0X04
#define R_Data_MSB_Z		0X05
#define R_Status			0X06
#define R_Temp_LSB			0x07
#define R_Temp_MSB			0x08
#define R_Contorl_1			0x09
#define R_Contorl_2         0x0A
#define R_Set_Reset			0x0B

#define TRIALS          			  (2)            /*Number of trials */
#define TIMEOUT        				 (100)                 /*100 ms*/
#define SIZE_OF_ADDRES   			  (1)                  /*1 byte*/
#define SIZE_OF_DATA				  (1) 				   /*1 byte*/

#define RAD_2_DEG  				(57.2957795131f)     /*radian to degree */



typedef enum{

	Standby    = 0x00,
	Continuous = 0x01

}Mode_Config;

typedef enum{

	ODR_10Hz   = (0x00 << 2),
	ODR_50Hz   = (0x01 << 2),
	ODR_100Hz  = (0x02 << 2),
	ODR_200Hz  = (0x03 << 2)

}ODR;

typedef enum{

    SCALE_2G = (0x00 << 4),
	SCALE_8G = (0x01 << 4)


}Scale;

typedef enum{

	OSR_512 = (0x00 << 6),
	OSR_256 = (0x01 << 6),
	OSR_128 = (0x02 << 6),
	OSR_64  = (0x03 << 6)

}OSR;

typedef enum{

	INT_ENB  =  0x01,
	ROL_PNT  = (0x01 << 6),
	SOFT_RST = (0x01 << 7)


}CR2;

typedef enum{


    Error_noneValid=     0x00,
	Data_Ready    =      0x01,
	Data_Overflow = 	 0x02,
	Overflof_and_Skipped=0x03,
	Data_Skipped  = 	 0x04,
	Ready_and_Skipped= 	 0x05,
	Overflow_and_Skipped=0x06,
	All_StatusSet = 	 0x07,
	sFlase        =      0x08,
	sTrue         =      0x09


}Dev_Status;


typedef struct{

	I2C_HandleTypeDef *i2cx_handle;

    int16_t QMCL_Axis_Raw[3];

	int16_t bias[3];

    uint8_t rx_buffer[6];

    int16_t QMCL_Axis_clb[3];

	float heading;

	float scale[3];

    volatile uint8_t RDY_flag;


}QMC5883L;


typedef struct{

	unsigned int filt_head;
	unsigned int filt_tail;
	int16_t total;
	int16_t buffer_t[NumPoints];
	float output;

}Filt_t;

 /*
  * @param  i2chandle that you use
  * @param  QMC5883L *ptr for reach the struct variables
  * @param  Mode_Config device mod config enum
  * @param  ODR data rate (data)
  * @param  Scale scale type device
  * @param  OSR Oversampling
  * @retval Dev_status enum variables
  * */
 Dev_Status Init_QMCL5883(I2C_HandleTypeDef*i2chandle, QMC5883L * dev , Mode_Config Mode ,ODR Rate,Scale scale,OSR Oversamp);
 /*
  * @param  data for decide the device state(taken by status reg)
  * @retval Dev_status enum variables*/
 Dev_Status Status_handler(uint8_t data );
 /*
  *@param  QMC5883L *ptr for reach the struct variables
  *@retval Dev_status enum variables */
 Dev_Status QMC5883L_Start(QMC5883L *dev);
 /*
  * @param  QMC5883L *ptr for reach the struct variables
  * @retval None*/
 void Int_Process_handler(QMC5883L*dev);
 /*
  * @param I2C_HandleTypeDef *hi2c port (ISR)
  * @retval none */
 void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
 /*
  *@param  declination_degs location degree
  *@param  declination_mins location mins
  *@param  declination_dir location directory(E,N)
  *@retval None */
 void SetDeclination( int declination_degs , int declination_mins, char declination_dir );
 /*
  * @param  QMC5883L *ptr for reach the struct variables
  *@retval None*/
 void heading_calc(QMC5883L*dev);
 /*
  * @param  QMC5883L *ptr for reach the struct variables
  *@param   xmax  maximum X axis value(positivie -> +)
  *@param   xmin  minimum X axis value(positivie -> +)
  *@param   ymax  maximum Y axis value(positivie -> +)
  *@param   ymin  minimum Y axis value(positivie -> +)
  *@param   zmax  maximum Z axis value(positivie -> +)
  *@param   zmin  minimum Z axis value(positivie -> +)
  *@retval  None  */
 void calibrating_init(QMC5883L *dev,int16_t xmax,int16_t xmin,int16_t ymax ,int16_t ymin,int16_t zmax ,int16_t zmin);

 /*@param  Filt *filt struct ptr
  *@retval None*/
 void filter_update(Filt_t *filt, int16_t input);
 /*@param  Filt *filt struct ptr
  *@retval None*/
 void Filter_init(Filt_t *filt);


#endif /* SENSOR_QMCL5883_H_ */
