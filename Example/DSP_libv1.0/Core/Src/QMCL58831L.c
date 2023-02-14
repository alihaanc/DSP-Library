/*
 * QMCL58831L.c
 *
 *  Created on: Jan 13, 2023
 *      Author: Alihan
 */
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "math.h"
#include "QMCL5883.h"
#include "string.h"


extern QMC5883L dev;

float offset_radians = 0.0f;
Filt_t filt[3];


Dev_Status Init_QMCL5883(I2C_HandleTypeDef*i2chandle, QMC5883L * dev , Mode_Config Mode ,ODR Rate,Scale scale,OSR Oversamp){

    for(unsigned int i =0 ;i<3; i++){


    Filter_init(&filt[i]);


    }

	dev->i2cx_handle = i2chandle;

	uint8_t config_data = 0x00 ;

	config_data |= ( Mode|Rate |scale |Oversamp );

	uint8_t CR2_data  = 0x40;

	uint8_t Set_reset = 0x01;

	uint8_t tempRx ;


	while(!(HAL_I2C_Mem_Write(dev->i2cx_handle, R_Device_address, R_Set_Reset, SIZE_OF_ADDRES, &Set_reset, SIZE_OF_DATA, TIMEOUT)==HAL_OK));
	while(!(HAL_I2C_Mem_Write(dev->i2cx_handle, R_Device_address, R_Contorl_1, SIZE_OF_ADDRES, &config_data,SIZE_OF_DATA, TIMEOUT)==HAL_OK));
	while(!(HAL_I2C_Mem_Write(dev->i2cx_handle, R_Device_address, R_Contorl_2, SIZE_OF_ADDRES, &CR2_data, SIZE_OF_DATA , TIMEOUT)==HAL_OK));


	HAL_Delay(10);

    while(!(HAL_I2C_Mem_Read(dev->i2cx_handle, R_Device_address,R_Status, SIZE_OF_ADDRES, &tempRx, SIZE_OF_DATA, TIMEOUT)==HAL_OK));

return Status_handler(tempRx);

}


Dev_Status QMC5883L_Start(QMC5883L *dev)
{
	uint8_t tempRx;
	HAL_StatusTypeDef status;


	tempRx = 0;

	status = HAL_I2C_Mem_Read(dev->i2cx_handle, R_Device_address,R_Status, SIZE_OF_ADDRES, &tempRx, SIZE_OF_DATA, TIMEOUT);

	if((tempRx & Data_Ready)== 1 && (status == HAL_OK)){
    dev->RDY_flag = 1;

	HAL_I2C_Mem_Read_IT(dev->i2cx_handle, R_Device_address, R_Data_LSB_X, SIZE_OF_ADDRES,  dev->rx_buffer, 6);


	/*for (unsigned int i =0;i<3;i++)
	{


	filter_update(&filt[i],dev->QMCL_Axis_clb[i]);

	 dev->QMCL_Axis_clb[i] = filt[i].output;


	}*/



	heading_calc(dev);




	return Status_handler(tempRx);

	}

	return Status_handler(tempRx);

}
void Int_Process_handler(QMC5883L*dev)
{

      dev->QMCL_Axis_Raw[0] = (int16_t)( (dev->rx_buffer[1] << 8)| dev->rx_buffer[0] );
      dev->QMCL_Axis_Raw[1] = (int16_t)( (dev->rx_buffer[3] << 8)| dev->rx_buffer[2] );
      dev->QMCL_Axis_Raw[2] = (int16_t)( (dev->rx_buffer[5] << 8)| dev->rx_buffer[4] );

     // for(unsigned int i =0 ; i<3 ;i++){


     // dev->QMCL_Axis_clb[i] = dev->scale[i]* (dev->QMCL_Axis_Raw[i] - dev->bias[i]);


     // }
      dev->RDY_flag = 0;

}
Dev_Status Status_handler(uint8_t data ){

    switch(data){

    case Data_Ready:
    	return Data_Ready;
    	break;
    case Data_Overflow:
        	return Data_Overflow;
        	break;
    case Overflof_and_Skipped:
        	return Overflof_and_Skipped;
        	break;
    case Data_Skipped:
        	return Data_Skipped;
        	break;
    case Ready_and_Skipped:
        	return Ready_and_Skipped;
        	break;
    case Overflow_and_Skipped:
           	return Overflow_and_Skipped;
           	break;
    case All_StatusSet:
           	return All_StatusSet;
           	break;
    case Error_noneValid:
               	return Error_noneValid;
               	break;
    case sFlase:
           	return sFlase;
           	break;
    case sTrue:
              	return sTrue;
              	break;
    default:
    	return Error_noneValid ;

    }


}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){


	if(hi2c->Instance ==dev.i2cx_handle->Instance){

		Int_Process_handler(&dev);

		dev.RDY_flag = 0;
	}

	}

/*thanks to riza celik for this function https://github.com/rizacelik/STM32-HAL-Compass-HMC5883L-
 * find your own value*/

void SetDeclination( int declination_degs , int declination_mins, char declination_dir )
{

  switch (declination_dir)
  {

    case 'E':
      offset_radians = 0 - ( declination_degs + (1 / 60 * declination_mins)) * (M_PI / 180);
      break;


    case 'W':
      offset_radians =  (( declination_degs + (1 / 60 * declination_mins) ) * (M_PI / 180));
      break;
  }

}


void heading_calc(QMC5883L*dev)
{
	   dev->heading = atan2f((float)dev->QMCL_Axis_Raw[1] ,(float)dev->QMCL_Axis_Raw[0] ) ;


	   dev->heading += offset_radians;
	   dev->heading  = dev->heading * RAD_2_DEG ;

	   if (dev->heading>0){

	   	  }

	   	  else
	   	  {

	   		  dev->heading += 360.0f;
	   	  }


}



void calibrating_init(QMC5883L *dev,int16_t xmax,int16_t xmin,int16_t ymax ,int16_t ymin,int16_t zmax ,int16_t zmin)
{

	 int16_t average_delta [3];
	 float average_delta_Sum;

	 dev->bias[0] = (xmax + xmin) / 2;
	 dev->bias[1] = (ymax + ymin) / 2;
	 dev->bias[2] = (zmax + zmin) / 2;

	 average_delta[0] = (xmax - xmin) / 2;
	 average_delta[1] = (ymax - ymin) / 2;
	 average_delta[2] = (zmax - zmin) / 2;

	 average_delta_Sum =(float)(( average_delta[0] + average_delta[1] + average_delta[2])/3);

	 dev->scale[0] = (float)(average_delta_Sum /average_delta[0]);
	 dev->scale[1] = (float)(average_delta_Sum /average_delta[1]);
	 dev->scale[2] = (float)(average_delta_Sum /average_delta[2]);


}


void Filter_init(Filt_t *filt)
{

	filt->filt_head = 0;
	filt->filt_tail = 0;
	filt->output =0;
	filt->total = 0;

	memset(filt->buffer_t,0,sizeof(filt->buffer_t));



}

void filter_update(Filt_t *filt, int16_t input){


   /* filt->filt_head = filt->filt_head % NumPoints;


    filt->buffer_t[filt->filt_head] = input;

    filt->filt_head++;

    if(filt->filt_head == NumPoints){

    	do{

    	filt->filt_tail = filt->filt_tail % NumPoints;

    	filt->total += filt->buffer_t[filt->filt_tail];

    	filt->filt_tail++ ;


    	}while(!(filt->filt_tail == NumPoints ));

    	filt->output_f =  (filt->total / NumPoints);

    	filt->total =0;

    }*/

    /*More effective algorithm*/

    filt->total =  filt->total + input - filt->buffer_t[filt->filt_head];


    filt->output =(float)( filt->total / NumPoints);

    filt->buffer_t[filt->filt_head] = input;

    filt->filt_head++;

    if (filt->filt_head == NumPoints){

    	filt->filt_head= 0;}





}







