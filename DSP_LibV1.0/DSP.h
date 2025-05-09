/*
 * DSP.h
 *
 *  Created on: Jan 25, 2023
 *      Author: Alihan
 */

#ifndef HW_DSP_DSP_H_
#define HW_DSP_DSP_H_

#define Max_bufferSize 50
#define OptNumpoint  5
#define MinNumpoint  2
#define MaxNumpoint 15
#define siglen 8    // for performance  use lower values

#include "main.h"
#include "arm_math.h"
#include "math.h"
#include  "cmsis_gcc.h"
/*
 * Filter struct*/
typedef struct{

	unsigned int filt_head;                 // Head index of ring buffer
	float32_t total;                        // Total value of buffer
	float32_t buffer_t[Max_bufferSize];     // Ring buffer
	float32_t output_f;                     // float32_t type filter output
	int32_t output_32;						// int32_t type filter output

}M_Filt_t;

/* kalman struct
 * 1-Dimensional Kalman Filter we don't need  vectors*/
typedef struct{

	 float32_t R;                           // Measurement Uncertainty
	 float32_t H;                      		// Observation constant
	 float32_t Q;							// Process Noise Uncertainty
	 float32_t P;                           // Estimate Uncertainty
	 float32_t X;							// State
	 float32_t K;							// Kalman Gain


}Kalman;

/*
 * Statitic struct*/
typedef struct{

	unsigned int head;          			// Head index of ring buffer
	float32_t mean;							// mean value final variable
	float32_t variance;						// variance value final variable
	float32_t std;							// standart deviation value final variable
	int16_t S_buffer[siglen];				// Ring buffer
	float32_t SNR;							// Signal to noise ratio final variable

}Statistic;

/*
 *Kalman enumeration for filtering*/
typedef enum{

	trust_measurmentslevel_low,				//  Q is higher then R
	trust_measurmentslevel_middle,			//  Q is equal R
	trust_measurmentslevel_high,			//  Q is lower then R

}Trust;



/*
 * @brief General filter  initialize function
 * @param M_Filt_t for reach struct variables
 * @retval none*/
void DSP_Filter_init(M_Filt_t *filt);
/*
 * @brief  this function provide simple initialize for kalman filter.
 * @param Enumartion operator for 3 state(Trust)
 * @retval none*/
void DSPkalman_initTrust(Kalman*kalman,Trust type);
/*
 * @brief Normal initialize for kalman filter this function provide Q,R constant values.This filter is 1-Dimensional so we don use vectors,
 * transpoze and inverse calculation.
 * @param Q modelling error, if Q is higher than R kalman filter trust the measurements for prediction.
 * @param R measurment error, if R is higher than Q kalman filter trust the model(scalar) for prediction
 * @retval none */
void DSPkalman_init(Kalman *kalman ,float32_t Q, float32_t R);
/*
 * @brief Exponential moving average filter
 * @param filter struct variable for reach the struct variables
 * @param Alpha value for filter characteristic. Alpha have to be 1 between 0 for stability
 * @retval none*/
void DSPexp_moving_avg(M_Filt_t *filt,float32_t input,float Alpha);
/*
 * @brief Moving average filter y[n]= (x[n] + x[n+1]...x[N-1])/N
 * @param average point(N)
 * @retval none*/
void DSPmoving_av(M_Filt_t *filt,float32_t input, unsigned int Numpoints);
/*
 * @brief Moving average filter y = median value(X[N]), X[N]={ sorted N numbers}
 * @param median point(N)
 * @retval none*/
void DSPmoving_med(M_Filt_t *filt ,float32_t input, unsigned int Numpoints);
/*
 * @brief FIR(Finit Impulse Response)filter
 * @param Float32_t type source(data)
 * @retval none*/
void DSP_firfloat(M_Filt_t *filt,float32_t Src);
/*
 * @brief FIR(Finit Impulse Response)filter
 * @param int16_t type source(data)
 * @retval none*/
void DSP_fir16(M_Filt_t *filt ,int16_t Src);
/*
 * @brief Fast FIR(Finit Impulse Response)filter with SIMD instructions.
 * @param int16_t type source(data)
 * @retval none
 * @warning ! coefficients size have to be power of 2 for now.. */
void DSP_fastfir16(M_Filt_t *filt ,int16_t Src);
/*
 * @brief Struct variable for reach the variables in struct
 * @param Signal(data)  float32_t
 * @param Sensor RMS noise for example MPU6050 RMS noise 0.003 in datasheet
 * @retval none*/
void DSPstatistic_32(Statistic *st , int16_t sig, float32_t Sens_RMSnoise);
/*
 * @brief Struct variable for reach the variables in struct
 * @param Signal(data)  float32_t
 * @param Sensor RMS noise for example MPU6050 RMS noise 0.003 in datasheet
 * @retval none*/
void DSPstatistic_16(Statistic *st , int16_t sig, float32_t Sens_RMSnoise);
/*
 * @brief Device FPU(floating point unit ) activation
 * @param none
 * @retval none*/
void FPU_enable();
/*
 * @brief  Kalman prediction
 * @param  float32_t data
 * @retval none*/
float32_t DSPkalman(Kalman*kalman, float32_t  data);
/*
 * @brief  instertion sort for sorting
 * @param  array for sorting
 * @param  (n) size value
 * @retval none*/
float insertionSort(float arr[], unsigned int n);


#endif /* HW_DSP_DSP_H_ */
