/*
 * DSP.c
 *
 *  Created on: Jan 25, 2023
 *      Author: Alihan
 */

#include "DSP.h"
/***********************************FILTER COEFFICIENT And SPECIFICATION*************************/
/* FILTER FIR:
 *
 * please define your filter design which you calculate for your source this coefficients probably doesnt work .
 * Data Type = float
 * Fs = 100 Hz
 * Fc = 3 Hz
 * Order = 3
 * Type = Butterworth
 * Return type Float*/

#define length 7

float FIRCoeff[length] = {
        0.05320701676022750200,
        0.12339934871587555000,
        0.20292328393882797000,
        0.24094070117013799000,
        0.20292328393882797000,
        0.12339934871587555000,
        0.05320701676022750200
};
/* FILTER FIR:
 *
 * please define your filter design which you calculate for your source this coefficients probably doesnt work .
 * Data Type = int16_t
 * Fs = 100 Hz
 * Fc = 3 Hz
 * Order = 3
 * Type = Butterworth
 * Return type = int32_t*/

#define length16 8
#define DCgain 131072

int16_t	 FIRCoeff16[length16] = {
	        15035,
	        16691,
	        17965,
	        18468,
	        17965,
	        16691,
	        15035,
	        13220
    };


/***********************************************Code************************************************/
void FPU_enable()
{

SCB->CPACR|=((3UL << 10*2) | (3UL << 11*2));


}



void DSP_Filter_init(M_Filt_t *filt)
{

	filt->filt_head = 0;
	filt->output_f =0;
	filt->total = 0;

	memset(filt->buffer_t,0,sizeof(filt->buffer_t));



}

void DSPkalman_init(Kalman *kalman ,float32_t Q, float32_t R)
{

	kalman->Q = Q;
	kalman->R = R;
	kalman->H = 1.0;                      // 1 Dimension  model constant
	kalman->K = 0.0f;
	kalman->P = 0.0f;
	kalman->X = 0.0f;

}



void DSPkalman_initTrust(Kalman*kalman,Trust type)
{

	switch(type){

	case trust_measurmentslevel_low:
		kalman->Q = 0.09;
		kalman->R = 0.01;
		kalman->K = 0.0f;
		kalman->P = 0.0f;
		kalman->X = 0.0f;
		kalman->H = 1;

		break;

	case trust_measurmentslevel_middle:
		kalman->Q =  0.09 ;
		kalman->R =  0.09 ;
		kalman->K = 0.0f;
		kalman->P = 0.0f;
		kalman->X = 0.0f;
		kalman->H = 1;

		break;

	case trust_measurmentslevel_high:
		kalman->Q =  0.01 ;
		kalman->R =  0.09 ;
		kalman->K = 0.0f;
		kalman->P = 0.0f;
		kalman->X = 0.0f;
		kalman->H = 1;

		break;

	default:
		kalman->Q = 0.09;
		kalman->R = 0.01;
		kalman->K = 0.0f;
		kalman->P = 0.0f;
		kalman->X = 0.0f;
		kalman->H = 1;
		break;



	}


}

void DSPexp_moving_avg(M_Filt_t *filt,float32_t input,float Alpha)
{
	  if(Alpha >  1)
	       {
	    	   Alpha = 0.99f;

	       }



	filt->output_f  = Alpha * filt->output_f + (1-Alpha) * input;

}


void DSPmoving_av(M_Filt_t *filt,float32_t input, unsigned int Numpoints)
{



       if(Numpoints > Max_bufferSize)
       {
    	   Numpoints = Max_bufferSize;

       }

	    filt->total =  filt->total + (float)input - filt->buffer_t[filt->filt_head];

	    filt->output_f =(float)( filt->total / Numpoints);

	    filt->buffer_t[filt->filt_head] = input;

	    filt->filt_head++;

	    filt->filt_head = filt->filt_head % Numpoints;


}



void DSPmoving_med(M_Filt_t *filt ,float32_t input, unsigned int Numpoints){

	if(Numpoints > Max_bufferSize)
	       {
	    	   Numpoints = Max_bufferSize;

	       }

filt->buffer_t[filt->filt_head] =input;

filt->filt_head ++;

if(filt->filt_head == Numpoints){


	filt->output_f = insertionSort(filt->buffer_t, Numpoints);
}


    filt->filt_head = filt->filt_head % Numpoints;




}



float insertionSort(float arr[], unsigned int n)
{
	float retval;
   register int i, key, j;
    for (i = 1; i < n; i++) {
        key = arr[i];
        j = i - 1;

        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j = j - 1;
        }
        arr[j + 1] = key;
    }
    if (n % 2 == 0){

    	 retval =(float)( arr[n/2] + arr[(n/2)-1])/2;

    	return retval;
    }
    else if(n % 2 != 0){

    	retval = arr[((n-1)/2)+1];
    	return retval;
    }
return 0;
}


void DSP_firfloat(M_Filt_t *filt , float32_t Src){

	 static float FIR_buf[length];
	 unsigned int tempIndex;
	 unsigned int n;

	  FIR_buf[filt->filt_head] = Src;

	  tempIndex =  filt->filt_head;

	  filt->filt_head++ ;

	  filt->filt_head =  filt->filt_head % length;



	  filt->output_f =0.0f;

	  for(n =0; n<length ;n++){

		  tempIndex > 0 ? (tempIndex--):(tempIndex = length-1);

		  filt->output_f += FIRCoeff[n] * FIR_buf[tempIndex];



	  }




}

void DSP_fir16(M_Filt_t *filt ,int16_t Src){

	  static int16_t FIR_buf[length16];
	  unsigned int tempIndex;
	  unsigned int n;

	  FIR_buf[filt->filt_head] = Src;

	  tempIndex =  filt->filt_head;

	  filt->filt_head++ ;

	  filt->filt_head =  filt->filt_head % length16;



	  filt->output_32 =0;

	  for(n =0; n<length16 ;n++){

		  tempIndex > 0 ? (tempIndex--):(tempIndex = length16-1);

		  filt->output_32 += FIRCoeff16[n] * FIR_buf[tempIndex];

	  }

	  filt->output_32 = filt->output_32/DCgain;
}


void DSP_fastfir16(M_Filt_t *filt ,int16_t Src){

	      static int16_t FIR_buf[length16];


	      int32_t acc_result[1]={0};

	      int16_t *pX; // Circular data buffer
	      int16_t *pB; // Coefficient
	      int32_t *pY; // Result buffer

		  FIR_buf[filt->filt_head] = Src;

		  filt->filt_head++ ;

		  filt->filt_head =  filt->filt_head % length16;

		  pX =  FIR_buf;
		  pB =  FIRCoeff16;
		  pY =  acc_result;

		  for(register unsigned int i =0; i<( length16 >> 1) ;i++ ){ //(length16 / 2 ) 4 conversion for 8 coefficient


		 *__SIMD32(pY) = __SMLAD(*__SIMD32(pX)++,*__SIMD32(pB)++,*__SIMD32(pY) );
		 /*
		  *
		  * p1 = pX[15:0]  * pB[15:0]
            p2 = pX[31:16] * pB[31:16]
            res[31:0] = p1 + p2 + res[31:0]*/


		  }
		  filt->output_32 =  acc_result[0]/DCgain;
		  /*
		   * !!! coefficient size have to be power of 2 for now.. !!!*/

}


void DSPstatistic_32(Statistic *st , int16_t sig, float32_t Sens_RMSnoise){

st->S_buffer[st->head] = (int16_t)sig;

st->head++;

st->head = st->head % siglen;
float32_t k=1;

if(st->head == 0)
{
	//statistic_calc;


	int16_t *pDest;
	int16_t *pVarDest;
	int16_t *pSrc;
	int16_t Accumulator[2]={0,0};
	int32_t *pVarResult;

	 Accumulator[0]= st->S_buffer[0];
	 Accumulator[1]= st->S_buffer[1];

     pDest = Accumulator;
     pSrc  = st->S_buffer;


	for(register unsigned int i = 0 ;i<siglen/2; i++){


		//_ __UHADD16
		 *__SIMD32(pDest) = __UHADD16(*__SIMD32(pSrc)++,*__SIMD32(pDest) );



		 /*  Accumulator[0],Accumulator[1]
		  *
		  +  (S_buffer[i] ,S_buffer[i])..............S_buffer[siglen-1] (i=i+1)

		 -----------------------------------------------------------
         result = half value for every  2 index
         */
	}

	st->mean = (float32_t)((Accumulator[0] +  Accumulator[1]) / 2.0f);

	Accumulator[0] = (int16_t)st->mean;
	Accumulator[1] = (int16_t)st->mean;

	int16_t VarAccm[2]={0,0};
	int32_t ResultAcc[1]={0};
	pVarDest = VarAccm;
	pVarResult = ResultAcc;
	pSrc  = st->S_buffer;

	for(register unsigned int i = 0 ;i<siglen/2; i++){

	   	*__SIMD32(pVarDest) = __QSUB16(*__SIMD32(pSrc)++,*__SIMD32(pDest) );
		*__SIMD32(pVarResult) += __SMUAD(*__SIMD32(pVarDest),*__SIMD32(pVarDest));

         /*variance; ((data[i] - mean)^2 +....data[siglen-1])/siglen =variance
          *
          *
          * */

	}
    st->variance =(float32_t)((float32_t)((ResultAcc[0]))/ siglen);
    st->std = sqrtf(st->variance);
    if(Sens_RMSnoise == 0.0){

       	k = 0;
       	Sens_RMSnoise=1.0;
       }
    st->SNR = k*10.0f * logf((st->mean*st->mean) / (Sens_RMSnoise*Sens_RMSnoise));






}

}

void DSPstatistic_16(Statistic *st , int16_t sig, float32_t Sens_RMSnoise){

st->S_buffer[st->head] = (int16_t)sig;

st->head++;

st->head = st->head % siglen;

int16_t k=1;
if(st->head == 0)
{

	//statistic_calc;


	int16_t *pDest;
	int16_t *pVarDest;
	int16_t *pSrc;
	int16_t Accumulator[2]={0,0};
	int32_t *pVarResult;
	 Accumulator[0]= st->S_buffer[0];
	 Accumulator[1]= st->S_buffer[1];

     pDest =Accumulator;
     pSrc  = st->S_buffer;


	for(register unsigned int i = 0 ;i<siglen/2; i++){


		//__UHADD16
		 *__SIMD32(pDest) = __UHADD16(*__SIMD32(pSrc)++,*__SIMD32(pDest));



		 /*  Accumulator[0],Accumulator[1]
		  *
		  +  (S_buffer[i] ,S_buffer[i])..............S_buffer[siglen-1] (i=i+1)

		 -----------------------------------------------------------
         result = half value for every 2 index
         */
	}

	st->mean = (float32_t)((Accumulator[0] +  Accumulator[1]) / 2.0f);

	Accumulator[0] = (int16_t)st->mean;
	Accumulator[1] = (int16_t)st->mean;

	int16_t VarAccm[2]={0,0};
	int32_t ResultAcc[1]={0};
	pVarDest = VarAccm;
	pVarResult = ResultAcc;
	pSrc  = st->S_buffer;

	for(register unsigned int i = 0 ;i<siglen/2; i++){

	   	*__SIMD32(pVarDest) = __QSUB16(*__SIMD32(pSrc)++,*__SIMD32(pDest) );
		*__SIMD32(pVarResult) += __SMUAD(*__SIMD32(pVarDest),*__SIMD32(pVarDest));

         /*variance; ((data[i] - mean)^2 +....data[siglen-1])/siglen =variance
          *
          *
          * */

	}
    st->variance =(float32_t)((float32_t)((ResultAcc[0]))/ siglen);
    st->std = sqrtf(st->variance);
    if(Sens_RMSnoise == 0.0){

    	k = 0;
    	Sens_RMSnoise=1;
    }
    st->SNR = k*10.0f * logf((st->mean*st->mean) / (Sens_RMSnoise*Sens_RMSnoise));




}

}
float32_t DSPkalman(Kalman*kalman, float32_t  data)
{



    kalman->K = kalman->P * kalman->H / (kalman->H * kalman->P * kalman->H + kalman->R);

    kalman->X = kalman->X + kalman->K *(data - kalman->H * kalman->X);

    kalman->P = (1- kalman->K * kalman->H ) * kalman->P + kalman->Q;



    return  kalman->X;


}





