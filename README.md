# DSP-Library
` DSP-Library with SIMD functions `
` SIMD stands for 'Single Instruction and Multiple Data Stream'. It represents an organization that includes many processing units under the supervision of a common control unit.`


 <img src ="https://user-images.githubusercontent.com/93796314/218828414-cdbd43b9-e83c-4db7-8050-409ef5bf62c4.png" align="right" width="380" height="200">
 <img src ="https://user-images.githubusercontent.com/93796314/218829497-64caa91e-285a-4c1d-a18a-a84690559340.png" align="left" width="380" height="200">




| Functions| Description |
| --- | ----------- |
| ``` DSP_Filter_init(M_Filt_t *filt) ```  | General filter  initialize function |
| ``` DSPkalman_initTrust(Kalman*kalman,Trust type) ``` | This function provide simple initialize for kalman filter |
| ``` DSPkalman_init(Kalman *kalman ,float32_t Q, float32_t R) ``` | Normal initialize for kalman filter this function provide Q,R constant values.This filter is 1-Dimensional so we don't use vectors,transpoze and inverse calculation. |
| ``` DSPexp_moving_avg(M_Filt_t *filt,float32_t input,float Alpha) ```  | Exponential moving average filter |
| ``` DSPmoving_med(M_Filt_t *filt ,float32_t input, unsigned int Numpoints) ``` |  Moving median filter y = median value(X[N]), X[N]={ sorted N numbers}  |
| ``` DSPmoving_av(M_Filt_t *filt,float32_t input, unsigned int Numpoints) ``` |Moving average filter y[n] = ( x[n] + x[n+1]...x[N-1] ) / N |
| ``` DSP_fir16(M_Filt_t *filt ,int16_t Src) ``` | FIR(Finit Impulse Response)filter |
| ``` DSP_fastfir16(M_Filt_t *filt ,int16_t Src) ``` | Fast FIR (Finit Impulse Response) filter with SIMD instructions. |
| ``` DSPstatistic_32(Statistic *st , int16_t sig, float32_t Sens_RMSnoise) ``` | 32 - bit Statistical calculation |
| ``` DSPstatistic_32(Statistic *st , int16_t sig, float32_t Sens_RMSnoise) ``` | 16 - bit Statistical calculation. |
| ``` FPU_enable() ``` |Device FPU(floating point unit ) activation |
| ``` DSPkalman(Kalman*kalman, float32_t  data ``` | Kalman prediction |
| ``` insertionSort(float arr[], unsigned int n) ``` | instertion sort|

## Statistic
### The graph below gives the standard deviations of the FIR filtered signal and the unfiltered signal.


![compar _std](https://user-images.githubusercontent.com/93796314/218825236-76ebfbb8-863a-4e09-b9d4-7f336aa4358c.JPG)

## Statistic Table 
![statistic](https://user-images.githubusercontent.com/93796314/218825956-ade8f305-a92f-4044-829a-1773e4f9cb59.JPG)

## Filters

### FIR filter result
![fir](https://user-images.githubusercontent.com/93796314/218826903-a1ed0733-fd04-4a03-b8c5-76f0c7b1eb32.JPG)

### Fast FIR filter result

![fastfir](https://user-images.githubusercontent.com/93796314/218827214-16345f5b-d320-4310-8f57-cd77daa00709.JPG)

## All Filter Types and Specification

- FIR Filter float
- FIR Filter 16 - bit 
- Fast FIR Filter
- Exponential Moving Average Filter
- Moving Average Filter
- Moving Median Filter
- Statistic calculation 16 and 32 - bit (Standard deviation,mean,SNR,variance)
- SIMD support

## Prerequisites
- CMSIS DSP Library 
- Cortex M4

<h3 align="left">Support:</h3>
<p><a href="https://www.buymeacoffee.com/alihancoban"> <img align="left" src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" height="50" width="210" alt="https://www.buymeacoffee.com/alihancoban" /></a></p><br><br>






