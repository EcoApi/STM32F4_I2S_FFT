/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* source used :
  https://github.com/Steppeschool/STM32-Audio-recording-/tree/Real-Time_FFT
  https://github.com/andysworkshop/usb-microphone
  https://github.com/YetAnotherElectronicsChannel/STM32_FFT_Spectrum_Analysis

plotter :
  https://github.com/hyOzd/serialplot

*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

//#include "audio_sd.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"
#include <math.h>


//#define DEBUG_SAMPLING_FREQUENCY      1
#define FFT_USE_WINDOWING             1
#define FFT_USE_FILTER_HIGH           0
#define FFT_USE_FILTER_LOW            0
#define FTT_USE_FREQ_FACTOR           1
#define FTT_USE_I2S_STANDBY_OFFSET    1
#define FFT_LOG_ENABLED               1
#define FFT_LOOP_ENABLED              1

#define FFT_SAMPLE_FREQ_HZ            (4000) /* only : 2000 / 4000 / 8000 / 16000 Hz */
#define FFT_OUTPUT_SIZE               1024UL
#define FTT_COUNT                     2 
#define I2S_N_U16_BY_CHANNEL          2
#define I2S_N_CHANNEL                 2
#define FFT_CYCLE                     (FTT_COUNT + (FTT_COUNT % 2))

#if (FTT_USE_I2S_STANDBY_OFFSET == 1)
  /*
    Compute :
      Use #define FFT_I2S_COMPUTE_STANDBY_OFFSET 1 for offset computation 
  */

  #if (FFT_SAMPLE_FREQ_HZ == 2000)
    #define FFT_I2S_STANDBY_OFFSET      (0) /* For sampling 2000Hz, res 1.91Hz */
  #elif (FFT_SAMPLE_FREQ_HZ == 4000)
    #define FFT_I2S_STANDBY_OFFSET      (4096)//(136) /* For sampling 4000Hz, res 3.79Hz */
  #elif  (FFT_SAMPLE_FREQ_HZ == 8000)
    #define FFT_I2S_STANDBY_OFFSET      (0) /* For sampling 8000Hz, res 7.6Hz */
  #elif  (FFT_SAMPLE_FREQ_HZ == 16000)
    #define FFT_I2S_STANDBY_OFFSET      (0) /* For sampling 16000Hz, res 15.20Hz */      
  #else
    //try sample 4100Hz for beep with ratio 1.95
    #define FFT_I2S_STANDBY_OFFSET      (0) /* For sampling 16000Hz */      
  #endif

  #define FFT_I2S_STANDBY_OFFSET_SIZE   (FFT_I2S_STANDBY_OFFSET * I2S_N_U16_BY_CHANNEL * I2S_N_CHANNEL)
#else
  #define FFT_I2S_STANDBY_OFFSET_SIZE   (0)
#endif 

#define FFT_I2S_SAMPLING_SIZE         (FFT_OUTPUT_SIZE * 2)
#define FFT_I2S_BUFFER_SIZE           (((FFT_OUTPUT_SIZE * I2S_N_U16_BY_CHANNEL * I2S_N_CHANNEL) + FFT_I2S_STANDBY_OFFSET_SIZE) * FFT_CYCLE)
#define FFT_COMPLEX_INPUT             (FFT_OUTPUT_SIZE * 2UL)

#if (FTT_USE_FREQ_FACTOR == 1)
  #if (FFT_SAMPLE_FREQ_HZ == 2000)
    #define FFT_SAMPLE_FREQ_FACTOR      (1.9526) /* For sampling 2000Hz, res 1.91Hz */
  #elif (FFT_SAMPLE_FREQ_HZ == 4000)
    #define FFT_SAMPLE_FREQ_FACTOR      (1.9845) /* For sampling 4000Hz, res 3.79Hz */
  #elif  (FFT_SAMPLE_FREQ_HZ == 8000)
    #define FFT_SAMPLE_FREQ_FACTOR      (1.9466) /* For sampling 8000Hz, res 7.6Hz */
  #elif  (FFT_SAMPLE_FREQ_HZ == 16000)
    #define FFT_SAMPLE_FREQ_FACTOR      (1.9451) /* For sampling 16000Hz, res 15.20Hz */      
  #else
    //try sample 4100Hz for beep with ratio 1.95
    #define FFT_SAMPLE_FREQ_FACTOR      (1.950) /* For sampling 16000Hz */      
  #endif
#else
  /*
    Apply in input 100Hz, 200Hz, 500Hz, 1000Hz, 1500Hz, 2000Hz, etc ....
    And read the frequency in trace (for exemple 54.69HZ): Max value: 15.6448 at [7] = 54.69 Hz" 
    And divide each target frequency by each measure : ex 1000 / 54.69 HZ = Ratio for 1000 Hz target
    Finish by compute average of each ratio (FFT_SAMPLE_FREQ_FACTOR = average of each ratio)
  */
  #define FFT_SAMPLE_FREQ_FACTOR      (1.0)
#endif 

#define FFT_SAMPLE_RES_HZ             ((float32_t)(((float32_t)FFT_SAMPLE_FREQ_HZ / (float32_t)FFT_COMPLEX_INPUT) * (float32_t)FFT_SAMPLE_FREQ_FACTOR))
#define FFT_HIGH_CUTT_OFF_FREQ        50 /* Hz */
#define FFT_LOW_CUTT_OFF_FREQ         50 /* Hz */

#ifndef M_PI
  #define M_PI       3.14159265358979323846
#endif

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

uint16_t i2sData[FFT_I2S_BUFFER_SIZE];

static uint8_t i2sDataCount = 0;
static uint8_t fftPerformed = 0;
static bool fftStarted = FALSE;

arm_rfft_fast_instance_f32 S;

static float m_fft_output_f32[FFT_OUTPUT_SIZE];             //!< FFT output data. Frequency domain.

FFT_RESULTS fftResult[FTT_COUNT];

char sz_traceBuffer[10000];
size_t sz_traceBufferSize;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART2_UART_Init(void);

static void print_dataShort(const char* sz_title, const uint16_t *p_data, uint16_t size);
static void print_dataInteger(const char* sz_title, const uint32_t *p_data, uint16_t size);
static void print_dataU16Hex(const char* sz_title, const uint16_t *p_data, uint16_t size);
static void print_dataU32Hex(const char* sz_title, const uint32_t *p_data, uint16_t size);
static void print_dataFloat(const char* sz_title, const float32_t *p_data, uint16_t size);
static void print_plotter(float const * p_data, uint16_t size);
static void print_fft_min(float * m_fft_output_f32, uint16_t data_size);
static void print_fft_max(float * m_fft_output_f32, uint16_t data_size);
static void print_fft_result(FFT_RESULTS *p_fftResult);
static void fft_create_result(FFT_RESULTS *p_fftResult, float *p_fftValue, uint16_t binOutputCount, uint16_t binOffset, uint16_t binOutputSize, uint16_t fftSize);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief Macro to be used in a formatted string to a pass float number to the log.
 *
 * Use this macro in a formatted string instead of the %f specifier together with
 * @ref NRF_LOG_FLOAT macro.
 * Example: NRF_LOG_INFO("My float number" TRACE_FLOAT_MARKER "\r\n", TRACE_FLOAT(f)))
 */
//#define TRACE_FLOAT_MARKER "%s%d.%02d"

#define TRACE_FLOAT_MARKER "%s%d.%d"

/**
 * @brief Macro for dissecting a float number into two numbers (integer and residuum).
 */

#define EXP2(a, b) a ## b
#define EXP(a, b) EXP2(a ## e,b)
#define POW(C,x) EXP(C, x)
#define TRACE_FLOAT_PRECISION 4

#define TRACE_FLOAT(val) (uint32_t)(((val) < 0 && (val) > -1.0) ? "-" : ""),   \
                           (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))* 10000)

#define TRACE(fmt, ...) trace(fmt, ##__VA_ARGS__)

void trace(const char * format, ... ) {
  va_list argptr;

  va_start(argptr, format);
  sz_traceBufferSize = vsnprintf(sz_traceBuffer, sizeof(sz_traceBuffer), format, argptr);
  va_end(argptr);

//  if(HAL_UART_Transmit_IT(&huart1, (uint8_t*)sz_traceBuffer, sz_traceBufferSize)!= HAL_OK) {
//    Error_Handler();
//  }

  if(HAL_UART_Transmit(&huart2, (uint8_t*)sz_traceBuffer, sz_traceBufferSize, 1000)!= HAL_OK) {
    Error_Handler();
  }
}

#if (FFT_USE_WINDOWING == 1)
//https://github.com/sidneycadot/WindowFunctions
//https://github.com/kichiki/WaoN/blob/master/fft.c
//https://github.com/kfrlib/kfr

/* Reference: "Numerical Recipes in C" 2nd Ed.
 * by W.H.Press, S.A.Teukolsky, W.T.Vetterling, B.P.Flannery
 * (1992) Cambridge University Press.
 * ISBN 0-521-43108-5
 * Sec.13.4 - Data Windowing
 */
double
parzen (int i, int nn)
{
  return (1.0 - fabs (((double)i-0.5*(double)(nn-1))
		      /(0.5*(double)(nn+1))));
}

double
welch (int i, int nn)
{
  return (1.0-(((double)i-0.5*(double)(nn-1))
	       /(0.5*(double)(nn+1)))
	  *(((double)i-0.5*(double)(nn-1))
	    /(0.5*(double)(nn+1))));
}

double
hanning (int i, int nn)
{
  return ( 0.5 * (1.0 - cos (2.0*M_PI*(double)i/(double)(nn-1))) );
}

/* Reference: "Digital Filters and Signal Processing" 2nd Ed.
 * by L. B. Jackson. (1989) Kluwer Academic Publishers.
 * ISBN 0-89838-276-9
 * Sec.7.3 - Windows in Spectrum Analysis
 */
double
hamming (int i, int nn)
{
  return ( 0.54 - 0.46 * cos (2.0*M_PI*(double)i/(double)(nn-1)) );
}

double
blackman (int i, int nn)
{
  return ( 0.42 - 0.5 * cos (2.0*M_PI*(double)i/(double)(nn-1))
	  + 0.08 * cos (4.0*M_PI*(double)i/(double)(nn-1)) );
}

double
steeper (int i, int nn)
{
  return ( 0.375
	  - 0.5 * cos (2.0*M_PI*(double)i/(double)(nn-1))
	  + 0.125 * cos (4.0*M_PI*(double)i/(double)(nn-1)) );
}

/* apply window function to data[]
 * INPUT
 *  flag_window : 0 : no-window (default -- that is, other than 1 ~ 6)
 *                1 : parzen window
 *                2 : welch window
 *                3 : hanning window
 *                4 : hamming window
 *                5 : blackman window
 *                6 : steeper 30-dB/octave rolloff window
 */
void
windowing (int n, const float32_t *data, int flag_window, float32_t scale, float32_t *out) {
  int i;
  for (i = 0; i < n; i ++) {
    switch (flag_window) {
	case 1: // parzen window
	  out [i] = data [i] * (float32_t) parzen (i, n) / scale;
	  break;

	case 2: // welch window
	  out [i] = data [i] * (float32_t) welch (i, n) / scale;
	  break;

	case 3: // hanning window
	  out [i] = data [i] * (float32_t) hanning (i, n) / scale;
	  break;

	case 4: // hamming window
	  out [i] = data [i] * (float32_t) hamming (i, n) / scale;
	  break;

	case 5: // blackman window
	  out [i] = data [i] * (float32_t) blackman (i, n) / scale;
	  break;

	case 6: // steeper 30-dB/octave rolloff window
	  out [i] = data [i] * (float32_t) steeper (i, n) / scale;
	  break;

	default:
	  //fprintf (stderr, "invalid flag_window\n");
	case 0: // square (no window)
	  out [i] = data [i] / scale;
	  break;
	  }
  }
}

#endif

/* all 2048 bytes */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
  if(++i2sDataCount >= FTT_COUNT)
    HAL_I2S_DMAStop(&hi2s2);
  
  if(!fftStarted)
    fftStarted = TRUE;  
}

/* all 4096 bytes */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
  i2sDataCount++;

  HAL_I2S_DMAStop(&hi2s2);

  if(!fftStarted) 
    fftStarted = TRUE;  
}

bool processDSP(void) {
  uint16_t i2sIndex = fftPerformed * FFT_OUTPUT_SIZE * I2S_N_CHANNEL;
  float32_t f32_fftSamples[FFT_OUTPUT_SIZE];
  float32_t dcComponent = 0.0;
  uint32_t i;
  uint16_t *p_i2sData = NULL;

  if(!fftStarted || (fftPerformed > FTT_COUNT))
    return FALSE;

  p_i2sData = (FFT_I2S_STANDBY_OFFSET_SIZE) ? &i2sData[i2sIndex + FFT_I2S_STANDBY_OFFSET_SIZE] : &i2sData[i2sIndex];

#if (FFT_LOG_ENABLED == 1)
  TRACE("New fft %d: index %d\r\n", fftPerformed, i2sIndex);  
  //print_dataInteger("I2S samples", (uint32_t*)p_i2sData, FFT_I2S_BUFFER_SIZE / 2);
  //print_dataShort("I2S samples", (uint16_t*) p_i2sData, FFT_I2S_BUFFER_SIZE / 2);
  //print_dataU16Hex("I2S samples", p_i2sData, FFT_I2S_BUFFER_SIZE / 2);
#endif

	/* extract 16bits from 24 bits left i2s mono sample and compute DC component */
	for(i = 0; i < (FFT_OUTPUT_SIZE); i++ ) { 
    //uint16_t part_1_left = i2sData[((i * sizeof(uint32_t)) + i2sIndex)];
    //uint16_t part_2_left = i2sData[((i * sizeof(uint32_t)) + i2sIndex) + 1];
    uint32_t part_left = (p_i2sData[i * sizeof(uint32_t)] << 16) | p_i2sData[(i * sizeof(uint32_t)) + 1];
    int32_t part_left_signed = (int32_t) part_left >> 8; 

    //uint16_t part_1_right = i2sData[((i * sizeof(uint32_t)) + i2sIndex) + 2];
    //uint16_t part_2_right = i2sData[((i * sizeof(uint32_t)) + i2sIndex) + 3];
    //uint32_t part_right = (i2sData[((i * sizeof(uint32_t)) + i2sIndex) + 2] << 16) | i2sData[((i * sizeof(uint32_t)) + i2sIndex) + 3];

    f32_fftSamples[i] = (float32_t) part_left_signed;

    //TRACE("%d: %08x %08x %d\r\n", i, part_left, part_left_signed, part_left_signed);

    dcComponent += f32_fftSamples[i];
  }
  dcComponent = dcComponent / (float32_t) FFT_OUTPUT_SIZE;

#if (FFT_LOG_ENABLED == 1)
  TRACE("FFT DC component %f\r\n", dcComponent);
#endif

  /* remove dc component for each sample */
  for(i=0;i<FFT_OUTPUT_SIZE;i++)
    f32_fftSamples[i] -= dcComponent;

#if (FFT_I2S_COMPUTE_STANDBY_OFFSET == 1)
  for(i=0;i<FFT_OUTPUT_SIZE;i++) {
    if(f32_fftSamples[i] >= -100000.0)
      continue;
    
    break;
  }

  TRACE("offset exit standby %d [index %d]\r\n", (i == FFT_OUTPUT_SIZE) ? 0 : i, i);
#endif

#if (FFT_LOG_ENABLED == 1)
  //print_dataFloat("I2S samples without dc component", f32_fftSamples, FFT_OUTPUT_SIZE);
  //print_plotter(f32_fftSamples, FFT_OUTPUT_SIZE);
#endif

#if (FFT_USE_WINDOWING == 1) 
/*  flag_window : 0 : no-window (default -- that is, other than 1 ~ 6)
 *                1 : parzen window
 *                2 : welch window
 *                3 : hanning window
 *                4 : hamming window
 *                5 : blackman window
 *                6 : steeper 30-dB/octave rolloff window
 */
  windowing(FFT_OUTPUT_SIZE, f32_fftSamples, 4 /* hamming */, 1.0, f32_fftSamples);

#if (FFT_LOG_ENABLED == 1)
  //print_dataFloat("ADC samples after windowing", f32_fftSamples, FFT_OUTPUT_SIZE);
#endif
#endif

  arm_rfft_fast_f32(&S, f32_fftSamples, m_fft_output_f32, 0);
	
#if (FFT_USE_FILTER_HIGH == 1)
  //high-pass filter
  float32_t FcutHigh = FFT_HIGH_CUTT_OFF_FREQ / FFT_SAMPLE_RES_HZ; 
  
  for(uint32_t i=0;i<FFT_OUTPUT_SIZE;i++) { //set frequencies <FFT_HIGH_CUTT_OFF_FREQ Hz to zero
    if(((float32_t)i < (FcutHigh * 2)) || ((float32_t)i > ((float32_t) FFT_OUTPUT_SIZE - (FcutHigh * 2)))) {
      m_fft_output_f32[i] = 0; // Real part.
    }
  }
#endif

#if (FFT_USE_FILTER_LOW == 1)
  //low-pass filter
  float32_t FcutLow = FFT_LOW_CUTT_OFF_FREQ / FFT_SAMPLE_RES_HZ;

  for(uint32_t i = 0; i< FFT_OUTPUT_SIZE; i++) { //set frequencies >FFT_HIGH_CUTT_OFF_FREQ Hz to zero
    if ((((float32_t)i > (FcutLow*2)) && (i < FFT_OUTPUT_SIZE)) || ((i > FFT_OUTPUT_SIZE) &&((float32_t)i < ((float32_t)FFT_OUTPUT_SIZE - (FcutLow * 2))))) {
      m_fft_output_f32[i] = 0; // Real part.
    }
  }  
#endif

  /* Calculate the magnitude */
	arm_cmplx_mag_f32(m_fft_output_f32, m_fft_output_f32, FFT_OUTPUT_SIZE / 2);

#if (FFT_LOG_ENABLED == 1)
  //print_plotter(m_fft_output_f32, FFT_OUTPUT_SIZE); // full 
  //print_plotter(m_fft_output_f32, FFT_OUTPUT_SIZE / 2); // N/2 (nyquist-theorm)
  //print_dataFloat("FFT uncompressed", m_fft_output_f32, FFT_OUTPUT_SIZE / 2);
  //print_fft_min(m_fft_output_f32, FFT_OUTPUT_SIZE / 2); // N/2 (nyquist-theorm)
  print_fft_max(m_fft_output_f32, FFT_OUTPUT_SIZE / 2); // N/2 (nyquist-theorm)
#endif

  float32_t min_value = 0;
  uint32_t  min_val_index = 0;

  arm_min_f32(m_fft_output_f32, FFT_OUTPUT_SIZE / 2, &min_value, &min_val_index);

  for(uint32_t i=0;i<FFT_OUTPUT_SIZE / 2;i++) { 
    m_fft_output_f32[i] -= min_value;
  }

  /* Remove first bin correspond to the DC component */
  m_fft_output_f32[0] = 0.0; 

  //print_plotter(m_fft_output_f32, FFT_OUTPUT_SIZE / 2); // N/2 (nyquist-theorm)

  /* Create new fft output and compress the data */
  fft_create_result(&fftResult[fftPerformed], m_fft_output_f32, 20 /* from flash */, 0 /* from flash */, 13 /* from flash */, FFT_OUTPUT_SIZE / 2); // N/2 (nyquist-theorm)

#if (FFT_LOG_ENABLED == 1)
  //print_dataShort("FFT compressed", fftResult[fftPerformed].values, fftResult[fftPerformed].bins);
  print_fft_result(&fftResult[fftPerformed]);
#endif

  if(++fftPerformed >= i2sDataCount)
    fftStarted = FALSE;

  return FALSE;
}

void print_dataShort(const char* sz_title, const uint16_t *p_data, uint16_t size) {
  uint16_t i;

  TRACE("%s :\r\n", sz_title);
  
  for (i = 0; i<size; i++) {
		if(i < (size - 1))
			TRACE("[ %d ] ", p_data[i]);
		else
			TRACE("[ %d ]\r\n", p_data[i]);
	}	
}

void print_dataInteger(const char* sz_title, const uint32_t *p_data, uint16_t size) {
  uint16_t i;

  TRACE("%s :\r\n", sz_title);
  
  for (i = 0; i<size; i++) {
		if(i < (size - 1))
			TRACE("[ %d ]", p_data[i]);
		else
			TRACE("[ %d ]\r\n", p_data[i]);
	}	
}

void print_dataU16Hex(const char* sz_title, const uint16_t *p_data, uint16_t size) {
  uint16_t i;

  TRACE("%s :\r\n", sz_title);
  
  for (i = 0; i<size; i++) {
		if(i < (size - 1))
			TRACE("[ %04x ]", p_data[i]);
		else
			TRACE("[ %04x ]\r\n", p_data[i]);
	}	
}

void print_dataU32Hex(const char* sz_title, const uint32_t *p_data, uint16_t size) {
  uint16_t i;

  TRACE("%s :\r\n", sz_title);
  
  for (i = 0; i<size; i++) {
		if(i < (size - 1))
			TRACE("[ %08x ]", p_data[i]);
		else
			TRACE("[ %08x ]\r\n", p_data[i]);
	}	
}

void print_dataFloat(const char* sz_title, const float32_t *p_data, uint16_t size) {
  uint16_t i;

  TRACE("%s :\r\n", sz_title);
  
  for(i=0;i<size;i++) {
		if(i < (size - 1))
			TRACE("[ %f ]", p_data[i]);
		else
			TRACE("[ %f ]\r\n", p_data[i]);
	}	
}

void print_plotter(float const * p_data, uint16_t size) {
  uint16_t i;
  for (i = 0; i<size; i++)
    TRACE(TRACE_FLOAT_MARKER"\r\n", TRACE_FLOAT(p_data[i]));
}

void print_fft_min(float * m_fft_output_f32, uint16_t data_size) {
  float32_t min_value = 0;
  uint32_t  min_val_index = 0;
  
  uint32_t offset = 0;//10;
  arm_min_f32(&m_fft_output_f32[offset], data_size - offset, &min_value, &min_val_index);
  min_val_index += offset;
  
  TRACE("Frequency sample: %f Hz\r\n", (float32_t) FFT_SAMPLE_RES_HZ);
  TRACE("Min magnitude value: "TRACE_FLOAT_MARKER", index %d, frequency %f Hz\r\n", TRACE_FLOAT(min_value), min_val_index, (float32_t) min_val_index * FFT_SAMPLE_RES_HZ);
}

void print_fft_max(float * m_fft_output_f32, uint16_t data_size) {
  float32_t max_value = 0;
  uint32_t  max_val_index = 0;
  
  uint32_t offset = 0; //10;
  arm_max_f32(&m_fft_output_f32[offset], data_size - offset, &max_value, &max_val_index);
  max_val_index += offset;
  
  TRACE("Frequency sample: %f Hz\r\n", (float32_t) FFT_SAMPLE_RES_HZ);
  TRACE("Max magnitude value: "TRACE_FLOAT_MARKER", index %d, frequency %f Hz\r\n", TRACE_FLOAT(max_value), max_val_index, (float32_t) max_val_index * FFT_SAMPLE_RES_HZ);
}

void print_fft_result(FFT_RESULTS *p_fftResult) {
  float32_t startFrequency = p_fftResult->start * FFT_SAMPLE_RES_HZ;
  float32_t binFrequency = (float32_t) p_fftResult->stop * FFT_SAMPLE_RES_HZ;
  uint16_t binEnd = p_fftResult->start + (p_fftResult->bins * p_fftResult->stop);
  uint16_t i;

  if(binEnd >= (FFT_OUTPUT_SIZE / 2))
    binEnd = FFT_OUTPUT_SIZE / 2;

  TRACE("FFT result start: %u / %0.2f Hz, end: %u / %0.2f Hz, out bin freq: %0.2f Hz, out bin count: %u, in bin by out bin: %u\r\n", p_fftResult->start, startFrequency,  binEnd, (float32_t) binEnd * FFT_SAMPLE_RES_HZ, binFrequency, p_fftResult->bins, p_fftResult->stop);

  for(i=0;i<p_fftResult->bins;i++) {
    TRACE("%02d -> brut s_bin%04.f_%04.fHz,\t\ts_bin%04d_%04dHz = %d\r\n", i,
                                               (float32_t) (startFrequency + ((float32_t) i * binFrequency)), //todo round
                                               (float32_t) (startFrequency + ((float32_t) i * binFrequency) + binFrequency), //todo round 
                                               (uint16_t) (startFrequency + ((float32_t) i * binFrequency)), //todo round
                                               (uint16_t) (startFrequency + ((float32_t) i * binFrequency) + binFrequency), //todo round
                                               p_fftResult->values[i]);


  }
}

void fft_create_result(FFT_RESULTS *p_fftResult, float *p_fftValue, uint16_t binOutputCount, uint16_t binOffset, uint16_t binOutputSize, uint16_t fftSize) {
  uint32_t binsOutput, binInputCurrent, i;
  float32_t binOutputSum;
  bool end = FALSE;

  if((p_fftResult == NULL) || (p_fftValue == NULL) || !binOutputCount || (binOutputCount > FFT_MAX_BINS) || !fftSize)
    return;

  p_fftResult->start = binOffset; // start input bin
  p_fftResult->stop = binOutputSize; // n input bin for one output bin

  for(binsOutput=0;binsOutput<binOutputCount;binsOutput++) {
    binOutputSum = 0.0;

    for(i=0;i<binOutputSize;i++) {
      binInputCurrent = binOffset + ((binsOutput * binOutputSize) + i);

      if(binInputCurrent >= fftSize) {
        end = TRUE;
        break;
      }  

      binOutputSum += p_fftValue[binInputCurrent];
    }

    p_fftResult->values[binsOutput] = (binOutputSum > UINT32_MAX) ? UINT32_MAX : (uint32_t) binOutputSum; // check diff on 32 bits !!!

    if(end)
      break;
  }

  p_fftResult->bins = binsOutput; //n output bin computed
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_USART2_UART_Init();

  HAL_I2S_DMAStop(&hi2s2);
  HAL_Delay(500);
	
  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *) i2sData, FFT_I2S_BUFFER_SIZE / 2);

  arm_rfft_fast_init_f32(&S, FFT_OUTPUT_SIZE);

  while (TRUE) {
		if(processDSP() == TRUE)
      break;

#if (FFT_LOOP_ENABLED == 1)
    if(fftPerformed >= FTT_COUNT) {
      __disable_irq(); 
      i2sDataCount = 0;
      fftStarted = FALSE; 
      __enable_irq();
    
      fftPerformed = 0;

      HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *) i2sData, FFT_I2S_BUFFER_SIZE / 2);
    }
#endif
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(16);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B; 
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = 4000U; //I2S_AUDIOFREQ_8K; //I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW; 
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

static void MX_GPIO_Init(void)
{
  //GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
