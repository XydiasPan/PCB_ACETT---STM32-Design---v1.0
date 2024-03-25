/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#define ARM_MATH_CM4
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math.h"
#include "usbd_cdc_if.h"
#include "KalmanFilter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DATASIZE 1024
#define FULLSIZE 2048
#define Fs 48000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac2;

TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t ADC1_buf[FULLSIZE]; //ADC1 DMA buffer
uint16_t ADC2_buf[FULLSIZE]; //ADC2 DMA buffer
uint32_t DAC_buf[FULLSIZE]; //DAC DMA buffer
float32_t data[DATASIZE]; //processing buffer for each window
float32_t datasqrd[DATASIZE]; //squared buffer
float32_t SPL; //SPL measurement

static volatile uint16_t* inbufPtr1;
static volatile uint16_t* inbufPtr2;
static volatile uint32_t* outbufPtr;

arm_rfft_fast_instance_f32 FFT_IFFT_struct; //handler of FFT process
float32_t fftBuffer[DATASIZE]; //main FFT buffer - complex values
float32_t fftMAG[DATASIZE/2]; //main magnitudes FFT buffer - real values
float32_t synthBuffer[DATASIZE]; //synthesize buffer to create the output signal
float32_t ifftBuffer[DATASIZE]; //output signal after RIFFT process
uint32_t maxIndex[3]; //holds the indexes of FFT magnitudes at max values
float32_t maxValue[3]; //holds the max values of FFT magnitudes

//initialize frequency bins to search speech formants
int bin100;
int bin530;
int bin1000;
int bin2000;
int bin2700;
int bin4000;
int bin4900;
int bin5000;
int bin6000;
int binCityCut; // LPF for city mode
int binMusicCut; // LPF for music mode

//buttons logic
int btn1Pressed = 0;
int btn2Pressed = 0;
int btn3Pressed = 0;
int cntBtnCycles = 0;

int state; // this is the state of the buttons' logic
float32_t VOLUME; // this is a constant gain applied to the output signal
float32_t CF; //frequency factor to convert bins to frequencies
uint32_t EXP_MALE[DATASIZE/2]; // this buffer holds male frequency transformations
uint32_t EXP_FEMALE[DATASIZE/2]; // this buffer holds male frequency transformations
uint32_t FRICATIVE[DATASIZE/2]; // this buffer holds fricative transformations
uint32_t CITY_Transform[DATASIZE/2]; // this buffer holds city transformations
uint32_t NATURE_Transform[DATASIZE/2]; // this buffer holds nature transformations
uint32_t MUSIC_Transform[DATASIZE/2]; // this buffer holds music transformations
float32_t Cosines[DATASIZE/2]; // this buffer hold cosine values so they are pre-evaluated for the phase transformation
float32_t Sines[DATASIZE/2]; // this buffer hold sine values so they are pre-evaluated for the phase transformation

// KALMAN filter variables
float Ptemp;
float prev_input;

void proccessSpeechMode(){

    int newbin;
	float32_t fft100_1000[bin1000 - bin100 + 1]; //FFT buffer from 100Hz to 1kHz
	float32_t fft530_2700[bin2700 - bin530 + 1]; //FFT buffer from 530Hz to 2.7kHz
	float32_t fft2000_6000[bin6000 - bin2000 + 1]; //FFT buffer from 2kHz to 6kHz

	float32_t BPF1[bin6000]; //bandpass filter for the first formant
	float32_t BPF2[bin6000]; //bandpass filter for the second formant
	float32_t BPF3[bin6000]; //bandpass filter for the third formant
	float32_t TF[2*bin6000]; //combination of all the filters in a single buffer as transfer function

	//clean synthesize buffer
	for(int n=0;n<DATASIZE;n++){
		synthBuffer[n] = 0;
	}

	//setup the formants search regions
	for(int n=bin100;n<=bin1000;n++){
		fft100_1000[n-bin100] = fftMAG[n];
	}
	for(int n=bin530;n<=bin2700;n++){
		 fft530_2700[n-bin530] = fftMAG[n];
	}
	for(int n=bin2000;n<=bin6000;n++){
		fft2000_6000[n-bin2000] = fftMAG[n];
	}

	//find the speech formants
	arm_max_f32(fft100_1000, bin1000 - bin100 + 1, &maxValue[0], &maxIndex[0]);
	maxIndex[0] += bin100; //shift the bin properly to represent the exact one
	//add zeros in the next search region in case the first frequency might be found again instead of the second formant. Zeros are put 5 bins before/after the first formant
	if(maxIndex[0]-bin530 >=5){
		for(int n=0;n<=5;n++){
			fft530_2700[maxIndex[0]-bin530+n] = 0;
			fft530_2700[maxIndex[0]-bin530-n] = 0;
		}
	}
	else if(maxIndex[0]-bin530 <5 && maxIndex[0]-bin530 >= 0){
		for(int n=0;n<5;n++){
			fft530_2700[maxIndex[0]-bin530+n] = 0;
		}
		for(int n=0;n<maxIndex[0]-bin530;n++){
			fft530_2700[n] = 0;
		}
	}
	arm_max_f32(fft530_2700, bin2700 - bin530 + 1, &maxValue[1], &maxIndex[1]);
	maxIndex[1] += bin530; //shift the bin properly to represent the exact one
	//add zeros in the next search region in case the second frequency might be found again instead of the third formant. Zeros are put 5 bins before/after the second formant
	if(maxIndex[1]-bin2000 >=5){
		for(int n=0;n<=5;n++){
			fft2000_6000[maxIndex[1]-bin2000+n] = 0;
			fft2000_6000[maxIndex[1]-bin2000-n] = 0;
		}
	}
	else if(maxIndex[1]-bin2000 <5 && maxIndex[1]-bin2000 >=0){
		for(int n=0;n<5;n++){
			fft2000_6000[maxIndex[1]-bin2000+n] = 0;
		}
		for(int n=0;n<maxIndex[1]-bin2000;n++){
			fft2000_6000[n] = 1;
		}
	}
	arm_max_f32(fft2000_6000, bin6000 - bin2000 + 1, &maxValue[2], &maxIndex[2]);
	maxIndex[2] += bin2000; //shift the bin properly to represent the exact one
	if(maxIndex[2] >= bin4000){
		maxIndex[2] = bin5000; //might be a consonant
	}

	// initializing the bandpass filters and the transfer function depending on the formants that were found
	for(int n=0;n<bin6000;n++){

		float32_t dif1 = fabs(n-maxIndex[0]);
		float32_t dif2 = fabs(n-maxIndex[1]);
		float32_t dif3 = fabs(n-maxIndex[2]);

		if(dif1<=15){
			BPF1[n] = 1;
		}
		else{
			BPF1[n] = 0.1;
		}

		if(dif2<=15){
			BPF2[n] = 1;
		}
		else{
			BPF2[n] = 0.1;
		}

		if(dif3<=2){
			BPF3[n] = 10;
		}
		else{
			BPF3[n] = 0.1;
		}

		TF[2*n] = (BPF1[n] + BPF2[n] + BPF3[n])*Cosines[n];
		TF[2*n + 1] = (BPF1[n] + BPF2[n] + BPF3[n])*Sines[n];
	}

	//Frequency compression is being applied in cases (fricative,non-fricative and male/female voice). The transfer function is being applied in the meantime.
	if(maxIndex[2] > bin2700 && maxIndex[2] < bin4900){
	  		for(int n=0;n<bin6000;n++){
	  	       newbin = EXP_FEMALE[n];   //SPEECH = FEMALE
	  	  	   synthBuffer[2*newbin] += (fftBuffer[2*n]*TF[2*n] - fftBuffer[2*n + 1]*TF[2*n+1]);
	  	  	   synthBuffer[2*newbin + 1] += (fftBuffer[2*n]*TF[2*n+1] + fftBuffer[2*n + 1]*TF[2*n]);
	  		}
	}
	else if(maxIndex[2] > bin4900){
  		for(int n=0;n<bin6000;n++){
  	       newbin = FRICATIVE[n];   //FRICATIVE
  	  	   synthBuffer[2*newbin] += (fftBuffer[2*n]*TF[2*n] - fftBuffer[2*n + 1]*TF[2*n+1]);
  	  	   synthBuffer[2*newbin + 1] += (fftBuffer[2*n]*TF[2*n+1] + fftBuffer[2*n + 1]*TF[2*n]);
  		}
	}
	else{
  		for(int n=0;n<bin6000;n++){
  	       newbin = EXP_MALE[n];   //SPEECH = MALE
  	  	   synthBuffer[2*newbin] += (fftBuffer[2*n]*TF[2*n] - fftBuffer[2*n + 1]*TF[2*n+1]);
  	  	   synthBuffer[2*newbin + 1] += (fftBuffer[2*n]*TF[2*n+1] + fftBuffer[2*n + 1]*TF[2*n]);
  		}
	}


}

void proccessCityMode(){

    int newbin;

	//clean synthesize buffer
	for(int n=0;n<DATASIZE;n++){
		synthBuffer[n] = 0;
	}

	//frequency compression
	for(int n=0;n<DATASIZE/2 && SPL >= 58.7 ;n++){
	  	 newbin = CITY_Transform[n];
	  	 if(n<=bin2000){
		  	 synthBuffer[2*newbin] += 0.4*fftBuffer[2*n];
		  	 synthBuffer[2*newbin + 1] += 0.4*fftBuffer[2*n + 1];
	  	 }
	  	 else{
		  	 synthBuffer[2*newbin] += 2*fftBuffer[2*n];
		  	 synthBuffer[2*newbin + 1] += 2*fftBuffer[2*n + 1];
	  	 }
	}

}

void proccessNatureMode(){

    int newbin;

	//clean synthesize buffer
	for(int n=0;n<DATASIZE;n++){
		synthBuffer[n] = 0;
	}

	//frequency compression
	for(int n=0;n<DATASIZE/2 && SPL <= 60;n++){
	  	 newbin = NATURE_Transform[n];
	  	 if(n<=bin2000){
		  	 synthBuffer[2*newbin] += 0.5*fftBuffer[2*n];
		  	 synthBuffer[2*newbin + 1] += 0.5*fftBuffer[2*n + 1];
	  	 }
	  	 else{
		  	 synthBuffer[2*newbin] += 3*fftBuffer[2*n];
		  	 synthBuffer[2*newbin + 1] += 3*fftBuffer[2*n + 1];
	  	 }
	}

}

void proccessMusicMode(){

    int newbin;

	//clean synthesize buffer
	for(int n=0;n<DATASIZE;n++){
		synthBuffer[n] = 0;
	}

	//frequency compression
	for(int n=0;n<binMusicCut;n++){
	  	 newbin = MUSIC_Transform[n];
		 synthBuffer[2*newbin] += fftBuffer[2*n];
		 synthBuffer[2*newbin + 1] += fftBuffer[2*n + 1];
	}

}

void process_DSP()
{

    for (int n=0; n<DATASIZE; n++)
    {
    	data[n] = (float32_t) ((inbufPtr1[n])*3.3/4096); //capturing the data from DMA ADC
    	datasqrd[n] = data[n]*data[n];
    	//applying a time domain KALMAN filtering
    	/*
		float *Kalman_params = KalmanFilter(Ptemp,data[n],prev_input);
		filtered_data[n] = (float32_t) (Kalman_params[0]);
		Ptemp = Kalman_params[1];
		prev_input = Kalman_params[2];
		*/
    }

    arm_max_f32(datasqrd,DATASIZE,&maxValue[0],&maxIndex[0]); // find max amplitude in data squared to normalize the measurements
    for (int n=0; n<DATASIZE; n++)
    {
    	SPL += (float32_t) (10*log10f(maxValue[0]/datasqrd[n])/DATASIZE);
    }
    SPL += 56;


    arm_rfft_fast_f32(&FFT_IFFT_struct, data, fftBuffer,0); //RFFT

    //zero out DC frequency
    fftBuffer[0] = 0;
    fftBuffer[1] = 0;

    //get magnitude values and cut noise from spectrum. There is also added a transient region for the values that are close to noise cutoff level but also could be considered as wanted signal
    for(int n=0;n<DATASIZE/2;n++){
	  	 fftMAG[n] = sqrtf(fftBuffer[2*n]*fftBuffer[2*n] +  fftBuffer[2*n+1]*fftBuffer[2*n+1]);
	  	 if(fftMAG[n] > 4  && fftMAG[n] <= 5){
	  		fftBuffer[2*n] = (fftMAG[n] - 4)*fftBuffer[2*n];
	  		fftBuffer[2*n + 1] = (fftMAG[n] - 4)*fftBuffer[2*n + 1];
	  	 }
	  	 else if(fftMAG[n] > 5){
	  		fftBuffer[2*n] = fftBuffer[2*n];
	  		fftBuffer[2*n + 1] = fftBuffer[2*n + 1];
	  	 }
	  	 else{
	  		fftBuffer[2*n] = 0;
	  		fftBuffer[2*n + 1] = 0;
	  	 }

    }

    //Apply frequency compression depending on the mode of application
    switch(state){
    case 1:{
    	proccessSpeechMode(); //Speech mode
    	break;
    }
    case 2:{
    	proccessCityMode(); //City mode
    	break;
    }
    case 3:{
    	proccessNatureMode(); //Nature mode
    	break;
    }
    case 4:{
    	proccessMusicMode(); //Music mode
    	break;
    }
    default : break;
    }


    arm_rfft_fast_f32(&FFT_IFFT_struct, synthBuffer, ifftBuffer,1); //RIFFT

    //Write back the values to the DMA DAC buffer with a DC offset of 1.65V and a programmable constant gain to prevent clipping
    for (int n=0; n<DATASIZE; n++)
    {
        outbufPtr[n] = (uint32_t) ((VOLUME*ifftBuffer[n] + 1.65)*(4096/3.3));
    }

}

void UserInterface(){

	//Check which button was pressed to start counting cycles
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6) == GPIO_PIN_RESET && btn1Pressed==0){
		btn1Pressed = 1;
		cntBtnCycles = 0;
	}
	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2) == GPIO_PIN_RESET && btn2Pressed==0){
		btn2Pressed = 1;
		cntBtnCycles = 0;
	}
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6) == GPIO_PIN_RESET && btn3Pressed==0){
		btn3Pressed = 1;
		cntBtnCycles = 0;
	}

	//Check which button was released depending on the counting cycles and then perform the appropriate actions (Mode switching, Volume switching, LED turning on/off). Finally reset the cycles and the button's logic state as "not pressed".
	if(btn1Pressed==1 && cntBtnCycles >= 30 && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6) == GPIO_PIN_SET){
		switch(state){
		case 1:{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			state = 2;
			VOLUME = 1;
			break;
		}
		case 2:{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			state = 3;
			VOLUME = 1;
			break;
		}
		case 3:{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			state = 4;
			VOLUME = 1;
			break;
		}
		case 4:{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			state = 1;
			VOLUME = 1;
			break;
		}
		default: break;
		}
		btn1Pressed = 0;
	}
	if(btn2Pressed==1 && cntBtnCycles >= 30 && HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2) == GPIO_PIN_SET && VOLUME<3){
		VOLUME +=0.1;
		btn2Pressed = 0;
	}
	if(btn3Pressed==1 && cntBtnCycles >= 30 && HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6) == GPIO_PIN_SET && VOLUME>0){
		VOLUME -=0.1;
		btn3Pressed = 0;
	}

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_USB_DEVICE_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_buf, FULLSIZE); //start ADC1 DMA
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2_buf, FULLSIZE); //start ADC2 DMA
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)DAC_buf, FULLSIZE, DAC_ALIGN_12B_R); //start DAC DMA
  HAL_TIM_Base_Start_IT(&htim8); //start ADC/DAC Timer
  arm_rfft_fast_init_f32(&FFT_IFFT_struct, DATASIZE); //initialize FFT process

  //initialize variables
  CF = Fs/DATASIZE;
  bin100 = (int)(100*DATASIZE/Fs);
  bin530 = (int)(530*DATASIZE/Fs);
  bin1000 = (int)(1000*DATASIZE/Fs);
  bin2000 = (int)(2000*DATASIZE/Fs);
  bin2700 = (int)(2700*DATASIZE/Fs);
  bin4000 = (int)(4000*DATASIZE/Fs);
  bin4900 = (int)(4900*DATASIZE/Fs);
  bin5000 = (int)(5000*DATASIZE/Fs);
  bin6000 = (int)(6000*DATASIZE/Fs);
  binCityCut = (int) (4000*DATASIZE/Fs);
  binMusicCut = (int) (2500*DATASIZE/Fs);

  // set speech mode by default
  state = 1;

  // initialize VOLUME
  VOLUME = 1;

  // initialize KALMAN variables
  Ptemp = 0.5;
  prev_input = 0.0;

  //initialize filters
  for(int n=0;n<DATASIZE/2;n++){
	  EXP_MALE[n] = (uint32_t) (floor((-400.0 + 800.0/(1.0 + expf(-0.003*n*CF)))/CF + 1.5));
	  EXP_FEMALE[n] = (uint32_t) (floor((-400.0 + 800.0/(1.0 + expf(-0.002*n*CF)))/CF + 1.5));
	  FRICATIVE[n] = (uint32_t) (floor(n*0.109  + 1.5));
	  CITY_Transform[n] = (uint32_t) floor(n*0.109  + 1.5);
	  NATURE_Transform[n] = (uint32_t) floor(n*0.109  + 1.5);
	  MUSIC_Transform[n] = (uint32_t) floor(n*0.109  + 1.5);
	  Cosines[n] = (float32_t) cosf(2*M_PI*n/Fs);
	  Sines[n] = (float32_t) sinf(2*M_PI*n/Fs);
  }
  //Run on speech mode on startup, so write the appropriate value on its LED
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 for(int k=0;k<DATASIZE;k++){
		 char TX[128];
		 sprintf(TX,"%f\r\n", SPL);
		 CDC_Transmit_FS((uint8_t *)TX,strlen(TX));
	 }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T8_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 3499;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AR1_Pin|GAIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GAIN2_Pin|AR2_Pin|MODE4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MODE3_Pin|MODE2_Pin|MODE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DAC_EN_GPIO_Port, DAC_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : AR1_Pin GAIN1_Pin */
  GPIO_InitStruct.Pin = AR1_Pin|GAIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GAIN2_Pin AR2_Pin */
  GPIO_InitStruct.Pin = GAIN2_Pin|AR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MODE4_Pin */
  GPIO_InitStruct.Pin = MODE4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MODE4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MODE3_Pin MODE2_Pin MODE1_Pin */
  GPIO_InitStruct.Pin = MODE3_Pin|MODE2_Pin|MODE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DAC_EN_Pin */
  GPIO_InitStruct.Pin = DAC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DAC_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN2_Pin */
  GPIO_InitStruct.Pin = BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN3_Pin */
  GPIO_InitStruct.Pin = BTN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){

	//reset SPL
    SPL = 0;

	//pointers to the data addresses of the DMA streams (for the first half stream, we point at the start)
	inbufPtr1 = &ADC1_buf[0];
	inbufPtr2 = &ADC2_buf[0];
	outbufPtr = &DAC_buf[0];

	//check case where one of the buttons was pressed and continue counting cycles of 1/SAMPLERATE seconds. The counting stops when the button is released within its specified time.
	if(btn1Pressed || btn2Pressed || btn3Pressed){
		cntBtnCycles++;
	}

	//Always run the main application as it is described in the following two functions : 1st is running the User Interface algorithm and 2nd is running the main DSP processing algorithm
	UserInterface();
	process_DSP();
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	//reset SPL
    SPL = 0;

	//pointers to the data addresses of the DMA streams (for the second half stream, we point at the DATASIZE index)
	inbufPtr1 = &ADC1_buf[DATASIZE];
	inbufPtr2 = &ADC2_buf[DATASIZE];
	outbufPtr = &DAC_buf[DATASIZE];

	//check case where one of the buttons was pressed and continue counting cycles of 1/SAMPLERATE seconds. The counting stops when the button is released within its specified time.
	if(btn1Pressed || btn2Pressed || btn3Pressed){
		cntBtnCycles++;
	}

	//Always run the main application as it is described in the following two functions : 1st is running the User Interface algorithm and 2nd is running the main DSP processing algorithm
	UserInterface();
	process_DSP();
}
/* USER CODE END 4 */

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
