/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "arm_const_structs.h"
#include "arm_math.h"

/** 
  * @{
  */

/** @addtogroup ADC_RegSimul_DualMode
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint64_t)0x4001244C)

ErrorStatus HSEStartUpStatus;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
int Buff[1200];
int C0[256];
int C1[256];
//int C2[128];
//int C3[128];
uint16_t window[4]={0};
//signed long int C2[128]; // diaxwrismos kanaliwn
//signed long int C3[32];
int k=128;
//FFT timwn
//int FFTC2[128];
///int FFTC3[128];


	
	
 //float32_t FFTC3[16];

long int add[4] ;
long int dcc[4] ;
int mag[128]={0};
//long int arx[4];

uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void CH(int , int );
void AV(void);

void END(void);
void FFT(void);
void ENERG(void);
volatile int ic,st ; // we have  interrupt , i take the values from it.c
//epeksergasia mia fora ka8e fora
int c=0;//counter gia ta windows twn 10 para8urwn

int abs1(int bu[256] ,int pos);
int i ,j,l1,l2,l3,l4;
int pos;
int sta=0;////////Global state=0 i need to know if it is the first time that start the procces

/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
	int max1=0;
	///////////////////DSP//////////////
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

  /* System clocks configuration ---------------------------------------------*/
  RCC_Configuration();

  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration();

  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Buff;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1200;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  /* Enable DMA1 Channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 2;
  ADC_Init(ADC1, &ADC_InitStructure);
  /* ADC1 regular channels configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
 // ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5);
 /*The frequency would be about 7200000/(55.5+12.5)= 111.029Hz*/
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);



  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_9b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
  /* Enable ADC1 reset calibration register */   
 ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);

  
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

 //* /* Test on DMA1 channel1 transfer complete flag */
 // while(!DMA_GetFlagStatus(DMA1_FLAG_TC1));
  /* Clear DMA1 channel1 transfer complete flag */
 // DMA_ClearFlag(DMA1_FLAG_TC1); *//


while (1)
  {

   if (ic==1)
	 {
	  if (st==1)//when the buffer is half full an interrupt called to process half buffer


		{
	
		CH(0,1);//separate the 2 channels 
		if (sta==0)
			{
				AV(); //calculate the dc compliment (only one time, we dont need to calculate it every time)
				sta=1;
			}
		
	
	  END();//remove dc compliment
		FFT(); abs1//start fft
		
		i=1;
   
		}
	  if (st==2){//after the buffer is full an interrupt start
	  DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);
	  CH(600,601);///Start the procces of second half oof array 
		
 	  END();
		FFT();
		
		i=2;
	}
	ic = 0;
  }
}
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
 /* Enable peripheral clocks ------------------------------------------------*/
 /* Enable DMA1 clock */
 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
 /* Enable ADC1 and GPIOC clock */
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA,ENABLE);
 /* RCC system reset(for debug purpose) */
 //RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
 
}



void END(void)
{
	//power[0]=0;
	//power[1]=0;
	//power[2]=0;
	//power[3]=0;
		for (i=0;i<128;i++)
					{ 
						
						C0[i]= C0[i]-dcc[0];
						//power[0]=power[0]+C0[i]^2;//
						C1[i]= C1[i]-dcc[1]; //remove dc coompliment
						//power[1]=power[1]+C1[i]^2;
						//C2[i]= C2[i]-dcc[2];
						//power[2]=power[2]+C2[i]^2;
						//C3[i]= C3[i]-dcc[3];
						//power[3]=power[3]+C3[i]^2;
						
					}
}

	void AV(void){
	     for (i=0;i<128;i++)
					{
						add[0]=add[0] + C0[i]; //caclulate the total sum of signal
						add[1]=add[1] + C1[i];
						//add[2]=add[2] + C2[i];
						//add[3]=add[3] + C3[i];
						
						
						
					}
					dcc[0]=add[0]/128;
					dcc[1]=add[1]/128; //calculate dc component
					//dcc[2]=add[2]/128;
					//dcc[3]=add[3]/128;
	
	
	}
	
/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void CH(int l1,int l2 ) 
	{

		for (i=0;i<128;i++)
					{ 
						
						C0[i]= Buff[l1];
						l1+=2;
						C1[i]=Buff[l2]; //Separate the chanels
						l2+=2;
						//C2[i]= Buff[l3];
						//l3+=4;
						//C3[i]= Buff[l4];
						//l4+=4;
						
					}
	
}	
	
void FFT(void)
{	

	  //	arm_rfft_instance_q31 	S;
   // arm_rfft_init_q31(&S,128,0,1);
	//   arm_rfft_q31(&S,C0,FFTC0);

		//FFT CHANEL 11

	 arm_cfft_q31(&arm_cfft_sR_q31_len128, C0,ifftFlag ,doBitReverse); 
   //arm_cmplx_mag_q31(C0, FFTC0, 128);
	//arm_max_f32(FFTC0, fftSize, &maxValue0, &testIndex);
   //pos= abs1(C0,pos);
  // ENERG();	
		//FFT CHANEL 12
	arm_cfft_q31(&arm_cfft_sR_q31_len128,C1,ifftFlag ,doBitReverse); 
	//arm_cmplx_mag_q31(C1, FFTC1, 128);
	pos= abs1(C1,pos);

	ENERG();
	
		//FFT CHANEL 13
	 //arm_cfft_q31(&arm_cfft_sR_q31_len128,C2,ifftFlag ,doBitReverse); 
	//arm_cmplx_mag_q31(C2, FFTC2, 128);
	
		//FFT CHANEL 14
	 //arm_cfft_q31(&arm_cfft_sR_q31_len128,C3,ifftFlag ,doBitReverse); 
	//arm_cmplx_mag_q31(C3, FFTC3, 128);
	//arm_max_f32(FFTC3, fftSize, &maxValue3, &testIndex); 
	

}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
   
   
/* Configure ADC1 channel1 */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
/* Configure ADC1 channel2 */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
/* Configure ADC1 channel2 */
   //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
   //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   //GPIO_Init(GPIOC, &GPIO_InitStructure);
	
/* Configure ADC1 channel3 */

   //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
   //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
   //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   //GPIO_Init(GPIOC, &GPIO_InitStructure);
}
 int abs1(int bu[128] ,int pos)///calculate the abs
{
	int k=0;
	int max=0;
	for(i=0;i<=127;i=i+2)
{
	mag[k]=sqrt((bu[i]*bu[i])+(bu[i+1]*bu[i+1]));
	
	if (max<=mag[k]&&(k<=63)&&k>0)
	{
		max=mag[k];
		pos=k;
	}
		k++;

}

return pos;
}





// Implementation of itoa() 
	void itoa1(int num, char str[5], int base) ;

// Implementation of itoa() 
void itoa1(int num, char str[5], int base) 
{ 
    int i = 0; 
  
    /* Handle 0 explicitely, otherwise empty string is printed for 0 */
    if (num == 0) 
    { 
        str[i++] = '0'; 
        str[i] = '\0'; 
        
    } 
  
    // In standard itoa(), negative numbers are handled only with  
    // base 10. Otherwise numbers are considered unsigned. 

  
    // Process individual digits 
    while (num != 0) 
    { 
        int rem = num % base; 
        str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0'; 
        num = num/base; 
    } 
  
    // If number is negative, append '-' 
    
  
    str[i] = '\0'; // Append string terminator 
  
		
		

		




   /// reverse(str, i); 
  
} 
void ENERG(void)
{
	window[1]=0;
	window[1]=0;
	window[2]=0;
  window[3]=0;


	int state=1;
	int dcenergy=mag[0]*mag[0];
  uint8_t pas;
	// i use the else state   take the 63 values of fft array ,  the i take 21 +21 +21=63 of valuses.
	// i calculate the energy of signals  of every 21 part and i stored to window[i=1,2,3]=sum(mag^2)
 	if (state ==0) //state 0 USART Returns the values only
	{
		 int k=0;
		char kat[5];
		USART_SendData(USART1,0);
		USART_SendData(USART1,'=');
			window[0]=dcenergy;
		   
		itoa1(window[0],kat,10);
		int l=0;
	for (l=0;l<=4;l++){
							if((4-l==4)||(4-l==3)){
								if ((kat[4]=='\0') && (kat[3]=='\0'))
									{
									l=2;
								}
									else if (kat[3]=='\0')
                  {
										l=1;
									}
									else if (kat[4]=='\0')
                   {
										l=1;
									 }
										 
							}
						USART_SendData(USART1,kat[4-l]);
						}
						j=j+1;
            kat[0]='\0';
						kat[1]='\0';
						kat[2]='\0';
						kat[3]='\0';
						kat[4]='\0';
      
      l=0;
		 int j=1;///communication with USART 
			for (i=1;i<=63;i++)
			{
				

				window[j]=mag[i]*mag[i]+window[j];
				if (k==20)
          { 
						int f=0;
						pas=window[j];
						k=0;
					  USART_SendData(USART1,j);
						USART_SendData(USART1,'=');
		        itoa1(window[j],kat,10);
						for (l=0;l<=4;l++){
							if((4-l==4)||(4-l==3)){
								if ((kat[4]=='\0') && (kat[3]=='\0'))
									{
									l=2;
								}
									else if (kat[3]=='\0')
                  {
										l=1;
									}
									else if (kat[4]=='\0')
                   {
										l=1;
									 }
										 
							}
						USART_SendData(USART1,kat[4-l]);
						}
						j=j+1;
            kat[0]='\0';
						kat[1]='\0';
						kat[2]='\0';
						
						
				
						kat[3]='\0';
						kat[4]='\0';
           
					}
				k=k+1;
			}
		
	}
	else 
	{
		
		int window[6]={0};
		 int k=0;
			window[0]=dcenergy;///energy of dc complement must be zero because i remove it before

		 int j=1;
			for (i=1;i<=63;i++)
			{
				window[j]=mag[i]*mag[i]+window[j];/// calculate energy of fft signal in 3 same size Arrays
				if (k==21)
          { 
						k=0;
						j=j+1;
						i=i-10;
					}
				k=k+1;
			}
	}

}
	
	
	#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 




/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/