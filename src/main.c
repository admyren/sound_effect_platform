/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

#include <stdio.h>
#include "drivers.h"
#include "dsp.h"
/* Includes for freeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define UART_BUFF_SIZE 50

xSemaphoreHandle ADCDMAInterruptSignal1;
xSemaphoreHandle ADCDMAInterruptSignal2;

volatile uint8_t UART_buff[UART_BUFF_SIZE];

// Buffers for ADC's and DAC's
uint16_t ADC1InBuff[BUFFER_SIZE];
uint16_t ADC2InBuff[BUFFER_SIZE];
uint16_t DAC1OutBuff[BUFFER_SIZE];
uint16_t DAC2OutBuff[BUFFER_SIZE];

// Destination buffers for the different blocks
float ADC1Buff[BUFFER_SIZE/2];
float ADC2Buff[BUFFER_SIZE/2];

// Intermediate buffers
float inBuff1[BUFFER_SIZE/2];
float inBuff2[BUFFER_SIZE/2];

float resultBuff1[BUFFER_SIZE/2];
float resultBuff2[BUFFER_SIZE/2];

volatile uint16_t ADCValue = 4000;
volatile uint16_t prev_ADCValue = 0;

Filter_TypeDef filters[2];


void SystemClock_Config(void);



#define FILTER_ON 1
static void DSPTask(void* params)
{
	while(1)
	{
		//ADCValue = (uint16_t)ADC3->DR;
		filters[0].cutOff[0] = 100+4*ADCValue;
		filters[1].cutOff[0] = 100+4*ADCValue;
		if(prev_ADCValue != ADCValue)
		{
			design_FIR(&filters[0]);
			design_FIR(&filters[1]);
		}
		/*** WAIT FOR FIRST HALF OF BUFFER ***/
		if(xSemaphoreTake(ADCDMAInterruptSignal1, 0xffff) == pdTRUE)
		{
			for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
			{
				ADC1Buff[i] = (float)(ADC1InBuff[i]-2048);
				ADC2Buff[i] = (float)(ADC2InBuff[i]-2048);
			}
#if FILTER_ON
			FIR_Filter(filters[0].FIR_instance, BUFFER_SIZE/2);
			FIR_Filter(filters[1].FIR_instance, BUFFER_SIZE/2);
#else

			for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
			{
				resultBuff1[i] = (((float)ADCValue)/4095)*ADC1Buff[i];
				resultBuff2[i] = (((float)ADCValue)/4095)*ADC2Buff[i];
			}

#endif
			for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
			{
				DAC1OutBuff[i] = (uint16_t)(resultBuff1[i] + 2048);
				DAC2OutBuff[i] = (uint16_t)(resultBuff2[i] + 2048);
			}
		}

		/*** WAIT FOR SECOND HALF OF BUFFER ***/
		if(xSemaphoreTake(ADCDMAInterruptSignal2, 0xffff) == pdTRUE)
		{
			for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
			{
				ADC1Buff[i] = (float)(ADC1InBuff[i+BUFFER_SIZE/2]-2048);
				ADC2Buff[i] = (float)(ADC2InBuff[i+BUFFER_SIZE/2]-2048);
			}
#if FILTER_ON
			FIR_Filter(filters[0].FIR_instance, BUFFER_SIZE/2);
			FIR_Filter(filters[1].FIR_instance, BUFFER_SIZE/2);
#else
			for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
			{
				resultBuff1[i] = (((float)ADCValue)/4095)*ADC1Buff[i];
				resultBuff2[i] = (((float)ADCValue)/4095)*ADC2Buff[i];
			}
#endif
			for(uint16_t i=0; i<BUFFER_SIZE/2; i++)
			{
				DAC1OutBuff[i+BUFFER_SIZE/2] = (uint16_t)(resultBuff1[i]+2048);
				DAC2OutBuff[i+BUFFER_SIZE/2] = (uint16_t)(resultBuff2[i]+2048);
			}
		}
	}
}



int main(void)
{
	HAL_Init();
	SystemClock_Config();
	UART_Init(UART4, 230400, (uint8_t*)UART_buff, UART_BUFF_SIZE);
	initControlInput();
	/*** FOR DEBUG ***/
	// PE8, PE9, PE10
	RCC->AHB1ENR |= (1 << 4);
	GPIOE->MODER |= (1 << 16) | (1 << 18) | (1 << 20);

	init_Filter(&filters[0], BUFFER_SIZE, ADC1Buff, resultBuff1);
	init_Filter(&filters[1], BUFFER_SIZE, ADC2Buff, resultBuff2);

	ADCDMAInterruptSignal1 = xSemaphoreCreateBinary();
	ADCDMAInterruptSignal2 = xSemaphoreCreateBinary();

	initSignalPath(ADC1InBuff, ADC2InBuff,
			    DAC1OutBuff, DAC2OutBuff, BUFFER_SIZE);
	initControlInput();

	xTaskCreate(DSPTask, "DSPTask", 2000, NULL, 5, NULL);
	vTaskStartScheduler();

	return 0;
}

// Synchronization for ADC1 buffer
void DMA2_Stream0_IRQHandler(void)
{
	portBASE_TYPE higherPrio = pdFALSE;
	// Half transfer complete interrupt?
	if(DMA2->LISR & (1 << 4))
	{
		// Clear interrupt flag
		DMA2->LIFCR |= 1 << 4;
		xSemaphoreGiveFromISR(ADCDMAInterruptSignal1, &higherPrio);
		portEND_SWITCHING_ISR(higherPrio);
	}
	// Transfer complete interrupt?
	else if(DMA2->LISR & (1 << 5))
	{
		// Clear interrupt flag
		DMA2->LIFCR |= 1 << 5;
		xSemaphoreGiveFromISR(ADCDMAInterruptSignal2, &higherPrio);
		portEND_SWITCHING_ISR(higherPrio);
	}
}

// Interrupt handler for control input on ADC3
void ADC_IRQHandler(void)
{
	// Check if EOC bit is set
	if(ADC3->SR & (1 << 1))
	{
		//char str[6];
		prev_ADCValue = ADCValue;
		ADCValue = (uint16_t)ADC3->DR;
		//sprintf(str, "Control value: %d\n", ADCValue);
		//UART_Transmit(UART4, (uint8_t*)str, 0);
		// Clear EOC bit
		ADC3->SR &= ~(1 << 1);
	}
}





/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
#if 0
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16; // 1MHz clock input to PLL
  RCC_OscInitStruct.PLL.PLLN = 360; // PLL multiplier, output of PLL = 1*360 = 360MHz
  RCC_OscInitStruct.PLL.PLLP = 2; // System clock = output of PLL / 2 = 180MHz
  RCC_OscInitStruct.PLL.PLLQ = 7;
#else
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  //RCC_OscInitStruct.HSECalibrationValue = 8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8; // 1MHz clock input to PLL
  RCC_OscInitStruct.PLL.PLLN = 360; // PLL multiplier, output of PLL = 1*360 = 360MHz
  RCC_OscInitStruct.PLL.PLLP = 2; // System clock = output of PLL / 2 = 180MHz
  RCC_OscInitStruct.PLL.PLLQ = 7;
#endif
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK |
		                        RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

  SystemCoreClockUpdate();


  /**Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /**Configure the Systick interrupt time
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}
