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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "usbd_cdc_if.h"
#include "aes.h"
#include "key.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_START_ADDRESS 0x8008000
#define FLASH_FLAG_ADDRESS 0x0807fffc
#define FLASH_FLAG 0xafb3
#define __RUN_TIME_REFRESH LastTickBootloderRunTime = HAL_GetTick()
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct AES_ctx ctx;

uint32_t AppRegister;
uint16_t FlashFlag;

volatile uint8_t MsgReady;
uint8_t BootloaderStep;

volatile uint8_t FlashDonwload;

enum Step{
	ECHO = 0x00,
	FULL_SIZE = 0x01,
	FULL_CRC = 0x02,
	EREASE = 0x03,
	PACKET_SIZE = 0x04,
	PACKET_ADDRESS = 0x05,
	PACKET_CRC = 0x06,
	RECEIVE_PACKET = 0x07,
	FULL_CRC_CALCULATE = 0x08,
	ENDED = 0x09
}BootStep;

enum Response_type{
	OK = 0x00,
	ERR = 0x01,
	CRC_ERR = 0x02,
	READY = 0x03
}Response_type;

const uint8_t Response[] = {0x00, 0x01, 0x02, 0x03};

volatile union Bufer{
	uint8_t FlashBuffer_8[1024];
	uint32_t FlashBuffer_32[1024/4];
}Buffer_Packet;

volatile union Command{
	uint8_t Buff[5];
	struct{
		uint32_t Argument;
		uint8_t Command_number;
	};
}Command;

volatile struct FlashInfoVariable{
	uint32_t CRC_packet, CRC_flash;
	uint32_t CRC_packet_Receive, CRC_flash_Receive;
	uint32_t PacketSize;
	uint32_t PacketAddress;
	uint32_t FullSize;
}FlashInfoVariable;

volatile uint32_t LastTickDownloadTimeout;

uint32_t LastTickBootloderRunTime;
uint8_t FlashIsErese = 0;

volatile uint32_t DownloadingBytes = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void jump_to_application(uint32_t const app_address);
void deinit_peripherals(void);

void updateIV(uint8_t* iv, const uint8_t* block);

void respondOK(void)
{
	CDC_Transmit_FS(&Response[OK], 1);
}
void respondERR(void)
{
	CDC_Transmit_FS(&Response[ERR], 1);
}
void respondCRC_err(void)
{
	CDC_Transmit_FS(&Response[CRC_ERR], 1);
}
void respondPacketReady(void)
{
	CDC_Transmit_FS(&Response[READY], 1);
}

uint8_t EreaseApp(uint32_t FirstPage, uint32_t NbPages)
{
	FLASH_EraseInitTypeDef eraseConfig;
	eraseConfig.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseConfig.PageAddress = FirstPage;
	eraseConfig.NbPages = NbPages;

	uint32_t PageError;

	if(HAL_FLASHEx_Erase(&eraseConfig, &PageError) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;

}

void FlashWriteProcedure(void);
void DownloadingPacket(uint8_t *Data, uint32_t Length);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	__asm__ volatile("mov %0, r10" : "=r" (AppRegister));
	__enable_irq();
	memcpy(&FlashFlag, FLASH_FLAG_ADDRESS, 2);
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
  MX_CRC_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  if(AppRegister != 0x1234 && HAL_GPIO_ReadPin(BOOT_PIN_GPIO_Port, BOOT_PIN_Pin) && FlashFlag == FLASH_FLAG) jump_to_application(APP_START_ADDRESS);
  LastTickBootloderRunTime = HAL_GetTick();

  AES_init_ctx_iv(&ctx, key, iv);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(FlashDonwload == 1 && HAL_GetTick() - LastTickDownloadTimeout > 10000)
	  {
			FlashDonwload = 0;
			MsgReady = 0;
			Command.Command_number = 0x00;
			DownloadingBytes = 0;
			respondERR();
	  }

	  if(HAL_GetTick() - LastTickBootloderRunTime > 45000 && FlashIsErese == 0 && FlashFlag == FLASH_FLAG)
	  {
		  jump_to_application(APP_START_ADDRESS);
	  }


	  if(MsgReady || Command.Command_number == ENDED)
	  {
		  __RUN_TIME_REFRESH;
		  switch(Command.Command_number){
		  case ECHO:
			  respondOK();
			  break;
		  case FULL_SIZE:
			  FlashInfoVariable.FullSize = Command.Argument;
			  respondOK();
			  break;
		  case FULL_CRC:
			  FlashInfoVariable.CRC_flash_Receive = Command.Argument;
			  respondOK();
			  break;
		  case EREASE:
			  if(HAL_FLASH_Unlock()!= HAL_OK)
			  {
				  respondERR();
				  break;
			  }
			  FlashIsErese = 1;
			  if(EreaseApp(APP_START_ADDRESS, 240) != HAL_OK)
			  {
				  respondERR();
			  }
			  if(HAL_FLASH_Lock() != HAL_OK)
			  {
				  respondERR();
			  }
			  else respondOK();
			  break;
		  case PACKET_SIZE:
			  FlashInfoVariable.PacketSize = Command.Argument;
			  respondOK();
			  break;
		  case PACKET_ADDRESS:
			  FlashInfoVariable.PacketAddress = Command.Argument;
			  respondOK();
			  break;
		  case PACKET_CRC:
			  FlashInfoVariable.CRC_packet_Receive = Command.Argument;
			  respondOK();
			  break;
		  case RECEIVE_PACKET:
			  FlashDonwload = 1;
			  LastTickDownloadTimeout = HAL_GetTick();
			  respondPacketReady();
			  break;
		  case FULL_CRC_CALCULATE:
			  FlashInfoVariable.CRC_flash = HAL_CRC_Calculate(&hcrc, APP_START_ADDRESS, FlashInfoVariable.FullSize / 4);
			  if(FlashInfoVariable.CRC_flash != FlashInfoVariable.CRC_flash_Receive)
			  {
				  respondERR();
			  }
			  else
			  {
				  respondOK();
			  }
			  break;
		  case ENDED:
			  if(HAL_FLASH_Unlock() != HAL_OK)
			  {
				  respondERR();
				  break;
			  }
			  HAL_Delay(100);
			  if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_FLAG_ADDRESS, FLASH_FLAG) != HAL_OK)
			  {
				  respondERR();
				  break;
			  }
			  if(HAL_FLASH_Lock() != HAL_OK)
			  {
				  respondERR();
				  break;
			  }
			  respondOK();
			  HAL_Delay(100);
			  jump_to_application(APP_START_ADDRESS);
			  break;
		  }
		  MsgReady = 0;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void FlashWriteProcedure(void)
{
	//
	//CRC
	//
	FlashInfoVariable.CRC_packet = HAL_CRC_Calculate(&hcrc, Buffer_Packet.FlashBuffer_32, FlashInfoVariable.PacketSize/4);
	if(FlashInfoVariable.CRC_packet != FlashInfoVariable.CRC_packet_Receive)
	{
		respondCRC_err();
		return;
	}

	if(HAL_FLASH_Unlock() != HAL_OK)
	{
		respondERR();
		return;
	}

    AES_ctx_set_iv(&ctx, iv);
    AES_CBC_decrypt_buffer(&ctx, Buffer_Packet.FlashBuffer_8, FlashInfoVariable.PacketSize);
    updateIV(iv, iv_update);

	for(uint32_t WordsCounter=0; WordsCounter<FlashInfoVariable.PacketSize/4; WordsCounter++)
	{
		uint32_t ActualAddr = APP_START_ADDRESS + FlashInfoVariable.PacketAddress + (WordsCounter * 4);
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ActualAddr, Buffer_Packet.FlashBuffer_32[WordsCounter]) != HAL_OK)
		{
			HAL_FLASH_Lock();
			respondERR();
			return;
		}
	}
	if(HAL_FLASH_Lock() != HAL_OK)
	{
		respondERR();
		return;
	}
	//
	//Verify
	//
	if(memcmp(Buffer_Packet.FlashBuffer_8, APP_START_ADDRESS + FlashInfoVariable.PacketAddress, FlashInfoVariable.PacketSize) != 0)
	{
		respondERR();
		return;
	}
	respondOK();
}

void DownloadingPacket(uint8_t *Data, uint32_t Length)
{
	memcpy(Buffer_Packet.FlashBuffer_8 + DownloadingBytes, Data, Length);
	DownloadingBytes = DownloadingBytes + Length;
	if(DownloadingBytes >= FlashInfoVariable.PacketSize)
	{
		FlashDonwload = 0;
		MsgReady = 0;
		Command.Command_number = 0x00;
		FlashWriteProcedure();
		DownloadingBytes = 0;
	}
}

void CDC_ReveiveCallback(uint8_t *Buffer, uint32_t *Length)
{
		if(FlashDonwload)
		{
			DownloadingPacket(Buffer, *Length);
		}
		else
		{
			memcpy(Command.Buff, Buffer, 5);
			FlashDonwload = 0;

		}
		if(!FlashDonwload)MsgReady = 1;

}



void deinit_peripherals(void)
{
	__disable_irq();
	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
	USBD_Stop(&hUsbDeviceFS);
	USBD_DeInit(&hUsbDeviceFS);
	__HAL_RCC_USB_CLK_DISABLE();
	memset(&hUsbDeviceFS, 0, sizeof(USBD_HandleTypeDef));

	  HAL_GPIO_DeInit(BOOT_PIN_GPIO_Port, BOOT_PIN_Pin);
	  HAL_DeInit();

	  SysTick->CTRL = 0;
	  SysTick->LOAD = 0;
	  SysTick->VAL = 0;

}

void jump_to_application(uint32_t const app_address) {
	for (uint8_t i = 0; i < 8; i++) {
	        NVIC->ICER[i] = 0xFFFFFFFF;
	    }
  typedef void (*jumpFunction)(); // helper-typedef
  uint32_t const jumpAddress = *(__IO uint32_t*) (app_address + 4); // Address of application's Reset Handler
  jumpFunction runApplication =  jumpAddress; // Function we'll use to jump to application


  deinit_peripherals(); // Deinitialization of peripherals and systick

  __set_MSP(*((__IO uint32_t*) app_address)); // Stack pointer setup
  runApplication(); // Jump to application
}

void updateIV(uint8_t* iv, const uint8_t* block) {
    for (int i = 0; i < 16; i++) {
        iv[i] ^= block[i];
    }
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
