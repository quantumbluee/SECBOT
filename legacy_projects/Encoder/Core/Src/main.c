/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "NRF905.h"
#include <stdbool.h>
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void initialise_monitor_handles(void);
NRF905_hw_t NRF905_hw;
NRF905_t NRF905;

int master=1;

#define NUM_OF_PINS 16

#define RADIO_CHANNEL 1
#define RADIO_ADDRESS 0x12345678

uint32_t nrf905_payload_buffer[NRF905_MAX_PAYLOAD];
uint32_t my_address;
#define ADDRESS_MASTER 0xA2B5D154
uint32_t last_PA4_state = 0;
bool flag = false;


// function to send data
void sendData(uint32_t data) {
  // create an array of 4 elements to store data
  uint8_t dataArray[4];
  // extract the first byte of data (most significant byte)
  dataArray[0] = (data >> 24) & 0xFF;
  // extract the second byte of data
  dataArray[1] = (data >> 16) & 0xFF;
  // extract the third byte of data
  dataArray[2] = (data >> 8) & 0xFF;
  // extract the fourth byte of data (least significant byte)
  dataArray[3] = data & 0xFF;
  // transmit data using NRF905 module
  // parameters: address of the NRF905 object, destination address, data to be transmitted, number of bytes to be transmitted, mode after transmission (NRF905_NEXTMODE_STANDBY for standby mode)
  NRF905_tx(&NRF905, RADIO_ADDRESS, dataArray, 4, NRF905_NEXTMODE_STANDBY);
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
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);


  // Initialize the random number generator with the unique device ID
  uint32_t uid = 0x00;
    	for (uint8_t i = 0; i < 3; ++i) {
    		uid += (uint32_t) (UID_BASE + i * 4);
    	}
    	srand(uid);


    	// Initialize the NRF905 hardware
    	NRF905_hw.gpio[NRF905_HW_GPIO_TXEN].pin = TXEN_Pin;
    	NRF905_hw.gpio[NRF905_HW_GPIO_TXEN].port = TXEN_GPIO_Port;
    	NRF905_hw.gpio[NRF905_HW_GPIO_TRX_EN].pin = TRX_CE_Pin;
    	NRF905_hw.gpio[NRF905_HW_GPIO_TRX_EN].port = TRX_CE_GPIO_Port;
    	NRF905_hw.gpio[NRF905_HW_GPIO_PWR].pin = PWR_Pin;
    	NRF905_hw.gpio[NRF905_HW_GPIO_PWR].port = PWR_GPIO_Port;

    	NRF905_hw.gpio[NRF905_HW_GPIO_CD].pin = CD_Pin;
    	NRF905_hw.gpio[NRF905_HW_GPIO_CD].port = CD_GPIO_Port;
    	NRF905_hw.gpio[NRF905_HW_GPIO_AM].pin = 0;
    	NRF905_hw.gpio[NRF905_HW_GPIO_AM].port = NULL;
    	NRF905_hw.gpio[NRF905_HW_GPIO_DR].pin = DR_Pin;
    	NRF905_hw.gpio[NRF905_HW_GPIO_DR].port = DR_GPIO_Port;

    	NRF905_hw.gpio[NRF905_HW_GPIO_CS].pin = SPI_CS_Pin;
    	NRF905_hw.gpio[NRF905_HW_GPIO_CS].port = SPI_CS_GPIO_Port;

    	NRF905_hw.tim = &htim14;
    	NRF905_hw.spi = &hspi2;

		// Initialize the NRF905 device
    	NRF905_init(&NRF905, &NRF905_hw);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    	// infinite loop
    	while (1) {
    	  // Check if PA4 is pressed
    	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {

    	    // Call the sendData function and pass 0xDEADBEEF as the data to be sent
    	    sendData(0xDEADBEEF);

    	    // Turn on the LED by setting PC13 high
    	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    	    // Wait for 0.5 seconds
    	    HAL_Delay(5);

    	    // Turn off the LED by setting PC13 low
    	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    	    // Wait until the button is released
    	    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {}
    	  }
    	}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	 if(htim->Instance == TIM3)
	 {
		flag = true;

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
