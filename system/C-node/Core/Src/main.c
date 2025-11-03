#include "main.h"
#include "string.h"

// Motor control pins
#define MOTOR1_EN_PIN    GPIO_PIN_2  // ENA for Motor 1 (GPIO Pin PF2)
#define MOTOR2_EN_PIN    GPIO_PIN_3  // ENB for Motor 2 (GPIO Pin PF3)
#define MOTOR1_DIR_PIN   GPIO_PIN_11 // IN2 for Motor 1 (GPIO Pin PE11)
#define MOTOR2_DIR_PIN   GPIO_PIN_3  // IN4 for Motor 2 (GPIO Pin PA3)

// PWM Duty Cycle Variables
uint16_t motor1_duty_cycle = 90; // 50% duty cycle
uint16_t motor2_duty_cycle = 90; // 75% duty cycle

UART_HandleTypeDef huart3;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);

// Debugging helper
void Debug_Print(const char *message)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}

int main(void)
{
    // Initialize the system
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART3_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();

    Debug_Print("System initialized.\n");

    // Enable motors
    HAL_GPIO_WritePin(GPIOF, MOTOR1_EN_PIN, GPIO_PIN_SET); // Enable Motor 1
    Debug_Print("Motor 1 enabled (ENA high).\n");
    HAL_GPIO_WritePin(GPIOF, MOTOR2_EN_PIN, GPIO_PIN_SET); // Enable Motor 2
    Debug_Print("Motor 2 enabled (ENB high).\n");

    // Set directions
    HAL_GPIO_WritePin(GPIOE, MOTOR1_DIR_PIN, GPIO_PIN_SET); // Motor 1 forward
    Debug_Print("Motor 1 set to forward direction (IN2 high).\n");
    HAL_GPIO_WritePin(GPIOA, MOTOR2_DIR_PIN, GPIO_PIN_RESET); // Motor 2 reverse
    Debug_Print("Motor 2 set to reverse direction (IN4 low).\n");

    // Start PWM
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) == HAL_OK)
        Debug_Print("PWM started for Motor 1.\n");
    else
        Debug_Print("Error: PWM not started for Motor 1.\n");

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) == HAL_OK)
        Debug_Print("PWM started for Motor 2.\n");
    else
        Debug_Print("Error: PWM not started for Motor 2.\n");

    // Set initial PWM duty cycles
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (htim1.Init.Period * motor1_duty_cycle) / 100);
    Debug_Print("Motor 1 PWM duty cycle set.\n");

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (htim2.Init.Period * motor2_duty_cycle) / 100);
    Debug_Print("Motor 2 PWM duty cycle set.\n");

    while (1)
    {
        HAL_Delay(2000);
        Debug_Print("Toggling motor directions.\n");

        HAL_GPIO_TogglePin(GPIOE, MOTOR1_DIR_PIN); // Toggle Motor 1 direction
        Debug_Print("Motor 1 direction toggled.\n");

        HAL_GPIO_TogglePin(GPIOA, MOTOR2_DIR_PIN); // Toggle Motor 2 direction
        Debug_Print("Motor 2 direction toggled.\n");

        // Update PWM duty cycles
        motor1_duty_cycle = (motor1_duty_cycle + 10) % 100;
        motor2_duty_cycle = (motor2_duty_cycle + 20) % 100;

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (htim1.Init.Period * motor1_duty_cycle) / 100);
        Debug_Print("Updated Motor 1 PWM duty cycle.\n");

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (htim2.Init.Period * motor2_duty_cycle) / 100);
        Debug_Print("Updated Motor 2 PWM duty cycle.\n");
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 100;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOF, MOTOR1_EN_PIN | MOTOR2_EN_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = MOTOR1_EN_PIN | MOTOR2_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTOR1_DIR_PIN;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTOR2_DIR_PIN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_TIM1_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 99;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 99;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim2);
}

void Error_Handler(void)
{
    Debug_Print("Error occurred!\n");
    while (1)
    {
    }
}
