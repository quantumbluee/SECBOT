#include "main.h"
#include "pw_rpc/server.h"
#include "pw_rpc_transport/uart.h" // Change to the correct UART transport header if necessary
#include "led_control.pb.h"
#include "stm32f4xx_hal.h" // Change to the correct header for your STM32 series

UART_HandleTypeDef huart2;

class LedControlServiceImpl : public mypackage::LedControl::Service {
public:
    pw::Status TurnOnLed(const LedRequest& request, LedResponse& response) override {
        if (request.on()) {
            // Turn on the LED
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);  // Adjust GPIO port and pin as necessary
            response.set_status("LED is turned ON");
        } else {
            // Turn off the LED
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);  // Adjust GPIO port and pin as necessary
            response.set_status("LED is turned OFF");
        }
        return pw::Status::OK;
    }
};

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);
void Error_Handler(void);

int main() {
    // Initialize the HAL Library
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_USART2_UART_Init();  // Initialize the UART peripheral used for transport

    // Set up the UART transport
    pw::rpc::Uart uart_transport(&huart2);  // Pass the UART handle

    // Set up the RPC server
    pw::rpc::Server server;
    server.RegisterService<LedControlServiceImpl>();

    // Start the server loop
    while (true) {
        server.ProcessPackets(uart_transport);
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO clocks
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // Configure GPIO pin : PD13 (Adjust to your specific GPIO pin)
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }
}

void Error_Handler(void) {
    // User can add their own implementation to report the HAL error return state
    while (1) {
    }
}
