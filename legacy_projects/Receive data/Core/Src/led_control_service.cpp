#include "pw_rpc/server.h"
#include "pw_rpc_transport/uart_transport.h"
#include "led_control.pb.h"

class LedControlServiceImpl : public mypackage::LedControl::Service {
 public:
  pw::Status TurnOnLed(const LedRequest& request, LedResponse& response) override {
    if (request.on()) {
      // Turn on the LED
      HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);
      response.set_status("LED is turned ON");
    } else {
      // Turn off the LED
      HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
      response.set_status("LED is turned OFF");
    }
    return pw::Status::OK;
  }
};

int main() {
  // Initialize the hardware
  HAL_Init();
  // Configure the system clock
  SystemClock_Config();
  // Initialize all configured peripherals
  MX_GPIO_Init();

  // Set up the UART transport
  pw::rpc::UartTransport uart_transport;

  // Set up the RPC server
  pw::rpc::Server server;
  server.RegisterService<LedControlServiceImpl>();

  // Start the server
  while (true) {
    server.ProcessPackets(uart_transport);
  }
}
