#include "pw_rpc/client.h"
#include "pw_rpc_transport/uart_transport.h"
#include "led_control.pb.h"

int main() {
  // Set up the UART transport
  pw::rpc::UartTransport uart_transport;

  // Set up the RPC client
  pw::rpc::Client client(uart_transport);

  // Prepare the request to turn on the LED
  LedRequest request;
  request.set_on(true);  // Turn on the LED

  // Call the RPC and handle the response
  LedResponse response;
  pw::Status status = client.CallMethod<mypackage::LedControl::TurnOnLed>(request, response);
  if (status.ok()) {
    // Handle the response
    printf("Response: %s\n", response.status().c_str());
  } else {
    // Handle the error
    printf("RPC call failed: %s\n", status.str());
  }

  return 0;
}
