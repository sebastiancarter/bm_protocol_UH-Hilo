#include "user_code.h"
#include "LineParser.h"
#include "OrderedSeparatorLineParser.h"
#include "app_util.h"
#include "array_utils.h"
#include "avgSampler.h"
#include "bristlefin.h"
#include "bsp.h"
#include "debug.h"
#include "lwip/inet.h"
#include "payload_uart.h"
#include "pubsub.h"
#include "sensors.h"
#include "spotter.h"
#include "stm32_rtc.h"
#include "task_priorities.h"
#include "uptime.h"
#include "usart.h"

#define LED_ON_TIME_MS 20
#define LED_PERIOD_MS 1000
#define DEFAULT_BAUD_RATE 9600
#define DEFAULT_LINE_TERM 10 // newline, '\n', 0x0A
#define BYTES_CLUSTER_MS 50 // used for console printing convenience
#define DEFAULT_UART_MODE MODE_RS232
// TODO: run the commands from the integrating sensor guide.
// TODO: try running bm pluart start
// TODO: try starting from the devkit itself to chase down what is going on if the above 2 things dont work

// A timer variable we can set to trigger a pulse on LED2 when we get payload serial data
static int32_t ledLinePulse = -1;
static u_int32_t baud_rate_config = DEFAULT_BAUD_RATE;
static u_int32_t line_term_config = DEFAULT_LINE_TERM;
static u_int32_t uart_mode_config = DEFAULT_UART_MODE;

// A buffer for our data from the payload uart
char payload_buffer[311]; // TODO: might need to change this, but feels like it was set to 311 for a good reason

void sendSensorCommand(const char *command_payload) {
  char full_command[128];
  snprintf(full_command, sizeof(full_command), "%s\r\n", command_payload);
  PLUART::write((uint8_t *)full_command, strlen(full_command));
  printf("Sent command to sensor: %s", full_command);
  vTaskDelay(pdMS_TO_TICKS(100)); // wait a bit for the command to be sent
}


void setup(void) {
  /* USER ONE-TIME SETUP CODE GOES HERE */
  // Retrieve user-set config values out of NVM.
  if (uart_mode_config >= MODE_MAX) {
    printf("ERROR - PLUART UART MODE %lu is not supported. Reverting to MODE_RS232\n", uart_mode_config);
    uart_mode_config = MODE_RS232;
  }
  // Setup the UART – the on-board serial driver that talks to the RS232 transceiver.
  PLUART::init(USER_TASK_PRIORITY);
  // Baud set per expected baud rate of the sensor.
  PLUART::setBaud(baud_rate_config);
  // Enable passing raw bytes to user app.
  PLUART::setUseByteStreamBuffer(true);
  // Enable parsing lines and passing to user app.
  /// Warning: PLUART only stores a single line at a time. If your attached payload sends lines
  /// faster than the app reads them, they will be overwritten and data will be lost.
  PLUART::setUseLineBuffer(true);
  // Set a line termination character per protocol of the sensor.
  PLUART::setTerminationCharacter((char)line_term_config);
  if (uart_mode_config == MODE_RS485_HD) {
    printf("Enabling RS485 Half-Duplex.\n");
    bristlefin.setRS485HalfDuplex();  // Set the RS485 transceiver to Half-Duplex.
    // Set up PLUART transactions for proper RS485 enable Tx / enable Rx
    PLUART::enableTransactions(Bristlefin::setRS485Tx, Bristlefin::setRS485Rx);
  }
  // Turn on the UART.
  PLUART::enable();
  // Enable the input to the Vout power supply.
  bristlefin.enableVbus();
  // ensure Vbus stable before enable Vout with a 5ms delay.
  vTaskDelay(pdMS_TO_TICKS(5));
  // enable Vout, 12V by default.
  bristlefin.enableVout();
  // enable 5V out.
  // bristlefin.enable5V();
  // Choose which command to run on startup
  // should send once things start
  // turns sensor on
  // sendSensorCommand("StartNow");
  // turns sensor off
  sendSensorCommand("Stop");
}

void loop(void) {
  /* USER LOOP CODE GOES HERE */
  static bool led2State = false;
  /// This checks for a trigger set by ledLinePulse when data is received from the payload UART.
  ///   Each time this happens, we pulse LED2 Green.
  // If LED2 is off and the ledLinePulse flag is set (done in our payload UART process line function), turn it on Green.
  if (!led2State && ledLinePulse > -1) {
    bristlefin.setLed(2, Bristlefin::LED_GREEN);
    led2State = true;
  }
  // If LED2 has been on for LED_ON_TIME_MS, turn it off.
  else if (led2State &&
           ((u_int32_t)uptimeGetMs() - ledLinePulse >= LED_ON_TIME_MS)) {
    bristlefin.setLed(2, Bristlefin::LED_OFF);
    ledLinePulse = -1;
    led2State = false;
  }

  /// This section demonstrates a simple non-blocking bare metal method for rollover-safe timed tasks,
  ///   like blinking an LED.
  /// More canonical (but more arcane) modern methods of implementing this kind functionality
  ///   would bee to use FreeRTOS tasks or hardware timer ISRs.
  static u_int32_t ledPulseTimer = uptimeGetMs();
  static u_int32_t ledOnTimer = 0;
  static bool led1State = false;
  // Turn LED1 on green every LED_PERIOD_MS milliseconds.
  if (!led1State &&
      ((u_int32_t)uptimeGetMs() - ledPulseTimer >= LED_PERIOD_MS)) {
    bristlefin.setLed(1, Bristlefin::LED_GREEN);
    ledOnTimer = uptimeGetMs();
    ledPulseTimer += LED_PERIOD_MS;
    led1State = true;
  }
  // If LED1 has been on for LED_ON_TIME_MS milliseconds, turn it off.
  else if (led1State &&
           ((u_int32_t)uptimeGetMs() - ledOnTimer >= LED_ON_TIME_MS)) {
    bristlefin.setLed(1, Bristlefin::LED_OFF);
    led1State = false;
  }

  // Read a cluster of bytes if available
  // -- A timer is used to try to keep clusters of bytes (say from lines) in the same output.
  static int64_t readingBytesTimer = -1;
  // Note - PLUART::setUseByteStreamBuffer must be set true in setup to enable bytes.
  if (readingBytesTimer == -1 && PLUART::byteAvailable()) {
    // Get the RTC if available
    RTCTimeAndDate_t time_and_date = {};
    rtcGet(&time_and_date);
    char rtcTimeBuffer[32];
    rtcPrint(rtcTimeBuffer, &time_and_date);
    printf("[payload-bytes] | tick: %llu, rtc: %s, bytes:", uptimeGetMs(),
           rtcTimeBuffer);
    // not very readable, but it's a compact trick to overload our timer variable with a -1 flag
    readingBytesTimer = (int64_t)((u_int32_t)uptimeGetMs());
  }
  while (PLUART::byteAvailable()) {
    readingBytesTimer = (int64_t)((u_int32_t)uptimeGetMs());
    uint8_t byte_read = PLUART::readByte();
    printf("%02X ", byte_read);
  }
  if (readingBytesTimer > -1 &&
      (u_int32_t)uptimeGetMs() - (u_int32_t)readingBytesTimer >= BYTES_CLUSTER_MS) {
    printf("\n");
    readingBytesTimer = -1;
  }

  // Read a line if it is available
  // Note - PLUART::setUseLineBuffer must be set true in setup to enable lines.
  if (PLUART::lineAvailable()) {
    // Shortcut the raw bytes cluster completion so the parsed line will be on a new console line
    if (readingBytesTimer > -1) {
      printf("\n");
      readingBytesTimer = -1;
    }
    uint16_t read_len =
        PLUART::readLine(payload_buffer, sizeof(payload_buffer));

    // Get the RTC if available
    RTCTimeAndDate_t time_and_date = {};
    rtcGet(&time_and_date);
    char rtcTimeBuffer[32];
    rtcPrint(rtcTimeBuffer, &time_and_date);

    // Print the payload data to a file, to the bm_printf console, and to the printf console.
    spotter_log(0, "payload_data.log", USE_TIMESTAMP, "tick: %llu, rtc: %s, line: %.*s\n",
               uptimeGetMs(), rtcTimeBuffer, read_len, payload_buffer);
    spotter_log_console(0, "[payload] | tick: %llu, rtc: %s, line: %.*s", uptimeGetMs(),
              rtcTimeBuffer, read_len, payload_buffer);
    printf("[payload-line] | tick: %llu, rtc: %s, line: %.*s\n", uptimeGetMs(),
           rtcTimeBuffer, read_len, payload_buffer);


    // Transmit over Spotter celluar or Iridium SBD fallback.
    if (spotter_tx_data(payload_buffer, sizeof(payload_buffer), BmNetworkTypeCellularIriFallback)) {
      printf("%llut - %s | Sucessfully sent Spotter transmit data request\n", uptimeGetMs(),
             rtcTimeBuffer);
    } else {
      printf("%llut - %s | Failed to send Spotter transmit data request\n", uptimeGetMs(),
             rtcTimeBuffer);
    }
    ledLinePulse = uptimeGetMs(); // trigger a pulse on LED2
  }
}
