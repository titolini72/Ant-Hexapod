#ifndef CONFIG_H
#define CONFIG_H

#ifdef SERIAL_DEBUG
#define DEBUG_PRINT(x)       Serial.print(x)
#define DEBUG_PRINTLN(x)     Serial.println(x)
#define DEBUG_PRINTF(x, ...) Serial.printf((x), __VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(x, ...)
#endif

#define CMD_MIT_APPINVENTOR 0
#define CMD_PC 1
#define CMD_TRANSMITTER 2
#define CMD_MANUAL_TEST 3

#ifndef COMMANDER
#define COMMANDER CMD_TRANSMITTER
#endif

#define LINK_TRANSPORT_BLE 0
#define LINK_TRANSPORT_NRF24 1
#define LINK_TRANSPORT_NONE 2

#ifndef LINK_TRANSPORT
#define LINK_TRANSPORT LINK_TRANSPORT_BLE
#endif

#if (LINK_TRANSPORT == LINK_TRANSPORT_BLE)
#define TRANSMISSION_BLE
#elif (LINK_TRANSPORT == LINK_TRANSPORT_NRF24)
#define NRF24_LINK
#elif (LINK_TRANSPORT == LINK_TRANSPORT_NONE)
#else
#error "Unsupported LINK_TRANSPORT selection."
#endif

#define POWER_MANAGER_DEBUG
#define SENSOR_MANAGER_DEBUG

#if defined(TRANSMISSION_BLE) && (COMMANDER == CMD_MANUAL_TEST)
#error "CMD_MANUAL_TEST and TRANSMISSION_BLE cannot be defined at the same time."
#endif

#if defined(NRF24_LINK) && (COMMANDER != CMD_TRANSMITTER)
#error "NRF24 transport is currently supported only with CMD_TRANSMITTER."
#endif

#if (COMMANDER == CMD_MANUAL_TEST)
/* This spoil console in Manual Test mode */
#warning "Manual Test mode active: Power and Sensor Manager debug disabled"
#undef POWER_MANAGER_DEBUG
#undef SENSOR_MANAGER_DEBUG
#elif (COMMANDER == CMD_TRANSMITTER)
#define NUM_JOYSTICKS 4
#define NUM_POTENTIOMETERS 2
#define NUM_BUTTONS 6
#endif

#if defined(TRANSMISSION_BLE)
#define UART_BAUDRATE 921600
#endif

#ifdef STM32H7
// Pin definitions
#define S1  PB8
#define S2  PB9
#define S3  PB6
#define S4  PB5
#define S5  PB4
#define S6  PB3
#define S7  PD7
#define S8  PD6
#define S9  PD5
#define S10 PB8 // Pin PD0 KO Move to PB8/S1
#define S11 PD1
#define S12 PA9
#define S13 PA10
#define S14 PD15
#define S15 PD14
#define S16 PD13
#define S17 PD12
#define S18 PD11
#define S19 PD10
#define S20 PD9
#define S21 PB15
#define S22 PD8
#define S23 PB14
#define S24 PB13

// Analog pin for voltage monitoring
#define A3 PB0

#define SONAR_DETECTION
//#define TOF_DETECTION

#if defined(SONAR_DETECTION) && defined(TOF_DETECTION)
#error "SONAR_DETECTION and TOF_DETECTION cannot be defined at the same time."
#endif

#ifdef SONAR_DETECTION
// Sonar sensor pins
#define SONAR_TRIG PB2
#define SONAR_ECHO PE7
#define SONAR_DETECTION_RANGE 30   // in cm
#define SONAR_MEASURE_DELAY 200    // in ms
#endif

#ifdef TOF_DETECTION
// ToF sensor I2C pins
#define TOF_I2C_SDA PB11
#define TOF_I2C_SCL PB10
#define TOF_IRQ PC0
#define TOF_DETECTION_RANGE 30   // in cm
#endif

// Led pin definitions
#define LED_ON_PIN   PE9
#define LED_LOW_PIN  PE8

// Leg and bone IDs
#define LEFT_ID   0
#define RIGHT_ID  9
#define FRONT_ID  0
#define MIDDLE_ID 3
#define BACK_ID   6
#define FEMUR_ID  0
#define TIBIA_ID  1
#define FEET_ID   2

#define HEAD_ROLL   (RIGHT_ID + BACK_ID + FEET_ID + 1)
#define HEAD_PITCH  (HEAD_ROLL + 1)
#define HEAD_GRIP   (HEAD_PITCH + 1)
#define TAIL_ID     (HEAD_GRIP + 1)

// Buffer size
#define MAX_BUFFER (6 + 1)               // Maximum size for incoming serial data buffer

#define SERIAL_BAUDRATE 115200     // Main serial interface baudrate

#if defined (TRANSMISSION_BLE)
#define ESP Serial4                // ESP serial interface
#endif

#if defined(NRF24_LINK)
// Default nRF24L01 wiring targets the SPI4-capable pin group on STM32H743.
// Change these macros to move the radio or match another SPI instance.
#define NRF24_CE_PIN PE3
#define NRF24_CSN_PIN PE4
#define NRF24_SCK_PIN PE2
#define NRF24_MISO_PIN PE5
#define NRF24_MOSI_PIN PE6
#define NRF24_IRQ_PIN PC13

#define NRF24_SPI_USE_CUSTOM_PINS 1
// If you prefer an existing SPI object instead of custom pins, set
// NRF24_SPI_USE_CUSTOM_PINS to 0 and define NRF24_SPI_INSTANCE to that object.
// Example: #define NRF24_SPI_INSTANCE SPI

#define NRF24_CHANNEL 76
#define NRF24_RETRY_DELAY 5
#define NRF24_RETRY_COUNT 15
#define NRF24_SPI_SPEED_HZ 8000000UL

#define NRF24_PA_LEVEL_MIN 0
#define NRF24_PA_LEVEL_LOW 1
#define NRF24_PA_LEVEL_HIGH 2
#define NRF24_PA_LEVEL_MAX 3
#define NRF24_PA_LEVEL NRF24_PA_LEVEL_LOW

#define NRF24_DATA_RATE_1MBPS 0
#define NRF24_DATA_RATE_2MBPS 1
#define NRF24_DATA_RATE_250KBPS 2
#define NRF24_DATA_RATE NRF24_DATA_RATE_250KBPS

#define NRF24_RX_ADDRESS "AntRx"
#define NRF24_TX_ADDRESS "AntTx"
#endif

#define ADC_PIN A3                 // Analog pin for voltage monitoring
#define ADC_MAX_VOLTAGE 3.3        // Maximum ADC reference voltage
#define ADC_RANGE 1023.0           // ADC Maximum range
#define ADC_RATIO 5.396            // Voltage divider ratio (measured not computed)
#define ADC_POLL_DELAY 1000        // in milliseconds
#define ADC_VOLTAGE_THRESHOLD 11.0 // in volts


#define DEFAULT_ANT_SPEED (20) // Default movement speed
#endif

#if defined(ESP32) && defined(TRANSMISSION_BLE)
// ============================================================
// Build selection
// 0 = BLE server
// 1 = BLE client
// ============================================================
#define BLE_FLAVOR_SERVER 0
#define BLE_FLAVOR_CLIENT 1

#ifndef BLE_FLAVOR
#define BLE_FLAVOR BLE_FLAVOR_SERVER
#endif

#define UART_RX_PIN 20
#define UART_TX_PIN 21

// BLE Service and Characteristic UUIDs (randomly generated)
#define SERVICE_UUID    "0db550cc-bf0e-46ff-84fc-06c08b1aff60"  // Service UUID
#define WRITE_UUID      "96df8658-427c-458b-86de-0f6703f28977"  // Characteristic UUID
#define NOTIFY_UUID     "96df8658-427c-458b-86de-0f6703f28978"  // Characteristic UUID

#define WEACT Serial1               // WeAct serial interface

#define CONSOLE_BAUDRATE 115200      // Main serial interface baudrate

#define STATUS_LED_PIN 8

#endif

#endif // CONFIG_H