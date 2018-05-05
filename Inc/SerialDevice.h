#ifndef SERIAL_DEVICE_H
#define SERIAL_DEVICE_H

#include <stdint.h>
#include <stdbool.h>
#ifdef  Q_OS_WIN

#include <windows.h>

#define Baudrate_9600       9600
#define Baudrate_115200     115200
#define DataBits_7          7
#define DataBits_8          8
#define Parity_Even         EVENPARITY
#define Parity_None         NOPARITY
#else
#include <usart.h>
#define Baudrate_9600       9600
#define Baudrate_115200     115200
#define DataBits_7          UART_WORDLENGTH_7B
#define DataBits_8          UART_WORDLENGTH_8B
#define Parity_Even         UART_PARITY_EVEN
#define Parity_None         UART_PARITY_NONE
#endif

typedef uint8_t             UINT8;
typedef uint32_t            UINT32;

// open serial device
bool
SerialDevice_Open(
#ifdef Q_OS_WIN
		const char*   comPort,
#else
		UART_HandleTypeDef * huart,
#endif
                  UINT32        baudRate,
                  int           dataBits,
                  UINT8         parity);

// close serial device
bool
SerialDevice_Close();

// send single byte
int
SerialDevice_SendByte(UINT8 txByte);

// send data
int
SerialDevice_SendData(UINT8*    txBuffer,
                      int       txLength);

// read data
int
SerialDevice_ReadData(UINT8*    rxBuffer,
                      int       rxBufferSize);


#endif // SERIAL_DEVICE_H
