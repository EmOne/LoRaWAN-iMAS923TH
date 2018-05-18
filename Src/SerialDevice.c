//------------------------------------------------------------------------------
//
//	File:		SerialDevice.cpp
//
//	Abstract:	Serial Device Abstraction
//
//	Version:	0.1
//
//	Date:		18.05.2016
//
//	Disclaimer:	This example code is provided by IMST GmbH on an "AS IS" basis
//				without any warranties.
//
//	Maintain by : Anol Paisal <anol.paisal@emone.co.th> @ 2018
//
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//
//  Include Files
//
//------------------------------------------------------------------------------

#include "SerialDevice.h"
#include "WiMOD_LoRaWAN_API.h"

//------------------------------------------------------------------------------
//
//  Section RAM
//
//------------------------------------------------------------------------------

#ifdef Q_OS_WIN

// File Handle
static HANDLE   ComHandle = INVALID_HANDLE_VALUE;

#else
// Todo : add your own platform specific variables here
// forward declarations
static void     Ping(void);
static void     GetNwkStatus(void);
static void 	GetCustomCFG(void);
static void     SetCustomCFG(void);
static void		ActivateABP(void);
static void		Reactivate(void);
static void		Deactivate(void);
static void		SetRadioStack(void);
static void     GetRadioStack(void);
static void 	GetSupportBand(void);
static void     GetDevEUI(void);
static void 	SetDevEUI(void);
static void 	GetOPMODE(void);
static void 	SetOPMODE(void);
static void 	ClearOPMODE(void);
static void     ClearRTCAlarm(void);
static void     GetRTC(void);
static void     SetRTC(void);
static void     GetAlarm(void);
static void     SetAlarm(void);
static void     GetFirmwareVersion(void);
static void     GetDeviceInfo(void);
static void     GetDeviceStatus(void);
static void     Reset(void);
static void     Join(void);
static void     SendUData(void);
static void     SendCData(void);
static void		FactoryReset(void);
static void		GetLinkADR(void);
static void		SetLinkADR(void);
#endif

#define USART_STATUS_REGISTER   ISR                     //!< HAL USART status register name adapter.
#define USART_DATA_REGISTER     RDR                     //!< HAL UART data register name adapter.

//------------------------------------------------------------------------------
//
//  Section Code
//
//------------------------------------------------------------------------------
uint8_t UsartTextString[USARTTEXTSTRINGSIZE];
//------------------------------------------------------------------------------
//
//  Open
//
//  @brief: open serial device
//
//------------------------------------------------------------------------------

bool
SerialDevice_Open(
#ifdef Q_OS_WIN
		const char*   comPort,
#else
		UART_HandleTypeDef * huart,
#endif
                  UINT32        baudRate,
                  int           dataBits,
                  UINT8         parity)
{

#ifdef Q_OS_WIN
    // handle valid ?
    if (ComHandle != INVALID_HANDLE_VALUE)
        SerialDevice_Close();

    char devName[80];

    // windows workaround for COM Ports higher than COM9
    strcpy(devName, "\\\\.\\");
    strcat(devName, comPort);


    ComHandle = CreateFileA(devName,
                            GENERIC_WRITE | GENERIC_READ,
                            0,
                            NULL,
                            OPEN_EXISTING,
                            0, //FILE_FLAG_WRITE_THROUGH, //0
                            NULL);

    // handle valid ?
    if (ComHandle != INVALID_HANDLE_VALUE)
    {
        DCB dcb;
        if (GetCommState(ComHandle, &dcb))
        {
            dcb.DCBlength           = sizeof(DCB);
            dcb.BaudRate            = baudRate;
            dcb.ByteSize            = dataBits;
            dcb.Parity              = parity; //EVENPARITY;// NOPARITY;
            dcb.StopBits            = ONESTOPBIT;
            dcb.fOutxCtsFlow        = FALSE;
            dcb.fOutxDsrFlow        = FALSE;
            dcb.fDtrControl         = DTR_CONTROL_DISABLE;
            dcb.fDsrSensitivity     = FALSE;
            dcb.fTXContinueOnXoff   = FALSE;
            dcb.fOutX               = FALSE; // no XON/XOFF
            dcb.fInX                = FALSE; // no XON/XOFF
            dcb.fErrorChar          = FALSE;
            dcb.fNull               = FALSE;
            dcb.fRtsControl         = RTS_CONTROL_DISABLE;
            dcb.fAbortOnError       = FALSE;

            if (SetCommState(ComHandle, &dcb))
            {
                COMMTIMEOUTS commTimeouts;
                commTimeouts.ReadIntervalTimeout 		= MAXDWORD;
                commTimeouts.ReadTotalTimeoutMultiplier = 0;
                commTimeouts.ReadTotalTimeoutConstant 	= 0;

                commTimeouts.WriteTotalTimeoutMultiplier = 10;
                commTimeouts.WriteTotalTimeoutConstant  = 1;

                SetCommTimeouts(ComHandle, &commTimeouts);

                // ok
                return true;
            }
        }
        // close device
        SerialDevice_Close();
    }
#else
    // Todo : add your own platform specific code here
//    if (huart->gState == HAL_UART_STATE_READY) {
//		return true;
//	}
    huart->Init.BaudRate = baudRate;
    huart->Init.WordLength = dataBits;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = parity;

	if (HAL_UART_Init(huart) == HAL_OK)
	{
		return true;
	}

	SerialDevice_Close();
	_Error_Handler(__FILE__, __LINE__);
//	if (HAL_UARTEx_SetTxFifoThreshold(&huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//	{
//	_Error_Handler(__FILE__, __LINE__);
//	}
//
//	if (HAL_UARTEx_SetRxFifoThreshold(&huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//	{
//	_Error_Handler(__FILE__, __LINE__);
//	}
//
//	if (HAL_UARTEx_DisableFifoMode(&huart) != HAL_OK)
//	{
//	_Error_Handler(__FILE__, __LINE__);
//	}

#endif
    // error
    return false;
}

//------------------------------------------------------------------------------
//
//  Close
//
//  @brief: close serial device
//
//------------------------------------------------------------------------------

bool
SerialDevice_Close()
{
#ifdef Q_OS_WIN
    // handle valid ?
    if (ComHandle != INVALID_HANDLE_VALUE)
    {
        // cancel last operation
        CancelIo(ComHandle);

        // wait 100us
        Sleep(100);

        // close device
        CloseHandle(ComHandle);

        // invalidate handle
        ComHandle = INVALID_HANDLE_VALUE;

        // ok
        return true;
    }
#else
    // Todo : add your own platform specific code here
    HAL_UART_DeInit(&huart3);
#endif
    // error
    return false;
}

//------------------------------------------------------------------------------
//
//  SendData
//
//  @brief: send data
//
//------------------------------------------------------------------------------

int
SerialDevice_SendData(UINT8* txBuffer, int txLength)
{
#ifdef Q_OS_WIN
    // handle valid ?
    if (ComHandle == INVALID_HANDLE_VALUE)
        return -1;

    UINT32  numTxBytes;

    // write chunk of data
    if (!WriteFile(ComHandle, txBuffer, txLength, (DWORD*)&numTxBytes, 0))
    {
        // error
        return -1;
    }
    // all bytes written ?
    if (numTxBytes == (UINT32)txLength)
    {
        // ok
        return numTxBytes;
    }
#else
    // Todo : add your own platform specific code here
	if(HAL_UART_Transmit(&huart3, txBuffer, txLength, HAL_MAX_DELAY) != HAL_ERROR)
	{
		return txLength;
	}
#endif
    // error
    return -1;
}

//------------------------------------------------------------------------------
//
//  SendByte
//
//  @brief: send single byte
//
//------------------------------------------------------------------------------

int
SerialDevice_SendByte(UINT8 txByte)
{
#ifdef Q_OS_WIN
    // handle valid ?
    if (ComHandle == INVALID_HANDLE_VALUE)
        return -1;

    UINT32  numTxBytes;

    // write chunk of data
    if (!WriteFile(ComHandle, &txByte, 1, (DWORD*)&numTxBytes, 0))
    {
        // error
        return -1;
    }
    // all bytes written ?
    if (numTxBytes == 1)
    {
        // ok
        return numTxBytes;
    }
#else
    // Todo : add your own platform specific code here
    if(HAL_UART_Transmit(&huart3, &txByte, 1, HAL_MAX_DELAY) != HAL_ERROR)
	{
		return 1;
	}
#endif
    // error
    return -1;
}

//------------------------------------------------------------------------------
//
//  ReadData
//
//  @brief: read data
//
//------------------------------------------------------------------------------

int
SerialDevice_ReadData(UINT8* rxBuffer, int rxBufferSize)
{
#ifdef  Q_OS_WIN
    // handle ok ?
    if (ComHandle == INVALID_HANDLE_VALUE)
        return -1;

    DWORD numRxBytes = 0;

    // read chunk of data
    if (ReadFile(ComHandle, rxBuffer, rxBufferSize, &numRxBytes, 0))
    {
        // return number of bytes read
        return (int)numRxBytes;
    }
#else
    // Todo : add your own platform specific code here
    if(HAL_UART_Receive(&huart3, rxBuffer, rxBufferSize, 500) != HAL_ERROR)
    {
    	return rxBufferSize;
    }
#endif
    // error
    return -1;
}

/**
 * @brief  Transmit the initial message to the PC which is connected to the
 *         NUCLEO board via UART.
 */
void USART_TxWelcomeMessage(void) {
	/* Send information to PC via USART */
	USART_Transmit(&hlpuart1, "\n\r");
	USART_Transmit(&hlpuart1, " LoRaWAN-iMAS923TH (iM980A) Demo\n\r");
	USART_Transmit(&hlpuart1, " -------------------------------------------\n\r");
	USART_Transmit(&hlpuart1, " iMAS923TH Expansion Board for STM32 NUCLEO\n\r\n\r");
	USART_Transmit(&hlpuart1, " Copyright 2018 @ EmOne; The MIT License (MIT)\n\r\n\r");
	USART_Transmit(&hlpuart1, " Welcome to eMOD HCL example (WiMOD compatible)!!!\n\r");

}


/**
 * @brief  Send a text string via USART.
 * @param  huart       pointer to a UART_HandleTypeDef structure that contains
 *                     the configuration information for the specified UART module.
 * @param  TextString  The text string to be sent.
 * @note It use the HAL_UART_Transmit function.
 */
void USART_Transmit(UART_HandleTypeDef* huart, const char* data) {
	USART_Transmit_Data(huart, (uint8_t*) data);
}

void USART_Transmit_Data(UART_HandleTypeDef* huart, uint8_t* TextString) {
	uint8_t TextStringLength;

	/* Calculate the length of the text string to be sent */
	TextStringLength = 0;
	while (TextString[TextStringLength++] != '\0')
		;
	TextStringLength--;

	/* Use the HAL function to send the text string via USART */
	HAL_UART_Transmit(huart, TextString, TextStringLength, HAL_MAX_DELAY);
}


/**
 * @brief  Handle text character just received.
 * @param  huart pointer to a UART_HandleTypeDef structure that contains
 *               the configuration information for the specified UART module.
 * @note To use inside USARx_IRQHandler function.
 */
void USART_ITCharManager(UART_HandleTypeDef* huart) {
	uint8_t UART_Receive_IT_Char;

	UART_Receive_IT_Char = (uint8_t) (huart->Instance->USART_DATA_REGISTER);
	/* Checks the buffer full or return carriage  */
	if ((huart->RxXferCount == 1) || (UART_Receive_IT_Char == '\r')) {
		huart->RxXferCount += 1;
		huart->pRxBuffPtr -= 1;
		*(huart->pRxBuffPtr) = '\0';

		USART_Transmit(huart, "\n\r" );

		while (HAL_IS_BIT_SET(huart->Instance->USART_STATUS_REGISTER,
				UART_FLAG_RXNE)) {
		}
		__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);

		/* Check if a transmit process is ongoing or not */
		if (huart->gState == HAL_UART_STATE_BUSY_TX_RX) {
			huart->gState = HAL_UART_STATE_BUSY_TX;
		} else {
			/* Disable the UART Parity Error Interrupt */
			__HAL_UART_DISABLE_IT(huart, UART_IT_PE);

			/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
			__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

			huart->gState = HAL_UART_STATE_READY;
		}
	}
}

/**
 * @brief  This function converts a text string into a number.
 * @param  str       The pointer to the text string to convert.
 * @param  pnum      The pointer to the numerical variable.
 * @retval FlagStatus  SET or RESET related to the conversion.
 */
FlagStatus str2num(uint8_t* str, uint32_t* pnum) {
	uint8_t TxtStr_digit, digit;
	uint32_t tenpwr;
	uint32_t number;

	digit = 0;

	while (*(str + digit) != '\0') {
		if (((*(str + digit) >= '0') && (*(str + digit) <= '9'))) {
			digit++;
		} else {
			*pnum = 0;
			return RESET;
		}
	}

	tenpwr = 1;
	number = 0;

	do {
		TxtStr_digit = (*(str + (--digit)));
		number += ((TxtStr_digit - '0') * tenpwr);
		tenpwr *= 10;
	} while (digit);

	*pnum = number;
	return SET;
}

/**
 * @brief  Convert a number nbr into a string str with 7 characters.
 * @param  nbr The number to be converted.
 * @param  str The container of the converted number into a text in decimal
 *         format.
 * @note   The decimal digits of the number must be maximum 7 so str has to be
 *         able to store at least 7 characters plus '\0'.
 */
void num2str(uint32_t nbr, uint8_t *str) {
	uint8_t k;
	uint8_t *pstrbuff;
	uint32_t divisor;

	pstrbuff = str;

	/* Reset the text string */
	for (k = 0; k < 7; k++)
		*(pstrbuff + k) = '\0';

	divisor = 1000000;

	if (nbr) // if nbr is different from zero then it is processed
	{
		while (!(nbr / divisor)) {
			divisor /= 10;
		}

		while (divisor >= 10) {
			k = nbr / divisor;
			*pstrbuff++ = '0' + k;
			nbr = nbr - (k * divisor);
			divisor /= 10;
		}
	}

	*pstrbuff++ = '0' + nbr;
	*pstrbuff++ = '\0';
}

/**
 * @brief  Convert an integer number into hexadecimal format.
 *
 * @param  num         The integer number to convert.
 * @param  HexFormat   The output format about hexadecimal number.
 *
 * @retval uint8_t*    The address of the string text for the converted hexadecimal number.
 */
uint8_t* num2hex(uint32_t num, eHexFormat HexFormat) {
	static uint8_t HexValue[8 + 1];
	uint8_t i;
	uint8_t dummy;
	uint8_t HexDigits = 0;

	switch (HexFormat) {
	case HALFBYTE_F:
		HexDigits = 1;
		break;
	case BYTE_F:
		HexDigits = 2;
		break;
	case WORD_F:
		HexDigits = 4;
		break;
	case DOUBLEWORD_F:
		HexDigits = 8;
		break;
	default:
		HexDigits = 2;
		break;
	}

	for (i = 0; i < HexDigits; i++) {
		HexValue[i] = '\0';
		dummy = (num & (0x0F << (((HexDigits - 1) - i) * 4)))
				>> (((HexDigits - 1) - i) * 4);
		if (dummy < 0x0A) {
			HexValue[i] = dummy + '0';
		} else {
			HexValue[i] = (dummy - 0x0A) + 'A';
		}
	}
	HexValue[i] = '\0';

	return HexValue;
}

/**
 * @brief  Check if any Application Command for L6470 has been entered by USART
 *         so to proceed to decode and perform the command.
 */
void USART_CheckAppCmd(void)
{
	/* Checks the UART2 is in idle state */
	if (hlpuart1.gState == HAL_UART_STATE_READY) {

		/* Checks one character has been at least entered */
		if (strchr((char *) UsartTextString, '\r') != NULL) {
			/* Decode the entered command string */
			strtok((char *) UsartTextString, "\r");
//			USART_DecodeTextString(UsartTextString, L6470_TextCommandBundle,
//					(uint8_t*) L6470_DaisyChainSpiTxStruct,
//					(uint8_t*) L6470_DaisyChainSpiRxStruct);

			// handle commands
			switch (UsartTextString[0]) {
	//		case 'e':
	//		case 'x':
	//			run = false;
	//			break;
			case 'N':
				GetNwkStatus();
				break;
			case 'O':
				SetCustomCFG();
				break;
			case 'C':
				GetCustomCFG();
				break;
			case 'A':
				ActivateABP();
				break;
			case 'R':
				Reactivate();
				break;
			case 'D':
				Deactivate();
				break;
			case 'S':
				SetRadioStack();
				break;
			case 'K':
				GetRadioStack();
				break;
			case 'G':
				GetSupportBand();
				break;
			case 'o':
				// get opmode
				GetOPMODE();
				break;
			case 'm':
				// set opmode
				SetOPMODE();
				break;
			case 'n':
				// clear opmode
				ClearOPMODE();
				break;
			case 'E':
				GetDevEUI();
				break;
			case 'e':
				// set dev eui
				SetDevEUI();
				break;

			case '-':
				// get rtc
				ClearRTCAlarm();
				break;

			case 't':
				// get rtc
				GetRTC();
				break;
			case '*':
				// set RTC
				SetRTC();
				break;
			case 'a':
				// get rtc alarm
				GetAlarm();
				break;
			case '+':
							// set rtc alarm
							SetAlarm();
							break;
			case 's':
				// device status
				GetDeviceStatus();
				break;

			case 'r':
				// reset device
				Reset();
				break;

			case 'f':
				// get firmware version
				GetFirmwareVersion();
				break;

			case 'i':
				// get device info
				GetDeviceInfo();
				break;

			case 'p':
				// ping device
				Ping();
				break;

			case 'j':
				// join network
				Join();
				break;

			case 'u':
				// send u-data
				SendUData();
				break;

			case 'c':
				// send c-data
				SendCData();
				break;

			case 'Z':
				// factory reset
				FactoryReset();
				break;
			case 'Q':
				//Get Link ADR
				GetLinkADR();
				break;
			case 'W':
				//Set Link ADR
				SetLinkADR();
				break;
			case ' ':
				USART_ShowMenu(
	#ifdef Q_OS_WIN
						comPort
	#endif
						);
				break;
			}

			HAL_UART_Abort(&hlpuart1);
			memset(&UsartTextString, 0x0, sizeof(UsartTextString));
		}

		/* Prepare to receive a text string via USART with UART_IT_RXNE */
		HAL_UART_Receive_IT(&hlpuart1, UsartTextString, USARTTEXTSTRINGSIZE);
	}
}
//------------------------------------------------------------------------------
//
//  ShowMenu
//
//  @brief: show main menu
//
//------------------------------------------------------------------------------
void USART_ShowMenu(
#ifdef Q_OS_WIN
		const char* comPort
#else
		void
#endif
		)
{
	USART_Transmit(&hlpuart1, "\n\r");
#ifdef Q_OS_WIN
    printf("------------------------------\n\r");
    printf("Using comport:%s\r\n", comPort);
#endif
    USART_Transmit(&hlpuart1, "------------------------------\n\r");
    USART_Transmit(&hlpuart1, "[SPACE] : show this menu\n\r");
    USART_Transmit(&hlpuart1, "[o]     : get OPMODE\n\r");
    USART_Transmit(&hlpuart1, "[m]     : set OPMODE\n\r");
    USART_Transmit(&hlpuart1, "[n]     : clear OPMODE\n\r");
	USART_Transmit(&hlpuart1, "[E]     : get DEV EUI\n\r");
//    USART_Transmit(&hlpuart1, "[e] <0123456789ABCDEF> : set DEV EUI\n\r");
    USART_Transmit(&hlpuart1, "[A]     : activate ABP\n\r");
    USART_Transmit(&hlpuart1, "[R]     : reactivate\n\r");
    USART_Transmit(&hlpuart1, "[D]     : deactivate\n\r");
    USART_Transmit(&hlpuart1, "[N]     : get network status\n\r");
    USART_Transmit(&hlpuart1, "[K]     : get Radio Stack Configuration\n\r");
    USART_Transmit(&hlpuart1, "[S]     : set Radio Stack Configuration\n\r");
    USART_Transmit(&hlpuart1, "[Q]     : get link ADR\n\r");
    USART_Transmit(&hlpuart1, "[W]     : set link ADR\n\r");
    USART_Transmit(&hlpuart1, "[G]     : get support band\n\r");
    USART_Transmit(&hlpuart1, "[C]     : get custom configuration\n\r");
    USART_Transmit(&hlpuart1, "[O]     : set custom configuration\n\r");
    USART_Transmit(&hlpuart1, "[t]     : get RTC\n\r");
    USART_Transmit(&hlpuart1, "[*]     : set RTC\n\r");
	USART_Transmit(&hlpuart1, "[a]     : get RTC alarm\n\r");
	USART_Transmit(&hlpuart1, "[+]     : set RTC alarm\n\r");
	USART_Transmit(&hlpuart1, "[-]     : clear RTC alarm\n\r");
    USART_Transmit(&hlpuart1, "[r]     : reset device\n\r");
    USART_Transmit(&hlpuart1, "[p]     : ping device\n\r");
    USART_Transmit(&hlpuart1, "[f]     : get firmware version\n\r");
    USART_Transmit(&hlpuart1, "[i]     : get device information\n\r");
    USART_Transmit(&hlpuart1, "[s]     : get device status\n\r");
    USART_Transmit(&hlpuart1, "[j]     : join network request\n\r");
    USART_Transmit(&hlpuart1, "[u]     : send unconfirmed radio message port 21 data 01020304\n\r");
    USART_Transmit(&hlpuart1, "[c]     : send confirmed radio message port 23 data 0A0B0C0D0E0F\n\r");
    USART_Transmit(&hlpuart1, "[Z]     : factory reset\n\r\n\r");
//    printf("[e|x]   : exit program\n\r");
    USART_Transmit(&hlpuart1, "-> enter command: ");

}

//------------------------------------------------------------------------------
//
//  GetNwkStatus
//
//  @brief: get Network Status
//
//------------------------------------------------------------------------------
static void
GetNwkStatus(void)
{
	USART_Transmit(&hlpuart1, "GetNwkStatus\n\r");

			// send get network status message
	WiMOD_LoRaWAN_GetNkwStatus();
}

//------------------------------------------------------------------------------
//
//  SetCustomCFG
//
//  @brief: Set Custom CFG
//
//------------------------------------------------------------------------------
static void
SetCustomCFG(void)
{
	int8_t payload = 0; //dBd
	USART_Transmit(&hlpuart1, "SetCustomCFG\n\r");

		// send set custom config message
	WiMOD_LoRaWAN_SetCustomConfig(payload);
}

//------------------------------------------------------------------------------
//
//  GetCustomCFG
//
//  @brief: Get Custom CFG
//
//------------------------------------------------------------------------------
static void
GetCustomCFG(void)
{
	USART_Transmit(&hlpuart1, "GetCustomCFG\n\r");

		// send get custom config message
	WiMOD_LoRaWAN_GetCustomConfig();
}

//------------------------------------------------------------------------------
//
//  ActivateABP
//
//  @brief: Activate ABP
//
//------------------------------------------------------------------------------
static void
ActivateABP(void)
{
	uint8_t payload[37] = {
			0x6c, 0x01, 0x00, 0x00, //Device address (LSB)
			0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,	//Network key (MSB)
			0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00, //App key (MSB)
			0x00 //End of payload
	};

	USART_Transmit(&hlpuart1, "Activate ABP\n\r");

	// send unconfirmed radio message
	WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_ACTIVATE_DEVICE_REQ, payload, 37);
}

//------------------------------------------------------------------------------
//
//  ReactivateABP
//
//  @brief: Reactivate ABP
//
//------------------------------------------------------------------------------
static void
Reactivate(void)
{
	USART_Transmit(&hlpuart1, "Reactivate\n\r");

	// send unconfirmed radio message
	WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_REACTIVATE_DEVICE_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  Deactivate
//
//  @brief: deactivate device
//
//------------------------------------------------------------------------------
static void
Deactivate(void)
{
	USART_Transmit(&hlpuart1, "Deactivate\n\r");

	// send unconfirmed radio message
	WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_DEACTIVATE_DEVICE_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  GetSupportBand
//
//  @brief: Get Support Band
//
//------------------------------------------------------------------------------
static void
GetSupportBand(void)
{
	USART_Transmit(&hlpuart1, "GetSupportBand\n\r");
	WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_GET_SUPPORTED_BANDS_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  GetRadioStack
//
//  @brief: Get Radio Stack
//
//------------------------------------------------------------------------------
static void
GetRadioStack(void)
{
	USART_Transmit(&hlpuart1, "GetRadioStack\n\r");
		WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_GET_RSTACK_CONFIG_REQ, NULL, 0);
}
//------------------------------------------------------------------------------
//
//  SetRadioStack
//
//  @brief: Set Radio Stack
//
//------------------------------------------------------------------------------
static void
SetRadioStack(void)
{
	USART_Transmit(&hlpuart1, "SetRadioStack\n\r");


	    UINT8 data[7];

	    data[0] = 0x5;	//Default Data Rate Index
	    data[1] = 0x10;	//Default TX Power Level (EIRP)
	    data[2] = 0b11000111;
//	    Bit 0: 0 = Adaptive Data Rate disabled	    1 = Adaptive Data Rate enabled
//	    Bit 1: 0 = Duty Cycle Control disabled 1 = Duty Cycle Control enabled (Customer Mode required)
//	    Bit 2: 0 = Class A selected 1 = Class C selected
//	    Bit 6: 0 = standard RF packet output format	    1 = extended RF packet output format: Tx/Rx channel info attached
//	    Bit 7: 0 = Rx MAC Command Forwarding disabled 	    1 = Rx MAC Command Forwarding enabled
	    data[3] = 0x0;	//Power Saving Mode 0x00 : off	    0x01 : automatic
	    data[4] = 0x7;	//Number of Retransmissions
	    data[5] = 0x12;	//Band Index AS923TH
	    data[6] = 0xf;// Header MAC Cmd Capacity

	    // send unconfirmed radio message
	    WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_SET_RSTACK_CONFIG_REQ, data, 7);
}

//------------------------------------------------------------------------------
//
//  GetDevEUI
//
//  @brief: get DEV EUI 64 bit
//
//------------------------------------------------------------------------------
static void
GetDevEUI(void)
{
	USART_Transmit(&hlpuart1, "GetDevEUI\n\r");
	WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_GET_DEVICE_EUI_REQ, NULL, 0);
}

//------------------------------------------------------------------------------
//
//  SetDevEUI
//
//  @brief: set DEV EUI 64 bit
//
//------------------------------------------------------------------------------
static void
SetDevEUI(void)
{
	//TODO: Set Device EUI
	//Valid Dev EUI number
//	uint8_t eui = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //70 B3 D5 8F F0 03 8D 39
	int i;
	for (i = 2; i < 18; ++i) {
		if((isxdigit(UsartTextString[i]) != true))
		{
			USART_Transmit(&hlpuart1, "bad input dev eui!!!\n\r");
			return;
		}
	}
//	WiMOD_LoRaWAN_Msg_Req(LORAWAN_MSG_SET_DEVICE_EUI_REQ, &UsartTextString[2], 8);
}
//------------------------------------------------------------------------------
//
//  SetOPMODE
//
//  @brief: set OPMODE
//	Index Description
//	0 Standard Application Mode / Default Mode
//	1 Reserved
//	2 Reserved
//	3 Customer Mode
//
//------------------------------------------------------------------------------
static void
SetOPMODE(void)
{
	USART_Transmit(&hlpuart1, "set OPMODE\n\r");

    WiMOD_LoRaWAN_SetOPMODE(3);
}

//------------------------------------------------------------------------------
//
//  ClearOPMODE
//
//  @brief: clear OPMODE
//
//------------------------------------------------------------------------------
static void
ClearOPMODE(void)
{
	USART_Transmit(&hlpuart1, "clear OPMODE\n\r");

    WiMOD_LoRaWAN_SetOPMODE(0);
}

//------------------------------------------------------------------------------
//
//  GetOPMODE
//
//  @brief: get OPMODE
//
//------------------------------------------------------------------------------
static void
GetOPMODE(void)
{
	USART_Transmit(&hlpuart1, "get OPMODE\n\r");

    WiMOD_LoRaWAN_GetOPMODE();
}

//------------------------------------------------------------------------------
//
//  ClearRTCAlarm
//
//  @brief: clear RTC alarm
//
//------------------------------------------------------------------------------
static void
ClearRTCAlarm(void)
{
	USART_Transmit(&hlpuart1, "clear RTC alarm\n\r");

    WiMOD_LoRaWAN_ClearRTCAlarm();
}

//------------------------------------------------------------------------------
//
//  GetRTC
//
//  @brief: get RTC
//
//------------------------------------------------------------------------------
static void
GetRTC(void)
{
	USART_Transmit(&hlpuart1, "get RTC\n\r");

    WiMOD_LoRaWAN_GetRTC();
}

//------------------------------------------------------------------------------
//
//  SetRTC
//
//  @brief: set RTC
//
//------------------------------------------------------------------------------
static void
SetRTC(void)
{
	USART_Transmit(&hlpuart1, "set RTC\n\r");
	uint32_t time = 0;
	//2018-5-20 [12:30:30]
	//Year
	time |= 18;
	time <<= 6;
	//Days
	time |= 20;
	time <<= 5;
	//Hours
	time |= 12;
	time <<= 5;
	//Month
	time |= 5;
	time <<= 4;
	//Minutes
	time |= 30 ;
			time <<=  6;
	//Second
	time |= 30;


    WiMOD_LoRaWAN_SetRTC(&time);
}

//------------------------------------------------------------------------------
//
//  GetAlarm
//
//  @brief: get RTC Alarm device
//
//------------------------------------------------------------------------------
static void
GetAlarm(void)
{
	USART_Transmit(&hlpuart1, "get RTC alarm\n\r");

    WiMOD_LoRaWAN_GetRTCAlarm();
}

//------------------------------------------------------------------------------
//
//  SetAlarm
//
//  @brief: set RTC Alarm device
//
//------------------------------------------------------------------------------
static void
SetAlarm(void)
{
	USART_Transmit(&hlpuart1, "set RTC alarm\n\r");
	uint8_t payload[4] = {0};

	payload[0] = 0x1;
	//0x00 : single alarm
	//0x01 : daily repeated alarm
	payload[1] = 12;	//Hour
	payload[2] = 30;	//Minute
	payload[3] = 30;	//Second
	WiMOD_LoRaWAN_SetRTCAlarm(payload);
}


//------------------------------------------------------------------------------
//
//  Ping
//
//  @brief: ping device
//
//------------------------------------------------------------------------------
static void
Ping(void)
{
	USART_Transmit(&hlpuart1, "ping request\n\r");

    WiMOD_LoRaWAN_SendPing();
}

//------------------------------------------------------------------------------
//
//  GetFirmwareVersion
//
//  @brief: get firmware version
//
//------------------------------------------------------------------------------
static void
GetFirmwareVersion(void)
{
	USART_Transmit(&hlpuart1, "get firmware version\n\r");

    WiMOD_LoRaWAN_GetFirmwareVersion();
}

//------------------------------------------------------------------------------
//
//  GetDeviceInfo
//
//  @brief: get device information
//
//------------------------------------------------------------------------------
static void
GetDeviceInfo(void)
{
	USART_Transmit(&hlpuart1, "get device information\n\r");

    WiMOD_LoRaWAN_GetDeviceInfo();
}

//------------------------------------------------------------------------------
//
//  GetDeviceStatus
//
//  @brief: get status device
//
//------------------------------------------------------------------------------
static void
GetDeviceStatus(void)
{
	USART_Transmit(&hlpuart1, "device status\n\r");

    WiMOD_LoRaWAN_GetDeviceStatus();
}

//------------------------------------------------------------------------------
//
//  Reset
//
//  @brief: reset device
//
//------------------------------------------------------------------------------
static void
Reset(void)
{
	USART_Transmit(&hlpuart1, "reset device\n\r");

    WiMOD_LoRaWAN_Reset();
}

//------------------------------------------------------------------------------
//
//  Join
//
//  @brief: ping device
//
//------------------------------------------------------------------------------
static void
Join(void)
{
	USART_Transmit(&hlpuart1, "join network request\n\r");

    WiMOD_LoRaWAN_JoinNetworkRequest();
}
//------------------------------------------------------------------------------
//
//  SendUData
//
//  @brief: send unconfirmed radio message
//
//------------------------------------------------------------------------------
static void
SendUData(void)
{
	USART_Transmit(&hlpuart1, "send U-Data\n\r");

    // port 0x21 (33)
    UINT8 port = 0x21;

    UINT8 data[4];

    data[0] = 0x01;
    data[1] = 0x02;
    data[2] = 0x03;
    data[3] = 0x04;

    // send unconfirmed radio message
    WiMOD_LoRaWAN_SendURadioData(port, data, 4);
}

//------------------------------------------------------------------------------
//
//  SendCData
//
//  @brief: send confirmed radio message
//
//------------------------------------------------------------------------------
static void
SendCData(void)
{
	USART_Transmit(&hlpuart1, "send C-Data\n\r");

    // port 0x23 (35)
    UINT8 port = 0x23;

    UINT8 data[6];

    data[0] = 0x0A;
    data[1] = 0x0B;
    data[2] = 0x0C;
    data[3] = 0x0D;
    data[4] = 0x0E;
    data[5] = 0x0F;

    // send unconfirmed radio message
    WiMOD_LoRaWAN_SendCRadioData(port, data, 6);
}

//------------------------------------------------------------------------------
//
//  FactoryReset
//
//  @brief: factory reset
//
//------------------------------------------------------------------------------
static void
FactoryReset(void)
{
	USART_Transmit(&hlpuart1, "FactoryReset\n\r");
	WiMOD_LoRaWAN_FactoryReset();
}

//------------------------------------------------------------------------------
//
//  GetLinkADR
//
//  @brief: get link ADR
//
//------------------------------------------------------------------------------
static void		GetLinkADR(void)
{
	USART_Transmit(&hlpuart1, "GetLinkADR\n\r");
	WiMOD_LoRaWAN_GetLinkADR();
}

//------------------------------------------------------------------------------
//
//  SetLinkADR
//
//  @brief: set link ADR
//
//------------------------------------------------------------------------------
static void		SetLinkADR(void)
{
	USART_Transmit(&hlpuart1, "SetLinkADR\n\r");
	uint8_t payload = 0;
//	LinkADRReq Option:
//	0: LoRaWAN v1.0.2
//	1: Semtech proposal
//	2: KPN/Actility proposal
	WiMOD_LoRaWAN_SetLinkADR(&payload);
}
//------------------------------------------------------------------------------
// end of file
//------------------------------------------------------------------------------
