//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************


//*****************************************************************************
//
// Application Name     -   SSL Demo
// Application Overview -   This is a sample application demonstrating the
//                          use of secure sockets on a CC3200 device.The
//                          application connects to an AP and
//                          tries to establish a secure connection to the
//                          Google server.
// Application Details  -
// docs\examples\CC32xx_SSL_Demo_Application.pdf
// or
// http://processors.wiki.ti.com/index.php/CC32xx_SSL_Demo_Application
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup ssl
//! @{
//
//*****************************************************************************

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_nvic.h"
#include "hw_types.h"
#include "interrupt.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "timer.h"
#include "uart.h"
#include "utils.h"
#include "spi.h"
#include "i2c_if.h"
#include "systick.h"

//Common interface includes
#include "uart_if.h"
#include "pinmux.h"
#include "timer_if.h"
#include "gpio_if.h"
#include "common.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "oled_test.h"
#include "utils/network_utils.h"
#include "simplelink.h"
// Custom includes
#include "utils/network_utils.h"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

// SysTick stuff
#define SYSCLKFREQ         80000000ULL  // 80 MHz
#define TICKS_TO_US(ticks) ( ((ticks) / (SYSCLKFREQ/1000000ULL)) )
#define US_TO_TICKS(us)    ( ((us)   * (SYSCLKFREQ/1000000ULL)) )
#define SYSTICK_RELOAD_VAL 3200000UL
#define MAX_PULSES         48

// Times (microseconds) for your IR protocol
#define START_BIT_MIN_US   3000  // ~3.0ms
#define START_BIT_MAX_US   6000  // ~5.0ms
#define BIT_0_MIN_US       300
#define BIT_0_MAX_US       1200
#define BIT_1_MIN_US       1300
#define BIT_1_MAX_US       1800
#define SPI_IF_BIT_RATE    100000
#define RECEIVED_BUFFER_SIZE 256
char rxBuffer[RECEIVED_BUFFER_SIZE];
int rxIndex = 0;
#define PIN_LENGTH        4    // Number of digits in the PIN
#define CORRECT_PIN       "9" // The correct PIN
#define OLED_WIDTH        128
#define OLED_HEIGHT       128
#define TOP_AREA_HEIGHT   64    // For sent/received messages
#define BOT_AREA_HEIGHT   64    // For composing messages
volatile bool g_oledUpdateNeeded = false;
#define DATE                12    /* Current Date */
#define MONTH               3    /* Month 1-12 */
#define YEAR                2025  /* Current year */
#define HOUR                20    /* Time - hours */
#define MINUTE              11    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

#define APPLICATION_NAME      "SSL"
#define APPLICATION_VERSION   "SQ24"
#define SERVER_NAME           "<add your endpoint here>" // CHANGE ME
#define GOOGLE_DST_PORT       8443


#define POSTHEADER "POST /things/<add your thing here>/shadow HTTP/1.1\r\n"           // CHANGE ME
#define HOSTHEADER  "Host: <add your endpoint here>\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define DATA1 "{" \
            "\"state\": {\r\n"                                              \
                "\"desired\" : {\r\n"                                       \
                    "\"default\" :\""                                       \
                        "BIKE IS IN MOTION, "                                     \
                        "Longitude Latitude"                                \
                        "\"\r\n"                                            \
                "}"                                                         \
            "}"                                                             \
        "}\r\n\r\n"

#define UART1_BAUDRATE     9600
#define UART1_PERIPH       PRCM_UARTA1
#define UART1_BASE_ADDR    UARTA1_BASE
#define GPS_UART           UARTA1_BASE


#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

volatile int     systick_cnt = 0;
volatile uint64_t last_time  = 0;
volatile uint32_t lastActivityTime = 0;
// We store all captured time differences here
volatile uint64_t time_buffer[MAX_PULSES];
volatile int       pulse_count       = 0;
volatile int       instruction_ready = 0;
volatile int       remoteFlag = 0;
volatile uint64_t ticks_buffer[MAX_PULSES];
volatile int ticks_counter = 0;
volatile uint64_t pulse_buffer[MAX_PULSES]; //changed this to uint
volatile int start_bit;
volatile uint64_t start_bit_time_difference;
volatile uint64_t data;
static char entered_pin[PIN_LENGTH + 1]; // Buffer to store entered PIN
static int pin_index = 0;  // Track PIN entry index
static bool pin_matched = false; // Track if PIN is correct
static bool isBikeLocked = false;
long lRetVal = -1;
volatile bool motionDetectionEnabled;
volatile bool motionDetected;  // Tracks if motion has been detected

bool emailSent = false;

char receivedBuffer[64];
//long lRetVal = -1;
int receive = 0;


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static int set_time();
static void BoardInit(void);
//static int http_post(int);

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void) {
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}




//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

typedef struct PinSetting {
    unsigned long port;
    unsigned int  pin;
} PinSetting;

static const PinSetting gpioread = { .port = GPIOA1_BASE, .pin = 0x10 };


#define MULTI_TAP_THRESHOLD_MS 500
//*****************************************************************************
// Forward Declarations
//*****************************************************************************
static void BoardInit(void);
static void SysTickInit(void);
static void IR_InterruptHandler(void);
static inline void SysTickReset(void);


char received[50];

typedef enum {
    STATE_LOCKED,
    STATE_ALERT,
    STATE_UNLOCKED
} SystemState;

volatile SystemState systemState = STATE_LOCKED;
volatile bool motionDetected = false;

#define SW3_PORT GPIOA1_BASE
#define SW3_PIN 0x20


void UARTIntHandler(void) {
    UARTIntClear(UARTA1_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX);

    while(UARTCharsAvail(UARTA1_BASE)) {
        char receivedChar = UARTCharGet(UARTA1_BASE);
        if(receivedChar == '\0') {
            fillRect(0, 0, 128, 64, BLACK);
            setCursor(0, 0);
            Outstr(received);
            receive = 0;
            memset(received, 0, sizeof(received));
        } else {
            received[receive++] = receivedChar;
        }
    }
}




static void IR_InterruptHandler(void) {

    unsigned long ulStatus = MAP_GPIOIntStatus(gpioread.port, true);
    MAP_GPIOIntClear(gpioread.port, ulStatus);
    remoteFlag = 1;

    uint64_t current_time = MAP_SysTickValueGet();
    uint64_t time_difference;
    uint64_t time_difference_ticks = SYSTICK_RELOAD_VAL - MAP_SysTickValueGet();
    time_difference = TICKS_TO_US(time_difference_ticks);


    if(time_difference >= 4500 && time_difference <= 5100) {
        start_bit = 1;
        start_bit_time_difference = time_difference;
    }
    if(start_bit) {
        if(time_difference >= 1100 && time_difference <= 1950){
            //bit is a 1
            time_buffer[pulse_count] = time_difference;
            pulse_buffer[pulse_count] = 1;
            data = data | (1<<pulse_count);
            pulse_count++;

        } else if(time_difference >= 600 && time_difference <= 900){
            //bit is a 0
            time_buffer[pulse_count] = time_difference;
            pulse_buffer[pulse_count] = 0;
            pulse_count++;
        }
    } else {

    }

    if(pulse_count>=48) {
        instruction_ready = 1;
        pulse_count=0;
        ticks_counter=0;
        start_bit = 0;
    }
    ticks_counter++;
    SysTickReset();
}
static void updateOLEDMessage(const char *msg, uint16_t color) {
    fillRect(0,90,OLED_WIDTH, 30, BLACK);
    setTextSize(2);
    setTextColor(color, BLACK);
    setCursor(10,90);

    char display[PIN_LENGTH + 1];

    int i;

    for(i = 0; i < PIN_LENGTH; i++){
        display[i] = (i < strlen(msg)) ? msg[i] : '_';
    }
    display[PIN_LENGTH] = '\0';
    Outstr(display);
}
static void showSuccessAnimation() {
    int i;
    for (i = 0; i < 2;i++) {
        fillRect(0, 0, OLED_WIDTH, OLED_HEIGHT, GREEN);
        delay(100);
        fillRect(0, 0, OLED_WIDTH, OLED_HEIGHT, BLACK);
        delay(100);
    }
}

static void showFailureAnimation() {
    int i;
    for (i = 0; i < 2;i++) {
        fillRect(0, 0, OLED_WIDTH, OLED_HEIGHT, RED);
        delay(100);
        fillRect(0, 0, OLED_WIDTH, OLED_HEIGHT, BLACK);
        delay(100);
    }
}

static void checkPIN(void) {
    entered_pin[PIN_LENGTH] = '\0';

    fillScreen(BLACK);
    setTextSize(2);

    if (strcmp(entered_pin, CORRECT_PIN) == 0) {
        setTextColor(GREEN, BLACK);

        // Centered "ACCESS GRANTED"
        setCursor(5, 50);
        //setTextSize(2);
        Outstr("CORRECT");
        Report("PIN CORRECT");
        systemState = STATE_UNLOCKED;
        //showSuccessAnimation();
        MAP_UtilsDelay(8000000); // brief delay for visibility

        fillScreen(BLACK);
        // Centered "UNLOCKED"
        setCursor(20, 50);
        Outstr("UNLOCKED");
        //remoteFlag = 0;

        pin_matched = true;
        isBikeLocked = false;
    } else {
        setTextColor(RED, BLACK);

        // Centered "ACCESS DENIED"
        setCursor(10, 50);
        Outstr("INCORRECT");
        pin_matched = false;
        Report("PIN IS INCORRECT");
        systemState = STATE_LOCKED;
        //remoteFlag = 0;

        //showFailureAnimation();
        MAP_UtilsDelay(8000000); // delay for visibility
        resetOLEDToLockedScreen();


    }

    // Clear the entered PIN buffer
    memset(entered_pin, 0, sizeof(entered_pin));
    pin_index = 0;
}

// Update the top half of the screen with an incoming message.
void updateOLEDIncomingMessage(const char *msg) {
    //Report("HERE!");
    fillRect(0, 0, OLED_WIDTH, TOP_AREA_HEIGHT, BLACK);  // Clear OLED screen
    setCursor(0, 0);
    setTextSize(2);
    setTextColor(WHITE, BLACK);  // Make sure text is visible
    Outstr(msg);  // Display full message
}



void updateOLEDOutgoingMessage(const char *msg) {
    //Report("msg: %c", msg);
    fillRect(0, TOP_AREA_HEIGHT, OLED_WIDTH, BOT_AREA_HEIGHT, BLACK);
    setCursor(0, TOP_AREA_HEIGHT);
    setTextSize(2);
    Outstr(msg);
}
int DisplayBuffer(unsigned char *pucDataBuf, unsigned char ucLen)
{
    unsigned char ucBufIndx = 0;
    int buffer;
    while(ucBufIndx < ucLen)
    {
        buffer = (int) pucDataBuf[ucBufIndx];
        if(buffer & 0x80) {
            buffer = buffer | 0xffffff00;
        }
        ucBufIndx++;
    }
    return buffer;
}

//
//! Parses the read command parameters and invokes the I2C APIs
//!
//! \param pcInpString pointer to the user command parameters
//!
//! This function
//!    1. Parses the read command parameters.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ProcessReadCommand(char *pcInpString)
{
    unsigned char ucDevAddr, ucLen;
    unsigned char aucDataBuf[256];
    char *pcErrPtr;
    int iRetVal;

    //
    // Get the device address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    //
    // Get the length of data to be read
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);
    //RETERR_IF_TRUE(ucLen > sizeof(aucDataBuf));

    //
    // Read the specified length of data
    //
    iRetVal = I2C_IF_Read(ucDevAddr, aucDataBuf, ucLen);

    if(iRetVal == SUCCESS)
    {
        printf("I2C Read complete\n\r");

        //
        // Display the buffer over UART on successful write
        //
        DisplayBuffer(aucDataBuf, ucLen);
    }
    else
    {
        printf("I2C Read failed\n\r");
        return FAILURE;
    }

    return SUCCESS;
}

//****************************************************************************
//
//! Parses the readreg command parameters and invokes the I2C APIs
//! i2c readreg 0x<dev_addr> 0x<reg_offset> <rdlen>
//!
//! \param pcInpString pointer to the readreg command parameters
//!
//! This function
//!    1. Parses the readreg command parameters.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ProcessReadRegCommand(char *pcInpString)
{
    unsigned char ucDevAddr, ucRegOffset, ucRdLen;
    unsigned char aucRdDataBuf[256];
    char *pcErrPtr;

    //
    // Get the device address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    //
    // Get the register offset address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucRegOffset = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);

    //
    // Get the length of data to be read
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucRdLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);
    //RETERR_IF_TRUE(ucLen > sizeof(aucDataBuf));

    //
    // Write the register address to be read from.
    // Stop bit implicitly assumed to be 0.
    //
    RET_IF_ERR(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0));

    //
    // Read the specified length of data
    //
    RET_IF_ERR(I2C_IF_Read(ucDevAddr, &aucRdDataBuf[0], ucRdLen));

    //
    // Display the buffer over UART on successful readreg
    //
    return DisplayBuffer(aucRdDataBuf, ucRdLen);
}

//****************************************************************************
//
//! Parses the writereg command parameters and invokes the I2C APIs
//! i2c writereg 0x<dev_addr> 0x<reg_offset> <wrlen> <0x<byte0> [0x<byte1> ...]>
//!
//! \param pcInpString pointer to the readreg command parameters
//!
//! This function
//!    1. Parses the writereg command parameters.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ProcessWriteRegCommand(char *pcInpString)
{
    unsigned char ucDevAddr, ucRegOffset, ucWrLen;
    unsigned char aucDataBuf[256];
    char *pcErrPtr;
    int iLoopCnt = 0;

    //
    // Get the device address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);

    //
    // Get the register offset to be written
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucRegOffset = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    aucDataBuf[iLoopCnt] = ucRegOffset;
    iLoopCnt++;

    //
    // Get the length of data to be written
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucWrLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);

    //
    // Get the bytes to be written
    //
    for(; iLoopCnt < ucWrLen + 1; iLoopCnt++)
    {
        //
        // Store the data to be written
        //
        pcInpString = strtok(NULL, " ");
        RETERR_IF_TRUE(pcInpString == NULL);
        aucDataBuf[iLoopCnt] =
                (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    }
    //
    // Write the data values.
    //
    RET_IF_ERR(I2C_IF_Write(ucDevAddr,&aucDataBuf[0],ucWrLen+1,1));

    printf("I2C Write To address complete\n\r");

    return SUCCESS;
}

//****************************************************************************
//
//! Parses the write command parameters and invokes the I2C APIs
//!
//! \param pcInpString pointer to the write command parameters
//!
//! This function
//!    1. Parses the write command parameters.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ProcessWriteCommand(char *pcInpString)
{
    unsigned char ucDevAddr, ucStopBit, ucLen;
    unsigned char aucDataBuf[256];
    char *pcErrPtr;
    int iRetVal, iLoopCnt;

    //
    // Get the device address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);

    //
    // Get the length of data to be written
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);
    //RETERR_IF_TRUE(ucLen > sizeof(aucDataBuf));

    for(iLoopCnt = 0; iLoopCnt < ucLen; iLoopCnt++)
    {
        //
        // Store the data to be written
        //
        pcInpString = strtok(NULL, " ");
        RETERR_IF_TRUE(pcInpString == NULL);
        aucDataBuf[iLoopCnt] =
                (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    }

    //
    // Get the stop bit
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucStopBit = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);

    //
    // Write the data to the specified address
    //
    iRetVal = I2C_IF_Write(ucDevAddr, aucDataBuf, ucLen, ucStopBit);
    if(iRetVal == SUCCESS)
    {
        printf("I2C Write complete\n\r");
    }
    else
    {
        printf("I2C Write failed\n\r");
        return FAILURE;
    }

    return SUCCESS;
}

//****************************************************************************
//
//! Parses the user input command and invokes the I2C APIs
//!
//! \param pcCmdBuffer pointer to the user command
//!
//! This function
//!    1. Parses the user command.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int ParseNProcessCmd(char *pcCmdBuffer)
{
    char *pcInpString;
    int iRetVal;

    pcInpString = strtok(pcCmdBuffer, " \n\r");
    if(pcInpString != NULL)

    {

        if(!strcmp(pcInpString, "read"))
        {
            //
            // Invoke the read command handler
            //
            iRetVal = ProcessReadCommand(pcInpString);
        }
        else if(!strcmp(pcInpString, "readreg"))
        {
            //
            // Invoke the readreg command handler
            //
            iRetVal = ProcessReadRegCommand(pcInpString);
        }
        else if(!strcmp(pcInpString, "writereg"))
        {
            iRetVal = ProcessWriteRegCommand(pcInpString);
        }
        else if(!strcmp(pcInpString, "write"))
        {
            //
            // Invoke the write command handler
            //
            iRetVal = ProcessWriteCommand(pcInpString);
        }
        else
        {
            printf("Unsupported command\n\r");
            return FAILURE;
        }
    }

    return iRetVal;
}

//*****************************************************************************
//
// SysTickInit
//
//*****************************************************************************
static inline void SysTickReset(void) {
    HWREG(NVIC_ST_CURRENT) = 1;
}

static void SysTickHandler(void) {
    systick_cnt++;
}

static void SysTickInit(void) {
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);
    MAP_SysTickIntRegister(SysTickHandler);
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();

    SysTickReset();
}

static char* createJsonPayload(const char *message)
{
    static char jsonBuf[256];
    sprintf(jsonBuf,
        "{"
           "\"state\": {"
               "\"desired\": {"
                   "\"default\":\"%s\""
               "}"
           "}"
        "}",
        message
    );

    return jsonBuf;
}

static int http_post(int iTLSSockID, const char *message)
{
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[50];
    char *pcBufHeaders;
    int  lRetVal = 0;
    char *jsonPayload = createJsonPayload(message);
    int dataLength    = strlen(jsonPayload);

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(pcBufHeaders);

    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(pcBufHeaders);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(pcBufHeaders);
    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(pcBufHeaders);
    strcpy(pcBufHeaders, CLHEADER1);
    pcBufHeaders += strlen(pcBufHeaders);

    sprintf(cCLLength, "%d", dataLength);
    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(pcBufHeaders);

    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(pcBufHeaders);
    strcpy(pcBufHeaders, jsonPayload);
    pcBufHeaders += strlen(pcBufHeaders);

    UART_PRINT("\r\n--- HTTP POST Request Start ---\r\n");
    UART_PRINT(acSendBuff);
    UART_PRINT("\r\n--- HTTP POST Request End ---\r\n");
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);

    if(lRetVal < 0)
    {
        UART_PRINT("POST failed. Error Number: %i\n\r", lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, acRecvbuff, sizeof(acRecvbuff), 0);
    if(lRetVal < 0)
    {
        UART_PRINT("Receive failed. Error Number: %i\n\r", lRetVal);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    else
    {
        acRecvbuff[lRetVal] = '\0';
        UART_PRINT("Response:\n\r%s\n\r", acRecvbuff);
    }

    return 0;
}


void resetOLEDToLockedScreen(void)
{
    // --- Define colors ---
    // A darker, more appealing background (midnight blue)
    unsigned int background = Color565(25, 25, 112);
    // A nice silver for the lock body and shackle
    unsigned int silver     = Color565(192, 192, 192);
    unsigned int white      = WHITE;
    unsigned int black      = BLACK;

    // --- Clear the display with the new background color ---
    fillScreen(background);

    // Calculate screen center
    int centerX = SSD1351WIDTH / 2;
    int centerY = SSD1351HEIGHT / 2;
    int bodySize  = 60;
    int bodyX     = centerX - (bodySize / 2);
    int bodyY     = centerY - (bodySize / 2);

    fillRect(bodyX, bodyY, bodySize, bodySize, silver);

    // Shackle dimensions
    int shackleThickness  = 4;   // how thick each bar is
    int shackleHeight     = 25;  // total height of the U shape
    int shackleInnerWidth = 20;  // space between the left and right bars
    int shackleOuterWidth = shackleInnerWidth + (2 * shackleThickness);

    // The top of the shackle sits slightly above the lock body so it overlaps with no gap.
    // We�ll offset by +1 so it visually merges with the top of the lock.
    int topBarX = centerX - (shackleOuterWidth / 2);
    int topBarY = bodyY - shackleHeight + 1;

    // Draw the top horizontal bar
    fillRect(topBarX, topBarY, shackleOuterWidth, shackleThickness, silver);

    // The vertical bars start below the top bar so it looks like one continuous piece.
    int verticalBarHeight = shackleHeight - shackleThickness;
    int leftBarX = topBarX;
    int leftBarY = topBarY + shackleThickness;

    // Draw the left vertical bar
    fillRect(leftBarX, leftBarY, shackleThickness, verticalBarHeight, silver);

    // Draw the right vertical bar
    int rightBarX = centerX + (shackleInnerWidth / 2);
    fillRect(rightBarX, leftBarY, shackleThickness, verticalBarHeight, silver);

    int keyholeRadius = 5;
    int keyholeX      = centerX;
    // Position slightly above the midpoint of the lock body
    int keyholeY      = bodyY + (bodySize / 2) - 8;

    fillCircle(keyholeX, keyholeY, keyholeRadius, black);

    int stemWidth  = 8;
    int stemHeight = 10;
    fillRect(keyholeX - (stemWidth / 2),
             keyholeY + keyholeRadius - 1,
             stemWidth,
             stemHeight,
             black);

    // --- Draw the "LOCKED" text below the lock ---
    setTextSize(2);
    setTextColor(white, background);
    // "LOCKED" has 6 chars, each ~12 px wide at text size 2 => 72 px total
    int textWidth = 72;
    int textX     = centerX - (textWidth / 2);
    int textY     = bodyY + bodySize + 10;
    setCursor(textX, textY);
    Outstr("LOCKED");

    // Update the system state to reflect that the bike is locked
    isBikeLocked = true;
}

static bool readOneGPSLine(char *gpsLine, int maxLength) {
    bool started = false;
    int index = 0;
    while(1) {
        if(MAP_UARTCharsAvail(UARTA1_BASE)) {
            char c = (char)MAP_UARTCharGet(UARTA1_BASE);
            //Report("%c",c);
            if(c == '$') {
                started = true;
                index = 0;
                gpsLine[index++] = c;
            } else if(started) {
                if(index < (maxLength-1)){
                    gpsLine[index++] = c;
                }
                if(c=='\n' || c=='\r'){
                    gpsLine[index] = '\0';
                    //Report("LINE RECEIVED");
                    return true;
                }
            }
        }
    }
    return false;
}
static bool parseGPSLine(const char *nmeaLine, double *outLat, double *outLon) {
    // Ensure it's the correct NMEA sentence
    if (strncmp(nmeaLine, "$GPRMC", 6) != 0) {
        return false;
    }

    char buffer[128];
    strncpy(buffer, nmeaLine, sizeof(buffer));
    buffer[sizeof(buffer)-1] = '\0'; // Ensure null-termination

    char latitude[20] = {0};
    char longitude[20] = {0};
    strncpy(latitude, buffer + 19, 10);
    latitude[10] = '\0'; // Null-terminate the string
    strncpy(longitude, buffer + 32, 11);
    longitude[11] = '\0'; // Null-terminate the string

    char latitudeLetter = buffer[30];  // 'N' or 'S'
    char longitudeLetter = buffer[44]; // 'E' or 'W'
    char finalMessage[256];
    //const char* message = "HELLO";
    snprintf(finalMessage, sizeof(finalMessage),
             "Your bike is at Latitude %s %c Longitude %s %c",
             latitude, latitudeLetter, longitude, longitudeLetter);
    Report("MESSAGE: %s",
           finalMessage);

    // Send the message to AWS IoT via HTTP POST
    if (!emailSent) {
        http_post(lRetVal, finalMessage);
        emailSent = true;
    }

    return true;
}
void alertOLED(void)
{

    int y;
    for (y = 0; y < SSD1351HEIGHT; y++)
    {
        float ratio = (float)y / (float)(SSD1351HEIGHT - 1);
        // Interpolate red channel from 128 -> 255
        int r = 128 + (int)(127.0f * ratio);
        int g = 0;
        int b = 0;

        unsigned int color = Color565(r, g, b);
        fillRect(0, y, SSD1351WIDTH, 1, color);
    }

    unsigned int yellow  = Color565(255, 255, 0);
    unsigned int black   = Color565(0, 0, 0);

    setTextSize(2);

    const char* alertMsg = "ALERT!";
    // ~12 px per char at size=2
    int alertTextWidth   = strlen(alertMsg) * 12;
    int centerX          = SSD1351WIDTH / 2;
    int centerY          = SSD1351HEIGHT / 2;

    // Position the text near the top
    int alertX = centerX - (alertTextWidth / 2);
    int alertY = 10; // 10 px down from the top

    // Draw a black highlight box behind "ALERT!"
    int pad        = 4;
    int textHeight = 16; // size=2 => about 16px tall
    int boxW       = alertTextWidth + (2 * pad);
    int boxH       = textHeight + (2 * pad);
    int boxX       = alertX - pad;
    int boxY       = alertY - pad;
    fillRect(boxX, boxY, boxW, boxH, black);

    // Print "ALERT!" in yellow over the black box
    setTextColor(yellow, black);
    setCursor(alertX, alertY);
    Outstr(alertMsg);
    // We'll make a black circle, then place a big yellow exclamation mark inside.
    int circleRadius = 30; // adjust as desired
    fillCircle(centerX, centerY, circleRadius, black);
    int barWidth    = 8;
    int barHeight   = 20;
    int barX        = centerX - (barWidth / 2);
    int barY        = centerY - (barHeight / 2) - 5;  // shift upward a bit
    fillRect(barX, barY, barWidth, barHeight, yellow);

    // Dot below the bar
    int dotSize     = 6;
    int dotX        = centerX - (dotSize / 2);
    int dotY        = barY + barHeight + 4; // some gap below
    fillRect(dotX, dotY, dotSize, dotSize, yellow);

    unsigned int white = Color565(255, 255, 255);
    setTextSize(1);
    setTextColor(white, black); // white text on black background

    char gpsLine[128];
    double latitude, longitude;
    while(1) {
        if(GPIOPinRead(SW3_PORT, SW3_PIN) == SW3_PIN) {
            Report("RESET BUTTON PRESSED..RESETTING SYSTEM");
            resetButtonPressed();
            emailSent = false;
            break;
        }
        if(readOneGPSLine(gpsLine, sizeof(gpsLine))) {
            if(parseGPSLine(gpsLine, &latitude, &longitude)){
                //Report("WORKSSSSSSSSSSS");
//                Report("GPS fix found! Lat=%.6f, Lon=%.6f\n\r", latitude, longitude);
                //break;
            }
        }
        MAP_UtilsDelay(1200000);
    }
}

void checkForMotion(void)
{
    Report("CHECKING FOR MOTION....");
    // Define a 15-second time window
    const double CHECK_DURATION = 15.0;  // seconds
    time_t startTime, currentTime;

    // Record when we started checking for motion
    time(&startTime);

    while (1)
    {
        // Read sensor data
        int xSpeed, ySpeed;
        char xMessage[256] = "readreg 0x18 0x5 1 \n\r"; // Read x-axis speed
        char yMessage[256] = "readreg 0x18 0x3 1 \n\r"; // Read y-axis speed

        xSpeed = ParseNProcessCmd(xMessage);
        ySpeed = ParseNProcessCmd(yMessage);

        // Check if the absolute speed is above our motion threshold
        int motionThreshold = 7;
        bool isMoving = (abs(xSpeed) > motionThreshold || abs(ySpeed) > motionThreshold);

        if (isMoving)
        {
            // As soon as we detect motion, print a message and return
            Report("MOTION DETECTED\n\r");
            alertOLED();
            return;
        }

        // Check how much time has elapsed since we started
        time(&currentTime);
        double elapsed = difftime(currentTime, startTime);

        if (elapsed >= CHECK_DURATION)
        {
            // If we've passed 15 seconds with no motion, just return
            break;
        }
    }
    return;
}

void processRemoteCommands(){
    while(remoteFlag == 1) {
    if(instruction_ready) {
        uint64_t localPulseBuffer[MAX_PULSES];
        memcpy(localPulseBuffer, (const void *)pulse_buffer, sizeof(localPulseBuffer));
        instruction_ready = 0;
        compareLast16Bits(localPulseBuffer);
        memset((void *)pulse_buffer, 0, sizeof(pulse_buffer));
    }
    }
}
void resetToLockedState(){
    systemState = STATE_LOCKED;
    motionDetected = false;
    resetOLEDToLockedScreen();
    Report("System reset to LOCKED state");
}

void checkForResetButton() {
    if(GPIOPinRead(SW3_PORT, SW3_PIN) == SW3_PIN) {
        Report("RESET BUTTON PRESSED..RESETTING SYSTEM");
        resetToLockedState();
    }
}

void unlockOLED(void)
{
    unsigned int forestGreen  = Color565(34, 139, 34); // Background
    unsigned int brightGreen  = Color565(0, 255, 0);   // Circle color
    unsigned int white        = WHITE;

    // Clear the screen with a green background
    fillScreen(forestGreen);

    // Calculate the center of the screen
    int centerX = SSD1351WIDTH / 2;
    int centerY = SSD1351HEIGHT / 2;

    // --- Draw a bright-green circle in the center ---
    int circleRadius = 30;
    fillCircle(centerX, centerY, circleRadius, brightGreen);

    // First line of the check (bottom-left portion).
    // Coordinates chosen for a small angled line inside the circle.
    //    Slope is ~1. We'll offset by (+1, -1) to produce a thicker line.
    int x1s = centerX - 12;  // start X
    int y1s = centerY + 2;   // start Y
    int x1e = centerX - 4;   // end X
    int y1e = centerY + 10;  // end Y

    // Main line
    drawLine(x1s, y1s, x1e, y1e, white);
    // Offset line (shifts in the perpendicular direction for thickness)
    // For slope=+1, a perpendicular offset is (1, -1).
    drawLine(x1s + 1, y1s - 1, x1e + 1, y1e - 1, white);
    int x2s = centerX - 4;
    int y2s = centerY + 10;
    int x2e = centerX + 14;
    int y2e = centerY - 8;

    // Main line
    drawLine(x2s, y2s, x2e, y2e, white);
    // Offset line (for slope=-1, a perpendicular offset is (1, 1))
    drawLine(x2s + 1, y2s + 1, x2e + 1, y2e + 1, white);

    setTextSize(2);
    setTextColor(white, forestGreen);

    const char* text = "UNLOCKED";
    // ~12px width per character at text size=2
    int textWidth = strlen(text) * 12;
    int textX     = centerX - (textWidth / 2);
    int textY     = centerY + circleRadius + 10;
    setCursor(textX, textY);
    Outstr(text);
    isBikeLocked = false;
}


void resetButtonPressed(void)
{
    // Dark purple for the background
    unsigned int darkPurple = Color565(48, 0, 48);
    // Bright purple for the circle
    unsigned int purple  = Color565(128, 0, 128);
    unsigned int white   = WHITE;
    fillScreen(darkPurple);

    // Calculate the center of the screen
    int centerX = SSD1351WIDTH / 2;
    int centerY = SSD1351HEIGHT / 2;

    // --- Draw a bright-purple circle in the center ---
    int circleRadius = 30;
    fillCircle(centerX, centerY, circleRadius, purple);
    // First line of the check (bottom-left portion).
    int x1s = centerX - 12;  // start X
    int y1s = centerY + 2;   // start Y
    int x1e = centerX - 4;   // end X
    int y1e = centerY + 10;  // end Y

    // Main line
    drawLine(x1s, y1s, x1e, y1e, white);
    drawLine(x1s + 1, y1s - 1, x1e + 1, y1e - 1, white);

    // Second line of the check (angled up to the right).
    int x2s = centerX - 4;
    int y2s = centerY + 10;
    int x2e = centerX + 14;
    int y2e = centerY - 8;

    // Main line
    drawLine(x2s, y2s, x2e, y2e, white);
    // Offset line (for slope=-1, a perpendicular offset is (1, 1))
    drawLine(x2s + 1, y2s + 1, x2e + 1, y2e + 1, white);
    setTextSize(2);
    setTextColor(white, darkPurple);

    const char* text = "RESETTING";
    int textWidth = strlen(text) * 12;
    int textX     = centerX - (textWidth / 2);
    int textY     = centerY + circleRadius + 10;
    setCursor(textX, textY);
    Outstr(text);

    isBikeLocked = false;
}



static void compareLast16Bits(const uint64_t *localPulseBuffer){

    char last16bits[17];
    int i;
    for(i=0; i <16; i++){
        last16bits[i] = (localPulseBuffer[32+i] == 1)? '1':'0';
    }
    last16bits[16] = '\0';
    if(strcmp(last16bits, "1001100010011001") == 0){
        Report("Button 0 Pressed\n\r");
    } else if(strcmp(last16bits, "0000100000001001") == 0){
        Report("Button 1 Pressed\n\r");
    } else if(strcmp(last16bits, "1000100010001001")== 0){
        Report("Button 2 Pressed\n\r");
        systemState = STATE_UNLOCKED;
        unlockOLED();
        } else if(strcmp(last16bits, "0100100001001001") == 0) {
        Report("Button 3 Pressed\n\r");
    } else if(strcmp(last16bits, "1100100011001001") == 0) {
        Report("Button 4 Pressed\n\r");
    } else if(strcmp(last16bits, "0010100000101001") == 0){
        Report("Button 5 Pressed\n\r");
    } else if(strcmp(last16bits, "1010100010101001") == 0){
        Report("Button 6 Pressed\n\r");
    } else if((strcmp(last16bits,"0110100001101001") == 0)|(strcmp(last16bits,"1100001101001010") == 0)) {
        Report("Button 7 Pressed\n\r");
    } else if(strcmp(last16bits, "1110100011101001") == 0){
        Report("Button 8 Pressed\n\r");
    } else if(strcmp(last16bits, "0001100000011001") == 0){
        Report("Button 9 Pressed\n\r");
        resetOLEDToLockedScreen();
        checkForMotion();
    } else if(strcmp(last16bits, "1110110011101101") == 0){
        Report("LAST Button Pressed\n\r");
    } else if(strcmp(last16bits, "0100110001001101") == 0) {
        Report("MUTE Button Pressed\n\r");
    } else {
    }
}

int main(void) {

    BoardInit();
    PinMuxConfig();
    I2C_IF_Open(I2C_MASTER_MODE_FST);
    InitTerm();      // Sets up UART0 @ 115200
    ClearTerm();
    Report("System has been initialized. Bike is in a LOCKED STATE\n\r");
    SysTickInit();
    // Configure the IR input GPIO
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_GPIOIntRegister(gpioread.port, IR_InterruptHandler);
    MAP_GPIOIntTypeSet(gpioread.port, gpioread.pin, GPIO_FALLING_EDGE);
    unsigned long ulStatus = MAP_GPIOIntStatus(gpioread.port, false);
    MAP_GPIOIntClear(gpioread.port, ulStatus);
    MAP_GPIOIntEnable(gpioread.port, gpioread.pin);
    // Initialize SPI for the OLED
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIReset(GSPI_BASE);
    MAP_SPIConfigSetExpClk(GSPI_BASE, MAP_PRCMPeripheralClockGet(PRCM_GSPI),
        SPI_IF_BIT_RATE, SPI_MODE_MASTER, SPI_SUB_MODE_0,
        (SPI_SW_CTRL_CS | SPI_4PIN_MODE | SPI_TURBO_OFF | SPI_CS_ACTIVEHIGH | SPI_WL_8));
    MAP_SPIEnable(GSPI_BASE);
    //UART1
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_UARTA1);

    MAP_UARTConfigSetExpClk(UARTA1_BASE,
                            MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                            UART1_BAUDRATE,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Enable UART1 (but do not necessarily enable interrupts yet)
    MAP_UARTEnable(UARTA1_BASE);
    MAP_UARTFIFODisable(UARTA1_BASE);
    //AWS Initialization
    g_app_config.host = SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    lRetVal = connectToAccessPoint();
    //lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }

    lRetVal = tls_connect();
        if(lRetVal < 0) {
            ERR_PRINT(lRetVal);
        }

    // Initialize the OLED
    Adafruit_Init();
    unlockOLED();



    while (1) {
        // If an IR instruction is ready, decode it (the decoding only updates the g_message buffer)
        if(instruction_ready) {
            uint64_t localPulseBuffer[MAX_PULSES];
            memcpy(localPulseBuffer, (const void *)pulse_buffer, sizeof(localPulseBuffer));
            instruction_ready = 0;
            compareLast16Bits(localPulseBuffer);
            memset((void *)pulse_buffer, 0, sizeof(pulse_buffer));
        }
      }
}