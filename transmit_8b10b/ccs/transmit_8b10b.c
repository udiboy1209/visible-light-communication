//*****************************************************************************
//
// usb_dev_bulk.c - Main routines for the generic bulk device example.
//
// Copyright (c) 2012-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
//#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "usb_bulk_structs.h"

#include "lut_8b10b.h"

volatile uint8_t g_ui8ParallelOut6b = 0;
volatile uint8_t g_ui8ParallelOut4b = 0;
volatile bool g_bIntDone = 0;
volatile bool g_bParallelDataReady = 0;
volatile uint32_t g_ui32SymbolsTx = 0;
//volatile bool g_bOutputMux = 0;

volatile bool g_bRxAvailable = 0;
volatile tUSBDBulkDevice * g_psRxDevice;
volatile uint8_t * g_pvRxData;
volatile uint32_t g_ui32NumBytes = 0;

//*****************************************************************************
//
// Variables tracking transmit and receive counts.
//
//*****************************************************************************
volatile uint32_t g_ui32TxCount = 0;
volatile uint32_t g_ui32RxCount = 0;
#ifdef DEBUG
uint32_t g_ui32UARTRxErrors = 0;
#endif


#define OUTPUT_PORT SYSCTL_PERIPH_GPIOB
#define OUTPUT_BASE GPIO_PORTB_BASE
//#define OUTPUT_PINS (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0)
#define OUTPUT_PINS (GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0)


#define OUTPUT2_PORT SYSCTL_PERIPH_GPIOE
#define OUTPUT2_BASE GPIO_PORTE_BASE
#define OUTPUT2_PINS ( GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0)

//*****************************************************************************
//
// Debug-related definitions and declarations.
//
// Debug output is available via UART0 if DEBUG is defined during build.
//
//*****************************************************************************
#ifdef DEBUG
//*****************************************************************************
//
// Map all debug print calls to UARTprintf in debug builds.
//
//*****************************************************************************
#define DEBUG_PRINT UARTprintf

#else

//*****************************************************************************
//
// Compile out all debug print calls in release builds.
//
//*****************************************************************************
#define DEBUG_PRINT while(0) ((int (*)(char *, ...))0)
#endif

//*****************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//*****************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile uint32_t g_ui32Flags = 0;

//*****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//*****************************************************************************
static volatile bool g_bUSBConfigured = false;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    UARTprintf("Error at line %d of %s\n", ui32Line, pcFilename);
    while(1)
    {
    }
}
#endif


void
Timer0BIntHandler(void)
{
    uint8_t outData6b = g_ui8ParallelOut6b, outData4b = g_ui8ParallelOut4b;

    if(!g_bParallelDataReady){
        outData6b = 0x00;
        outData4b = 0x00;
    }

    //
    // Clear the timer interrupt flag.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    //if(g_bOutputMux)
    GPIOPinWrite(OUTPUT_BASE,OUTPUT_PINS,outData6b);
    GPIOPinWrite(OUTPUT2_BASE,OUTPUT2_PINS,outData4b);
    //else
        //GPIOPinWrite(OUTPUT2_BASE,OUTPUT2_PINS,outData);

    g_bIntDone = 1;
    g_bParallelDataReady = 0;
    g_ui32SymbolsTx ++;
}


/*static uint32_t
EchoNewDataToHost(tUSBDBulkDevice *psDevice, uint8_t *pui8Data,
                  uint32_t ui32NumBytes)
{
    uint32_t ui32Loop, ui32Space, ui32Count;
    uint32_t ui32ReadIndex;
    uint32_t ui32WriteIndex;
    tUSBRingBufObject sTxRing;

    USBBufferInfoGet(&g_sTxBuffer, &sTxRing);
    ui32Space = USBBufferSpaceAvailable(&g_sTxBuffer);

    ui32Loop = (ui32Space < ui32NumBytes) ? ui32Space : ui32NumBytes;
    ui32Count = ui32Loop;

    g_ui32RxCount += ui32NumBytes;

    DEBUG_PRINT("Received %d bytes\n", ui32NumBytes);

    ui32ReadIndex = (uint32_t)(pui8Data - g_pui8USBRxBuffer);
    ui32WriteIndex = sTxRing.ui32WriteIndex;

    while(ui32Loop)
    {
        if((g_pui8USBRxBuffer[ui32ReadIndex] >= 'a') &&
           (g_pui8USBRxBuffer[ui32ReadIndex] <= 'z'))
        {
            g_pui8USBTxBuffer[ui32WriteIndex] =
                (g_pui8USBRxBuffer[ui32ReadIndex] - 'a') + 'A';
        }
        else
        {
            if((g_pui8USBRxBuffer[ui32ReadIndex] >= 'A') &&
               (g_pui8USBRxBuffer[ui32ReadIndex] <= 'Z'))
            {
                g_pui8USBTxBuffer[ui32WriteIndex] =
                    (g_pui8USBRxBuffer[ui32ReadIndex] - 'Z') + 'z';
            }
            else
            {
                g_pui8USBTxBuffer[ui32WriteIndex] =
                        g_pui8USBRxBuffer[ui32ReadIndex];
            }
        }

        ui32WriteIndex++;
        ui32WriteIndex = (ui32WriteIndex == BULK_BUFFER_SIZE) ?
                         0 : ui32WriteIndex;

        ui32ReadIndex++;
        ui32ReadIndex = (ui32ReadIndex == BULK_BUFFER_SIZE) ?
                        0 : ui32ReadIndex;

        ui32Loop--;
    }
    USBBufferDataWritten(&g_sTxBuffer, ui32Count);

    DEBUG_PRINT("Wrote %d bytes\n", ui32Count);

    return(ui32Count);
}*/

//*****************************************************************************
//
// Handles bulk driver notifications related to the transmit channel (data to
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    if(ui32Event == USB_EVENT_TX_COMPLETE)
    {
        g_ui32TxCount += ui32MsgValue;
    }

    DEBUG_PRINT("TX complete %d\n", ui32MsgValue);

    return(0);
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event,
               uint32_t ui32MsgValue, void *pvMsgData)
{
    switch(ui32Event)
    {
        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;
            UARTprintf("Host connected.\n");

            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            break;
        }

        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBConfigured = false;
            UARTprintf("Host disconnected.\n");
            break;
        }

        case USB_EVENT_RX_AVAILABLE:
        {
            //tUSBDBulkDevice *psDevice;
            g_psRxDevice = (tUSBDBulkDevice *)pvCBData;

            g_bRxAvailable = 1;
            g_pvRxData = pvMsgData;
            g_ui32NumBytes += ui32MsgValue;

        }

        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        {
            break;
        }

        default:
        {
            break;
        }
    }

    return(0);
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// This is the main application entry function.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32BufferPos;
    uint32_t ui16CurrData;
    //uint32_t ui8CurrDataAdded;
    //uint32_t ui8NextByte;
    //uint32_t ui8NextByteFilled;
    uint32_t ui8Next4b, ui8Next6b;
    int32_t i8RunningDisparity = 0;
    bool bStopTimer = 0;

    //ROM_FPULazyStackingEnable();

    // Set the clocking to run from the PLL at 50MHz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    // Enable the GPIO pins for the LED (PF2 & PF3).
    ROM_SysCtlPeripheralEnable(OUTPUT_PORT);
    ROM_GPIOPinTypeGPIOOutput(OUTPUT_BASE, OUTPUT_PINS);

    ROM_SysCtlPeripheralEnable(OUTPUT2_PORT);
    ROM_GPIOPinTypeGPIOOutput(OUTPUT2_BASE, OUTPUT2_PINS);

    // Open UART0 and show the application name on the UART.
    ConfigureUART();

    UARTprintf("\033[2JTiva C Series USB bulk device example\n");
    UARTprintf("---------------------------------\n\n");

    // Not configured initially.
    g_bUSBConfigured = false;

    // Enable the GPIO peripheral used for USB, and configure the USB
    // pins.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
    //ROM_SysTickIntEnable();
    //ROM_SysTickEnable();

    // Enable timer B0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_B, ROM_SysCtlClockGet() / 100000);

    UARTprintf("Timer load: %d\n", ROM_SysCtlClockGet() / 100000);

    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
    ROM_IntEnable(INT_TIMER0B);


    // Tell the user what we are up to.
    UARTprintf("Configuring USB\n");

    // Initialize the transmit and receive buffers.
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);

    // Set the USB stack mode to Device mode with VBUS monitoring.
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    // Pass our device information to the USB library and place the device
    // on the bus.
    USBDBulkInit(0, &g_sBulkDevice);

    // Wait for initial configuration to complete.
    UARTprintf("Waiting for host...\n");


    ui32BufferPos = 0;
    //ui8NextByteFilled = 0;
    //ui8NextByte = 0;

    while(1)
    {
        if(g_bRxAvailable){

            ROM_TimerEnable(TIMER0_BASE, TIMER_B);

            if(g_ui32NumBytes>0){
                ui16CurrData = g_pui8USBRxBuffer[ui32BufferPos]; // Get 8b data

                if(i8RunningDisparity < 0){
                    i8RunningDisparity += D_lut[ui16CurrData];

                    ui16CurrData = encode_lut[ui16CurrData]; // change to 10b data
                } else {
                    i8RunningDisparity += D_lut[ui16CurrData+256];

                    ui16CurrData = encode_lut[ui16CurrData+256]; // change to 10b data
                }


                ui32BufferPos = (ui32BufferPos+1)%BULK_BUFFER_SIZE;
                g_ui32NumBytes --;
            } else {
                ui16CurrData = 0x2AA; // TODO put control character
                g_bRxAvailable = 0;

                bStopTimer = 1;
            }

            //ui8CurrDataAdded = 0;

            /*while(ui8CurrDataAdded < 10){
                ui8NextByte = (ui8NextByte << 1) + ui16CurrData%512;
                ui16CurrData = (ui16CurrData << 1)%1024;
                ui8CurrDataAdded ++;
                ui8NextByteFilled ++;

                if(ui8NextByteFilled == 8){
                    while(!g_bIntDone); // Wait for prev data to be output
                    g_bIntDone = 0;
                    g_ui8ParallelOut = ui8NextByte;
                    g_bParallelDataReady = 1;

                    ui8NextByte = 0;
                    ui8NextByteFilled =0;
                }
            }*/

            ui8Next6b = ui16CurrData % 64; // Lower 6 bits of 10 bit data
            ui8Next4b = (ui16CurrData>>6) % 16; // Upper 4 bits

            while(!g_bIntDone); // Wait for prev data to be output
            g_ui8ParallelOut6b = ui8Next6b;
            g_ui8ParallelOut4b = ui8Next4b;
            g_bParallelDataReady = 1;
            g_bIntDone = 0;

            if(bStopTimer)
                ROM_TimerDisable(TIMER0_BASE,TIMER_B);

            bStopTimer = 0;
        }
    }
}
