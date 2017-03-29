#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "usb_bulk_structs.h"

volatile uint8_t g_ui8ParallelOutLow[3] =  {0b1001, 0b1111, 0b0101};
volatile uint8_t g_ui8ParallelOutHigh[3] = {0b0110, 0b0110, 0b0010};
volatile uint8_t g_i=0;
volatile bool g_bIntDone = 0;
//volatile bool g_bHighLowSwitch = 0;
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


#define OUTPUTL_PORT SYSCTL_PERIPH_GPIOC
#define OUTPUTL_BASE GPIO_PORTC_BASE
#define OUTPUTL_PINS (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4)

#define OUTPUTH_PORT SYSCTL_PERIPH_GPIOE
#define OUTPUTH_BASE GPIO_PORTE_BASE
#define OUTPUTH_PINS ( GPIO_PIN_5 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1)

#define CLK16_TRIG_PORT SYSCTL_PERIPH_GPIOF
#define CLK16_TRIG_BASE GPIO_PORTF_BASE
#define CLK16_TRIG_INT INT_GPIOF
#define CLK16_TRIG_PIN GPIO_PIN_1



void
ExternalClockIntHandler(void)
{
    //if(g_bOutputMux)

    GPIOIntClear(CLK16_TRIG_BASE, CLK16_TRIG_PIN);
    if(GPIOPinRead(CLK16_TRIG_BASE, CLK16_TRIG_PIN)){ // Low
        GPIOPinWrite(OUTPUTL_BASE,OUTPUTL_PINS,g_ui8ParallelOutLow[g_i]);
        //GPIOPinWrite(CLK16_TRIG_PORT, GPIO_PIN_2|GPIO_PIN_3, 0b10);
    }
    else
    {
        GPIOPinWrite(OUTPUTH_BASE,OUTPUTH_PINS,g_ui8ParallelOutHigh[g_i]);
        g_bIntDone = 1;
        g_bParallelDataReady = 0;
        g_ui32SymbolsTx ++;

        g_i = (g_i+1)%3;
        //GPIOPinWrite(CLK16_TRIG_PORT, GPIO_PIN_2|GPIO_PIN_3, 0b01);
    }

    //g_bHighLowSwitch = !g_bHighLowSwitch;
    //else
        //GPIOPinWrite(OUTPUTH_BASE,OUTPUTH_PINS,outData);

    //ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
}

void
InitPWMClock(void)
{
    // Init PWM to output clock
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set frequency to 50kHz
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 1000);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 500);

    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
}

void
InitParallelOut(void)
{
    // Enable the GPIO pins for Parallel out
    ROM_SysCtlPeripheralEnable(OUTPUTL_PORT);
    ROM_GPIOPinTypeGPIOOutput(OUTPUTL_BASE, OUTPUTL_PINS);

    ROM_SysCtlPeripheralEnable(OUTPUTH_PORT);
    ROM_GPIOPinTypeGPIOOutput(OUTPUTH_BASE, OUTPUTH_PINS);
}

void
InitExternalClockInt(void)
{
    // Enable Edge-Triggered pin
    ROM_SysCtlPeripheralEnable(CLK16_TRIG_PORT);
    ROM_GPIOPinTypeGPIOInput(CLK16_TRIG_BASE, CLK16_TRIG_PIN);

    GPIOIntDisable(CLK16_TRIG_BASE, CLK16_TRIG_PIN);        // Disable interrupt for PF4 (in case it was enabled)
    GPIOIntClear(CLK16_TRIG_BASE, CLK16_TRIG_PIN);      // Clear pending interrupts for PF4
    GPIOIntRegister(CLK16_TRIG_BASE, ExternalClockIntHandler);     // Register our handler function for port F
    GPIOIntTypeSet(CLK16_TRIG_BASE, CLK16_TRIG_PIN, GPIO_BOTH_EDGES);
    GPIOIntEnable(CLK16_TRIG_BASE, CLK16_TRIG_PIN);
    ROM_IntEnable(CLK16_TRIG_INT);
}


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

void
LifiTxByte(uint8_t byte){
    while(!g_bIntDone);
    g_bIntDone = 0;
    g_ui8ParallelOutLow[g_i] = byte%16;
    g_ui8ParallelOutHigh[g_i] = byte/16;
}

int
main(void)
{
    uint32_t ui32BufferPos;

    // Set the clocking to run from the PLL at 50MHz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    // Enable parallel output pins
    InitParallelOut();

    // Enable interrupt for external clock
    InitExternalClockInt();

    // Init PWM output clock
    InitPWMClock();

    // Open UART0 and show the application name on the UART.
    ConfigureUART();

    UARTprintf("\033[2JLifi Transmitter\n");
    UARTprintf("---------------------------------\n\n");

    // Not configured initially.
    g_bUSBConfigured = false;

    // Enable the GPIO peripheral used for USB, and configure the USB
    // pins.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);


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


    // Start clock and data output
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    ui32BufferPos = 0;

    while(1)
    {
        if(g_bRxAvailable){
            LifiTxByte(0b01110001); // Phase alignment bytes
            //LifiTxByte(0b11001001); // Phase alignment bytes

            while(g_ui32NumBytes){
                LifiTxByte(g_pui8USBRxBuffer[ui32BufferPos]);
                g_ui32NumBytes -- ;
                ui32BufferPos = (ui32BufferPos+1)%BULK_BUFFER_SIZE;
            }

            g_bRxAvailable = 0;
        }
    }
}
