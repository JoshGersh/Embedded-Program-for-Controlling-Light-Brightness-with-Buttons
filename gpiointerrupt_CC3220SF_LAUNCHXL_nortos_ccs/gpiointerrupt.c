/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>


/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART_write(uart, &output, x);

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
    } sensors[3] = {
                    { 0x48, 0x0000, "11X" },
                    { 0x49, 0x0000, "116" },
                    { 0x41, 0x0001, "006" }
    };

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// UART Global Variables
uint8_t output[64];
int bytesToSend;

// button Global Variables
uint8_t topButtonFlag = 0;
uint8_t bottomButtonFlag = 0;

// Driver Handles - Global variables
I2C_Handle i2c;

// Driver Handles - Global variables
UART_Handle uart;

// Driver Handles - Global variables
Timer_Handle timer0;

// values for state machines
enum buttons{
    increaseTemp, decreaseTemp, noPress
};

enum buttons buttons = noPress; // Initialized button variable

enum heatingState{
    heatOff,
    heatOn
};

enum heatingState heatingState = heatOff; // Initialized heatIdle variable

// Variables for thermostat
uint8_t temperatur;
uint8_t setpoint = 28;
uint8_t heat = 0;
unsigned long timer = 0;

// Variables for task scheduler
typedef struct task {
  int state; // Current state of the task
  unsigned long period; // Rate at which the task should tick
  unsigned long elapsedTime; // Time since task's previous tick
  int (*TickFct)(int); // Function to call for task's tick
} task;
const unsigned char tasksNum = 3;

// time intervals for task scheduler in milliseconds
const unsigned long tasksPeriodGCD = 100;
const unsigned long periodThermostat = 500;
const unsigned long periodDesplay = 1000;
const unsigned long periodButtons = 200;

task tasks[tasksNum];

volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status){
    TimerFlag = 1;
    }

void initTimer(void){
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000; // the period the timer is checked is every 100 milliseconds or 100000 microseconds
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}



void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}


// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL){
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i){
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction)){
            DISPLAY(snprintf(output, 64, "Found\n\r"))
                found = true;
                break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found){
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else{
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

int16_t readTemp(void)
{
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)){
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
            /*
            * If the MSB is set '1', then we have a 2's complement
            * negative value which needs to be sign extended
            */
            if (rxBuffer[0] & 0x80){
                temperature |= 0xF000;
                }
    }
    else{
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}

/**
 * @brief Updates the temperature control system based on the temperature reading from the sensor.
 *
 * This function reads the current temperature using the I2C protocol and updates the heating state accordingly.
 * If the temperature is higher than the setpoint, it transitions to the heating state and turns on the LED.
 * If the temperature is lower than or equal to the setpoint, it transitions back to the idle state and turns off the LED.
 *
 * @note Ensure that the setpoint is properly initialized before calling this function.
 */
void updateTemperature(){

    temperatur = readTemp(); // calls the readTemp that uses the I2C protocol to check the temperature

    switch (heatingState) { // Transitions
        case heatOff:
            if (temperatur > setpoint) {
                // Transition to the heating state
                heatingState = heatOn;
            }
            break;

        case heatOn:
            if (temperatur <= setpoint) {
                // Transition back to the idle state
                heatingState = heatOff;
            }
            break;

        default:
            // Handle potential errors or unexpected states here
            break;
    }

    switch (heatingState) {// State actions
        case heatOff:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // Turn off the LED in idle state
            break;

        case heatOn:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON); // Turn on the LED when heating
            break;
    }
}

/**
 * @brief Changes the setpoint based on the button inputs.
 *
 * This function adjusts the setpoint temperature based on the button pressed.
 * If the increase temperature button is pressed, the setpoint is incremented.
 * If the decrease temperature button is pressed, the setpoint is decremented.
 * If no button is pressed, the function does nothing.
 *
 * @note Ensure that the 'buttons' and 'setpoint' variables are properly initialized before calling this function.
 */
void changeSetPoint(){
    switch (buttons) { // Switch based on the 'button' variable

        case increaseTemp:
            ++setpoint;
            buttons = noPress;
            break;

        case decreaseTemp:
            --setpoint;
            buttons = noPress;
            break;

        case noPress:
            break; // No action needed if no button press

        default:
            // Handle potential errors or unexpected states here
            break;
    }
}

void serverReport(){
    DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperatur, setpoint, heatingState, timer));
}

/**
 * @brief Timer interrupt function for task scheduling.
 *
 * This function is the core of the scheduler code.
 * It iterates through the tasks and checks if each task is ready to be executed based on its period.
 * If a task is ready, it updates the task state using its corresponding tick function and resets the elapsed time for the task.
 * The function ensures proper scheduling of tasks based on their respective periods and elapsed times.
 *
 * @note Ensure that the 'tasks' array, 'tasksNum', 'tasksPeriodGCD' variables, and the structure 'Task' are properly defined and initialized before calling this function.
 */
void TimerInterrupt() {
  unsigned char i;
  for (i = 0; i < tasksNum; ++i) { // Heart of the scheduler code
     if ( tasks[i].elapsedTime >= tasks[i].period ) { // Ready
        tasks[i].state = tasks[i].TickFct(tasks[i].state);
        tasks[i].elapsedTime = 0;
     }
     tasks[i].elapsedTime += tasksPeriodGCD;
  }
}
/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_0);
    buttons = increaseTemp;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_1);
    buttons = decreaseTemp;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{



    /* Call driver init functions */
    GPIO_init();
    initTimer();
    initUART();
    initI2C();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    //GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }


    unsigned char i=0;

    // Configure the first task
    tasks[i].state = heatOff;
    tasks[i].period = periodButtons;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &changeSetPoint;

    // Configure the second task
    ++i;
    tasks[i].state = noPress;
    tasks[i].period = periodThermostat;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &updateTemperature;

    // Configure the third task
    ++i;
    tasks[i].period = periodDesplay;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &serverReport;


    while (1) {

        TimerInterrupt();

        while(!TimerFlag){} //every 100ms
        TimerFlag = 0;
        ++timer;//every 1s
    }

    return (NULL);
}
