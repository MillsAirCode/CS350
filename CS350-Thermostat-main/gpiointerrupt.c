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
#include <stdio.h>

// Texas Instruments Driver Include Files
#include <ti/drivers/UART2.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/I2C.h>

// Configuration for device drivers
#include "ti_drivers_config.h"

// Macro for simplifying UART printing
#define DISPLAY(x) UART2_write(uart, &output, x, &bytesToSend);

// Global variables for driver handles
Timer_Handle timer0;
UART2_Handle uart;
I2C_Handle i2c;

// Flags and Variables for Application Logic
volatile unsigned char TimerFlag = 0;
int setpoint;                  // Desired temperature setpoint
int temperature;               // Current temperature reading
float seconds;                 // Timer to track elapsed time
char heat;                     // Heater state indicator ('0' off, '1' on)

// Global variables for UART communication
char output[64];               // Buffer for UART output
int bytesToSend;               // Number of bytes to send over UART

// Structure defining sensors for I2C communication
static const struct {
    uint8_t address;           // I2C address of the sensor
    uint8_t resultReg;         // Register to read the sensor data from
    char *id;                  // Sensor ID string for identification
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];           // Buffer for I2C transmissions
uint8_t rxBuffer[2];           // Buffer for I2C receptions
I2C_Transaction i2cTransaction; // I2C transaction structure

// Timer callback function - called on timer interrupt
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    seconds += 0.1;
    TimerFlag = 1;
}

// Initialize Timer0 with continuous callback mode
void initTimer(void) {
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 100000;  // Set period to 100 milliseconds
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        // Timer initialization failed
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        // Timer start failed
        while (1) {}
    }
}

// Initialize UART for communication
void initUART(void) {
    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.writeMode = UART2_Mode_BLOCKING;
    uartParams.readMode = UART2_Mode_BLOCKING;
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {
        // UART initialization failed
        while (1);
    }
}

// Initialize I2C for sensor communication
void initI2C(void) {
    int8_t i, found = 0;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));
    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Attempt to identify connected sensor by trying different addresses
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    for (i = 0; i < 3; ++i) {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
        if (I2C_transfer(i2c, &i2cTransaction)) {
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = 1;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    }
    if (found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress));
    } else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
}

// Read temperature from sensor
int16_t readTemp(void) {
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        // Extract temperature in degrees C from received data (refer to TMP sensor datasheet)
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        if (rxBuffer[0] & 0x80) {
            // If MSB is set, it's a negative value in 2's complement
            temperature |= 0xF000;
        }
    } else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }
    return temperature;
}

// GPIO button callback function for increasing temperature setpoint
void gpioButtonFxn0(uint_least8_t index) {
    setpoint++;
}

// GPIO button callback function for decreasing temperature setpoint
void gpioButtonFxn1(uint_least8_t index) {
    setpoint--;
}

// Heater state machine
enum Heater_States { Heater_SMStart, Heater_LedOff, Heater_LedOn } Heater_State;

void TickFct_Heater() {
    switch (Heater_State) { // State transitions
        case Heater_SMStart:
            Heater_State = Heater_LedOff; // Initial state
            break;
        case Heater_LedOff:
            if (setpoint > temperature) {
                Heater_State = Heater_LedOn;
            }
            break;
        case Heater_LedOn:
            if (temperature > setpoint) {
                Heater_State = Heater_LedOff;
            }
            break;
        default:
            Heater_State = Heater_SMStart;
            break;
    }

    switch (Heater_State) { // State actions
        case Heater_LedOff:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            heat = '0';
            break;
        case Heater_LedOn:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            heat = '1';
            break;
        default:
            break;
    }
}

// Main function
void *mainThread(void *arg0) {
    // Initialize drivers
    GPIO_init();
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    // Additional setup for devices with more than one button
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    initUART();
    initI2C();
    initTimer();

    // Task scheduler initializations
    unsigned long readTemp_elapsedTime = 500000;       // Every 500 milliseconds
    unsigned long checkButtonPress_elapsedTime = 200000; // Every 200 milliseconds
    unsigned long updateAndReport_elapsedTime = 1000000; // Every 1 second

    while (1) {
        TimerFlag = 0; // Reset timer flag
        while (!TimerFlag) {} // Wait for timer flag

        // Periodic tasks
        if (readTemp_elapsedTime >= 500000) {
            temperature = readTemp();
            readTemp_elapsedTime = 0;
        }
        if (updateAndReport_elapsedTime >= 1000000) {
            DISPLAY(snprintf(output, 64, "<%02d,%02d,%c,%04f>\n\r", temperature, setpoint, heat, seconds));
            TickFct_Heater();
            updateAndReport_elapsedTime = 0;
        }
        if (checkButtonPress_elapsedTime >= 200000) {
            GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
            GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
            checkButtonPress_elapsedTime = 0;
        }

        // Disable button interrupts to debounce
        GPIO_disableInt(CONFIG_GPIO_BUTTON_0);
        GPIO_disableInt(CONFIG_GPIO_BUTTON_1);

        // Increment elapsed times
        readTemp_elapsedTime += 100000;  // Add timer period
        checkButtonPress_elapsedTime += 100000;
        updateAndReport_elapsedTime += 100000;
    }

    return NULL;
}

