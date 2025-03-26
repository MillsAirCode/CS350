# TI Microcontroller Thermostat Firmware

**Author:** Brad Mills
**Date:** March 23, 2024

## Overview

This firmware transforms a Texas Instruments (TI) microcontroller development board into a basic thermostat. It reads temperature from an I2C sensor, allows setpoint adjustment via buttons, controls a heater (simulated by an LED), and reports status over UART. The system uses TI’s driver libraries and a timer-based scheduler to manage tasks efficiently.

---

## Features

- **Temperature Monitoring:** Reads temperature every 500ms from an I2C sensor (e.g., TMP116).
- **Setpoint Control:** Adjusts the desired temperature using two buttons (increase/decrease).
- **Heater Control:** Toggles an LED (simulating a heater) based on the setpoint:
  - On when temperature < setpoint.
  - Off when temperature > setpoint.
- **Real-Time Reporting:** Sends `<temperature,setpoint,heat,seconds>` to UART every second (e.g., `<23,25,0,10.5000>`).
- **Timer-Based Scheduling:** Executes tasks at 100ms, 200ms, and 1s intervals.

---

## Hardware Requirements

- **TI Development Board:** Any TI board compatible with TI’s driver libraries (e.g., MSP430, CC26XX).
- **I2C Temperature Sensor:** Supported models include TMP11X, TMP116, or TMP006.
- **Buttons:** Two GPIO-connected push buttons (e.g., CONFIG_GPIO_BUTTON_0, CONFIG_GPIO_BUTTON_1).
- **LED:** One GPIO-connected LED (e.g., CONFIG_GPIO_LED_0) to simulate the heater.
- **UART Connection:** USB-to-serial interface for monitoring output (e.g., via a terminal like PuTTY).

---

## Installation

### Prerequisites
- **TI Code Composer Studio (CCS)** or equivalent IDE for building and flashing the firmware.
- **TI SimpleLink SDK** or appropriate driver library for your board.
- **Hardware Setup:**
  - Connect the I2C sensor to the board’s I2C pins (check `ti_drivers_config.h`).
  - Wire buttons to GPIO pins with pull-up resistors.
  - Connect an LED to a GPIO pin (anode to pin, cathode to ground via resistor).

### Steps
1. **Clone or Download the Code:**
   - Obtain `gpiointerrupt.c` and `ti_drivers_config.h` from your project source or this repository.

2. **Configure the Project:**
   - Import the code into CCS.
   - Ensure `ti_drivers_config.h` matches your board’s pinout (e.g., update `CONFIG_GPIO_LED_0`, `CONFIG_I2C_0`).

3. **Build and Flash:**
   - Build the project in CCS.
   - Flash the firmware to your TI board using a debugger (e.g., XDS110).

4. **Connect UART:**
   - Open a serial terminal (e.g., PuTTY, Tera Term) at 115200 baud, connected to the board’s UART port.

---

## Usage

1. **Power On the Board:**
   - The firmware initializes, detects the sensor, and starts the thermostat loop.
   - The LED may briefly turn on during startup.

2. **Monitor Output:**
   - View UART messages like `<23,25,0,10.5000>`:
     - `23`: Current temperature (°C).
     - `25`: Setpoint (°C).
     - `0`: Heater off (1 = on).
     - `10.5000`: Elapsed time (seconds).

3. **Adjust Setpoint:**
   - Press Button 0 to increase the setpoint.
   - Press Button 1 to decrease the setpoint.
   - The LED turns on if the temperature is below the setpoint, off if above.

4. **Observe Behavior:**
   - The system updates every second, adjusting the heater and reporting status.

---

## Project Structure

project_directory/
├── gpiointerrupt.c         # Main thermostat firmware
├── ti_drivers_config.h     # Hardware configuration file
└── README.md               # This documentation


---

## Notes

- **Heater Simulation:** The LED simulates a heater. In a real application, replace it with a relay or transistor to control an actual heating element.
- **Sensor Detection:** The firmware tries three I2C addresses (0x48, 0x49, 0x41). Ensure your sensor matches one of these.
- **Error Handling:** If the sensor isn’t detected or I2C fails, check UART messages for debugging hints (e.g., "Temperature sensor not found").

---

## Troubleshooting

- **No UART Output:** Verify baud rate (115200) and connection to the correct COM port.
- **LED Not Toggling:** Check `ti_drivers_config.h` for correct GPIO pin mapping.
- **Sensor Not Found:** Confirm I2C wiring and sensor address compatibility.

---

## License

This code is adapted from Texas Instruments’ examples under their BSD-3-Clause license. See the source file (`gpiointerrupt.c`) for full license details.

---

# CS350
Emerging Sys Arch &amp; Tech

Thermostat Project
Summarize the project and what problem it was solving.
This project aimed to integrate various built-in board peripherals to simulate a working thermostat. By responding to temperature changes with an LED indicator and adjusting values, it mimicked the functionality of a heater, combining theoretical knowledge with practical application.

What did you do particularly well?
The structured approach to the code and the logical naming of variables were my strong points. The code's readability, aided by comprehensive comments and a logical flow, made the project's purpose and functionality clear and accessible.

Where could you improve?
My tendency to overthink solutions led to multiple rewrites of the program. I also see a need to enhance my understanding of programming the board's peripherals more efficiently.

What tools and/or resources are you adding to your support network?
Engaging in project discussions proved highly beneficial, not only in clarifying aspects of the project but also in highlighting the importance of collaborative problem-solving. Delving into the header files for the drivers used in the project was also enlightening.

What skills from this project will be particularly transferable to other projects and/or course work?
This project significantly improved my ability to code peripherals to work together seamlessly, a skill set that promises to be highly transferable to a wide range of future projects.

How did you make this project maintainable, readable, and adaptable?
I prioritized maintaining clean, well-commented code to facilitate understanding and future modifications. The project's structure is designed to be easily adjustable, allowing for straightforward alterations or expansions as needed.
