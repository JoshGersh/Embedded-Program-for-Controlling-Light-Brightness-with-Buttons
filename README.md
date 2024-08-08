# Thermostat System using TI LaunchPad and Code Composer Studio

## Overview
This project implements a thermostat system using the Texas Instruments (TI) LaunchPad development kit and Code Composer Studio (CCS). The system monitors the ambient temperature and adjusts a connected heating/cooling device to maintain a user-defined temperature setpoint. The system is designed with embedded C programming and utilizes Pulse Width Modulation (PWM) for controlling the output device.

## Hardware Requirements
TI LaunchPad Development Kit: CC3220SF-LAUNCHXL
User Interface: Buttons for adjusting the temperature setpoint, an LCD or LED display for showing the current temperature and setpoint
Breadboard, Jumper Wires, Resistors, etc.: For connecting the components
Software Requirements
Code Composer Studio (CCS): IDE for writing, compiling, and debugging the code
TI Driver Libraries: For interfacing with the hardware peripherals

## Features
Temperature Monitoring: Continuously monitors the ambient temperature using a connected sensor.
Setpoint Adjustment: Allows users to set the desired temperature using onboard buttons.
Automatic Control: Adjusts the heating/cooling device using PWM to maintain the set temperature.
User Feedback: Displays the current temperature and setpoint on an LCD or LED display.
Low Power Mode: Enters a low-power state when the system is idle to conserve energy.
Getting Started
1. Clone the Repository
Clone the project repository to your local machine.

bash
Copy code
git clone git@github.com:JoshGersh/Embedded-Program-for-Controlling-Light-Brightness-with-Buttons.git
cd thermostat-launchpad
2. Install Code Composer Studio (CCS)
Ensure you have Code Composer Studio installed. If not, download and install it from the TI website.

3. Open the Project
Open Code Composer Studio, go to File > Import and select Code Composer Studio > CCS Projects. Browse to the cloned repository directory and import the project.

4. Build and Debug
Connect your TI LaunchPad to your computer and click on Build and then Debug in CCS. The program should be compiled, and you can start debugging on the LaunchPad.

5. Run the Program
Once debugging is successful, you can run the program on the LaunchPad. Adjust the temperature setpoint using the buttons, and observe the system controlling the heating/cooling device.

Customization
Sensor Calibration: Adjust the calibration parameters in temperature_sensor.c if your sensor requires specific calibration.
PWM Frequency: Modify the PWM frequency in pwm_control.c based on the requirements of your heating/cooling device.
User Interface: Customize the display format or control buttons in ui_control.c.
Troubleshooting
Sensor Reading Errors: Ensure the temperature sensor is correctly connected and the pins are configured properly in the code.
PWM Not Working: Verify the PWM initialization and check the connections to the heating/cooling device.
Display Issues: Confirm that the LCD/LED connections match the pin assignments in the code.
