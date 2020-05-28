# GestureControl
This is my Embedded Systems Project. 
## Project requirements specified by the lecturer:
    
####    Project topics

    The project may relate to the issues and needs of your everyday life.
    The project may be related to another subject during studies.
    The project may concern topics related to the future MA thesis.
    The project may relate to issues related to professional work, if the employer has no opposition.
    The project cannot be a presentation of previously made systems.

####    Level of difficulty

    The project should have a minimum level of difficulty, which can be defined as:

     1. Using the UART system for debug communication with a computer.
     2. Using a minimum of two communication peripherals, for example SPI, I2C, USB, 1-Wire.
     3. Using a minimum of two interrupts.
     4. Use of the DMA mechanism.

    The listed set of requirements is negotiable depending on the subject of the project.

## Gesture Control System
The main goal of the project is to design and construct a system which allows for gesture control over another system (e.g. simple household object - curtain/display).
The system will read a user's gesture and process it. The processing results in a set of commands to be sent to another system using either BLE or WiFi.

For this project, the "other system" will be a web server on Raspberry Pi Zero, which acts as master controller. 
It receives gestures via WiFi and dispatches proper commands to another slave system - for example a split-flap display. 
However, what the other system does with the commands is not in scope of the project yet.

#### Most important system functions:

    read and recognize user's basic gestures using a gesture sensor
    process gestures to set control commands
    send control commands through WiFi/BLE to controlled device
    dump few last gestures through UART to PC for debug

#### List of necessary equipment:

    Development board: STM32 NUCLEO-F103RB
    Gesture sensor: APDS9960 or PAJ7620U2
    Communication module: NodeMCU v3
    breadboard,wires,etc.

#### Additional comments:

    Upper Master - Raspberry Pi Zero Web Server
    Local Master - NodeMCU
    Medium Slave - STM32F103RB
    Lower Slave - PAJ7620 Gesture Sensor
##### Main flow:


    Sensor: 
    {read gesture}---Interrupt->Send Over I2C--->STM32
    STM32: 
    {ISR: recognize gesture, send via SPI if master ready}<---SPI_Interrupt--->NodeMCU
    NodeMCU: {get gesture, send gesture}---WiFi/BLE--->...other system

In the project, both the STM32 and the NodeMCU will have to be programmed. Debug functionality can be realized as button click interrupt or command from connected PC.

#### Possible extensions of the project:

    1. Adding mode selection - from website or potentially application, user can choose what to control with gestures.
    2. Adding more user defined gestures - depends on implementation of gesture-decoding.
