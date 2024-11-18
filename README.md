# TriantaduoOlimexRT1010
Port of the Arduino based TriantaduoWS2811 Teensy 4.0 board project to the RT1010 processor on an Olimex board.

## Getting started
Installed MCUXpresso and created a new workspace on my google drive and did the following:
* Selected the "Download and Install SDK's" from the "Welcome" page.
* Selected the "evkmimxrt1010" board, and then "Install".
* This resulted in the installation of "SDK_2.x_EVK-MIMXRT1010"
* Next, selected "Import SDK Examples" from the "Welcome" page.
* Selected "Go straight to the Wizard".
* Selected the "evkmimxrt1010" board by clicking on it's image, the "Next"
* Expanded the "driver_examples" section and selected the "flexio" subsection, and the "flexio_pwm" project, then selected "Finish".
* This installed the FlexIO PWM example project in my workspace.
* I used this Example Project as a starting point for development.

## Building the Test Version that outputs partially working "Serial Data", "Bit Clock" and "Latch Clock"
* This Test version is a partial port of the Arduino code. It does use some DMA, hence the addition of the following files, that were stolen from other "Exmaple Projects":
    * fsl_dmamux.c/.h
    * fsl_edma.c/.h
* This test version outputs the following signals/ Olimex header pins (See the function tdws2811.configurePins() for more details):
    * Serial Data: RT1010  FLEXIO1_D06 / GPIO_SD_00 -> Olimex Board -> GPIO2_IO00 -> CON2:1
    * Bit Clock:   RT1010  FLEXIO1_D07 - GPIO_SD_01 -> Olimex Board -> GPIO2_IO01 -> CON2:2
    * Latch Clock: RT1010  FLEXIO1_D08 - GPIO_SD_02 -> Olimex Board -> GPIO2_IO02 -> CON2:3
* THIS IS STILL A WORK IN PROGRESS!!

## Debugging:
* Select the Green "Bug" icon dropdown, then "Debug Configurations"
* Double Click the "JLink GDB SEGGER Interface Debugging", this will open a dialog titled:
    * "evkmimxrt1010_flex_pwm JLink Debug" - Use the default selections!
* Press Green "Play" button (Yellow Rectangle on Left, Green Triangle on right - Not the Green Circle with the white triangle inside, that opens the "Run" dialog!)
* Using a scope you should see output on CON2 pin 1 (GPIO2_IO00)
* After stopping the debug session just do the following to Debug Again:
    * Press the Green "Bug" button, it will download.
    * Press the Green "Play" button to run from Main.