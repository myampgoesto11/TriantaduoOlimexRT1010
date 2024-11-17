# TriantaduoOlimexRT1010
Port of the Arduino based TriantaduoWS2811 project on a Teensy 4.0 board to the RT1010 processor on an Olimex board

## Getting started
Created a new MCUXpresso workspace on my google drive and did the following:
* Selected the "Download and Install SDK's" from the "Welcome" page.
* Selected the "evkmimxrt1010" board, and then "Install".
* This resulted in the installation of "SDK_2.x_EVK-MIMXRT1010"
* Next, selected "Import SDK Examples" from the "Welcome" page.
* Selected "Go straight to the Wizard".
* Selected the "evkmimxrt1010" board by clicking on it's image, the "Next"
* Expanded the "driver_examples" section and selected the "flexio" subsection, and the  "flexio_pwm" project, then selected "Finish".

## Building the Test Version that outputs partially working "Serial Data" only, NO "Bit Clock" or "Latch Clock"
* This Test version is a partial port of the Arduino code. It does use some DMA, hence the addition of the following files, that were stolen from other "Exmaple Projects":
    * fsl_dmamux.c/.h
    * fsl_edma.c/.h
* This test version outputs only the Serial Data signal, NOT the bit clock or the Latch clock.
* Also, the Serial Data Clock isn't correct yet.