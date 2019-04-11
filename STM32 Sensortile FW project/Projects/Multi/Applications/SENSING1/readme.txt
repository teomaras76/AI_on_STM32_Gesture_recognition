/**
  ******************** (C) COPYRIGHT STMicroelectronics ***********************
  * @file    readme.txt
  * @author  Central LAB
  * @version V1.0.0
  * @date    30-Nov-2018
  * @brief   Description of the SENSING1 application firmware
  ******************************************************************************
  * Attention
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

Application Description 

 This firmware package includes Components Device Drivers, Board Support Package
 and example application for the following STMicroelectronics elements:
 - X-NUCLEO-IDB05A1 Bluetooth Low energy expansion boards
 - X-NUCLEO-IKS01A2 Expansion board for four MEMS sensor devices:
       HTS221, LPS22HB, LSM6DSL, LSM303AGR
 - X-NUCLEO-CCAM02M1 Digital MEMS microphones expansion board
 - NUCLEO-L476RG Nucleo boards
 - STEVAL-STLKT01V1 (SensorTile) evaluation board that contains the following components:
	. MEMS sensor devices: HTS221, LPS22HB, LSM303AGR, LSM6DSM
	. digital microphone: MP34DT04
	. Gas Gauge IC with alarm output: STC3115
 - USB device library (for only STEVAL-STLCS01V1).
 - SD Data Log capabilities
 - Middleware libraries generated thanks to STM32CubeMx extension called X-CUBE-AI featuring example implementation of neural networks for real-time:
	- human activity recognition (HAR)
	- acoustic scene classification (ASC)
 - FatFs generic FAT file system module (for only STEVAL-STLCS01V1) provides access the storage devices 
   such as memory card and hard disk.
 - FreeRTOS Real Time Kernel/Scheduler that allows applications to be organized as a collection of independent threads of execution
   (under MIT open source license)

 The Example application initializes all the Components and Library creating some Custom Bluetooth services:
 - The first service exposes all the HW and SW characteristics:
   . HW characteristics:
		. related to MEMS sensor devices: Temperature, Humidity, Pressure, Magnetometer, Gyroscope and Accelerometer
		  and Microphones Signal Noise dB level.
		. battery Gas Gauge and SD Data Log (for only SensorTile)
 - The second Service exposes the console services where we have stdin/stdout and stderr capabilities
 - The last Service is used for configuration purpose

 When using a Nucleo board, the user application initialization phase can be monitored via UART.
 Launch a terminal application and set the UART port to 115200 bps, 8 bit, No Parity, 1 stop bit.
 When using a SensorTile board, similar debug messages can be activated if the SD card data logging functionality is disabled.
 It is necessary to re-compile the code enabling the define:
	//#define SENSING1_ENABLE_PRINTF
 on file:
	Projects\Multi\Applications\SENSING1\Inc\SENSING1_config.h

 This enables the UART that starts with a delay of 10 seconds for allowing the time to open a terminal application for looking the initialization phase.

 This example must be used with the related BlueMS Android/iOS application available on Play/itune store (Version 4.1.0 or higher),
 in order to read the sent information by Bluetooth Low Energy protocol

                   ---------------------------------------------
                   | Important Hardware Additional Information |
                   ---------------------------------------------
1) Before to connect X-NUCLEO-IKS01A2 with X-NUCLEO-CCAM02M1 expansion board through the Arduino UNO R3 extension connector with the NUCLEO-L476RG:
   on to X-NUCLEO-IKS01A2 board remove the SB25 0-ohm resistor
2) Before to connect the board X-NUCLEO-CCA02M1 with the STM32 L4 Nucleo motherboard through the Morpho connector layout, on to X-NUCLEO-CCA02M1 board 
   close the solder bridges SB12, SB16 and open the solder bridges SB7, SB15 and SB17

                              ------------------
                              | VERY IMPORTANT |
                              ------------------
 1) This example support the Firmware-Over-The-Air (FOTA) update using the BlueMS Android/iOS application (Version 4.1.0 or higher)
 2) This example must run starting at address 0x08004000 in memory and works ONLY if the BootLoader 
 is saved at the beginning of the FLASH (address 0x08000000)
 
 For each IDE (IAR/µVision/System Workbench), and for each platform (NUCLEO-L476RG/SensorTile),
 there are some scripts *.bat/*.sh that makes the following operations:
 - Full Flash Erase
 - Load the BootLoader on the rigth flash region
 - Load the Program (after the compilation) on the rigth flash region (This could be used for a FOTA)
 - Dump back one single binary that contain BootLoader+Program that could be 
   flashed at the flash beginning (address 0x08000000) (This COULD BE NOT used for FOTA)
 - Reset the board

 
Inside the Binary Directory there are the following binaries:
Binary/
|-- STM32L476RG-Nucleo:
|   +-- SENSING1_HAR_GMP.bin         (Program with HAR_GMP Activity Recognition      without BootLoader. COULD BE USED     for FOTA)
|   +-- SENSING1_HAR_GMP_BL.bin      (Program with HAR_GMP Activity Recognition      with    BootLoader. COULD NOT BE USED for FOTA)
|   |
|   +-- SENSING1_HAR_IGN.bin         (Program with HAR_IGN Activity Recognition      without BootLoader. COULD BE USED     for FOTA)
|   +-- SENSING1_HAR_IGN_BL.bin      (Program with HAR_IGN Activity Recognition      with    BootLoader. COULD NOT BE USED for FOTA)
|   |
|   +-- SENSING1_HAR_IGN_WSDM.bin    (Program with HAR_IGN_WSDM Activity Recognition without BootLoader. COULD BE USED     for FOTA)
|   +-- SENSING1_HAR_IGN_WSDM_BL.bin (Program with HAR_IGN_WSDM Activity Recognition with    BootLoader. COULD NOT BE USED for FOTA)
|   |
|   +-- SENSING1_ASC.bin             (Program with Audio Scene Classification        without BootLoader. COULD BE USED     for FOTA)
|   +-- SENSING1_ASC_BL.bin          (Program with Audio Scene Classification        with    BootLoader. COULD NOT BE USED for FOTA)
|
+-- STM32L476RG-SensorTile:
    +-- SENSING1_HAR_GMP.bin         (Program with HAR_GMP Activity Recognition      without BootLoader. COULD BE USED     for FOTA)
    +-- SENSING1_HAR_GMP_BL.bin      (Program with HAR_GMP Activity Recognition      with    BootLoader. COULD NOT BE USED for FOTA)
    |
    +-- SENSING1_HAR_IGN.bin         (Program with HAR_IGN Activity Recognition      without BootLoader. COULD BE USED     for FOTA)
    +-- SENSING1_HAR_IGN_BL.bin      (Program with HAR_IGN Activity Recognition      with    BootLoader. COULD NOT BE USED for FOTA)
    |
    +-- SENSING1_HAR_IGN_WSDM.bin    (Program with HAR_IGN_WSDM Activity Recognition without BootLoader. COULD BE USED     for FOTA)
    +-- SENSING1_HAR_IGN_WSDM_BL.bin (Program with HAR_IGN_WSDM Activity Recognition with    BootLoader. COULD NOT BE USED for FOTA)
    |
    +-- SENSING1_ASC.bin             (Program with Audio Scene Classification        without BootLoader. COULD BE USED     for FOTA)
    +-- SENSING1_ASC_BL.bin          (Program with Audio Scene Classification        with    BootLoader. COULD NOT BE USED for FOTA)


@par Hardware and Software environment

  - This example runs on Sensor expansion board attached to STM32L476RG device
    can be easily tailored to any other supported device and development board.
    Or it runs on STEVAL-STLKT01V1 (SensorTile) evaluation board

  - This example must be used with the related BlueMS Android/iOS application (Version 4.1.0 or higher) available on
    the Google Play or Apple App Store, in order to read the sent information by Bluetooth Low Energy protocol

@par STM32Cube packages:
  - STM32L4xx drivers from STM32CubeL4 V1.11.0
@par X-CUBE packages:
  - X-CUBE-BLE1 V3.3.0
  - X-CUBE-MEMS1 V4.3.0
  - X-CUBE-MEMSMIC1 V3.0.1
@par STEVAL-STLCS01V1:
  - STEVAL-STLCS01V1 V1.3.3

@par How to use it ?

This package contains projects for 3 IDEs viz. IAR, µVision and System Workbench.
In order to make the  program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:
 - Open IAR toolchain (this firmware has been successfully tested with Embedded Workbench V8.20.2).
 - Open the IAR project file EWARM\STM32F401RE-Nucleo\SENSING1.eww or EWARM\STM32L476RG-SensorTile\SENSING1.eww.
 - Rebuild all files and run the script that you find on the same directory: CleanSENSING1.bat

For µVision:
 - Open µVision toolchain (this firmware has been successfully tested with MDK-ARM Professional Version: 5.22.0)
 - Open the µVision project file MDK-ARM\STM32L476RG-Nucleo\Project.uvprojx or MDK-ARM\STM32L476RG-SensorTile\Project.uvprojx.
 - Rebuild all files and run the script that you find on the same directory: CleanSENSING1.bat
		
For System Workbench:
 - Open System Workbench for STM32 (this firmware has been successfully tested with System Workbench for STM32 Version 2.7.0.201811131241)
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the System
   Workbench project is located (it should be SW4STM32\STM32L476RG-Nucleo\ or SW4STM32\STM32L476RG-SensorTile). 
 - Rebuild all files and and run one of these scripts that you find on the same directory:
   - if you are on windows and you had installed the STM32 ST-Link utility: CleanSENSING1.bat
   - Otherwise (Linux/iOS or Windows without the STM32 ST-Link Utility): CleanSENSING1.sh
		
 /******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
