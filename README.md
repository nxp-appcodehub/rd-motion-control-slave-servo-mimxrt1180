# NXP Application Code Hub
[<img src="https://mcuxpresso.nxp.com/static/icon/nxp-logo-color.svg" width="100"/>](https://www.nxp.com)

## How to set up the motion control reference design on iMX.RT1180


This application demonstrates how to implement EtherCAT+  PMSM motor control on the iMX.RT1180. 
The reference design includes two projects, the master project and the slave project.The Master project is responsible for completing the processing of the EtherCAT master stack, and the Slave project is responsible for completing the processing of the EtherCAT slave stack and servo motor control.Based on this reference design, two demos were made for demonstration:
**Demo1:** 1 master with four slave nodes, each node controls two motors. Two motors from different slave nodes are grouped together, and the synchronization performance of EtherCAT is demonstrated through laser penetration and shielding.
**Demo2:** 1 master with two slave nodes, each node controls two motors.Motor 1 on different slave boards is responsible for controlling the left and right movement of the motor 2 platform. The two motors 2 on different boards will move synchronously when they are close to each other, and will move separately when they are separated.


#### Boards: XSERVO-MTR-DRV Board, MIMXRT1180-EVKC Board
#### Categories: Motor Control, Networking
#### Peripherals: ADC, PWM, ETHERNET
#### Toolchains: MCUXpresso IDE

## Table of Contents
1. [Software](#step1)
2. [Hardware](#step2)
3. [Setup](#step3)
4. [Results](#step4)
5. [Support](#step5)
6. [Release Notes](#step6)

## 1. Software<a name="step1"></a>
- [MCUXpresso IDE](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE) 
- [SDK for EVK-MIMXRT1180](https://mcuxpresso.nxp.com/en/builder?hw=MIMXRT1180-EVK) 

- [Download the code from Git repository ](https://github.com/nxp-appcodehub/rd-motion-control-slave-servo-mimxrt1180/tree/main). 
- [Freemaster 3.2](https://www.nxp.com.cn/design/design-center/software/development-software/freemaster-run-time-debugging-tool:FREEMASTER)

## 2. Hardware<a name="step2"></a>
- Personal Computer
- NET Cable and 24V adaptor.
- XSERVO-MTR-DRV Board and [MIMXRT1180-EVK](https://www.nxp.com.cn/products/i.MX-RT1180)
- J-Link Debugger ([v7.88e](https://www.segger.com/downloads/jlink/))

## 3. Setup<a name="step3"></a>

### 3.1 Step 1
#### Slave Project
**If you have download the project, please see the next section.**
1. Open MCUXpresso IDE, in the Quick Start Panel, choose **Import from Application Code Hub**

   ![](images/import_project_1.png)
2. Enter the demo name in the search bar.

   ![](images/import_project_2.png)
3. Click **Copy GitHub link**, MCUXpresso IDE will automatically retrieve project attributes, then click **Next>**.

   ![](images/import_project_3.png)
4. Select **main** branch and then click **Next>**, Select the MCUXpresso project, click **Finish** button to complete import.

   ![](images/import_project_4.png)
5. Click **Build** to start compiling the project.

   ![](images/build_project.png)

####   If you have download this project:
1. Click **file** to import the project.

   ![](images/import_project_5.png)

2. Follow the steps below to import 'motion_control_servo_rt1180_cm33' project.

   ![](images/import_project_6.png)
3. Same as step 2, import 'motion_control_servo_rt1180_cm7' project.
#### Master Project
1. Import the ‘evkmimxrt1180_soem_servo_motor_cm33’ project.
 ![](images/import_project_8.png)

2. Manually apply the changes listed in the file `rd-motion-control-slave-servo-mimxrt1180\changes.diff` to the project that has already been imported.


### 3.2 Step 2
#### MIMXRT-1180-EVK RevC board set up
**If you just don't want to use SOEM, please see the next section.**
![](./images/Board1.jpg)
1. 5V supply via J2 (JP1 pin1 and pin2 are connected using a jumper).  

2. Connect J-Link Debugger to J37 of the MIMXRT-1180-EVK RevC board.

3. Import '[motion_control_master_soem]()'  project to MCUXpresso IDE.

4. Choose the SDP mode.(SW5->off-off-off-on).

5. Build motion_control_master_soem project（**Debug build**） and download the project to the board(JP5 need be connected with jumper).

6. Choose the Flashboot mode.(SW5->off-on-off-off).

7. Reset borad.D31 will be light up all the time.

8. Connect the ETH4 to the Slave network port using the network cable.

#### XSERVO-MTR-PSG RevA board set up
![](./images/Board2.jpg)
1. 5V supply via J16/17. 

2. Connect J-Link Debugger to J15 of the XSERVO-MTR-PSG RevA board.

3. Import ['motion_control_servo_rt1180_cm33' and 'motion_control_servo_rt1180_cm7']() project to MCUXpresso IDE.

4. Choose the SDP mode.(SW2->off-off-off-on).

5. Build 'motion_control_servo_rt1180_cm7' project first（**Release build**）.(Right click the project and perform the following operations)
![](./images/Board5.png)

6. Build 'motion_control_servo_rt1180_cm33'project（**Release build**）.(same as step 5)
7. To import the downloaded SDK file, just drag the compressed package into the specified project, as shown below:
![](images/import_project_7.png)
7. Download the project to the board.

**Note:** This project **CAN NOT** be built directly. Please see the below.Details can be found in chapters 4 to 6.3 of the document in the path '\rd-motion-control-imxrt1180\motion_control_servo_rt1180_cm33\docs'.

**Please use the patch file in the project (..\motion_control_servo_rt1180_cm33\ecat\SSC\MotionControl.patch).**

**（1）** Download the patch.exe and Unix2Dos.exe tools.

**（2）** Use the Unix2Dos.exe tool to convert the patch file format.
![](./images/patch1.jpg)
![](./images/patch2.jpg)


**（3）** Apply patch by using the command.
![](./images/patch3.jpg)
![](./images/patch4.jpg)
![](./images/patch5.jpg)
##### Prepare the Demo
```
1. Generate the SSC source code
	- Download Slave Stack Code Tool(SSC Tool) from BECKHOFF official website and install it
	- Open Slave Stack Code Tool
	- Click 'File' -> 'New' -> 'Import' to import the Congfig file
		Select servo_motor.xml under path '.\boards\evkmimxrt1180\ecat_examples\servo_motor\<core>\SSC'.
	- Click 'Project' -> 'Create new Slave Files'
		'Source Folder'(default): '.\boards\evkmimxrt1180\ecat_examples\servo_motor\<core>\SSC\Src'.
		Click 'Start'.
	
2. Copy SSC souce code to '\rd-motion-control-imxrt1180\motion_control_servo_rt1180_cm33\ecat\SSC\Src', and change to '.\rd-motion-control-imxrt1180\motion_control_servo_rt1180_cm33\ecat\SSC' path to apply the patch.
	Linux:
		1. Download dos2unix command
			apt-get install dos2unix
		2. Transfer SSC source code format
			dos2unix Src/*
		3. Apply patch
			patch -d Src <  CiA402-combine-the-SSC-slave-with-ec_pmsm-support.patch
	Windows:
		1. Download patch.exe and Unix2Dos.exe tool
			Download Windows Patch Utility from http://gnuwin32.sourceforge.net/downlinks/patch-bin-zip.php .
			Download Dos2Unix/Unix2Dos-Text file format converters from https://sourceforge.net/projects/dos2unix/ .
		2. Transfer the patch format
			$(Dos2Unix/Unix2Dos-DIR)/bin/unix2dos.exe CiA402-combine-the-SSC-slave-with-ec_pmsm-support.patch
		3. Apply patch
			patch.exe -i CiA402-combine-the-SSC-slave-with-ec_pmsm-support.patch -d ./Src

3. Download the program to the Master board

4. Update ESI file to EEPROM
	- Copy ESI file 'ECAT-SERVO-MOTOR.xml' generated by SSC TOOL to <TwinCAT_installation_folder>/<Version>/Config/io/EtherCAT/
	- Restart TwinCAT3 IDE
	- Scan device and Update EEPROM
		
```

7. Choose the Flashboot mode.(SW2->off-on-off-off).

8. Reset borad through SW6.LED1 will light up every 1s.

9. Connect the power and encoder port of the motor1&2.

10. Connect the network cable, J3 is the input of slave and J7 is the output of slave.

### 3.2 Step 3
#### Demo1 set up
1. Network cable connection between master and slave.

![](./images/Board3.jpg)

2. 8 motors and 4 slave boards mounted to brackets.

![](./images/Board4.png)

3. Two 24V power supplies power the four boards. One power supply powers boards 1 and 2 /1 and 3, the other powers boards 3 and 4 /2 and 4

3. Make sure the motors are all in the correct position before powering up (is the laser able to pass through the round hole).

4. 24V power up for slave board.

5. 5V power up for master board.

#### Demo2 set up
1. Network cable connection between master and slave.

![](./images/Demo2.jpg)

2. Install the four motors onto the slide and tighten the couplings.
![](./images/Board11.png)

3. Docking the M2 gears on both boards together as shown above.

4. Rotate the motor couplings on both sides to turn the position of the two motors in the center to the position, the two motors are docked in the center. At the same time, adjust the relative positions of the two motors according to the figure below.
![](./images/Board12.png)

4. 24V power up for slave board.

5. 5V power up for master board.

#### Only debug motor
1. Open C:\Users\nxf87476\Desktop\MotionControl\motion_control_servo_rt1180\motion_control_servo_rt1180_cm7\source\motor_control_task.c file.
2. Change the macro **DEBUGMOTOR** form 0 to 1.
![](./images/Board6.png)
3. Bulid project and download the image.
4. Connect the J17 USB cable, which used for freemaster.
5. Open the freemaster file in ...\motion_control_servo_rt1180\motion_control_servo_rt1180_cm7\freemaster_exe\pmsm_demo_QUAD_release.pmp.
![](./images/Board7.png)
6. Click **run** .
![](./images/Board8.png)
7. Open the Position Controller window.
![](./images/Board9.png)
8. Select Position Ctrl Mode.
![](./images/Board13.png)
9. Change the Switch from off to on.
![](./images/Board14.png)
10. Enter you target position in **M1 Position Cmd Required**.


## 4. Results<a name="step4"></a>
Connect a USB cable between the host PC and the OpenSDA USB port on the Master board.Open a serial terminal with the following settings
	- 115200 baud rate
	- 8 data bits
	- No parity
	- One stop bit
	- No flow control
The log below shows the output of this demo in the terminal window:
```
Init success!

Init 3 success!

Start SOEM SERVO MOTOR EXAMPLE!

NETC EP frame loopback example start.
Starting motion task
ec_init on netc0 succeeded.
ec_config_init 0
1 slaves found and configured.
ec_config_map_group IOmap:20001214 group:0
 >Slave 1, configadr 1001, state  2
  CoE Osize:192 Isize:192
	 ISIZE:192 192 OSIZE:192
  SM programming
	SM2 Type:3 StartAddr:1100 Flags:   10064
	SM3 Type:4 StartAddr:1400 Flags:   10020
  OUTPUT MAPPING
	FMMU 0
	  SM2
	slave 1 Outputs 20001214 startbit 0
 =Slave 1, INPUT MAPPING
	FMMU 1
	  SM3
	Inputs 2000122C startbit 0
IOmapSize 48
Slaves mapped, state to SAFE_OP.
segments : 1 : 48 0 0 0
Request operational state for all slaves
Calculated workcounter 3
Request operational state for all slaves
Calculated workcounter 3
Operational state reached for all slaves.
```
#### Demo1 Result
[Demo1 Video](https://nxp1.sharepoint.com/teams/23_49/Shared%20Documents/Demo%20Videos/i.MX%20RT%20&%20DSC/RT1180Demo/Demo1.mp4)
#### Demo2 Result
[Demo2 Video](https://nxp1.sharepoint.com/teams/23_49/Shared%20Documents/Demo%20Videos/i.MX%20RT%20&%20DSC/RT1180Demo/Demo2.mp4)
## 5. Support<a name="step5"></a>
*Provide URLs for help here.*

#### Project Metadata

<!----- Boards ----->

<!----- Categories ----->
[![Category badge](https://img.shields.io/badge/Category-MOTOR%20CONTROL-yellowgreen)](https://github.com/search?q=org%3Anxp-appcodehub+motor_control+in%3Areadme&type=Repositories)
[![Category badge](https://img.shields.io/badge/Category-NETWORKING-yellowgreen)](https://github.com/search?q=org%3Anxp-appcodehub+networking+in%3Areadme&type=Repositories)

<!----- Peripherals ----->
[![Peripheral badge](https://img.shields.io/badge/Peripheral-ADC-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+adc+in%3Areadme&type=Repositories)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-PWM-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+pwm+in%3Areadme&type=Repositories)
[![Peripheral badge](https://img.shields.io/badge/Peripheral-ETHERNET-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+ethernet+in%3Areadme&type=Repositories)

<!----- Toolchains ----->
[![Toolchain badge](https://img.shields.io/badge/Toolchain-MCUXPRESSO%20IDE-orange)](https://github.com/search?q=org%3Anxp-appcodehub+mcux+in%3Areadme&type=Repositories)

Questions regarding the content/correctness of this example can be entered as Issues within this GitHub repository.

>**Warning**: For more general technical questions regarding NXP Microcontrollers and the difference in expected functionality, enter your questions on the [NXP Community Forum](https://community.nxp.com/)

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/NXP_Semiconductors)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/nxp-semiconductors)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/nxpsemi/)
[![Follow us on Twitter](https://img.shields.io/badge/X-Follow%20us%20on%20X-black.svg)](https://x.com/NXP)

## 6. Release Notes<a name="step6"></a>
| Version | Description / Update                           | Date                        |
|:-------:|------------------------------------------------|----------------------------:|
| 1.0     | Initial release on Application Code Hub        | September 9<sup>th</sup> 2024 |

