Product Info:
   Name     C55_Connected_Audio_Framework 
   Platform ALL
   Version  01.52.01.00
   Date     Fri 10/28/2011
   Hash     b2705c04b634227c0dcf25bbbf556ed8bda5b0e8 
 
Release Notes:

================================================================================
Table of Contents
================================================================================
 01.52.01.00-01. Overview
 01.52.01.00-02. Bugs Fixed in This Release

 01.52.00.00-01. Overview
 01.52.00.00-02. Added Features and Bugs Fixed in This Release

 01.51.01.00-01. Overview
 01.51.01.00-02. Bugs Fixed in This Release


================================================================================
 01.52.01.00-01. Overview
================================================================================
This release is a regular update to c55_caf to fix
reported defects.

================================================================================
 01.52.01.00-02. Bugs Fixed in This Release (since version 01.52.00.00)
================================================================================
Documentation
    Updated documentation contained in <install directory>\doc\


================================================================================
 01.52.00.00-01. Overview
================================================================================
This release is a regular update to c55_caf to add new features and fix
reported defects.

================================================================================
 01.52.00.00-02. Added Features and Bugs Fixed (since version 01.51.01.00)
================================================================================
Added Features
1) Updated ASRC algorithm library to include options for 15-taps/phase polyphase
filter and linear interpolation of two polyphase outputs
2) Added power optimizations, including CPU core voltage scaling, clock gating 
of unused peripherals, and idle function usage in DSP/BIOS idle loop. In default
build, CPU core voltage is set to 1.3V to allow system PLL output of 100 MHz. 
These settings can be changed in pll_sample(). Also, by default the idle 
function is commented out (see userIdle() in csl_usb_iso_fullspeed_example.c).
3) Added framework and record datapath support for stereo record. AIC3204 ADC 
needs to be configured AIC3254_init() to fully support stereo record.

Bugs Fixed
1) I2C pre-scaling divider and assumed system clock frequency changed to provide
I2S bus frequency passed to I2C_Init().
2) Fixed PSP enumeration for I2S FS divider.
3) Updated descriptors for case of single playback sampling frequency to not 
have sampling frequency control.
4) Fixed spelling mistake in manufacturer string descriptor.
5) Fixed selection of i2s_txIsr() for various compile-time options.


================================================================================
 01.51.01.00-01. Overview
================================================================================
This release is a regular update to c55_caf to fix
reported defects.

================================================================================
 01.51.01.00-02. Bugs Fixed in This Release (since version 01.51.00.04)
================================================================================
Readme.txt
    Updated the readme.txt file to correct spelling mistakes and to update build 
instructions.

Documentation
    Updated documentation contained in 
<install directory>\connected-audio-documentation-evm.ppt. The DSP clock source 
is now the RTC instead of CLKIN (remove jumper from JP9=>CLK_SEL=0). See slides 
1-2 for details.

Project Directory
    The directory build_script\ under 
<install directory>\c55_caf was mistakenedly named 
build_script_saved\ which caused click2build.bat to fail. This is corrected 
in this release.
 
To build c55_caf project automatically, please follow the steps below: 

1. Make sure CCS 4.2.4 or later is installed in C:\Program Files\Texas Instruments\ccsv4 (Windows XP) or C:\Program Files (x86)\Texas Instruments\ccsv4 (Windows 7)
2. Run C55_Connected_Audio_Framework-xx.xx.xx.xx-Setup.exe installer and select the install directory
3. cd to the install directory and run click2build.bat
4. The project will be created and the CCS4 workspace is at <install directory>\c55_caf



If the automatic build fails, please follow the steps below to build the project using CCS4:
 
1.	Delete the .metadata\ directory from the install if it exists
2.	Open CCS4
3.	Enter <install directory>\c55_caf as the workspace. Click OK
4.	Window->Preferences. Expand CCS. Click RTSC. 
5.	In ISA family, select C5500.
6.	In Code Generation Tool, select TI 4.3.9. If you do not see this option, you can download the codegen tool using either of the following techniques:
	1) Inside CCS, perform the following steps
	Help->Software Updates->Seach for new Features to install
	Select Code Generation Tools Updates
	Click Finish
	Select Code Generation Tools Updates->C5500 Code Generation Tools->C5500 Code Generation Tools 4.3.9
	Click Next
	
	2) Download from the following URL
	https://www-a.ti.com/downloads/sds_support/TICodegenerationTools/download.htm
	

7.	Check DSP/BIOS and check 5.41.10.36 from the menu if you see it. If the selection of DSP/BIOS is successful,  click OK and go to step 11
8.	Click Add. Check Select product version. Check if you see 5.41.10.36 in the menu. If yes. Select it.  Click OK.
9.	If not, Check Select product from the file-system.  Browse the file system and search for the directory where DSP/BIOS 5.41.10.36 is installed (bios_5_41_10_36). Click OK. 
10.	If you do not have 5.41.10.36 installed. You need to download it from the link below
	
	http://software-dl.ti.com/dsps/dsps_public_sw/sdo_sb/targetcontent/dspbios/index.html
 
11.	CCS4 will create a skeleton .metadata\ directory under <install directory>\c55_caf at this point
12.	Use CCS4 to import from import and rebuild the project
13.	Project->Import existing CCS/CCE Eclipse Project. Select <install directory>\c55_caf  as the search directory.
14.	View->C/C++ Projects
15.	Project->Rebuild All
 
