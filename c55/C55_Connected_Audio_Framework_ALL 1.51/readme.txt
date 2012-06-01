Product info:
   Name     C55_Connected_Audio_Framework 
   Platform ALL
   Version  01.51.00.04
   Hash     c4b8071921d2c77fd76519c1b73f007da35f804e 
 
To build c55-connected-audio-framework project automatically, please follow the steps below: 

1. Make sure CCS4 4.2.4 or later is insatlled in C:\Program Files\Texas Instruments\ccsv4
2. Make sure DSP/BIOS 5.41.10.36 is installed in C:\Program Files\Texas Instruments\bios_5_41_10_36
3. Make sure Codegn tool 4.3.6 or 4.3.9 for CCS4 is insatlled in C:\Program Files\Texas Instruments\....
4. Run C55_Connected_Audio_Framework-xx.xx.xx.xx-Setup.exe installer and select the install directory
5. cd to the install directory and run click2build.bat
6. The project will be created and the CCS4 workspace is at <install directory>\c55-connected-audio-framework



If the automatic build fails, please follow the steps below to build the project using CCS4:
 
1.	Delete the .metadata directory mentioned above from the install if it exists
2.	Open CCS4
3.	Enter <install directory>\c55-connected-audio-framework as the workspace. Click OK
4.	Window->Preferences. Expand CCS. Click RTSC. 
5.	In ISA family, select C5500.
6.	In Code Generation Tool, select TI 4.3.6 or 4.3.9. If you do not see this option, you need to download the codegen tool from 
	
	https://www-a.ti.com/downloads/sds_support/TICodegenerationTools/download.htm

7.	Check DSP/BIOS and check 5.41.10.6 from the menu if you see it. If the selection of DSP/BIOS is successful,  click OK and go to step 11
8.	Click Add. Check Select product version. Check if you see 5.41.10.36 in the menu. If yes. Select it.  Click OK.
9.	If not, Check Select product from the file-system.  Browse the file system and search for the directory where DSP/BIOS 5.41.10.36 is installed (bios_5_41_10_36). Click OK. 
10.	If you do not have 5.41.10.36 installed. You need to download it from the link below
	
	http://software-dl.ti.com/dsps/dsps_public_sw/sdo_sb/targetcontent/dspbios/index.html
 
11.	CCS4 will create a skeleton .metadata\ directory under <install directory>\c55-connected-audio-framework at this point
12.	Use CCS4 to import from import and rebuild the project
13.	Project->Import existing CCS/CCE Eclipse Project. Select <install directory>\c55-connected-audio-framework  as the search directory.
14.	View->C/C++ Projects
15.	Project->Rebuild All
