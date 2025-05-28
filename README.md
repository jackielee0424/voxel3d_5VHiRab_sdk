# voxel3d_5VHiRab_sdk
Library and utilities for 5Voxel 5VHiRab all-in-one RGB-D &amp; thermal camera  
* For usage of voxel3d library  
  1. Open voxel3d_5VHiRab_sdk/html/index.html in browser  
  2. Click "Files -> File List", then "voxel3d.h" for detail info  

-------------------------------------------------------------------------------
# Windows
Usage: voxel3d_tools.exe [options]  
  
Version 1.5  
Options:  
&emsp;-h | --help&emsp;&emsp;&emsp;&emsp;&emsp;&nbsp;&nbsp;Print this message  
&emsp;-A | --set_auto_expo&emsp;set auto exposure mode  
&emsp;-b | --build_date&emsp;&emsp;&nbsp;&nbsp;&nbsp;show firmware build date  
&emsp;-i | --show_info&emsp;&emsp;&emsp;&nbsp;show device info  
&emsp;-S | --scan_dev&emsp;&emsp;&emsp;&nbsp;&nbsp;scan devices and list device S/N  
&emsp;-s | --dev_sn&emsp;&emsp;&emsp;&emsp;&nbsp;&nbsp;specify device S/N to access  
&emsp;-t | --get_conf&emsp;&emsp;&emsp;&nbsp;&nbsp;&nbsp;get confidence threshold  
&emsp;-T | --set_conf&emsp;&emsp;&emsp;&nbsp;&nbsp;&nbsp;set confidence threshold  
&emsp;-u | --fw_upgrade&emsp;&emsp;device firmware upgrade  
&emsp;-v | --version&emsp;&emsp;&emsp;&emsp;&nbsp;show lib & firmware version  
  
  
Example:  
voxel3d_tools.exe
  
  
Supported Deivce(s)
-------------------------------------------------------------------------------
5Voxel 5VHiRab series  
1. 5VHiRab887CF60  
2. 5VHiRab887CF80  
3. 5VHiRab976F60  
4. 5VHiRab976F80  
5. 5VHiRabv1976F80  

Supported OS/Platform
-------------------------------------------------------------------------------
$ Windows11 (Visual Studio Community 2022)  
  
  
Kernel module needed
-------------------------------------------------------------------------------

Build steps
-------------------------------------------------------------------------------
1. Install Vistual Studion Community 2022  
2. Go to platform/win and click on 'voxel3d_tools.sln' (Visual Studio Solution File)  
3. Build x64 release project  
4. Open a command window and go to following directory  
        platform/win/Bin/x64-Release/voxel3d_tools  
5. Execute 'voxel3d_tools.exe -h' to show menu  
