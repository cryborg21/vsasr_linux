# vsasr_linux

README in Japanese/日本語解説記事:http://cryborg.hatenablog.com/entry/2016/09/04/153224

C/C++ samples for Linux to manipulating Vstone VS-ASR robot.

Vstone Academic Scara Robot VS-ASR  
https://www.vstone.co.jp/products/scara_robot/index.html

This software includes the work that is distributed in
https://www.vstone.co.jp/products/scara_robot/download.html#03-3

## Description

### position_to_axis
Linux version of [the PositionToAxis sample for Windows](https://www.vstone.co.jp/products/scara_robot/download/scaraSample_PotisionToAxis.cpp).  
You can move the robot's arm by setting target xyz-position, yaw and width of crew.

### get_position
This sample works almost the same with [the GetMotorAxis sample for Windows](https://www.vstone.co.jp/products/scara_robot/download/scaraSample_GetMotorAxis.cpp), but the controller functions are implemented with a class.  
You can get the robot's current xyz-position, yaw and width of crew with 60 Hz.

## Requirements

* Ubuntu 16.04
* cmake 3.5.1
* Silicon Labs CP2110/4 SDK v1.31
  * The author found that the latest SDK v6.7.2 is not suitable for the VS-ASR robot. The communication rate with the robot would be limited to about 1Hz under the SDK v6.7.2.
  * The version of the SDK is found on [Release Notes](http://www.silabs.com/products/interface/Pages/CP2110EK.aspx).

## Usage

### Installation - CP2110/4 HID USB-to-UART SDK v1.31

1. Download [old SDK(v1.31) for linux](https://web.archive.org/web/20140901060026/http://www.silabs.com/products/interface/Pages/CP2110EK.aspx).
* Add execute permission to it, and run.
  * `chmod 755 CP2110_4_linux.bin`
  * `./CP2110_4_Linux.bin`
* Install to default directory `~/Silabs/MCU`.
* Unlink useless symbolic links.
  * `unlink CP2110_4 Device Customization Utility (AN721)`
  * `unlink Uninstall CP2110_4 Software Development Kit`
  * If you want to uninstall, just remove `~/Silabs` directory.
* Copy includes and libraries.
  * Make `/usr/local/include/silabs` and `/usr/local/lib/silabs` directories.
  * Copy `~/Silabs/MCU/CP2110_4_SDK/Library/Linux/*.h` files to `/usr/local/include/silabs`.
  * Copy `~/Silabs/MCU/CP2110_4_SDK/Library/Linux/libslabhidtouart.so.1.0` to `/usr/local/lib/silabs`.
* Generate two symblic links of .so file.
  * `cd /usr/local/lib/silabs`
  * `sudo ln -s libslabhidtouart.so.1.0 libslabhidtouart.so.1`
  * `sudo ln -s libslabhidtouart.so.1.0 libslabhidtouart.so`

### Build
1. `git clone https://github.com/cryborg21/vsasr_linux.git`
* `cd vsasr_linux`
* `mkdir build`
* `cd build`
* `cmake ../`
* `make`
* Two executables `position_to_axis`,`get_position` will be generated to your `vsasr_linux/build` directory.

### Run

Permit access to the usb device.

1. Connect the robot with your computer via USB.
* Check the USB Bus and Device number that the robot connected.
  * `lsusb`
  * You will find the line like:
    * `> Bus 001 Device 007: ID 10c4:ea80 Cygnal Integrated Products, Inc. CP210x UART Bridge`
    * In this case, your robot is connected to USB Bus **001**, Device **007**. These numbers would be alterd depending on your environment.
* Permit read/write access to the device.
  * `cd /dev/bus/usb/001`
  * `sudo chmod 666 007`


If you did not permit access to the usb, you need to run executables with `sudo`.

* Run position_to_axis
  * `./position_to_axis`

* Run get_position: you must set the num of servos as argument.
  * `./get_position 3` or `./get_position 5`

## Author

[@cryborg21](https://github.com/cryborg21)  
Copyright (c) 2016 @cryborg21
