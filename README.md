<div align="center">
  <img width="1600" height="400" alt="goBILDA-Pinpoint-Combo-Photo" src="https://github.com/user-attachments/assets/a4da71cb-ba2c-428e-936a-b5cb205b31c7" />
</div>

<div align="center">
	<h1>goBILDA Pinpoint Odometry Computer</h1>
</div>
  	
<p>
Help your robot know exactly where it’s at with the Pinpoint Odometry Computer! This coprocessor performs sensor fusion between two Dead-Wheel Odometry Pods (like SKU: 3110-0001-0001 or 3110-0001-0002) and an internal IMU to locate your robot to a precise pinpoint. It reads your robot’s estimated position back to the controller via I²C, giving you access to data in a snap.
</p>

## Features
* **Full 3D Orientation Estimation** - Track X, Y position and heading angle
* **User-Configurable Bulk-Read Windows** - Optimize data transfer for your application
* **CRC8 Data Protection** - Ensures data integrity over I²C communication

## Hardware Requirements
* 1x [goBILDA Pinpoint Odometry Computer](https://www.gobilda.com/pinpoint-v2-odometry-computer-imu-sensor-fusion-for-2-wheel-odometry/)
* 2x Identical Dead-Wheel Odometry Pods:
  * [4-Bar Odometry Pod (32mm Wheel)](https://www.gobilda.com/4-bar-odometry-pod-32mm-wheel/)
  * [Swingarm Odometry Pod (48mm Wheel)](https://www.gobilda.com/swingarm-odometry-pod-48mm-wheel/)
* 1x 3.3v Arduino-compatible microcontroller with I²C support

## Installation

### Arduino Library Manager

* Open the Arduino IDE and select the Library Manager on the left side of the screen
* Type "goBILDA Pinpoint" in the search bar and select "INSTALL"

### Manual Install

* Download the library by clicking [here](https://github.com/goBILDA-Official/goBILDA-Pinpoint-Arduino-Library/archive/refs/heads/main.zip)
* Unzip the zip file by right clicking it and selecting "Extract All..."
* Extract the zip file to your Arduino libraries folder:
  * Windows: `Documents\Arduino\libraries`
  * macOS: `~/Documents/Arduino/libraries`
  * Linux: `~/Arduino/libraries`

## Examples

The goBILDA Pinpoint library includes example sketches to help you get started quickly.

### Accessing Examples in Arduino IDE

After installing the library, you can access the example sketches:

1. Open the Arduino IDE
2. Click on **File** -> **Examples** -> **goBILDA Pinpoint** (scroll down to find it under "Examples from Custom Libraries")
3. Select an example sketch to open it in a new window

## Setup
1. Include the goBILDA Pinpoint Library
```cpp
#include <goBILDA_Pinpoint.h>
```

2. Create the Pinpoint object that will communicate with your pinpoint and initialize communication
```cpp
goBILDA::Pinpoint pinpoint;

void setup(){
	pinpoint.begin();

  pinpoint.setPosX(0.0);                                                                                  // Set the current x-position        (millimeters)
  pinpoint.setPosY(0.0);                                                                                  // Set the current y-position        (millimeters)
  pinpoint.setHeading(0.0);                                                                               // Set the current heading           (radians)
  pinpoint.setOffsets(0.0, 0.0);                                                                          // X/Y offset from the robots center (millimeters)
  pinpoint.setEncoderResolution(goBILDA::EncoderResolution::goBILDA_4_BAR_POD);                           // Set the encoder resolution        (millimeters per tick)
  pinpoint.setEncoderDirections(goBILDA::EncoderDirection::Forward, goBILDA::EncoderDirection::Forward);  // Set X and Y encoder directions
}
```

## Usage
You can use this library in many ways, but here are a few ways to grab data from the Odometry Computer.
```cpp
	float x_position = pinpoint.getPositionXInMM();
    float y_position = pinpoint.getPositionYInMM();
    float heading    = pinpoint.getNormalizedHeading();
```
