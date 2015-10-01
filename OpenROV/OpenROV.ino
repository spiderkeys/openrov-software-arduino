// Includes
#include <EEPROM.h>
#include <SPI.h>

#include "NArduinoManager.h"
#include "NConfigManager.h"
#include "NDataManager.h"
#include "NCommManager.h"
#include "NModuleManager.h"

Settings settings;


#if(HAS_STD_CAPE)
  #include "Cape.h"
  Cape cape;
#endif

#if(HAS_OROV_CONTROLLERBOARD_25)
  #include "controllerboard25.h"
  Controller25 controller;
#endif

#if(HAS_STD_LIGHTS)
  #include "Lights.h"
  Lights lights;
#endif

#if(HAS_STD_CALIBRATIONLASERS)
  #include "CalibrationLaser.h"
  CalibrationLaser calibrationLaser;
#endif

#if(HAS_STD_2X1_THRUSTERS)
  #include "Thrusters2X1.h"
  Thrusters thrusters;
#endif

#if(HAS_STD_PILOT)
  #include "Pilot.h"
  Pilot pilot;
#endif

#if(HAS_STD_CAMERAMOUNT)
  #include "CameraMount.h"
  CameraMount cameramount;
#endif



#if(HAS_POLOLU_MINIMUV)
  #define COMPASS_ENABLED 1
  #define GYRO_ENABLED 1
  #define ACCELEROMETER_ENABLED 1
  #include "MinIMU9.h"
  #include <Wire.h> //required to force the Arduino IDE to include the library in the path for the I2C code
  MinIMU9 IMU;
#endif

#if(HAS_MPU9150)
  #define COMPASS_ENABLED 1
  #define GYRO_ENABLED 1
  #define ACCELEROMETER_ENABLED 1
  #include "MPU9150.h"
  #include <Wire.h> //required to force the Arduino IDE to include the library in the path for the I2C code

  MPU9150 IMU;
#endif

#if(HAS_MS5803_XXBA)
  #define DEPTH_ENABLED 1
  #include "MS5803_XXBA.h"
  #include <Wire.h> //required to force the Arduino IDE to include the library in the path for the I2C code
  #include <SPI.h> //required to force the Arduino IDE to include the library in the path for the SPI code
  MS5803_XXBA DepthSensor;
#endif

#if(DEADMANSWITCH_ON)
  #include "DeadManSwitch.h"
  DeadmanSwitch DMS;
#endif

#if(HAS_BNO055)
  #define COMPASS_ENABLED 1
  #define GYRO_ENABLED 1
  #define ACCELEROMETER_ENABLED 1
  #include "BNO055.h"
  #include <Wire.h> //required to force the Arduino IDE to include the library in the path for the I2C code
  BNO055 IMU2;
#endif

// include plugins
#include "PluginConfig.h"

Command cmd;

volatile byte wdt_resets = 0; //watchdog resets

// IMPORTANT!
// array[0] will be the number of arguments in the command
int array[MAX_ARGS];
Timer Output1000ms;
Timer Output100ms;
int loops_per_sec;

/* sets the watchdog timer both interrupt and reset mode with an 8 second timeout */
void enableWatchdog()
{
	// Initialize main subsystems
	NArduinoManager::Initialize();
	NCommManager::Initialize();
	NConfigManager::Initialize();
	NModuleManager::Initialize();
	NDataManager::Initialize();

	// Boot complete
	Serial.println( F( "boot:1;" ) );
}

void loop()
{
	// Reset the watchdog timer
	wdt_reset();

	// Attempt to read a current command off of the command line
	NCommManager::GetCurrentCommand();

	// Handle any config change requests
	NConfigManager::HandleMessages( NCommManager::m_currentCommand );

	// Handle update loops for each module
	NModuleManager::HandleModuleUpdates( NCommManager::m_currentCommand );

	// Handle update loops that send data back to the beaglebone
	NDataManager::HandleOutputLoops();
}
