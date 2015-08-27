#pragma once

#include "AConfig.h"
#if(HAS_BNO055_EXP)


#include <Arduino.h>
#include "Device.h"

class BNO055_EXP : public Device
{
public:
	BNO055_EXP(): Device() {};
	void device_setup();
	void device_loop( Command cmd );
};
#endif


