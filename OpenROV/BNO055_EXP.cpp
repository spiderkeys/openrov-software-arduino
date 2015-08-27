#include "AConfig.h"
#if(HAS_BNO055_EXP)

#include "BNO055_EXP.h"
#include "CAdaBNO055.h"
#include "Timer.h"

namespace
{
	bool initalized = false;
	Timer bno055_sample_timer;
	Timer report_timer;

	CAdaBNO055 bno;

	imu::Vector<3> euler;

	void InitializeSensor()
	{
		// Attempt to initialize. If this fails, we try every 30 seconds in its update loop
		if( !bno.Initialize() )
		{
			Serial.println( "BNO_INIT_STATUS:FAILED;" );
			initalized = false;
		}
		else
		{
			Serial.println( "BNO_INIT_STATUS:SUCCESS;" );


			Serial.print( "BNO055.SW_Revision_ID:" );
			Serial.print( bno.m_softwareVersionMajor, HEX );
			Serial.print( "." );
			Serial.print( bno.m_softwareVersionMinor, HEX );
			Serial.println( ";" );

			Serial.print( "BNO055.bootloader:" );
			Serial.print( bno.m_bootloaderRev );
			Serial.println( ";" );

			initalized = true;
		}
	}
}


void BNO055_EXP::device_setup()
{
	InitializeSensor();

	// Reset timers
	bno055_sample_timer.reset();
	report_timer.reset();
}

void BNO055_EXP::device_loop( Command command )
{
	// 1000 / 21
	if( bno055_sample_timer.elapsed( 47 ) )
	{
		if( !initalized )
		{
			// Attempt every 30 secs
			if( report_timer.elapsed( 30000 ) )
			{
				// Attempt to initialize the chip again
				device_setup();
			}

			return;
		}

		// System status checks
		if( report_timer.elapsed( 1000 ) )
		{
			// System calibration
			if( bno.GetCalibration() )
			{
				Serial.print( "BNO055.CALIB_MAG:" );
				Serial.print( bno.m_magCal );
				Serial.println( ';' );
				Serial.print( "BNO055.CALIB_ACC:" );
				Serial.print( bno.m_accelCal );
				Serial.println( ';' );
				Serial.print( "BNO055.CALIB_GYR:" );
				Serial.print( bno.m_gyroCal );
				Serial.println( ';' );
				Serial.print( "BNO055.CALIB_SYS:" );
				Serial.print( bno.m_systemCal );
				Serial.println( ';' );
			}
			else
			{
				Serial.print( "BNO055.CALIB_MAG:" );
				Serial.print( "N/A" );
				Serial.println( ';' );
				Serial.print( "BNO055.CALIB_ACC:" );
				Serial.print( "N/A" );
				Serial.println( ';' );
				Serial.print( "BNO055.CALIB_GYR:" );
				Serial.print( "N/A" );
				Serial.println( ';' );
				Serial.print( "BNO055.CALIB_SYS:" );
				Serial.print( "N/A" );
				Serial.println( ';' );
			}

			// Operating mode
			if( bno.GetOperatingMode() )
			{
				Serial.print( "BNO055.MODE:" );
				Serial.print( bno.m_operatingMode );
				Serial.println( ';' );
			}
			else
			{
				Serial.print( "BNO055.MODE:" );
				Serial.print( "N/A" );
				Serial.println( ';' );
			}

			// System status
			if( bno.GetSystemStatus() )
			{
				Serial.print( "BNO055_STATUS:" );
				Serial.print( bno.m_systemStatus, HEX );
				Serial.println( ";" );
			}
			else
			{
				Serial.print( "BNO055_STATUS:" );
				Serial.print( "N/A" );
				Serial.println( ";" );
			}

			// System Error
			if( bno.GetSystemError() )
			{
				Serial.print( "BNO055_ERROR_FLAG:" );
				Serial.print( bno.m_systemError );
				Serial.println( ";" );
			}
			else
			{
				Serial.print( "BNO055_ERROR_FLAG:" );
				Serial.print( "N/A" );
				Serial.println( ";" );
			}

		}

		// Get orientation data
		if( bno.GetVector( CAdaBNO055::VECTOR_EULER, euler ) )
		{
			// These may need adjusting
			navdata::PITC = euler.z();
			navdata::ROLL = -euler.y();
			navdata::YAW = euler.x();
			navdata::HDGD = euler.x();
		}
	}
}
#endif
