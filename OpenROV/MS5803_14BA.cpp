#include "AConfig.h"
#if(HAS_MS5803_14BA)

#include "Device.h"
#include "MS5803_14BA.h"
#include "Settings.h"
#include "Timer.h"
#include "MS5803_14BALib.h"

#include "I2C.h"

/*
Sketch to read a MS5803-14BA pressure sensor, written from scratch.
Will output data to the serial console.

Written by Walt Holm
Initial revision 10 Oct 2013
Rev 1 12 Oct 2013 -- Implements 2nd order temperature compensation
*/


namespace
{
	const uint8_t DevAddress = MS5803___BA_I2C_ADDRESS;  // 7-bit I2C address of the MS5803

	// Here are the commands that can be sent to the 5803
	// Page 6 of the data sheet
	const byte Reset = 0x1E;
	const byte D1_256 = 0x40;
	const byte D1_512 = 0x42;
	const byte D1_1024 = 0x44;
	const byte D1_2048 = 0x46;
	const byte D1_4096 = 0x48;
	const byte D2_256 = 0x50;
	const byte D2_512 = 0x52;
	const byte D2_1024 = 0x54;
	const byte D2_2048 = 0x56;
	const byte D2_4096 = 0x58;
	const byte AdcRead = 0x00;
	const byte PromBaseAddress = 0xA0;
	const bool FreshWater = 0;
	const bool SaltWater = 1;
	//io_timeout is the max wait time in ms for I2C requests to complete
	const int io_timeout = 20;


	unsigned int CalConstant[8];  // Matrix for holding calibration constants

	int32_t AdcTemperature, AdcPressure;  // Holds raw ADC data for temperature and pressure
	float Temperature, Pressure, TempDifference, Offset, Sensitivity;
	float T2, Off2, Sens2;  // Offsets for second-order temperature computation
	float AtmosPressure = 1015;
	float Depth;
	float DepthOffset = 0;
	float WaterDensity = 1.019716;
	Timer DepthSensorSamples;
	Timer InitTimer;
	byte ByteHigh, ByteMiddle, ByteLow;  // Variables for I2C reads
	bool WaterType = FreshWater;

	bool isInitialized = false;

	// Program initialization starts here

	bool WriteRegisterByte( uint8_t addressIn )
	{
		uint8_t ret = I2c.write( DevAddress, addressIn );

		// Non-zero is failure
		if( ret )
		{
			return false;
		}

		return true;
	}

	bool WriteDataByte( uint8_t addressIn, uint8_t dataIn )
	{
		uint8_t ret = I2c.write( DevAddress, addressIn, dataIn );

		// Non-zero is failure
		if( ret )
		{
			return false;
		}

		return true;
	}

	bool ReadByte( uint8_t addressIn, uint8_t& dataOut )
	{

		// Set address to request from
		uint8_t ret = I2c.read( DevAddress, ( uint8_t )addressIn, ( uint8_t )1 );

		// Non-zero failure
		if( ret )
		{
			return false;
		}

		// Request single byte from slave
		dataOut = I2c.receive();

		return true;
	}

	bool GetDepthAndTemperature()
	{

		// Signal sensor to perform pressure ADC conversion (takes 8.22ms)
		WriteRegisterByte( D1_512 );

		// Todo: break these waits up into a state machine so that the rest of the system doesn't have to wait for the pressure sensor

		// Signal intent to read adc value
		WriteRegisterByte( AdcRead );

		// Read three bytes from the device
		if( I2c.read( DevAddress, ( uint8_t )3 ) )
		{
			// Failed to read bytes, probably due to timeout
			Serial.println( "log:Failed to read pressure from depth sensor." );
			return false;
		}

		// Receive the three bytes
		ByteHigh	= I2c.receive();
		ByteMiddle	= I2c.receive();
		ByteLow		= I2c.receive();

		// Combine the bytes
		AdcPressure = ( ( long )ByteHigh << 16 ) + ( ( long )ByteMiddle << 8 ) + ( long )ByteLow;

		// Signal sensor to perform pressure ADC conversion (takes 8.22ms)
		WriteRegisterByte( D2_512 );

		// Signal intent to read adc value
		WriteRegisterByte( AdcRead );

		// Read three bytes from the device
		if( I2c.read( DevAddress, ( uint8_t )3 ) )
		{
			// Failed to read bytes, probably due to timeout
			Serial.println( "log:Failed to read temperature from depth sensor." );
			return false;
		}

		// Read bytes
		ByteHigh	= I2c.receive();
		ByteMiddle	= I2c.receive();
		ByteLow		= I2c.receive();

		// Combine bytes
		AdcTemperature = ( ( long )ByteHigh << 16 ) + ( ( long )ByteMiddle << 8 ) + ( long )ByteLow;

		// Calculate the temperature using the calibration constants
		envdata::TEMP = CorrectedTemperature( AdcTemperature, CalConstant );

		// Calculate the pressure using cal constants and the temperature
		Pressure = TemperatureCorrectedPressure( AdcPressure, AdcTemperature, CalConstant );

		envdata::PRES = Pressure;

		// Calculate the depth using pressure
		if( Settings::water_type == FreshWater )
		{
			//FreshWater
			Depth = ( Pressure - AtmosPressure ) * WaterDensity / 100.0f;
		}
		else
		{
			// Salt Water
			// See Seabird App Note #69 for details
			// 9.72659 / 9.780318 = 0.9945
			Depth = ( Pressure - AtmosPressure ) * 0.9945f / 100.0f;
		}

		// Set output depth, adjusting for offset if necessary
		navdata::DEAP = Depth - DepthOffset;

		return true;
	}

	bool GetCalibrationConstants()
	{
		bool ret = true;
		bool localRet = true;

		// Get the calibration constants and store in array
		for( byte i = 0; i < 8; i++ )
		{
			localRet = true;

			localRet &= WriteRegisterByte( PromBaseAddress + ( 2 * i ) );

			if( I2c.read( DevAddress, ( uint8_t )2 ) )
			{
				// Failure
				localRet &= false;

				// Print cal constant
				Serial.print( "Depth.C" );
				Serial.print( i );
				Serial.print( ":" );
				Serial.print( CalConstant[i] );
				Serial.println( "|INVALID;" );
			}
			else
			{
				// Success
				ByteHigh	= I2c.receive();
				ByteLow		= I2c.receive();

				// Combine bytes
				CalConstant[i] = ( ( ( unsigned int )ByteHigh << 8 ) + ByteLow );

				// Print cal constant
				Serial.print( "Depth.C" );
				Serial.print( i );
				Serial.print( ":" );
				Serial.print( CalConstant[i] );
				Serial.println( ";" );
			}

			if( localRet == false )
			{
				Serial.print( "log: Failed to fetch depth cal constant number " );
				Serial.print( i );
				Serial.println( ";" );
			}

			ret &= localRet;
		}

		return ret;
	}

	bool GetCalculationsTwo()
	{
		// Signal sensor to perform pressure ADC conversion (takes 8.22ms)
		WriteRegisterByte( D1_512 );

		// Todo: break these waits up into a state machine so that the rest of the system doesn't have to wait for the pressure sensor

		// Signal intent to read adc value
		WriteRegisterByte( AdcRead );

		// Read three bytes from the device
		if( I2c.read( DevAddress, ( uint8_t )3 ) )
		{
			// Failed to read bytes, probably due to timeout
			Serial.println( "log:Failed to read pressure from depth sensor." );
			return false;
		}

		// Receive the three bytes
		ByteHigh	= I2c.receive();
		ByteMiddle	= I2c.receive();
		ByteLow		= I2c.receive();

		// Combine the bytes
		AdcPressure = ( ( uint32_t )ByteHigh << 16 ) + ( ( uint32_t )ByteMiddle << 8 ) + ByteLow;

		// Signal sensor to perform pressure ADC conversion (takes 8.22ms)
		WriteRegisterByte( D2_512 );

		// Signal intent to read adc value
		WriteRegisterByte( AdcRead );

		// Read three bytes from the device
		if( I2c.read( DevAddress, ( uint8_t )3 ) )
		{
			// Failed to read bytes, probably due to timeout
			Serial.println( "log:Failed to read temperature from depth sensor." );
			return false;
		}

		// Read bytes
		ByteHigh	= I2c.receive();
		ByteMiddle	= I2c.receive();
		ByteLow		= I2c.receive();

		// Combine bytes
		AdcTemperature = ( ( uint32_t )ByteHigh << 16 ) + ( ( uint32_t )ByteMiddle << 8 ) + ByteLow;

		//Create Variables for calculations
		int32_t temp_calc;
		int32_t pressure_calc;

		int32_t dT;

		//Now that we have a raw temperature, let's compute our actual.
		dT = AdcTemperature - ( ( int32_t )CalConstant[5] << 8 );
		temp_calc = ( ( ( int64_t )dT * CalConstant[6] ) >> 23 ) + 2000;

		//Now we have our first order Temperature, let's calculate the second order.
		int64_t T2, OFF2, SENS2, OFF, SENS; //working variables

		if( temp_calc < 2000 )
			// If temp_calc is below 20.0C
		{
			T2 = 3 * ( ( ( int64_t )dT * dT ) >> 33 );
			OFF2 = 3 * ( ( temp_calc - 2000 ) * ( temp_calc - 2000 ) ) / 2;
			SENS2 = 5 * ( ( temp_calc - 2000 ) * ( temp_calc - 2000 ) ) / 8;

			if( temp_calc < -1500 )
				// If temp_calc is below -15.0C
			{
				OFF2 = OFF2 + 7 * ( ( temp_calc + 1500 ) * ( temp_calc + 1500 ) );
				SENS2 = SENS2 + 4 * ( ( temp_calc + 1500 ) * ( temp_calc + 1500 ) );
			}
		}
		else
			// If temp_calc is above 20.0C
		{
			T2 = 7 * ( ( uint64_t )dT * dT ) / pow( 2, 37 );
			OFF2 = ( ( temp_calc - 2000 ) * ( temp_calc - 2000 ) ) / 16;
			SENS2 = 0;
		}

		// Now bring it all together to apply offsets

		OFF = ( ( int64_t )CalConstant[2] << 16 ) + ( ( ( CalConstant[4] * ( int64_t )dT ) ) >> 7 );
		SENS = ( ( int64_t )CalConstant[1] << 15 ) + ( ( ( CalConstant[3] * ( int64_t )dT ) ) >> 8 );

		temp_calc = temp_calc - T2;
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;

		// Now lets calculate the pressure


		pressure_calc = ( ( ( SENS * AdcPressure ) / 2097152 ) - OFF ) / 32768;

		float pressureFinal = ( float )pressure_calc;
		float tempFinal = ( float )temp_calc;


		// Calculate the depth using pressure
		if( Settings::water_type == FreshWater )
		{
			//FreshWater
			Depth = ( pressureFinal - AtmosPressure ) * WaterDensity / 100.0f;
		}
		else
		{
			// Salt Water
			// See Seabird App Note #69 for details
			// 9.72659 / 9.780318 = 0.9945
			Depth = ( pressureFinal - AtmosPressure ) * 0.9945f / 100.0f;
		}

		// Calculate the temperature using the calibration constants
		envdata::TEMP = tempFinal;

		envdata::PRES = pressureFinal;

		// Set output depth, adjusting for offset if necessary
		navdata::DEAP = Depth - DepthOffset;
	}
}


void MS5803_14BA::device_setup()
{
	Settings::capability_bitarray |= ( 1 << DEAPTH_CAPABLE );

	Serial.println( "log:Resetting depth sensor;" );

	// Reset the device and check for device presence
	WriteRegisterByte( Reset );

	delay( 100 );

	Serial.println( "log:Reset depth sensor;" );

	DepthSensorSamples.reset();
	InitTimer.reset();
}


void MS5803_14BA::device_loop( Command command )
{
	if( isInitialized )
	{
		// Zero the depth value
		if( command.cmp( "dzer" ) )
		{
			DepthOffset = Depth;
		}
		else if( command.cmp( "dtwa" ) )
		{
			// Print water type value
			if( Settings::water_type == FreshWater )
			{
				Settings::water_type = SaltWater;
				Serial.println( F( "dtwa:1;" ) );
			}
			else
			{
				Settings::water_type =  FreshWater;
				Serial.println( F( "dtwa:0;" ) );
			}
		}

		// Read depth value once per second
		if( DepthSensorSamples.elapsed( 1000 ) )
		{
			//GetDepthAndTemperature();
			GetCalculationsTwo();
		}
	}
	else
	{
		// Try to reinitialize every 10 seconds
		if( InitTimer.elapsed( 10000 ) )
		{
			Serial.println( "log:Initializing depth sensor;" );

			if( GetCalibrationConstants() )
			{
				Serial.println( "log:Depth sensor initialized!;" );

				isInitialized = true;
			}
		}
	}
}


#endif

#if(HAS_MS5803_14BA_EXP)

#include "Device.h"
#include "MS5803_14BA.h"
#include "CMS5803_14.h"
#include "Settings.h"
#include "Timer.h"
#include "I2C.h"

namespace
{
	CMS5803 dev( 512U );

	Timer DepthSensorSamples;
	Timer InitTimer;

	float depth_m = 0;
	float depthOffset_m = 0;

	bool isInitialized = false;

	bool GetDepthAndTemperature()
	{
		dev.readSensor();

		return true;
	}

	bool GetCalibrationConstants()
	{
		return dev.GetCalibrationCoefficients();
	}
}


void MS5803_14BA::device_setup()
{
	Settings::capability_bitarray |= ( 1 << DEAPTH_CAPABLE );

	// Reset the chip
	dev.Initialize( false );

	DepthSensorSamples.reset();
	InitTimer.reset();
}


void MS5803_14BA::device_loop( Command command )
{
	if( isInitialized )
	{
		// Zero the depth value
		if( command.cmp( "dzer" ) )
		{
			dev.m_depthOffset = dev.m_depth;
		}
		else if( command.cmp( "dtwa" ) )
		{
			// Print water type value
			if( Settings::water_type == CMS5803::FRESH_WATER )
			{
				Settings::water_type = CMS5803::SALT_WATER;
				Serial.println( F( "dtwa:1;" ) );
			}
			else
			{
				Settings::water_type =  CMS5803::FRESH_WATER;
				Serial.println( F( "dtwa:0;" ) );
			}
		}

		// Read depth value once per second
		if( DepthSensorSamples.elapsed( 1000 ) )
		{
			//GetDepthAndTemperature();
			GetDepthAndTemperature();
		}
	}
	else
	{
		// Try to reinitialize every 10 seconds
		if( InitTimer.elapsed( 10000 ) )
		{
			Serial.println( "log:Attempting to fetch depth cal coeffs;" );

			if( dev.GetCalibrationCoefficients() == true )
			{
				Serial.println( "log:Depth sensor coeffs obtained!;" );

				isInitialized = true;
			}
		}
	}
}

#endif

