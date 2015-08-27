#include "Device.h"
#include "Settings.h"
#include "I2C.h"

int Settings::capability_bitarray = 0;
int Settings::smoothingIncriment = 40; //How aggressive the throttle changes
int Settings::deadZone_min = 50;
int Settings::deadZone_max = 50;
bool Settings::water_type = 0; //Freshwater


void Settings::device_setup()
{

}

void Settings::device_loop( Command command )
{
	if( command.cmp( "reportSetting" ) )
	{
		Serial.print( F( "*settings:" ) );
		Serial.print( F( "smoothingIncriment|" ) );
		Serial.print( String( Settings::smoothingIncriment ) + "," );
		Serial.print( F( "deadZone_min|" ) );
		Serial.print( String( Settings::deadZone_min ) + "," );
		Serial.print( F( "deadZone_max|" ) );
		Serial.print( String( Settings::deadZone_max ) + ";" );
		Serial.print( F( "water_type" ) );
		Serial.println( String( Settings::water_type ) + ";" );

	}
	else if( command.cmp( "rcap" ) ) //report capabilities
	{
		Serial.print( F( "CAPA:" ) );
		Serial.print( capability_bitarray );
		Serial.print( ';' );

		// I2c.scan();
	}
	else if( command.cmp( "updateSetting" ) )
	{
		//TODO: Need to update the motors with new deadZone setting. Probably move
		//deadzone to the thruster resposibilitiy
		Settings::smoothingIncriment = command.args[1];
		Settings::deadZone_min = command.args[2];
		Settings::deadZone_max = command.args[3];
		Settings::water_type = command.args[4];
	}

}
