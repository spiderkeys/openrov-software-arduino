#include "AConfig.h"
#if( THRUSTER_CONFIGURATION == THRUSTER_CONFIG_2X1 )

// Includes
#include "CThrusters.h"
#include "NConfigManager.h"
#include "NDataManager.h"
#include "CMotor.h"
#include "CTimer.h"
#include "CPin.h"

#if(HAS_STD_CAPE)
    #include "CCape.h"
#endif

#if(HAS_OROV_CONTROLLERBOARD_25)
    #include "CControllerBoard.h"
#endif


// Static variable initialization
const int CThrusters::kMotorCount = 3;

namespace
{
    CMotor m_portMotor( PORT_PIN );
    CMotor m_verticalMotor( VERTICAL_PIN );
    CMotor m_starboardMotor( STARBOARD_PIN );

    int portTargetNew_us	= MOTOR_TARGET_NEUTRAL_US;
    int starTargetNew_us	= MOTOR_TARGET_NEUTRAL_US;
    int vertTargetNew_us	= MOTOR_TARGET_NEUTRAL_US;
    int portTarget_us		= MOTOR_TARGET_NEUTRAL_US;
    int vertTarget_us		= MOTOR_TARGET_NEUTRAL_US;
    int starTarget_us		= MOTOR_TARGET_NEUTRAL_US;

	float m_targetThrottle;
	float m_targetYaw;
	float m_targetLift;

    int m_targetPower;


    CTimer m_controlTimer;
    CTimer m_thrusterOutputTimer;

    bool m_bypassSmoothing;

#ifdef ESCPOWER_PIN
    bool m_canPowerESCs = true;
    CPin m_escPowerPin( "escpower", ESCPOWER_PIN, CPin::kDigital, CPin::kOutput );
#else
    boolean m_canPowerESCs = false;
#endif
}

void CThrusters::Initialize()
{
	// Set up initial motor parameters
    m_portMotor.m_negativeDeadzoneBuffer		= NConfigManager::m_deadZoneMin;
    m_portMotor.m_positiveDeadzoneBuffer		= NConfigManager::m_deadZoneMax;
    m_portMotor.Activate();

    m_verticalMotor.m_negativeDeadzoneBuffer	= NConfigManager::m_deadZoneMin;
    m_verticalMotor.m_positiveDeadzoneBuffer	= NConfigManager::m_deadZoneMax;
    m_verticalMotor.Activate();

    m_starboardMotor.m_negativeDeadzoneBuffer	= NConfigManager::m_deadZoneMin;
    m_starboardMotor.m_positiveDeadzoneBuffer	= NConfigManager::m_deadZoneMax;
    m_starboardMotor.Activate();

	// Reset timers
    m_thrusterOutputTimer.Reset();
    m_controlTimer.Reset();

    m_bypassSmoothing = false;

	// Turn on ESCs, if possible
    #ifdef ESCPOWER_PIN
    m_escPowerPin.Reset();
    m_escPowerPin.Write( 1 ); //Turn on the ESCs
    #endif
}

void CThrusters::Update( CCommand& command )
{
	// Change positive motor modifiers
    if( command.Equals( "mtrmod1" ) )
    {
        m_portMotor.m_positiveModifier		= (float)command.m_arguments[1] / 100.0f;
        m_verticalMotor.m_positiveModifier	= (float)command.m_arguments[2] / 100.0f;
        m_starboardMotor.m_positiveModifier = (float)command.m_arguments[3] / 100.0f;
    }

	// Change negative motor modifiers
    if( command.Equals( "mtrmod2" ) )
    {
        m_portMotor.m_negativeModifier		= (float)command.m_arguments[1] / 100.0f;
        m_verticalMotor.m_negativeModifier	= (float)command.m_arguments[2] / 100.0f;
        m_starboardMotor.m_negativeModifier = (float)command.m_arguments[3] / 100.0f;
    }

	// Report motor modifiers
    if( command.Equals( "rmtrmod" ) )
    {
        Serial.print( F( "mtrmod:" ) );
        Serial.print( m_portMotor.m_positiveModifier );
        Serial.print( "," );
        Serial.print( m_verticalMotor.m_positiveModifier );
        Serial.print( "," );
        Serial.print( m_starboardMotor.m_positiveModifier );
        Serial.print( "," );
        Serial.print( m_portMotor.m_negativeModifier );
        Serial.print( "," );
        Serial.print( m_verticalMotor.m_negativeModifier );
        Serial.print( "," );
        Serial.print( m_starboardMotor.m_negativeModifier );
        Serial.println( ";" );
    }

	// Set all three motor modifiers at once
    if( command.Equals( "go" ) )
    {
        // Ignore commands that lie outside of the allowable PWM range
        if( command.m_arguments[1] > 999 && command.m_arguments[2] > 999 && command.m_arguments[3] > 999 && command.m_arguments[1] < 2001 && command.m_arguments[2] < 2001 && command.m_arguments[3] < 2001 )
        {
            portTarget_us = command.m_arguments[1];
            vertTarget_us = command.m_arguments[2];
            starTarget_us = command.m_arguments[3];

            if( command.m_arguments[4] == 1 )
            {
                m_bypassSmoothing = true;
            }
        }
    }

	// Set port motor target
    if( command.Equals( "port" ) )
    {
		// Ignore commands that lie outside of the allowable PWM range
        if( command.m_arguments[1] > 999 && command.m_arguments[1] < 2001 )
        {
            portTarget_us = command.m_arguments[1];

            if( command.m_arguments[2] == 1 )
            {
                m_bypassSmoothing = true;
            }
        }
    }

	// Set vertical motor target
    if( command.Equals( "vertical" ) )
    {
		// Ignore commands that lie outside of the allowable PWM range
        if( command.m_arguments[1] > 999 && command.m_arguments[1] < 2001 )
        {
            vertTarget_us = command.m_arguments[1];

            if( command.m_arguments[2] == 1 )
            {
                m_bypassSmoothing = true;
            }
        }
    }

	// Set starboard motor target
    if( command.Equals( "starboard" ) )
    {
		// Ignore commands that lie outside of the allowable PWM range
        if( command.m_arguments[1] > 999 && command.m_arguments[1] < 2001 )
        {
            starTarget_us = command.m_arguments[1];

            if( command.m_arguments[2] == 1 )
            {
                m_bypassSmoothing = true;
            }
        }
    }

	// Handle yaw/throttle commands for both port and starboard thrusters
    if( command.Equals( "thro" ) || command.Equals( "yaw" ) )
    {
		// Throttle
        if( command.Equals( "thro" ) )
        {
			// Ignore values not between -100% and +100%
            if( command.m_arguments[1] >= -100 && command.m_arguments[1] <= 100 )
            {
                m_targetThrottle = (float)command.m_arguments[1] / 100.0f;
            }
        }

        if( command.Equals( "yaw" ) )
        {
            // Ignore values not between -100% and +100%
            if( command.m_arguments[1] >= -100 && command.m_arguments[1] <= 100 )
            {
                m_targetYaw = (float)command.m_arguments[1] / 100.0f;
            }
        }

        // The code below spreads the throttle spectrum over the possible range of the motor. 
		// Not sure this belongs here or should be placed with deadzone calculation in the motor code.
        if( m_targetThrottle >= 0 )
        {
            portTarget_us = 1500 + ( 500.0 / abs( m_portMotor.m_positiveModifier ) ) * m_targetThrottle;
            starTarget_us = portTarget_us;
        }
        else
        {
            portTarget_us = 1500 + ( 500.0 / abs( m_portMotor.m_negativeModifier ) ) * m_targetThrottle;
            starTarget_us = portTarget_us;
        }

        m_targetPower = starTarget_us;

        int turn = m_targetYaw * 250; //max range due to reverse range

        if( m_targetThrottle >= 0 )
        {
            int offset = ( abs( turn ) + m_targetPower ) - 2000;

            if( offset < 0 )
            {
                offset = 0;
            }

            portTarget_us = m_targetPower + turn - offset;
            starTarget_us = m_targetPower - turn - offset;
        }
        else
        {
            int offset = 1000 - ( m_targetPower - abs( turn ) );

            if( offset < 0 )
            {
                offset = 0;
            }

            portTarget_us = m_targetPower + turn + offset;
            starTarget_us = m_targetPower - turn + offset;
        }

    }

    if( command.Equals( "lift" ) )
    {
        if( command.m_arguments[1] >= -100 && command.m_arguments[1] <= 100 )
        {
            m_targetLift = command.m_arguments[1] / 100.0;
            vertTarget_us = 1500 + 500 * m_targetLift;
        }
    }


    #ifdef ESCPOWER_PIN
    else if( command.Equals( "escp" ) )
    {
        m_escPowerPin.Write( command.m_arguments[1] ); //Turn on the ESCs
        Serial.print( F( "log:escpower=" ) );
        Serial.print( command.m_arguments[1] );
        Serial.println( ';' );
    }

    #endif
    else if( command.Equals( "start" ) )
    {
        m_portMotor.Activate();
        m_verticalMotor.Activate();
        m_starboardMotor.Activate();
    }
    else if( command.Equals( "stop" ) )
    {
        portTarget_us = MOTOR_TARGET_NEUTRAL_US;
        vertTarget_us = MOTOR_TARGET_NEUTRAL_US;
        starTarget_us = MOTOR_TARGET_NEUTRAL_US;
        // Not sure why the reset does not re-attach the servo.
        //port_motor.stop();
        //vertical_motor.stop();
        //starboard_motor.stop();
    }

    #ifdef ESCPOWER_PIN
    else if( ( command.Equals( "mcal" ) ) && ( m_canPowerESCs ) )
    {
        Serial.println( F( "log:Motor Callibration Staring;" ) );
        //Experimental. Add calibration code here
        Serial.println( F( "log:Motor Callibration Complete;" ) );
    }

    #endif

    //to reduce AMP spikes, smooth large power adjustments out. This incirmentally adjusts the motors and servo
    //to their new positions in increments.  The incriment should eventually be adjustable from the cockpit so that
    //the pilot could have more aggressive response profiles for the ROV.
    if( m_controlTimer.HasElapsed( 50 ) )
    {
        if( portTarget_us != portTargetNew_us || vertTarget_us != vertTargetNew_us || starTarget_us != starTargetNew_us )
        {
            portTargetNew_us = portTarget_us;
            vertTargetNew_us = vertTarget_us;
            starTargetNew_us = starTarget_us;

            // Check to see if any motors are non-neutral to signal system that at least one motor is running
            if( portTarget_us != MOTOR_TARGET_NEUTRAL_US || vertTarget_us != MOTOR_TARGET_NEUTRAL_US || starTarget_us != MOTOR_TARGET_NEUTRAL_US )
            {
                NDataManager::m_thrusterData.MotorsActive = true;
            }
            else
            {
                NDataManager::m_thrusterData.MotorsActive = false;
            }

            Serial.print( F( "motors:" ) );
            Serial.print( m_portMotor.SetMotorTarget( portTargetNew_us ) );
            Serial.print( ',' );
            Serial.print( m_verticalMotor.SetMotorTarget( vertTargetNew_us ) );
            Serial.print( ',' );
            Serial.print( m_starboardMotor.SetMotorTarget( starTargetNew_us ) );
            Serial.println( ';' );
        }

    }

    NDataManager::m_navData.FTHR = map( ( portTargetNew_us + starTargetNew_us ) / 2, 1000, 2000, -100, 100 );

    //The output from the motors is unique to the thruster configuration
    if( m_thrusterOutputTimer.HasElapsed( 1000 ) )
    {
        Serial.print( F( "mtarg:" ) );
        Serial.print( portTarget_us );
        Serial.print( ',' );
        Serial.print( vertTarget_us );
        Serial.print( ',' );
        Serial.print( starTarget_us );
        Serial.println( ';' );
        NDataManager::m_thrusterData.MATC = m_portMotor.IsActive() || m_portMotor.IsActive() || m_portMotor.IsActive();
        Serial.print( F( "mtrmod:" ) );
        Serial.print( m_portMotor.m_positiveModifier );
        Serial.print( "," );
        Serial.print( m_verticalMotor.m_positiveModifier );
        Serial.print( "," );
        Serial.print( m_starboardMotor.m_positiveModifier );
        Serial.print( "," );
        Serial.print( m_portMotor.m_negativeModifier );
        Serial.print( "," );
        Serial.print( m_verticalMotor.m_negativeModifier );
        Serial.print( "," );
        Serial.print( m_starboardMotor.m_negativeModifier );
        Serial.println( ";" );
    }
}

#endif