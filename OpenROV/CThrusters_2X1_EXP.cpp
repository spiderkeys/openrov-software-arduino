#include "AConfig.h"
#if( THRUSTER_CONFIGURATION == THRUSTER_CONFIG_2X1_EXP )

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

	float m_portTargetNew	= 0.0f;
	float m_starTargetNew	= 0.0f;
	float m_vertTargetNew	= 0.0f;

	float m_portTarget		= 0.0f;
	float m_starTarget		= 0.0f;
	float m_vertTarget		= 0.0f;

	// These are your user inputs
	float m_targetThrottle;
	float m_targetYaw;
	float m_targetLift;

    CTimer m_controlTimer;
    CTimer m_thrusterOutputTimer;

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

	// Turn on ESCs, if possible
    #ifdef ESCPOWER_PIN
    m_escPowerPin.Reset();
    m_escPowerPin.Write( 1 ); //Turn on the ESCs
    #endif
}

void CThrusters::Update( CCommand& command )
{
	bool lateralMotorUpdated = false;

	// Throttle
    if( command.Equals( "thro" ) )
    {
		// Ignore values not between -100% and +100%
        if( command.m_arguments[1] >= -100 && command.m_arguments[1] <= 100 )
        {
            m_targetThrottle = (float)command.m_arguments[1] / 100.0f;
        }

		lateralMotorUpdated = true;
    }
    else if( command.Equals( "yaw" ) )
    {
        // Ignore values not between -100% and +100%
        if( command.m_arguments[1] >= -100 && command.m_arguments[1] <= 100 )
        {
            m_targetYaw = (float)command.m_arguments[1] / 100.0f;
        }

		lateralMotorUpdated = true;
    }
	else if (command.Equals( "autoYaw" ))
	{
		// Ignore values not between -100% and +100%
		if (command.m_arguments[ 1 ] >= -10000 && command.m_arguments[ 1 ] <= 10000 )
		{
			m_targetYaw = (float)command.m_arguments[ 1 ] / 10000.0f;
		}

		lateralMotorUpdated = true;
	}

	// Handle either lateral motor commands or other commands that may have come in (only one command at a time, after all!)
	if( lateralMotorUpdated == true )
	{
		// Now that we have user inputs, let's calculate what we need to do!

		// This roughly equals ( reverse force at 1000us PWM ) / ( forward force at 2000us PWM )
		//float maxSymmetricThrust = 0.5f
		float maxSymmetricThrust = CMotor::m_forwardReverseRatio;

		float portYawInput = 0.0f;
		float starYawInput = 0.0f;

		// First step is to map the inputs to the appropriate yaw outputs. We only have half the thrust range available to us because of the reverse dir
		// (-maxSym to maxSym)
		portYawInput = m_targetYaw * maxSymmetricThrust;
		starYawInput = -m_targetYaw * maxSymmetricThrust;

		// translate yaw input to motor input scale (-1 to 1)
		float starYawOutput = CMotor::CalcMotorOutput( starYawInput );
		float portYawOutput = CMotor::CalcMotorOutput( portYawInput );

		float availableThrottle = 0.0f;
		float utilizedSymmetricThrust = ( fabs( m_targetYaw ) * maxSymmetricThrust );

		if( m_targetThrottle >= 0 )
		{
			availableThrottle = 1.0f - utilizedSymmetricThrust;
		}
		else
		{
			availableThrottle = maxSymmetricThrust - utilizedSymmetricThrust;
		}

		float appliedThrottle = CMotor::CalcMotorOutput( availableThrottle * m_targetThrottle );

		float portMotorInput = portYawOutput + appliedThrottle;
		float starMotorInput = starYawOutput + appliedThrottle;

		// Validate inputs with the motor
		if( CMotor::ValidateInput( portMotorInput ) && CMotor::ValidateInput( starMotorInput ) )
		{
			// If valid, update port and star targets
			m_portTarget = portMotorInput;
			m_starTarget = starMotorInput;
		}
	}
	else
	{
		// We'll treat this dumbly for now
		if (command.Equals( "lift" ))
		{
			if (command.m_arguments[ 1 ] >= -100 && command.m_arguments[ 1 ] <= 100)
			{
				m_targetLift = command.m_arguments[ 1 ] / 100.0;

				if( CMotor::ValidateInput( m_targetLift ) )
				{
					m_vertTarget = m_targetLift;
				}
				
			}
		}
		else if (command.Equals( "autoLift" ))
		{
			// Ignore values not between -100% and +100%
			if (command.m_arguments[ 1 ] >= -10000 && command.m_arguments[ 1 ] <= 10000)
			{
				m_targetLift = (float)command.m_arguments[ 1 ] / 10000.0f;

				if (CMotor::ValidateInput( m_targetLift ))
				{
					m_vertTarget = m_targetLift;
				}
			}
		}

#ifdef ESCPOWER_PIN
		else if (command.Equals( "escp" ))
		{
			m_escPowerPin.Write( command.m_arguments[ 1 ] ); //Turn on the ESCs
			Serial.print( F( "log:escpower=" ) );
			Serial.print( command.m_arguments[ 1 ] );
			Serial.println( ';' );
		}
#endif

		else if (command.Equals( "start" ))
		{
			m_portMotor.Activate();
			m_verticalMotor.Activate();
			m_starboardMotor.Activate();
		}
		else if (command.Equals( "stop" ))
		{
			m_portTarget = 0.0f;
			m_starTarget = 0.0f;
			m_vertTarget = 0.0f;
		}

#ifdef ESCPOWER_PIN
		else if (( command.Equals( "mcal" ) ) && ( m_canPowerESCs ))
		{
			Serial.println( F( "log:Motor Callibration Staring;" ) );
			//Experimental. Add calibration code here
			Serial.println( F( "log:Motor Callibration Complete;" ) );
		}
#endif
	}

	

    //to reduce AMP spikes, smooth large power adjustments out. This incirmentally adjusts the motors and servo
    //to their new positions in increments.  The incriment should eventually be adjustable from the cockpit so that
    //the pilot could have more aggressive response profiles for the ROV.
    if( m_controlTimer.HasElapsed( 50 ) )
    {
		// If new values have emerged for the motors, use them!
        if( m_portTarget != m_portTargetNew || m_starTarget != m_starTargetNew || m_vertTarget != m_vertTargetNew )
        {
            m_portTargetNew = m_portTarget;
            m_starTargetNew = m_starTarget;
            m_vertTargetNew = m_vertTarget;

			Serial.print( F( "motors:" ) );
			Serial.print( m_portMotor.SetMotorTarget( m_portTarget ) );
			Serial.print( ',' );
			Serial.print( m_verticalMotor.SetMotorTarget( m_vertTarget ) );
			Serial.print( ',' );
			Serial.print( m_starboardMotor.SetMotorTarget( m_starTarget ) );
			Serial.println( ';' );

            // Check to see if any motors are non-neutral to signal system that at least one motor is running
            if( m_portMotor.IsDriving() || m_verticalMotor.IsDriving() || m_starboardMotor.IsDriving() )
            {
                NDataManager::m_thrusterData.MotorsActive = true;
            }
            else
            {
                NDataManager::m_thrusterData.MotorsActive = false;
            }
        }

    }

	// This needs to reflect the applied forward throttle calculated earlier
    NDataManager::m_navData.FTHR = ( m_portTarget + m_starTarget ) / 2.0f;

    // This is just for reporting
    if( m_thrusterOutputTimer.HasElapsed( 1000 ) )
    {
        Serial.print( F( "mtarg:" ) );
        Serial.print( m_portTarget );
        Serial.print( ',' );
        Serial.print( m_starTarget );
        Serial.print( ',' );
        Serial.print( m_vertTarget );
        Serial.println( ';' );

        NDataManager::m_thrusterData.MATC = m_portMotor.IsActive() || m_portMotor.IsActive() || m_portMotor.IsActive();
    }
}

#endif