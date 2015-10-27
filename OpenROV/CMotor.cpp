// Includes
#include "CMotor.h"
#include "SystemConstants.h"

const float CMotor::m_forwardReverseRatio = 0.4f;

// Default constructor plus pin-setting
CMotor::CMotor( int motorPinIn )
	: m_motorPin( -1 )
	, m_positiveModifier( MOTOR_DEFAULT_POSITIVE_MOD )
	, m_negativeModifier( MOTOR_DEFAULT_NEGATIVE_MOD )
	, m_positiveDeadzoneBuffer( MOTOR_DEFAULT_NEGATIVE_DEADZONE_BUFFER )
	, m_negativeDeadzoneBuffer( MOTOR_DEFAULT_POSITIVE_DEADZONE_BUFFER )
	, m_currentInput( 0.0f )
{
	SetPin( motorPinIn );
}

CMotor::CMotor()
	: m_motorPin( -1 )
	, m_positiveModifier( MOTOR_DEFAULT_POSITIVE_MOD )
	, m_negativeModifier( MOTOR_DEFAULT_NEGATIVE_MOD )
	, m_positiveDeadzoneBuffer( MOTOR_DEFAULT_NEGATIVE_DEADZONE_BUFFER )
	, m_negativeDeadzoneBuffer( MOTOR_DEFAULT_POSITIVE_DEADZONE_BUFFER )
	, m_currentInput( 0.0f )
{
}

void CMotor::SetPin( int motorPinIn )
{
	m_motorPin = motorPinIn;
}

void CMotor::Activate()
{
	m_servo.Activate( m_motorPin );
}

void CMotor::Deactivate()
{
	// Set the servo back to the neutral PWM target and deactivate the pin associated with it
	m_servo.WriteMicroseconds( MOTOR_TARGET_NEUTRAL_US );
	m_servo.Detach();
}

bool CMotor::IsActive()
{
	// Find out if the motor is active in the list of servos
	return m_servo.IsActive();
}

bool CMotor::IsDriving()
{
	return !IsEffectivelyZero( m_currentInput );
}

bool CMotor::IsEffectivelyZero( float input )
{
	// Check for zero'ish
	if (input < 0.01f && input > -0.01f)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CMotor::ValidateInput( float thrustPercentageIn )
{
	// Out of percentage bounds
	if( thrustPercentageIn < -1.0f || thrustPercentageIn > 1.0f )
	{
		return false;
	}

	// Check for zero'ish
	if (IsEffectivelyZero( thrustPercentageIn))
	{
		return true;
	}

	// Inside the deadband
	if( thrustPercentageIn < 0.12f && thrustPercentageIn > -0.12f )
	{
		return false;
	}

	return true;
}

int CMotor::SetMotorTarget( int targetIn_us )
{
	float modifier( 1.0f );

	// Get the appropriate modifier value
	if( targetIn_us > MOTOR_TARGET_NEUTRAL_US )
	{
		modifier = m_positiveModifier;
	}
	else if( targetIn_us < MOTOR_TARGET_NEUTRAL_US )
	{
		modifier = m_negativeModifier;
	}

	int midpointDelta_us = targetIn_us - MOTOR_TARGET_NEUTRAL_US;

	// First, apply a floor and ceiling on the target PWM value
	int constrainedTarget_us = constrain( ( MOTOR_TARGET_NEUTRAL_US + static_cast<int>( midpointDelta_us * modifier ) ), MOTOR_TARGET_MIN_US, MOTOR_TARGET_MAX_US );

	int finalTargetValue_us = MOTOR_TARGET_NEUTRAL_US;

	// Re-map the constrained target to a target value that falls outside of the ESC's deadband.
	// There are basically small regions around the neutral value where the motor will not actuate.
	// This makes sure that we are either exactly equal to the neutral value or fall on either side of the deadband.
	if( constrainedTarget_us < MOTOR_TARGET_NEUTRAL_US )
	{
		finalTargetValue_us = map( constrainedTarget_us, MOTOR_TARGET_MIN_US, MOTOR_TARGET_NEUTRAL_US, MOTOR_TARGET_MIN_US, MOTOR_TARGET_NEUTRAL_US - m_negativeDeadzoneBuffer );
	}
	else if( constrainedTarget_us > MOTOR_TARGET_NEUTRAL_US )
	{
		finalTargetValue_us = map( constrainedTarget_us, MOTOR_TARGET_NEUTRAL_US, MOTOR_TARGET_MAX_US, MOTOR_TARGET_NEUTRAL_US + m_positiveDeadzoneBuffer, MOTOR_TARGET_MAX_US );
	}

	// Set the final target value in the servo interface and return the value that was set
	m_servo.WriteMicroseconds( constrain( finalTargetValue_us, MOTOR_TARGET_MIN_US, MOTOR_TARGET_MAX_US ) );
	return m_servo.ReadMicroseconds();
}

// This function maps input percentage to a motor input scale that linearizes forward and reverse thrust.
float CMotor::CalcMotorOutput( float inputPercentageIn )
{
	if (inputPercentageIn >= 0.0f)
	{
		return fmin( inputPercentageIn, 1.0f );
	}
	else
	{
		if (inputPercentageIn >= -m_forwardReverseRatio )
		{
			return inputPercentageIn / m_forwardReverseRatio;
		}
		else
		{
			return -1.0f;
		}
	}
}

int CMotor::SetMotorTarget( float input )
{
	m_currentInput = input;

	int pwmOutput = MOTOR_TARGET_NEUTRAL_US;

	pwmOutput = MOTOR_TARGET_NEUTRAL_US + static_cast<int>( 500.0f * m_currentInput );

	// Set the final target value in the servo interface and return the value that was set
	m_servo.WriteMicroseconds( constrain( pwmOutput, MOTOR_TARGET_MIN_US, MOTOR_TARGET_MAX_US ) );
	return m_servo.ReadMicroseconds();
}

