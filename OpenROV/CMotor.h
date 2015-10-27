#pragma once

// Includes
#include <Arduino.h>
#include "CServo.h"

class CMotor
{
private:
    CServo	m_servo;
    int		m_motorPin;

public:
    float	m_positiveModifier;
    float	m_negativeModifier;
    int		m_positiveDeadzoneBuffer;
    int		m_negativeDeadzoneBuffer;

	float	m_currentInput;


	static const float m_forwardReverseRatio;


    CMotor( int motorPinIn );
    CMotor();

    void SetPin( int motorPinIn );
    void Activate();
    void Deactivate();

    int SetMotorTarget( int ms );
    bool IsActive();
	bool IsDriving();

	static float CalcMotorOutput( float inputPercentageIn );


	int SetMotorTarget( float input );

	static bool IsEffectivelyZero( float input );
	static bool ValidateInput( float thrustPercentageIn );
};