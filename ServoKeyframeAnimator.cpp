/*
 * ServoKeyframeAnimator.cpp
 *
 *  Created on: 19.01.2021
 *      Author: butch
 */

#include "ServoKeyframeAnimator.h"
#include <arduino.h>
#include "globalDefines.h"
#include <ArduinoLog.h>

ServoKeyframeAnimator::ServoKeyframeAnimator() {

	Log.trace(F("ServoKeyframeAnimator::ServoKeyframeAnimator() start"CR));
	_duration = 0;

	// default for servos is center
	_currentPosition = 90;
	_targetKeyframePosition= 90;
	_previousKeyframePosition=90;

	// smooth mode is default. Its the only one which is implemented yet.
	//this -> _keyframeMode = KEYFRAME_MODE_SMOOTH;
	_keyframeMode = KEYFRAME_MODE_SMOOTH;
	Log.trace(F("ServoKeyframeAnimator::ServoKeyframeAnimator() end: _duration=%d, _currentPosition=%d, _targetKeyframePosition=%d, _previousKeyframePosition=%d, _keyframeMode=%d" CR), _duration, _currentPosition, _targetKeyframePosition, _previousKeyframePosition, _keyframeMode );

}

ServoKeyframeAnimator::~ServoKeyframeAnimator() {
	// TODO Auto-generated destructor stub
}


/**
 * set keyframeMode
 */
void ServoKeyframeAnimator::setKeyframeMode(unsigned char keyframeMode)
{
	_keyframeMode=keyframeMode;
	//Log.trace(F("ServoKeyframeAnimator::setKeyframeMode - set _keyframeMode=%d"CR), _keyframeMode);
}


/**
 * sets the target position of the servo. This is the position where the servo will be on next keyframe
 */
void ServoKeyframeAnimator::setServoPositionNextKeyframe(unsigned char targetPos)
{
	_targetKeyframePosition = targetPos;
}

/**
 * sets the previous keyframe position of the servo. This will be internally called, when a keyframe is passed
 */
void ServoKeyframeAnimator::setServoPositionPreviousKeyframe(unsigned char previousPos)
{
	_previousKeyframePosition = previousPos;
}

/**
 * sets the _currentPostion variable to an absolute value. Useful to have direct servo access within the same functionset.
 */
void ServoKeyframeAnimator::setServoAbsolutePosition(unsigned char absolutePos)
{
	_currentPosition = absolutePos;
}


/**
 * increase (+) / decrease (-) the _currentPosition variable by the absolutePosChange value.
 */
void ServoKeyframeAnimator::setServoAbsolutePositionChange(signed int absolutePosChange)
{
	_currentPosition = _currentPosition + absolutePosChange;
}


/**
 * returns the servo's target position at next keyframe
 */
unsigned char ServoKeyframeAnimator::getServoTargetPositon()
{
	return _targetKeyframePosition;
}


/**
 * returns the servo's current position
 */

unsigned char ServoKeyframeAnimator::getServoCurrentPositon()
{
	return _currentPosition;
}




/**
 *
 * This function returns the position of a servo. Base for (de-)accelleration is sinus curve.
 * timePrevKey:			time of previous keyframe / start of the move
 * targetDuration:		time used to move from posPrevKey to posNextKey in millis

 * posPrevKey:  	servo position of at the start (previous keyframe)
 * posNextKey:		target position of the servo (next keyframe)
 *
 * No Speed change is allowed within a move! (should make things much easier)
 */
unsigned char ServoKeyframeAnimator::getCalculatedServoPosition( unsigned long timePrevKey,	unsigned int targetDuration)
{

	#if DEBUG_SERVO_KEYFRAME_ANIMATOR_GET_CALCULATED_SERVO_POSITION == 1
		DEBUG_SERIAL_NAME.print("_keyframeMode=");
		DEBUG_SERIAL_NAME.println(_keyframeMode);
		Log.trace(F("ServoKeyframeAnimator:getCalculatedServoPosition args timePrevKey=%l, targetDuration=%d (_keyframeMode=%d)"CR), timePrevKey, targetDuration,_keyframeMode);
	#endif

	unsigned long currentTime = millis();
	unsigned char servoPos=90; // if calculation fails / not implemented yet -> return center value. Should not break the model / robot.


	if (_keyframeMode == KEYFRAME_MODE_SMOOTH)
	{
		// map timeframe to sinus function
		float currentTimeInFunction = (float) map(currentTime, timePrevKey, targetDuration + timePrevKey, 0, PI*1000)/1000;

		long resultInFunction = (long) ((sin(currentTimeInFunction+1.5*PI)+1)*1000);


		// map to servo position
		servoPos = map(resultInFunction, 0, 2000, _previousKeyframePosition, _targetKeyframePosition);

		#if DEBUG_SERVO_KEYFRAME_ANIMATOR_GET_CALCULATED_SERVO_POSITION == 1
			Log.trace(F("ServoKeyframeAnimator:getCalculatedServoPosition timePrevKey=%l targetDuration=%d currentTime=%l currentTimeInFunction=%F resultInFunction=%F servoPos=%d"CR), timePrevKey, targetDuration, currentTime, currentTimeInFunction,resultInFunction,servoPos );
		#endif
	}
	else
	{
		Log.error(F("ServoKeyframeAnimator:getCalculatedServoPosition: mode not implemented yet: %d"CR), _keyframeMode);
	}

	// return the servo position
	return servoPos;
}
