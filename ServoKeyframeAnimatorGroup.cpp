/*
 * ServoKeyframeAnimatorGroup.cpp
 *
 *  Created on: 19.01.2021
 *      Author: butch
 */

#include "ServoKeyframeAnimatorGroup.h"
#include <arduino.h>
#include "globalDefines.h"
#include <ArduinoLog.h>


ServoKeyframeAnimatorGroup::ServoKeyframeAnimatorGroup()
{
	// do nothing. Wait for init call


	// initialize empty ServoKeyframeAnimator. Initialization will be done init procedure.
//	//ServoKeyframeAnimator* _keyframeAnimators=0;
//	_numberOfServos=0;
//
//	// _numbeOfServos=numberOfServos;
//	_timePreviousKeyframe=0;
//	_isInMove=false;
//	_duration=0;

//	Log.trace(F("ServoKeyframeAnimatorGroup::ServoKeyframeAnimatorGroup (numberOfServos=%d) - _timePreviousKeyframe=%l _isInMove=%T _duration=%d" CR), numberOfServos,_timePreviousKeyframe, _isInMove, _duration);
}

void ServoKeyframeAnimatorGroup::init(ServoKeyframeAnimator* keyframeAnimators, EnhancedServo* servos, unsigned char numberOfServos)
{
	Log.verbose(F("ServoKeyframeAnimatorGroup::init - args numberOfServos=%d"CR ),numberOfServos);

		_timePreviousKeyframe=0;
		_isInMove=false;
		_duration=0;
		_keyframeAnimators=keyframeAnimators;
		_numberOfServos=numberOfServos;
		_servos = servos;

//	_timePreviousKeyframe=0;
//	_isInMove=false;
//	_duration=0;
//	//_keyframeAnimators = new ServoKeyframeAnimator[numberOfServos];
//
//
//
//
//	// Allocation (let's suppose size contains some value discovered at runtime,
//	// e.g. obtained from some external source or through other program logic)
//	if (_keyframeAnimators != 0) {
//	    delete [] _keyframeAnimators;
//	}
////	DEBUG_SERIAL_NAME.println("AAAAAAAAAAAAAAAAAAAAAAA");
//	_keyframeAnimators = new ServoKeyframeAnimator [numberOfServos];
//	_numberOfServos=numberOfServos;


	Log.verbose(F("ServoKeyframeAnimatorGroup::init - numServos=%d=%d _timePreviousKeyframe=%d _isInMove=%T _duration=%d, "CR), getNumberOfServos(), _numberOfServos, _timePreviousKeyframe, _isInMove, _duration );
}


ServoKeyframeAnimatorGroup::ServoKeyframeAnimatorGroup(ServoKeyframeAnimator* keyframeAnimators, EnhancedServo* servos, unsigned char numberOfServos)
{
	Log.verbose(F("ServoKeyframeAnimatorGroup (ServoKeyframeAnimator)"CR ));
	_timePreviousKeyframe=0;
	_isInMove=false;
	_duration=0;
	_keyframeAnimators=keyframeAnimators;
	_numberOfServos=numberOfServos;
	_servos = servos;
	}

/**
 * returns the number of servos
 */
unsigned char ServoKeyframeAnimatorGroup::getNumberOfServos()
{
	// return sizeof _keyframeAnimators  ;
	 // return (sizeof(_keyframeAnimators)/sizeof(*_keyframeAnimators));
	// Log.trace(F("ServoKeyframeAnimatorGroup::getNumberOfServos %d=%d"CR), _numberOfServos, sizeof _servos / sizeof (_servos[0]));
	return _numberOfServos;
}


/**
 * this returns a child ServoKeyframeAnimator. ID is the position in the array
 */
ServoKeyframeAnimator ServoKeyframeAnimatorGroup::getServoKeyframeAnimator (unsigned char id)
{
	return _keyframeAnimators[id];
}


ServoKeyframeAnimatorGroup::~ServoKeyframeAnimatorGroup() {
	// TODO Auto-generated destructor stub
}

/**
 * get complete time which is targeted for the complete move.
 * It is the time between previous and next keyframe
 */

unsigned int ServoKeyframeAnimatorGroup::getMoveDuration()
{
	return _duration;
}

/**
 * sets the time / duration which is used to move the servo from previous keyframe to target keyframe
 */
void ServoKeyframeAnimatorGroup::setServoMoveDuration(unsigned int duration)
{
	_duration = duration;
}


/**
 * calculate which position all servos of the groups should have now. Update the current servo position.
 * this does not move the servo. This needs to be done externally. Here is only the calculation.
 */
void ServoKeyframeAnimatorGroup::calculateServoPositions()
{
	if (!_isInMove)
	{
		// not in move jet, so lets start it
		_isInMove=true;
		// remember the time when the move starts for later calucation
		_timePreviousKeyframe=millis();

		#if DEBUG_SERVO_KEYFRAME_ANIMATOR_GROUP_CALCULATE_SERVO_POSITIONS == 1
			Log.trace(F("ServoKeyframeAnimatorGroup::calculateServoPositions : set _isInMove=%T at _timePreviousKeyframe=%l"CR), _isInMove, _timePreviousKeyframe );
		#endif

	}
	unsigned long currentTime = millis();


	// check if time is reached
	if (currentTime > _timePreviousKeyframe + _duration)
	{
		// time is reached -> end the move and set servo to final position
		_isInMove=false;

		#if DEBUG_SERVO_KEYFRAME_ANIMATOR_GROUP_CALCULATE_SERVO_POSITIONS == 1
			Log.trace(F("ServoKeyframeAnimatorGroup::calculateServoPositions : time is up (%d > %d + %d = %d, set _isInMove=%T, setting servos to final position"CR), currentTime, _timePreviousKeyframe, _duration, _timePreviousKeyframe + _duration,  _isInMove);
		#endif

		for (unsigned char s=0; s < getNumberOfServos() ; s++)
		{
			_keyframeAnimators[s].setServoAbsolutePosition(_keyframeAnimators[s].getServoTargetPositon());
			_keyframeAnimators[s].setServoPositionPreviousKeyframe(_keyframeAnimators[s].getServoTargetPositon());
		}
	}
	else
	{
		#if DEBUG_SERVO_KEYFRAME_ANIMATOR_GROUP_CALCULATE_SERVO_POSITIONS == 1
			Log.trace(F("ServoKeyframeAnimatorGroup::calculateServoPositions : time is NOT up calculating positions."CR));
		#endif

		// time is not up, we are within the move. calculate the new servo position and set it as current position.
		for (unsigned char s=0; s < getNumberOfServos() ; s++)
		{
			_keyframeAnimators[s].setServoAbsolutePosition(_keyframeAnimators[s].getCalculatedServoPosition(_timePreviousKeyframe, _duration));
		}
	}


//	#if DEBUG_SERVO_KEYFRAME_ANIMATOR_GROUP_CALCULATE_SERVO_POSITIONS == 1
//		Log.trace(F("ServoKeyframeAnimatorGroup::calculateServoPositions : end. calculated Positions are: %d, %d, %d, &d set _isInMove=%T, setting servos to final position"CR), currentTime, _timePreviousKeyframe, _duration, _timePreviousKeyframe + _duration,  _isInMove);
//	#endif
}

/**
 * get the servo calculated position for a _specific_ servo.
 * calculation needs to be done before.
 */
unsigned char ServoKeyframeAnimatorGroup::getCalculatedServoPositionById (unsigned char id)
{
	return _keyframeAnimators[id].getServoCurrentPositon();
}


/**
 * sets the target position for next keyframe for a _specific_ servo identified by id
 */
void ServoKeyframeAnimatorGroup::setServoPositionNextKeyframeById (unsigned char id, unsigned char targetPosition)
{
	_keyframeAnimators[id].setServoPositionNextKeyframe(targetPosition);
}


/**
 * sets the target position for next keyframe for _all_ servos, given as array
 */
void ServoKeyframeAnimatorGroup::setServoPositionsNextKeyframe (unsigned char* targetPositions)
{

	for (unsigned char s=0; s < _numberOfServos; s++)
	{
		_keyframeAnimators[s].setServoPositionNextKeyframe(targetPositions[s]);

	}

	Log.trace(F("ServoKeyframeAnimatorGroup::setServoPositionsNextKeyframe: array is updated to: "));
	for (unsigned char s=0; s < _numberOfServos; s++)
	{
		Log.trace(F("%d "), _keyframeAnimators[s].getServoTargetPositon());
	}
}


/**
 *
 * returns if we are within a move
 */
bool ServoKeyframeAnimatorGroup::isInMove()
{
	Log.trace(F("ServoKeyframeAnimatorGroup::isInMove() - returning %T" CR), _isInMove);
	return _isInMove;
}


/**
 * moves the linked servos of the group to its calculated positions.
 * calculation needs to be called before!
 */
void ServoKeyframeAnimatorGroup::driveServosToCalculatedPosition()
{
	for (unsigned char i=0; i < getNumberOfServos(); i++)
	{
		//_servos[i].enhancedWrite(_keyframeAnimators.getCalculatedServoPositionById(i) , 0, 180);

		_servos[i].enhancedWrite(getCalculatedServoPositionById(i), 0, 180);
		Log.trace(F("ServoKeyframeAnimatorGroup::driveServosToCalculatedPosition -> servo[%d]=%d" CR), i,getCalculatedServoPositionById(i) );
	}
}
