/*
 * ServoKeyframeAnimatorGroup.h
 *
 * This will group the keyframing for some servos: eg: legs, arms, head, ...
 *
 * use as few groups as possible to save RAM.
 *
 *  Created on: 19.01.2021
 *      Author: butch
 */

#ifndef SERVOKEYFRAMEANIMATORGROUP_H_
#define SERVOKEYFRAMEANIMATORGROUP_H_

#include "ServoKeyframeAnimator.h"
#include <EnhancedServo.h>

class ServoKeyframeAnimatorGroup {
public:
	ServoKeyframeAnimatorGroup();
	ServoKeyframeAnimatorGroup(ServoKeyframeAnimator* keyframeAnimators, EnhancedServo* servos, unsigned char numberOfServos);


	virtual ~ServoKeyframeAnimatorGroup();

	ServoKeyframeAnimator getServoKeyframeAnimator (unsigned char id);
	void ServoKeyframeAnimatorGroup::init(unsigned char numberOfServos);


	unsigned char getNumberOfServos();


	unsigned char getCalculatedServoPositionById (unsigned char id);
	void setServoPositionNextKeyframeById (unsigned char id, unsigned char targetPosition);
	void setServoPositionsNextKeyframe (unsigned char targetPositions[]);

	unsigned int getMoveDuration();
	void setServoMoveDuration(unsigned int duration);

	void calculateServoPositions();

	bool isInMove();

	void driveServosToCalculatedPosition();

private:
	ServoKeyframeAnimator* 	_keyframeAnimators;
	unsigned long 			_timePreviousKeyframe;
	unsigned int            _duration;
	unsigned char			_numberOfServos;
	bool 					_isInMove;
	EnhancedServo*          _servos;
};

#endif /* SERVOKEYFRAMEANIMATORGROUP_H_ */
