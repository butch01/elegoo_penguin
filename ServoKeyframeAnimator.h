/*
 * ServoKeyframeAnimator.h
 *
 *  Created on: 19.01.2021
 *      Author: butch
 */

#ifndef SERVOKEYFRAMEANIMATOR_H_
#define SERVOKEYFRAMEANIMATOR_H_

#define KEYFRAME_MODE_ABSOLUTE 0 // absolute control for direct servo access
#define KEYFRAME_MODE_SMOOTH 1 // smooth start / stop
#define KEYFRAME_MODE_LINEAR 2 // linear movement, sudden start / stop at / from full speed
#define KEYFRAME_MODE_SMOOTH_START 3 // only smooth start, sudden_stop
#define KEYFRAME_MODE_SMOOTH_STOP 4 // only smooth start, sudden_stop



class ServoKeyframeAnimator {
public:
	ServoKeyframeAnimator();
	virtual ~ServoKeyframeAnimator();


	unsigned char getCalculatedServoPosition( 			// calculates position where the servo needs to be now.
	unsigned long timePrevKey,
	unsigned int targetDuration);

	void setServoPositionPreviousKeyframe(unsigned char previousPos);
	void setServoPositionNextKeyframe(unsigned char targetPos);



	void setServoAbsolutePosition(unsigned char absolutePos);
	void setServoAbsolutePositionChange(signed int absolutePosChange);

	unsigned char getServoTargetPositon();
	unsigned char getServoCurrentPositon();

	void setKeyframeMode(unsigned char keyframeMode);


private:
	unsigned int 	_duration=0;
	unsigned char 	_currentPosition;
	unsigned char 	_targetKeyframePosition;
	unsigned char 	_previousKeyframePosition;
	unsigned char	_keyframeMode;

};

#endif /* SERVOKEYFRAMEANIMATOR_H_ */
