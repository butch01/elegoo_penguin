/*
 * moveConstants.h
 *
 *  Created on: 25.01.2021
 *      Author: butch
 */

#ifndef MOVECONSTANTS_H_
#define MOVECONSTANTS_H_

#define ITERATION_CONTINUATION_MODE_LOOP 0 		// start from 0 again when last iteration is ended
#define ITERATION_CONTINUATION_MODE_REVERSE 1	// make iterations backward in reverse order if end is reached
#define ITERATION_CONTINUATION_MODE_ONCE 2		// stop when last iteration is done.

#define KEYFRAME_MODE_ABSOLUTE 0 // absolute control for direct servo access
#define KEYFRAME_MODE_SMOOTH 1 // smooth start / stop
#define KEYFRAME_MODE_LINEAR 2 // linear movement, sudden start / stop at / from full speed
#define KEYFRAME_MODE_SMOOTH_START 3 // only smooth start, sudden_stop
#define KEYFRAME_MODE_SMOOTH_STOP 4 // only smooth start, sudden_stop


#define MOVE_01_CENTER 0
#define MOVE_01_WALKFORWARD 1
#define MOVE_01_TEST 2

#define MOVE_DEBUG_SINGLE_ARRAY 99





#endif /* MOVECONSTANTS_H_ */
