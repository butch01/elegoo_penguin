/*
 * RobotMoves01.cpp
 *
 *  Created on: 24.01.2021
 *      Author: butch
 */

#include "RobotMoves01.h"
#include <arduino.h>
#include "moveConstants.h"
#include <ArduinoLog.h>

RobotMoves01::RobotMoves01() {
	// TODO Auto-generated constructor stub
	// initialize the array
//	for (unsigned int i=0; i<(sizeof _keyframe / sizeof _keyframe[0]); i++)
//	{
//		_keyframe[i]=90;
//	}

	// initialize array with 0
	for (unsigned char i=0; i< MOVE_BUFFER_ARRAY_ITERATIONS; i++)
	{
		for (unsigned char e=0; e < MOVE_BUFFER_ARRAY_ELEMENTS; e++)
				_moveBuffer[i][e]=0;
	}

	// set the number of move iterations to 0;
	_moveIterations=0;


}

RobotMoves01::~RobotMoves01() {
	// TODO Auto-generated destructor stub
}


/**
 * returns the continuation mode. Default is ITERATION_MODE_LOOP
 */
unsigned char RobotMoves01::getContinuationMode(unsigned char moveId)
{
	// smooth as default
	unsigned char continuationMode = ITERATION_CONTINUATION_MODE_LOOP;


	return continuationMode;
}

/**
 * transfers the keyframe configuration of a specific move[iteration] to a target array.
 * keyframe has following format [time in 0.01 sec;servoPos0; servoPos1; servoPos2; servoPos3)
 * time example: value 200 means 200 * 0.01s = 2s. 10 = 0.1s
 *
 * moveId - ID of move
 * iteration - the iteration to return
 * targetMove - Reference if the array where to copy the move's iteration to
 */
void RobotMoves01::getKeyframe(unsigned char moveId, unsigned char iteration, unsigned char* targetMove)
{

	// Log.trace(F("RobotMoves01::getKeyframe args moveId=%d, iteration=%d, targetMoveAddr=%X, targetMove[0]=%d, targetMove[1]=%d" CR ), moveId, iteration, &targetMove, targetMove[0], targetMove[1]);


	if (moveId == MOVE_01_CENTER)
	{
		unsigned char move[][5] =
		{
				{100,90,90,90,90}
		};
		memcpy(targetMove, move[iteration], sizeof move[iteration]);
	}
	else if (moveId == MOVE_01_WALKFORWARD)
	{
		unsigned char move[6][5] =
		{
			{100, 90, 90 + 35, 90 + 15, 90 + 15},
			{100, 90 + 25, 90 + 30, 90 + 15, 90 + 15},
			{100, 90 + 20, 90 + 20, 90 - 15, 90 - 15},
			{100, 90 - 35, 90, 90 - 15, 90 - 15},
			{100, 90 - 40, 90 - 30, 90 - 15, 90 - 15},
			{100, 90 - 20, 90 - 20, 90 + 15, 90 + 15}
		};
		memcpy(targetMove, move[iteration], sizeof move[iteration]);
	}
	else if (moveId == MOVE_01_TEST)
	{
		unsigned char move[4][5] =
		{
			{100, 90,    90,    90, 90},
			{100, 90,    90-80, 90, 90},
			{100, 90,    90,    90, 90},
			{100, 90,    90+80, 90, 90}
		};
		memcpy(targetMove, move[iteration], sizeof move[iteration]);
	}
	else if (moveId == MOVE_01_WALKTBACKWARDS)
	{
		unsigned char move[6][5] =
		{
			{100, 90,      90 + 35, 90 - 15, 90 - 15},
			{100, 90 + 25, 90 + 30, 90 - 15, 90 - 15},
			{100, 90 + 20, 90 + 20, 90 + 15, 90 + 15},
			{100, 90 - 35, 90,      90 + 15, 90 + 15},
			{100, 90 - 40, 90 - 30, 90 + 15, 90 + 15},
			{100, 90 - 20, 90 - 20, 90 - 15, 90 - 15},
		};
		memcpy(targetMove, move[iteration], sizeof move[iteration]);
	}
	else if (moveId == MOVE_01_TURN_RIGHT)
	{
		unsigned char move[8][5] =
		{
			{100, 90 - 55, 90 - 20, 90 + 20, 90 + 20},
			{100, 90 - 20, 90 - 20, 90 + 20, 90 - 20},
			{100, 90 + 20, 90 + 55, 90 + 20, 90 - 20},
			{100, 90 + 20, 90 + 20, 90 - 20, 90 + 20},
			{100, 90 - 55, 90 - 20, 90 - 20, 90 + 20},
			{100, 90 - 20, 90 - 20, 90 + 20, 90 - 20},
			{100, 90 + 20, 90 + 55, 90 + 20, 90 - 20},
			{100, 90 + 20, 90 + 20, 90 - 20, 90 + 20}
		};
		memcpy(targetMove, move[iteration], sizeof move[iteration]);
	}
	else if (moveId == MOVE_01_TURN_LEFT)
	{
		unsigned char move[8][5] =
		{
			{90 + 20, 90 + 55, 90 + 20, 90 + 20},
			{90 + 20, 90 + 20, 90 + 20, 90 - 20},
			{90 - 55, 90 - 20, 90 + 20, 90 - 20},
			{90 - 20, 90 - 20, 90 - 20, 90 - 20},
			{90 + 20, 90 + 55, 90 - 20, 90 + 20},
			{90 + 20, 90 + 20, 90 + 20, 90 - 20},
			{90 - 55, 90 - 20, 90 + 20, 90 - 20},
			{90 - 20, 90 - 20, 90 - 20, 90 - 20}
		};
		memcpy(targetMove, move[iteration], sizeof move[iteration]);
	}


}


//unsigned char RobotMoves01::getNumberOfServosUsed(unsigned char moveId)
//{
//	// -1 because first element is base speed.
//	 return (unsigned char) (sizeof getMove(moveId)[0] / sizeof getMove(moveId)[0][0]) -1;
//}

//
//
///**
// * returns a the reference to the move object.
// * Hint: Will it drain the memory with the always new?
// */
//void RobotMoves01::getMoveToBuffer(unsigned char moveId)
//{
//
//
//	if (moveId == MOVE_01_CENTER)
//	{
//
//		unsigned char move[1][5]=
//		{
//			{100,90,90,90,90}
//
//		};
//
//		memcpy(move, _moveBuffer, sizeof (move) );
//	}
//
//	if (moveId == MOVE_01_WALKFORWARD)
//	{
//		unsigned char move[6][5] =
//		{
//			{100, 90, 90 + 35, 90 + 15, 90 + 15},
//			{100, 90 + 25, 90 + 30, 90 + 15, 90 + 15},
//			{100, 90 + 20, 90 + 20, 90 - 15, 90 - 15},
//			{100, 90 - 35, 90, 90 - 15, 90 - 15},
//			{100, 90 - 40, 90 - 30, 90 - 15, 90 - 15},
//			{100, 90 - 20, 90 - 20, 90 + 15, 90 + 15}
//		};
//
//	}
//}




/**
 * returns the number of available iterations of the move.
 * ATTENTION: Number of iterations are hardcoded here. They need to stay in Sync with the moves
 */
unsigned char RobotMoves01::getNumberOfIterations(unsigned char moveId )
{
	unsigned char iterationCount=0;
	switch (moveId)
	{
	case MOVE_01_CENTER:
		iterationCount= 1;
		break;
	case MOVE_01_WALKFORWARD:
		iterationCount=6;
		break;
	case MOVE_01_TEST:
		iterationCount=4;
		break;
	case MOVE_01_WALKTBACKWARDS:
		iterationCount=6;
		break;
	case MOVE_01_TURN_LEFT:
		iterationCount=8;
		break;
	case MOVE_01_TURN_RIGHT:
		iterationCount=8;
		break;
	}
	return iterationCount;

}

