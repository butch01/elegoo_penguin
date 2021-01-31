/*
 * RobotMoves01.h
 *
 *  Created on: 24.01.2021
 *      Author: butch
 */

#ifndef ROBOTMOVES01_H_
#define ROBOTMOVES01_H_
#define MOVE_BUFFER_ARRAY_ITERATIONS 10
#define MOVE_BUFFER_ARRAY_ELEMENTS 5





class RobotMoves01 {
public:
	RobotMoves01();
	virtual ~RobotMoves01();


// 	unsigned char   getNumberOfServosUsed(unsigned char moveId);
//	void getMoveToBuffer(unsigned char moveId);


	unsigned char getNumberOfIterations(unsigned char moveId );
	unsigned char getContinuationMode(unsigned char moveId);
	void getKeyframe(unsigned char moveId, unsigned char iteration, unsigned char* targetMove);

private:
//	unsigned char* getKeyframeByIdWalkForward(unsigned int id);
//	unsigned char getNumberOfKeyframeIterationsWalkForward();
//	unsigned char* getKeyframeByIDCenter(unsigned int id);




	unsigned char _moveBuffer[MOVE_BUFFER_ARRAY_ITERATIONS][MOVE_BUFFER_ARRAY_ELEMENTS];
	unsigned char _moveIterations;

};

#endif /* ROBOTMOVES01_H_ */
