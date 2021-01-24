/*
 * MY169016S.h
 *
 *  Created on: 24.01.2021
 *      Author: butch
 */

#ifndef MY1690_16S_H_
#define MY1690_16S_H_

#include <arduino.h>
#include <NeoSWSerial.h>

class MY1690_16S {
public:
	MY1690_16S(NeoSWSerial *mp3Serial);
	virtual ~MY1690_16S();

	void test();

	int volume;
	String playStatus[5] = {"0", "1", "2", "3", "4"}; // STOP PLAYING PAUSE FF FR

	/* Music Playing Choice*/
	void playSong(unsigned char num, unsigned char vol);

	/* Get playback status*/
	String getPlayStatus();

	/* Get status*/
	String getStatus();


	/* Stop broadcasting*/
	void stopPlay();


	/* Volume setting*/
	void setVolume(unsigned char vol);


	/* Voice Enhancement*/
	void volumePlus();


	/* Lower volume*/
	void volumeDown();


	void setPlayMode(unsigned char mode);


	void checkCode(unsigned char *vs);


	void ampMode(int p, bool m);


	void init(unsigned char HT6871_PIN);



private:
	byte CMD_MusicPlay[5] = {0x7E, 0x03, 0x11, 0x12, 0xEF};
	byte CMD_MusicStop[5] = {0x7E, 0x03, 0x1E, 0x1D, 0xEF};
	byte CMD_MusicNext[5] = {0x7E, 0x03, 0x13, 0x10, 0xEF};
	byte CMD_MusicPrev[5] = {0x7E, 0x03, 0x14, 0x17, 0xEF};
	byte CMD_VolumePlus[5] = {0x7E, 0x03, 0x15, 0x16, 0xEF};
	byte CMD_VolumeDown[5] = {0x7E, 0x03, 0x16, 0x15, 0xEF};
	byte CMD_VolumeSet[6] = {0x7E, 0x04, 0x31, 0x00, 0x00, 0xEF};
	byte CMD_PlayMode[6] = {0x7E, 0x04, 0x33, 0x00, 0x00, 0xEF};
	byte CMD_SongSelet[7] = {0x7E, 0x05, 0x41, 0x00, 0x00, 0x00, 0xEF};
	byte CMD_getPlayStatus[5] = {0x7E, 0x03, 0x20, 0x23, 0xEF};

	NeoSWSerial *_mp3Serial;
};

#endif /* MY1690_16S_H_ */
