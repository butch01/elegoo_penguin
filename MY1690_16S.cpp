/*
 * MY169016S.cpp
 *
 *  Created on: 24.01.2021
 *      Author: butch
 */

#include "MY1690_16S.h"

#include <arduino.h>

MY1690_16S::MY1690_16S (NeoSWSerial *mp3Serial)
{
	_mp3Serial= mp3Serial;
	volume = 10;
}

MY1690_16S::~MY1690_16S()
{
	// TODO Auto-generated destructor stub
}



/* Music Playing Choice*/
void MY1690_16S::playSong(unsigned char num, unsigned char vol)
{
	setVolume(vol);
	setPlayMode(4);
	CMD_SongSelet[4] = num;
	checkCode(CMD_SongSelet);
	_mp3Serial->write(CMD_SongSelet, 7);
	delay(50);
}

/* Get playback status*/
String MY1690_16S::getPlayStatus()
{
	_mp3Serial->write(CMD_getPlayStatus, 5);
	delay(50);
	return getStatus();
}

/* Get status*/
String MY1690_16S::getStatus()
{
	String statusMp3 = "";
	while (_mp3Serial->available())
	{
		statusMp3 += (char)_mp3Serial->read();
	}
	return statusMp3;
}


/* Stop broadcasting*/
void MY1690_16S::stopPlay()
{
	setPlayMode(4);
	_mp3Serial->write(CMD_MusicStop, 5);
	delay(50);
}

/* Volume setting*/
void MY1690_16S::setVolume(unsigned char vol)
{
	CMD_VolumeSet[3] = vol;
	checkCode(CMD_VolumeSet);
	_mp3Serial->write(CMD_VolumeSet, 6);
	delay(50);
}


/* Voice Enhancement*/
void MY1690_16S::volumePlus()
{
	_mp3Serial->write(CMD_VolumePlus, 5);
	delay(50);
}

/* Lower volume*/
void MY1690_16S::volumeDown()
{
	_mp3Serial->write(CMD_VolumeDown, 5);
	delay(50);
}

void MY1690_16S::setPlayMode(unsigned char mode)
{
	CMD_PlayMode[3] = mode;
	checkCode(CMD_PlayMode);
	_mp3Serial->write(CMD_PlayMode, 6);
	delay(50);
}

void MY1690_16S::checkCode(unsigned char *vs)
{
	int val = vs[1];
	int i;
	for (i = 2; i < vs[1]; i++)
	{
		val = val ^ vs[i];
	}
	vs[i] = val;
}

void MY1690_16S::ampMode(int p, bool m)
{
	pinMode(p, OUTPUT);
	if (m)
	{
		digitalWrite(p, HIGH);
	}
	else
	{
		digitalWrite(p, LOW);
	}
}


void MY1690_16S::init(unsigned char HT6871_PIN)
{
	ampMode(HT6871_PIN, HIGH);
	stopPlay();
	volume = 15;
}


