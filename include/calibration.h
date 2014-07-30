#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

typedef struct
{
	uint16_t offsets[6];
	float scales[6];
	float magic;
}__attribute__((packed))CalibVals;

int writeCalibration(unsigned char *data, unsigned int len);
int readCalibration(unsigned char *data, unsigned int len);

#endif // _CALIBRATION_H_
