#ifndef _FREE_IMU_SERIAL_H_
#define _FREE_IMU_SERIAL_H_

typedef struct
{
	void (*GetVersion)(void);
	void (*GetIMURaw)(void);
	void (*GetBaseValues)(char count);
	void (*GetQuat)(char count);
	void (*LoadCalibrationValues)(void);
	void (*ClearCalibration)(void);
	void (*Calibrate)(void);
	void (*Debug)(void);
	void (*GetAttitude)(int count);
	void (*GetAttitudeBytes)(void);
	void (*FormatFlash)(void);
}FreeIMU_Func;

int FreeIMU_serial(FreeIMU_Func *funcs);
void FreeIMUVersion(void);
void FreeIMURaw(void);
void FreeIMUBaseInt(char count);
void FreeIMUSendYawPitchRoll(int count);
void FreeIMUWriteCalibration(void);
void FreeIMUReadCalibration(void);
void FreeIMUClearCalibration(void);
void FreeIMUFormatFlash(void);

#endif // _FREE_IMU_SERIAL_H_
