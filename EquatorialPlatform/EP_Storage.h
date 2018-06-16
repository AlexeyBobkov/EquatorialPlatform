/*
 * EP_Storage.h
 *
 * Created: 12/1/2013 7:20:46 PM
 *  Author: Alexey
 */ 


#ifndef EP_STORAGE_H_
#define EP_STORAGE_H_

///////////////////////////////////////////////////////////////////////////////////////
// read/write fast movement RPS
double EP_ReadFastRPS();
void EP_WriteFastRPS(double val);

///////////////////////////////////////////////////////////////////////////////////////
// read/write fast movement stepping
uint8_t EP_ReadFastStepping();
void EP_WriteFastStepping(uint8_t val);

///////////////////////////////////////////////////////////////////////////////////////
// read/write slow movement stepping
uint8_t EP_ReadSlowStepping();
void EP_WriteSlowStepping(uint8_t val);

///////////////////////////////////////////////////////////////////////////////////////
void EP_WriteCurrentAnglePos(double angle);
bool EP_ReadCurrentAnglePos(double *pAngle);

///////////////////////////////////////////////////////////////////////////////////////
void EP_WriteCapabilities(uint8_t capabilities);
bool EP_ReadCapabilities(uint8_t *pCapabilities);

///////////////////////////////////////////////////////////////////////////////////////
void EP_WriteSessionId(uint8_t sessionId);
bool EP_ReadSessionId(uint8_t *pSessionId);

///////////////////////////////////////////////////////////////////////////////////////
void EP_WriteMotoAngle(bool fStart, double angle);
bool EP_ReadMotoAngle(bool fStart, double *pAngle, double defaultVal);


#endif /* EP_STORAGE_H_ */