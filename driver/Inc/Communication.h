#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include "packet.h"

void CommunicationInit(void);
void ReceiveByte(uint8_t c);
void TransmitRun(void);
void TransmitReturnNone(uint8_t cmd);
void TransmitByte(uint8_t cmd, uint8_t data);
void TransmitFloat(uint8_t cmd, float data);
void TransmitPacket(uint8_t cmd, uint8_t* data, uint8_t length);
void TransmitNoDataCmd(uint8_t cmd);
void TransmitMotorStop(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMMUNICATION_H */
