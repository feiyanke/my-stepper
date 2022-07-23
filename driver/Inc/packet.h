#ifndef __PACKET_H
#define __PACKET_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include "stdint.h"
//Packet��ʽ: SYNC(0x2C) CMD LENGTH DATA[] CHKSUM
#define SYNC   0x2C
#define MAX_DATA_LENGTH 255
#define MAX_PACKET_LENGTH (MAX_DATA_LENGTH+4)
    
__packed typedef struct
{
    uint8_t  cmd;
    uint8_t  length;
    uint8_t  data[MAX_DATA_LENGTH];
} Packet;

//���յ���Ϣ
typedef enum
{
    CMD_MOVE_ANGLE = 0,
    CMD_MOVE_SPEED,
    CMD_MOVE_RESET,
    CMD_MOVE_SPEED_DIRECT,
    CMD_MOVE_WAKEUP,
    
    CMD_FORWARD_SETTING = 0x50,
    
    CMD_LED_SCENE_CONTROL,
    CMD_LED_RGB_CALIBRATION,
    
    CMD_READ_POSITION = 0x60,
    CMD_READ_FORWARD,
    CMD_READ_MOTOR_STOP,    //�յ����������������ֹͣ״̬������������CMD_MOTOR_ALL_STOP������ȵ����ֹͣ����
    CMD_READ_INFORMATION,   //��ȡ�汾����Ϣ
    
    //����ΪBootloader������
    CMD_BOOT_UPDATE_SYN=200,	//0xC8=200  ��λ��������������������
    CMD_BOOT_APP_DATA=202,		//0xCA=202  ��λ���·���Ӧ�ó�������
    CMD_BOOT_APP_END=203,		//0xCB=203  ��λ���·���Ӧ�ó������ݡ�������ɡ�

}ReceiveCommand;

//���������˶�
__packed typedef struct
{
    uint8_t type;       
    float angle;        //��
    float speed;        //��/s
} CmdMoveAngle;

//��λ,�������ͬʱ����Ӳ��λ
__packed typedef struct
{
    uint8_t mode;   //0-����λ����Ϊ�˶������Ӳ��λ��1-��λ
    float speed;
} CmdReset;

//�ٶȿ���
__packed typedef struct
{
    float velocity;   //��/s
    float accelerate; //��/s/s
} CmdMoveSpeed;

__packed typedef struct
{
    float velocity;   //��/s
} CmdMoveSpeedDirect;

__packed typedef struct
{
    uint8_t type;   //0-relative, 1-absolut
    float angle;         //��/s/s 
} CmdForwardSetting;


//���͵���Ϣ
typedef enum
{
    CMD_RETURN = 0,
    CMD_QUEUE_OVERFLOW,
    
    CMD_UPDATE_POSITION = 0x60,
    CMD_UPDATE_FORWARD,
    CMD_UPDATE_MOTOR_STOP,
    CMD_UPDATE_INFORMATION,        //���µ�ǰ�İ汾��Ϣ
    
    //����ΪBootloader������
    CMD_BOOT_REQ_DATA = 0xC9,        //0xC9=201  ��λ������λ����������
    CMD_BOOT_UPDATE_ERROR = 0xCC,	//0xCC=204  ��λ������λ���ġ�����ʧ�ܡ�Ӧ��
    CMD_BOOT_UPDATE_SUCCESS = 0xCD,	//0xCD=205  ��λ������λ���ġ������ɹ���Ӧ��
    CMD_BOOT_UP_INFO = 0xCE,	    //0xCE=206  ��λ������λ����������Ϣ
    
}SendCommand;

/*__packed typedef struct
{
    uint8_t cmd;
} CmdReturnNone; */

/*__packed typedef struct
{
    uint8_t cmd;
    uint8_t code; //0-success 1-fail
} CmdReturnErrorCode; */

/*__packed typedef struct
{
    float angle;
} CmdUpdatePosition; */

/*__packed typedef struct
{
    float forward;
} CmdUpdateForward; */

__packed typedef struct
{
    float forward;
    float angle;
} CmdMotorStop; 

__packed typedef struct
{
    uint32_t nAppVer;
    uint32_t nAppChecksum;
} CmdUpdateInformation;

        
#ifdef __cplusplus
}
#endif

#endif /* __PACKET_H */
