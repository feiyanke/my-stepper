#ifndef __PACKET_H
#define __PACKET_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include "stdint.h"
//Packet格式: SYNC(0x2C) CMD LENGTH DATA[] CHKSUM
#define SYNC   0x2C
#define MAX_DATA_LENGTH 255
#define MAX_PACKET_LENGTH (MAX_DATA_LENGTH+4)
    
__packed typedef struct
{
    uint8_t  cmd;
    uint8_t  length;
    uint8_t  data[MAX_DATA_LENGTH];
} Packet;

//接收的消息
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
    CMD_READ_MOTOR_STOP,    //收到这个后如果电机处于停止状态，则立即返回CMD_MOTOR_ALL_STOP，否则等到电机停止后发送
    CMD_READ_INFORMATION,   //读取版本等信息
    
    //以下为Bootloader命令码
    CMD_BOOT_UPDATE_SYN=200,	//0xC8=200  上位机发来的升级握手命令
    CMD_BOOT_APP_DATA=202,		//0xCA=202  上位机下发的应用程序数据
    CMD_BOOT_APP_END=203,		//0xCB=203  上位机下发的应用程序数据“发送完成”

}ReceiveCommand;

//电机的相对运动
__packed typedef struct
{
    uint8_t type;       
    float angle;        //°
    float speed;        //°/s
} CmdMoveAngle;

//复位,三个电机同时进行硬复位
__packed typedef struct
{
    uint8_t mode;   //0-以软复位方向为运动方向的硬复位，1-软复位
    float speed;
} CmdReset;

//速度控制
__packed typedef struct
{
    float velocity;   //°/s
    float accelerate; //°/s/s
} CmdMoveSpeed;

__packed typedef struct
{
    float velocity;   //°/s
} CmdMoveSpeedDirect;

__packed typedef struct
{
    uint8_t type;   //0-relative, 1-absolut
    float angle;         //°/s/s 
} CmdForwardSetting;


//发送的消息
typedef enum
{
    CMD_RETURN = 0,
    CMD_QUEUE_OVERFLOW,
    
    CMD_UPDATE_POSITION = 0x60,
    CMD_UPDATE_FORWARD,
    CMD_UPDATE_MOTOR_STOP,
    CMD_UPDATE_INFORMATION,        //更新当前的版本信息
    
    //以下为Bootloader命令码
    CMD_BOOT_REQ_DATA = 0xC9,        //0xC9=201  下位机向上位机请求数据
    CMD_BOOT_UPDATE_ERROR = 0xCC,	//0xCC=204  下位机给上位机的“升级失败”应答
    CMD_BOOT_UPDATE_SUCCESS = 0xCD,	//0xCD=205  下位机给上位机的“升级成功”应答
    CMD_BOOT_UP_INFO = 0xCE,	    //0xCE=206  下位机给上位机的自身信息
    
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
