#ifndef __COMMAND_H
#define __COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Packet.h"

void CmdMoveEnableExecute(CmdMoveEnable* cmd);    
void CmdMoveRelativeExecute(CmdMoveRelative* cmd);
void CmdMoveAbsoluteExecute(CmdMoveAbsolute* cmd);
void CmdMovePositionExecute(CmdMovePosition* cmd);
//void CmdMoveFrameExecute(CmdMoveFrame* cmd);
void CmdMoveByTimeExecute(CmdMoveByTime* cmd);
void CmdMoveResetExecute(CmdMoveReset* cmd);
void CmdMoveSpeedExecute(CmdMoveSpeed* cmd);
    void CmdMoveSpeedDirectExecute(CmdMoveSpeed* cmd);
void CmdMoveAccelerateSettingExecute(CmdMoveAccelerateSetting* cmd);
void CmdForwardSettingExecute(CmdForwardSetting* cmd);
void CmdMoveLimitSettingExecute(CmdMoveLimitSetting* cmd);
void CmdMoveAttitudeExecute(CmdMoveAttitude* cmd);
void CmdMovePitchExecute(CmdMovePitch* cmd);
    
#ifdef __cplusplus
}
#endif

#endif /* __COMMAND_H */
