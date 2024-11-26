/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "api_motorcontrol.h"
#include "multicore.h"
#include "fsl_rgpio.h"
#include "pin_mux.h"


SHARED_MEMORY_IN_CORES volatile static mc_motor_command_t sMotorCommandSHM[2];    //define the command data from CM33 to CM7
SHARED_MEMORY_IN_CORES volatile static mc_motor_status_t  sMotorStatusSHM[2];     //define the status data from CM7 to CM3
/*
 *  Set status and get command functions
 * */
#if (__CORTEX_M == 7)

mc_motor_command_t g_sM1Cmd,g_sM2Cmd;

/*!
 * @brief Use this function to trigger MU interrupt , store the motor status into shared memory
 *
 * @param None
 *
 * @return None
 */
void MC_SetMotorStatusFromISR(mc_motor_status_t *ptrM1, mc_motor_status_t *ptrM2)
{

	sMotorStatusSHM[0].eMotorId = ptrM1->eMotorId;
	sMotorStatusSHM[0].sSlow = ptrM1->sSlow;
	sMotorStatusSHM[0].sFast = ptrM1->sFast;
	sMotorStatusSHM[0].g_eM1StateRun = ptrM1->g_eM1StateRun;
	sMotorStatusSHM[0].eControlMethodSel = ptrM1->eControlMethodSel;

	sMotorStatusSHM[1].eMotorId = ptrM2->eMotorId;
	sMotorStatusSHM[1].sSlow = ptrM2->sSlow;
	sMotorStatusSHM[1].sFast = ptrM2->sFast;
	sMotorStatusSHM[1].g_eM1StateRun = ptrM2->g_eM1StateRun;
	sMotorStatusSHM[1].eControlMethodSel = ptrM2->eControlMethodSel;

	SendEventToRemoteCore(Slave_SendMotorStatus);
}

/*!
 * @brief MU interrupt, when CM33 receive the command and store it into share memory, CM7 triggers this MU interrupt.
 *
 * @param None
 *
 * @return None
 */
static void MC_GetMotorCommand_fromSHM(void *ptr)
{
	g_sM1Cmd.ui16MUPospeSensor= sMotorCommandSHM[0].ui16MUPospeSensor;
	g_sM1Cmd.eAppSwitch = sMotorCommandSHM[0].eAppSwitch;
	g_sM1Cmd.eControlMethodSel = sMotorCommandSHM[0].eControlMethodSel;
	if(g_sM1Cmd.eControlMethodSel == kMC_FOC_SpeedControl)
	{
		g_sM1Cmd.uSpeed_pos.fltSpeed = sMotorCommandSHM[0].uSpeed_pos.fltSpeed;
	}
	else if(g_sM1Cmd.eControlMethodSel == kMC_FOC_PositionControl)
	{
		g_sM1Cmd.uSpeed_pos.sPosParam.bIsRandomPosition = sMotorCommandSHM[0].uSpeed_pos.sPosParam.bIsRandomPosition;
		g_sM1Cmd.uSpeed_pos.sPosParam.uPosition.i32Raw = sMotorCommandSHM[0].uSpeed_pos.sPosParam.uPosition.i32Raw;
	}
	else if(g_sM1Cmd.eControlMethodSel == kMC_ScalarControl)
	{
		g_sM1Cmd.uSpeed_pos.sScalarParam.fltScalarControlFrequency = sMotorCommandSHM[0].uSpeed_pos.sScalarParam.fltScalarControlFrequency;
		g_sM1Cmd.uSpeed_pos.sScalarParam.fltScalarControlVHzGain = sMotorCommandSHM[0].uSpeed_pos.sScalarParam.fltScalarControlVHzGain;
	}

	g_sM2Cmd.ui16MUPospeSensor= sMotorCommandSHM[1].ui16MUPospeSensor;
	g_sM2Cmd.eAppSwitch = sMotorCommandSHM[1].eAppSwitch;
	g_sM2Cmd.eControlMethodSel = sMotorCommandSHM[1].eControlMethodSel;
	if(g_sM2Cmd.eControlMethodSel == kMC_FOC_SpeedControl)
	{
		g_sM2Cmd.uSpeed_pos.fltSpeed = sMotorCommandSHM[1].uSpeed_pos.fltSpeed;
	}
	else if(g_sM2Cmd.eControlMethodSel == kMC_FOC_PositionControl)
	{
		g_sM2Cmd.uSpeed_pos.sPosParam.bIsRandomPosition = sMotorCommandSHM[1].uSpeed_pos.sPosParam.bIsRandomPosition;
		g_sM2Cmd.uSpeed_pos.sPosParam.uPosition.i32Raw = sMotorCommandSHM[1].uSpeed_pos.sPosParam.uPosition.i32Raw;
	}
	else if(g_sM2Cmd.eControlMethodSel == kMC_ScalarControl)
	{
		g_sM2Cmd.uSpeed_pos.sScalarParam.fltScalarControlFrequency = sMotorCommandSHM[1].uSpeed_pos.sScalarParam.fltScalarControlFrequency;
		g_sM2Cmd.uSpeed_pos.sScalarParam.fltScalarControlVHzGain = sMotorCommandSHM[1].uSpeed_pos.sScalarParam.fltScalarControlVHzGain;
	}

}

void installCoreCommTable(void)
{
	EventTableAddItem(Master_SendMotorCmd, MC_GetMotorCommand_fromSHM, Event_NULL);
	MCMGR_Core1Init(NULL);
}

#endif

#if (__CORTEX_M == 33)
#include "freemaster.h"
#include "ecat_hw.h"
mc_motor_status_t  g_sM1Status, g_sM2Status;
static mc_motor_command_t sM1CmdBuffer[2];
static mc_motor_command_t sM2CmdBuffer[2];
static bool bM1Buf0WriteInProgress = false;
static bool bM2Buf0WriteInProgress = false;

/*!
 * @brief This function is used to solve bus conflict issues;
 *
 * @param None
 *
 * @return None
 */
bool MC_ReadReadWriteFlag()
{
	uint32_t ui32Temp;
	do
	{
		SYSTICK_STOP_COUNT(ui32Temp);  //Read the time stamp of CM33 to determine if CM33 can read/write ECAT register
	}while((ui32Temp %7500)>6000);     //If (ui32Temp %7500)>6300) ,which means CM7 is reading/writing PWM/ADC register
}

/*!
 * @brief This function is used to solve bus conflict issues;
 *  CM33 accessing ECAT register operations are divided into 32-bit read/write, and each 32-bit read/write takes 400 ns;
 * @param None
 *
 * @return None
 */
void  Ecat_Read(ECAT_Type *ethercat,uint8_t* pdata,uint32_t Add, uint32_t L)
{
	uint32_t re;
	uint32_t length = L / 4;
	uint32_t index = 0;
	uint32_t *source = &((uint32_t *)(ethercat))[(Add) >> 2];
	while (index < length) {
		MC_ReadReadWriteFlag();
		*(((uint32_t*)pdata) + index) =  source[index];
		index++;
	}
	MC_ReadReadWriteFlag();
	re = source[index];
	memcpy((uint8_t *)(((uint32_t*)pdata) + index), (uint8_t *)&re, L % 4 );
}
/*!
 * @brief This function is used to solve bus conflict issues;
 *  CM33 accessing ECAT register operations are divided into 32-bit read/write, and each 32-bit read/write takes 400 ns;
 * @param None
 *
 * @return None
 */
void  Ecat_Write(ECAT_Type *ethercat,uint8_t* pdata,uint32_t Add, uint32_t L)
{
	uint32_t re;
	uint32_t length = L / 4;
	uint32_t index = 0;
	uint32_t *source = (uint32_t*)pdata;
	while (index < length) {
		MC_ReadReadWriteFlag();
		*((&((uint32_t *)(ethercat))[(Add) >> 2]) + index) =  source[index];
		index++;
	}
	MC_ReadReadWriteFlag();
	re = source[index];
	memcpy((uint8_t *)((&((uint32_t *)(ethercat))[(Add) >> 2]) + index), (uint8_t *)&re, L % 4 );
}


/*!
 * @brief This function is used to update the motor1 command from Cia402;
 *
 * @param None
 *
 * @return None
 */
void MC_SetMotor1Command(mc_motor_command_t *ptr)
{
	if(bM1Buf0WriteInProgress == false)
	{
		bM1Buf0WriteInProgress = true;
		sM1CmdBuffer[0] = *ptr;
		bM1Buf0WriteInProgress = false;
	}
}
/*!
 * @brief This function is used to update the motor1 command from Cia402;
 *
 * @param None
 *
 * @return None
 */
void MC_SetMotor2Command(mc_motor_command_t *ptr)
{
	if(bM2Buf0WriteInProgress == false)
	{
		bM2Buf0WriteInProgress = true;
		sM2CmdBuffer[0] = *ptr;
		bM2Buf0WriteInProgress = false;
	}
}

/*!
 * @brief CM33 receive the command and store it into share memory.
 *
 * @param None
 *
 * @return None
 */
RAM_FUNC_CRITICAL void MC_SetMotorCommandFromISR(void)
{
	if(bM1Buf0WriteInProgress == false)
	{
		sMotorCommandSHM[0] = sM1CmdBuffer[0];
		sM1CmdBuffer[1] = sM1CmdBuffer[0];
	}
	else
	{
		sMotorCommandSHM[0] = sM1CmdBuffer[1];
	}

	if(bM2Buf0WriteInProgress == false)
	{
		sMotorCommandSHM[1] = sM2CmdBuffer[0];
		sM2CmdBuffer[1] = sM2CmdBuffer[0];
	}
	else
	{
		sMotorCommandSHM[1] = sM2CmdBuffer[1];
	}

	SendEventToRemoteCore(Master_SendMotorCmd);
}


/*!
 * @brief MU interrupt, when CM7 store the motor status into share memory, triggers the MU interrupt
 *
 * @param None
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void MC_GetMotorStatus_fromSHM(void *ptr)
{
	g_sM1Status.sFast = sMotorStatusSHM[0].sFast;
	g_sM1Status.sSlow = sMotorStatusSHM[0].sSlow;
	g_sM1Status.eMotorId = kMC_Motor1;
	g_sM1Status.g_eM1StateRun = sMotorStatusSHM[0].g_eM1StateRun;
	g_sM1Status.eControlMethodSel = sMotorStatusSHM[0].eControlMethodSel;

	g_sM2Status.sFast = sMotorStatusSHM[1].sFast;
	g_sM2Status.sSlow = sMotorStatusSHM[1].sSlow;
	g_sM2Status.eMotorId = kMC_Motor2;
	g_sM2Status.g_eM1StateRun = sMotorStatusSHM[1].g_eM1StateRun;
	g_sM2Status.eControlMethodSel = sMotorStatusSHM[1].eControlMethodSel;

	FMSTR_Recorder(0);
}

volatile uint16_t ui16SlaveReady = 0U;
RAM_FUNC_CRITICAL static void SlaveReady(void *ptr)
{
    ui16SlaveReady = 1;
}

void installCoreCommTable(void)
{
	sM1CmdBuffer[0].eAppSwitch = kMC_App_Off;
	sM1CmdBuffer[1].eAppSwitch = kMC_App_Off;
	sM2CmdBuffer[0].eAppSwitch = kMC_App_Off;
	sM2CmdBuffer[1].eAppSwitch = kMC_App_Off;
	EventTableAddItem(Slave_Ready, SlaveReady, Event_NULL);
	EventTableAddItem(Slave_SendMotorStatus, MC_GetMotorStatus_fromSHM, Event_NULL);
	MCMGR_Core0Init_StartCore1(NULL);

}

#endif
