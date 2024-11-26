/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef API_MOTORCONTROL_H_
#define API_MOTORCONTROL_H_

#include "fsl_common.h"

#if (__CORTEX_M == 7)
#include "state_machine.h"
#include "sm_ref_sol_comm.h"
#endif

#if (__CORTEX_M == 33)
#define RAM_FUNC_CRITICAL __attribute__((section(".ramfunc.$SRAM_ITC_cm33")))
#else
#define RAM_FUNC_CRITICAL
#endif


/*******************************************************************************
 * Definitions => Enumerations
 ******************************************************************************/
/*!
 * @brief Current motor control application status.
 */
typedef enum _mc_app_switch
{
    kMC_App_Off           = 0U, /*!< Motor control switched off */
    kMC_App_On            = 1U, /*!< Manual control of the position/speed enabled */
	kMC_App_Freeze        = 2U, /*!< All control of the position/speed disabled */
	kMC_App_FreezeAndStop = 3U, /*!< All control of the position/speed disabled and motors turned off */
} mc_app_switch_t;

typedef enum _mc_fault
{
    kMC_NoFaultMC              = 0x00000000U, /*!< No fault occured or a previous fault reported from the MC loop was resolved . */
	kMC_NoFaultBS			   = 0x00000004U, /*!< No fault occured or a previous fault reported from the Board service task was resolved . */
    kMC_OverCurrent            = 0x00000008U, /*!< Over-current event detected. */
    kMC_UnderDcBusVoltage      = 0x00000010U, /*!< DC bus under-voltage event detected */
    kMC_OverDcBusVoltage       = 0x00000020U, /*!< DC bus over-voltage event detected */
    kMC_OverLoad               = 0x00000040U, /*!< Motor overloaded */
    kMC_OverSpeed              = 0x00000080U, /*!< Motor ran into overspeed */
    kMC_RotorBlocked           = 0x00000100U, /*!< Rotor shaft is blocked */
    kMC_PsbOverTemperature1    = 0x00000200U, /*!< Over-temperature on power stage board (sensor 1) */
    kMC_PsbOverTemperature2    = 0x00000400U, /*!< Over-temperature on power stage board (sensor 2) */
	kMC_GD3000_OverTemperature = 0x00000800U, /*!< GD3000 over temperature */
	kMC_GD3000_Desaturation    = 0x00001000U, /*!< GD3000 desaturation detected */
	kMC_GD3000_LowVLS          = 0x00002000U, /*!< GD3000 VLS under voltage */
	kMC_GD3000_OverCurrent     = 0x00004000U, /*!< GD3000 over current */
	kMC_GD3000_PhaseError      = 0x00008000U, /*!< GD3000 phase error */
	kMC_GD3000_Reset           = 0x00010000U, /*!< GD3000 is in reset state */
} mc_fault_t;
 /*!
 * @brief List the available motors
 */
typedef enum _mc_motor_id
{
    kMC_Motor1 = 0U, /*!< First motor */
    kMC_Motor2 = 1U, /*!< Second motor */
    kMC_Motor3 = 2U, /*!< Third motor */
    kMC_Motor4 = 3U, /*!< Fourth motor */
} mc_motor_id_t;

/*!
 * @brief Motor control state.
 */
typedef enum _mc_state
{
    kMC_Fault = 0U, /*!< Any of the motor control faults has happened */
    kMC_Init  = 1U, /*!< Initialization phase of motor control (ramp up) */
    kMC_Stop  = 2U, /*!< Motors stopped */
    kMC_Run   = 3U, /*!< Motors run in one of the control methods defined by mc_method_selection_t */
} mc_state_t;


/*!
 * @brief Lists available motor control method types
 */
typedef enum _mc_method_selection
{
    kMC_ScalarControl       = 0U, /*!< Scalar motor control (V/f) */
    kMC_FOC_SpeedControl    = 1U, /*!< Direct speed setting (based on Field-Oriented-Control) */
    kMC_FOC_PositionControl = 2U, /*!< Direct position setting (based on Field-Oriented-Control) */
} mc_method_selection_t;

#if (__CORTEX_M == 33)
/*! @brief Application state identification enum */
typedef enum sm_app_state
{
    kSM_AppFault = 0,
    kSM_AppInit = 1,
    kSM_AppStop = 2,
    kSM_AppRun = 3,
} mc_sm_app_state_t;

/* The GMCLIB_2COOR_DQ_T_FLT structure type corresponds to the two-phase rotating
coordinate system based on the D and Q orthogonal components. */
typedef struct
{
    float fltD;
    float fltQ;
} mc_GMCLIB_2COOR_DQ_T_FLT;


typedef struct mcs_mcat_ctrl_a1
{
	mc_GMCLIB_2COOR_DQ_T_FLT sIDQReqMCAT;          /* required dq current entered from MCAT tool */
	mc_GMCLIB_2COOR_DQ_T_FLT sUDQReqMCAT;          /* required dq voltage entered from MCAT tool */
    uint16_t ui16PospeSensor;                   /* position sensor type information */
} mc_mcat_ctrl_t;





#endif

/*! @brief Control modes of the motor */
typedef enum mc_mcs_ctrl_mode
{
	mc_kControlMode_Scalar = 0,
	mc_kControlMode_VoltageFOC = 1,
	mc_kControlMode_CurrentFOC = 2,
	mc_kControlMode_SpeedFOC = 3,
	mc_kControlMode_PositionFOC = 4,
	mc_kControlMode_VoltageOpenloop = 5,
	mc_kControlMode_CurrentOpenloop = 6
} mc_mcs_ctrl_mode_t;

/*! @brief States of machine enumeration */
typedef enum mc_run_substate
{
    mc_kRunState_Calib = 0,
	mc_kRunState_Ready = 1,
	mc_kRunState_Align = 2,
	mc_kRunState_Startup = 3,
	mc_kRunState_Spin = 4,
	mc_kRunState_Freewheel = 5,
	mc_kRunState_Measure = 6,
} mc_run_substate_t; /* Run sub-states */

/*******************************************************************************
 * Definitions => Structures
 ******************************************************************************/
/*!
 * @brief A motor position that can be accessed as a raw Q15.Q16 value or as individual components.
 */
typedef union _mc_motor_position
{
    struct {
        uint16_t ui16RotorPosition; /*!< Value represents one revolution angle fragment 360degree / 65536 = 0.0055 degree */
        int16_t  i16NumOfTurns;     /*!< Value represents number of revolutions */
    } sPosValue;
    int32_t i32Raw;                 /*!< Actual motor position; signed 32-bit (fractional Q15.16 value) */
} mc_motor_position_t;


/*!
 * @brief Holds the motor command parameters for the motor indicated by eMotorId : mc_motor_id_t.
 */

typedef struct _mc_motor_command
{
    mc_motor_id_t         eMotorId;          /*!< Motor to which this command applies */
    mc_app_switch_t       eAppSwitch;        /*!< Defines whether motor control is enabled */
    mc_method_selection_t eControlMethodSel; /*!< Control method to be used */
    uint16_t ui16MUPospeSensor;                   /* position sensor type information */

    union
	{

        float             fltSpeed;                         /*!< [RPM] Speed parameter for FOC speed control method */

        struct
		{
            float               fltScalarControlVHzGain;   /*!< [V/Hz] Voltage/Frequency parameter for scalar control method */
            float               fltScalarControlFrequency; /*!< [Hz] Frequency parameter for scalar control method */

        } sScalarParam;
        struct
		{
            mc_motor_position_t uPosition;                 /*!< Position parameter for FOC position control method */
            bool                bIsRandomPosition;         /*!< Indicates that the position is not on a trajectory and may need filtering */
        } sPosParam;
    } uSpeed_pos;

} mc_motor_command_t;

/*!
 * @brief Holds the parameters of the motor status that are written by the Fast Motor Control Loop.
 */
typedef struct _mc_motor_status_fast
{
    mc_state_t eMotorState;  /*!< Actual motor control state */
    mc_fault_t eFaultStatus; /*!< Actual motor control fault */
    float      fltIa;        /*!< Actual phase A current */
    float      fltIb;        /*!< Actual phase B current */
    float      fltIc;        /*!< Actual phase C current */
    float      fltValpha;    /*!< Actual alpha component voltage */
    float      fltVbeta;     /*!< Actual beta component voltage */
    float      fltVDcBus;    /*!< Actual DC bus voltage */
} mc_motor_status_fast_t;

/*!
 * @brief Holds the parameters of the motor status that are written by the Slow Motor Control Loop.
 */
typedef struct _mc_motor_status_slow
{
    mc_app_switch_t     eAppSwitch; /*!< Actual motor control application status (on / off) */
    float               fltSpeed;   /*!< Actual motor speed */
    mc_motor_position_t uPosition;  /*!< Actual motor position */
} mc_motor_status_slow_t;




/*!
 * @brief Holds the motor status parameters of the motor indicated by motorId.
 */
typedef struct _mc_motor_status
{
    mc_motor_status_fast_t sFast;    /*!< Part of the status that is written by the Fast Motor Control Loop */
    mc_motor_status_slow_t sSlow;    /*!< Part of the status that is written by the Slow Motor Control Loop */
    mc_motor_id_t          eMotorId; /*!< Indicates which motor this status belongs to */

    mc_run_substate_t g_eM1StateRun;
    mc_mcs_ctrl_mode_t eControlMethodSel; /*!< Control method to be used */

} mc_motor_status_t;

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_TP151_GPIO                                         RGPIO6   /*!< GPIO peripheral base pointer */
#define BOARD_INITPINS_TP151_GPIO_PIN                                        15U   /*!< GPIO pin number */
#define BOARD_INITPINS_TP151_GPIO_PIN_MASK                           (1U << 15U)   /*!< GPIO pin mask */

#define SHARED_MEMORY_IN_CORES __attribute__((section(".bss.$SHMEM_REGION")))

#if (__CORTEX_M == 7)
extern mc_motor_command_t g_sM1Cmd,g_sM2Cmd;
extern void MC_SetMotorStatusFromISR(mc_motor_status_t *ptrM1, mc_motor_status_t *ptrM2);
extern void installCoreCommTable(void);
#endif

#if (__CORTEX_M == 33)
extern mc_motor_status_t  g_sM1Status, g_sM2Status;
extern volatile uint16_t ui16SlaveReady;
extern void MC_SetMotorCommandFromISR(void);
extern void MC_SetMotor1Command(mc_motor_command_t *ptr);
extern void MC_SetMotor2Command(mc_motor_command_t *ptr);
extern void installCoreCommTable(void);
extern bool MC_ReadReadWriteFlag(void);
extern void  Ecat_Read(ECAT_Type *ethercat,uint8_t* pdata,uint32_t Add, uint32_t L);
extern void  Ecat_Write(ECAT_Type *ethercat,uint8_t* pdata,uint32_t Add, uint32_t L);
#endif

#endif /* API_MOTORCONTROL_H_ */
