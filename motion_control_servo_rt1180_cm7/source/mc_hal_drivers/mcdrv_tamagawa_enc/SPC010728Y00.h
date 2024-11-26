/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SPC010728Y00_H_
#define SPC010728Y00_H_

#include <stdint.h>
#include "mc_common.h"
#include "fsl_device_registers.h"

#define BIT0_MASK 0x0001
#define BIT1_MASK 0x0002
#define BIT2_MASK 0x0004
#define BIT3_MASK 0x0008
#define BIT4_MASK 0x0010
#define BIT5_MASK 0x0020
#define BIT6_MASK 0x0040
#define BIT7_MASK 0x0080
#define BIT8_MASK 0x0100
#define BIT9_MASK 0x0200
#define BIT10_MASK 0x0400
#define BIT11_MASK 0x0800
#define BIT12_MASK 0x1000
#define BIT13_MASK 0x2000
#define BIT14_MASK 0x4000
#define BIT15_MASK 0x8000

#define INIT_REQUEST 0x28
#define SEND_REQEUST 0x55
#define REQUEST_ACKNOWLEDGED 0xAA

/*  (CF) Control field definitions */
#define READ_DATA_ANGLE       0x02
#define READ_DATA_TURNS       0x8A
#define READ_DATA_ID          0x92
#define READ_DATA_ALL         0x1A
#define WRITE_EEPROM          0x32
#define READ_EEPROM           0xEA
#define RESET_ALL_ERROR       0xBA
#define RESET_ONE_REVOLUTION  0xC2
#define RESET_ALL_ERROR_TURNS 0x62

/*  (ALMC) Encoder error bit field definitions */
#define OVER_SPEED                   0x01
#define FULL_ABSOLUTE_ACCURACY_ERROR 0x02
#define COUNTING_ERROR               0x04
#define COUNTER_OVERFLOW             0x08
#define OVER_HEAT                    0x10
#define MULTI_TURN_ERROR             0x20
#define BATTERY_ERROR                0x40
#define BATTERY_ALARM                0x80

/* CRC error flag which is issued when received frame crc doesn't match */
#define CRC_ERROR                    0x100

/* Communication alarm flags from (SF) Status field */
#define COMM_PARITY_ERROR            0x200
#define COMM_DELIMITER_ERROR         0x400

/* EEPROM busy flag from received (ADF) Address field */
#define EEPROM_BUSY                  0x800

/* Communication cable disconnected error flag */
#define COMM_CABLE_DISCONNECTED      0x1000

/**********************************************************/

typedef union
{
	uint8_t ui8ErrorStatus;
	struct
	{
		uint8_t OverSpeed : 1;
		uint8_t FullAbsoluteStatus : 1;
		uint8_t CountingError : 1;
		uint8_t CounterOverflow : 1;
		uint8_t OverHeat : 1;
		uint8_t MultiTurnError : 1;
		uint8_t BatteryError : 1;
		uint8_t BatteryAlarm : 1;

	} Bits;
} tama_error_field_t;

typedef union
{
	uint8_t ui8AddressField;
	struct
	{
		uint8_t Address : 7;  // 7bit address, address 127 represent page setting which includes total 6 pages, each page has address 0~126
		uint8_t BusyFlag : 1; // Flag that indicates eeprom is busy

	} Bits;
} tama_eeprom_addr_field_t;

typedef union
{
	uint8_t ui8AbsStatus;
	struct
	{
		uint8_t Information : 4;		// fixed 0
		uint8_t CountingError : 1;		// Counting error
		uint8_t OtherError : 1;			// logic OR of Over-heat, multi-turn error,battery error and battery alarm
		uint8_t CommParityError : 1;	// Parity error in request frame
		uint8_t CommDelimiterError : 1; // Delimiter error in request frame

	} Bits;
	tama_eeprom_addr_field_t uEepromAddr;  // Eeprom address
} tama_status_field_t;

typedef struct
{
	uint8_t ui8ControlField;
	tama_eeprom_addr_field_t uEepromAddressField;
	uint8_t ui8EepromField;
	uint8_t ui8CrcField;

} tama_request_frame_t;

typedef union
{
	uint8_t ui8DataReceiveArray[11];
	struct
	{
		uint8_t ui8ControlField;
		tama_status_field_t uStatusField; // This field is Address field(ADF) in eeprom read/write response
		uint8_t ui8DataField0; // This field is eeprom data field (EDF) in eeprom read/write response
		uint8_t ui8DataField1; // This field is crc field in eeprom read/write response
		uint8_t ui8DataField2;
		uint8_t ui8DataField3;
		uint8_t ui8DataField4;
		uint8_t ui8DataField5;
		uint8_t ui8DataField6;
		uint8_t ui8DataField7;
		uint8_t ui8CrcField;
	}sReceivedData;
		
} tama_response_frame_t;


typedef struct
{
	
	uint32_t ui32RawPosCnt; // 24bit raw position counter data
	uint32_t ui32RawPosCnt_1;
	float    fltSpeedElecRadScale; // A scale to turn fractional speed to electrical radian speed
	int32_t  i32Q23SpeedCalConst;   // Q9.23
	frac32_t f32Speed;
	frac16_t f16Speed;
	float    fltSpeed; // Electrical radian speed
} tama_speed_t;

typedef struct abs_to
{
	frac16_t    f16PosErr; // Position error to the tracking observer
	AMCLIB_TRACK_OBSRV_T_FLT    sTO; // Tracking observer
	frac16_t    f16PosEstim;
	float_t     fltSpeedEstim;
}abs_to_t;

typedef struct tamagawa_abs_t
{
	/* Abs parameter */
	uint8_t ui8OneTurnBitNum;     // Indicate the position(angle) counter resolution of one turn, such as 17bits, or 23bits 
	uint8_t  ui8PolePair;

	/* Communication physical interface */
	uint8_t ui8DMAChannel;   // DMA4 is used for RT1180, this field specifies which channel is used to receive response data
	LPUART_Type *LPUART;     // The UART that is used for communication
	uint32_t ui32DADDR;      // DMA destination address, which is the start address of uResponseFrame.  

	/* Protocol related frames */
	tama_request_frame_t sRequestFrame;
	tama_response_frame_t uResponseFrame;
	void (*pResponseFrameLengthConfig)(struct tamagawa_abs_t *this, uint8_t ui8Val);
	void (*pSendCFRequest)(struct tamagawa_abs_t *this); // Send only Control Field(1 byte)
	void (*pSendEepromReadRequest)(struct tamagawa_abs_t *this); // Send CF, ADF, CRC (3 bytes)
	void (*pSendEepromWriteRequest)(struct tamagawa_abs_t *this); // Send CF, ADF, EDF, CRC(4 bytes)
	void (*pTransmitEnable)(void);
	void (*pReceiveEnable)(void);
	bool_t (*pGetPWMForceFlag)(void);
	void (*pClearPWMForceFlag)(void);
	/* Speed calculation related variables */
	tama_speed_t sSpeed;

	/* Rotor position&revolution related variables for FOC algorithm */
	uint32_t ui32InitCount;	   // Abs position(turn) counter value when the rotor initial position has been identified
	uint32_t ui32CurrentCount; // Abs position(turn) current counter value, which reflects current rotor position
	frac32_t f32PosMechInit;   // The mechanical rotor position corresponding to the initial Abs position counter value
	frac32_t f32PosMechOffset; // Rotor real mechanical position at the initial position(against A-axis) when the initial value of bPosAbsoluteFlag is FALSE
	                           // When the initial value of bPosAbsoluteFlag is TRUE, it represents the error between the position(Q1.31) from Abs position counter directly and the real rotor position (against A-axis). 
	frac32_t f32PosMech;	   // Rotor real mechanical position, Q1.31
	frac32_t f32PosMech_1;	   // Rotor real mechanical position, Q1.31
	frac32_t f32PosMechOffsetforAlign;	   // when run in the position mode,record the Offset
	frac32_t f32Q16DeltaPosMech;
	frac32_t f32PosElec;	   // Rotor real electrical position, Q1.31
	frac16_t f16PosElec;	   // Rotor real electrical position, Q1.15
	frac16_t f16PosElec_1;	   // Rotor real electrical position, Q1.15

	uint16_t ui16RevCnt;       // Abs revolution counter current value
	uint32_t ui32PosCurCnt;       // Abs revolution counter current value
	int32_t  i32Q16InitRev;	   // Revolution value when initial rotor position has been identified. Q16.16 format, integer part represents revolutions, fractional part represents the part which is "less than 1 revolution"
	int32_t  i32Q16Rev;		   // Current revolution value, Q16.16
	int32_t  i32Q16DeltaRev;   // Revolutions between current rotor position and the initial rotor position, Q16.16

	bool_t   bPosAbsoluteFlag;	   // A flag indicating whether rotor position is an absolute correct value

	/* Error Status */
	uint16_t ui16Status;

	/* Request acknowledge flag */
	uint8_t  ui8RequestAckFlag; // A flag to indicate whether a request has been acknowledged by the encoder. It has 3 states: INIT_REQUEST, SEND_REQEUST, REQUEST_ACKNOWLEDGED.

	uint8_t ui8IllTriggerCnt;

	abs_to_t    sSpeedEstim;    // A module to estimate speed by tracking observer

} mcdrv_tamagawa_abs_t;

/*!
 * @brief Get how many rounds the rotor has revolved with regard to its initial position:i32Q16InitRev, the result is stored in i32Q16DeltaRev, which is Q16.16 format.
		 It can represent -32768 ~ 32767.99974 revolutions.

 * @param this   Pointer to the speed calculate object
 *
 * @return boot_t true on success
 */
ALWAYS_INLINE inline void MCDRV_AbsGetRotorDeltaRev(mcdrv_tamagawa_abs_t *this)
{
	this->i32Q16DeltaRev = this->i32Q16Rev - this->i32Q16InitRev;
}

extern void MCDRV_AbsRequestData(mcdrv_tamagawa_abs_t *this);
extern void MCDRV_AbsRequestEepromRead(mcdrv_tamagawa_abs_t *this);
extern void MCDRV_AbsRequestEepromWrite(mcdrv_tamagawa_abs_t *this);
extern void MCDRV_AbsGetDataResponse(mcdrv_tamagawa_abs_t *this);
extern uint8_t MCDRV_AbsGetEepromResponse(mcdrv_tamagawa_abs_t *this);
extern void MCDRV_AbsSpeedCalculation(mcdrv_tamagawa_abs_t *this);
extern void MCDRV_AbsGetRotorInitPos(mcdrv_tamagawa_abs_t *this, frac32_t f32PosMechOffset);
extern void MCDRV_AbsGetRotorCurrentPos(mcdrv_tamagawa_abs_t *this);
extern void MCDRV_AbsGetRotorInitRev(mcdrv_tamagawa_abs_t *this);
extern void MCDRV_AbsGetRotorCurrentRev(mcdrv_tamagawa_abs_t *this); 
extern void MCDRV_TamagawaAbsInit(mcdrv_tamagawa_abs_t *this);
extern bool_t MCDRV_AbsToSpeedCalInit(mcdrv_tamagawa_abs_t *this);
extern void MCDRV_AbsToSpeedCalUpdate(mcdrv_tamagawa_abs_t *this);
#endif /* SPC010728Y00_H_ */
