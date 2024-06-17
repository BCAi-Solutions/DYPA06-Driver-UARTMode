/**
 * ********************************************************************************
 * @file 		dypa06.h
 * @created-on 	Jun 5, 2024
 * @author 		Joshua Tucker
 * @company 	BCA Engineers Pty Ltd
 * @breif		DYP-A06-v1.1  header driver file
 * ********************************************************************************
 * @attention
 *
 *  Driver for interaction with the DFRobot DYP-A06-v1.1 Ultrasonic Distance Sensor
 *  This driver is designed for use with a polled UART BSP
 *  Current support for low power serial mode only.
 *
 * ********************************************************************************
 */

#ifndef DRIVERS_BSP_DYPA06_DYPA06_H_
#define DRIVERS_BSP_DYPA06_DYPA06_H_

/**
 * Driver Includes
 */

#include "platform.h"
//#include "stm32l0xx.h"
#include <stdint.h>


/**
 * Driver Typedefs
 */

typedef int32_t (*DYPA06_Init_Func)(void);
typedef int32_t (*DYPA06_DeInit_Func)(void);
typedef int32_t (*DYPA06_Send_Func)(uint8_t *, uint16_t);
typedef int32_t (*DYPA06_Recv_Func)(uint8_t *, uint16_t);

typedef struct
{
  DYPA06_Init_Func			Init;
  DYPA06_DeInit_Func		DeInit;
  DYPA06_Send_Func			Send;
  DYPA06_Recv_Func			Recv;
  GPIO_TypeDef*				EnablePort;
  uint16_t					EnablePin;
} DYPA06_IO_t;

typedef struct
{
  DYPA06_IO_t				IO;
  uint8_t					is_initialised;
  uint8_t					dist_is_enabled;
  uint8_t					dist_samples;
} DYPA06_Object_t;

typedef struct
{
  uint16_t					Distance;
  uint8_t					DistMaxSmpls;
} DYPA06_Capabilities_t;

typedef struct
{
  int32_t (*Init)(DYPA06_Object_t *);
  int32_t (*DeInit)(DYPA06_Object_t *);
  int32_t (*GetCapabilities)(DYPA06_Object_t *, DYPA06_Capabilities_t *);
} DYPA06_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(DYPA06_Object_t *);
  int32_t (*Disable)(DYPA06_Object_t *);
  int32_t (*GetDistance)(DYPA06_Object_t *, uint16_t *);
  int32_t (*GetSamples)(DYPA06_Object_t *, uint8_t *);
  int32_t (*SetSamples)(DYPA06_Object_t *, uint8_t);
} DYPA06_DIST_Drv_t;


/**
 * Driver Exported Constants
 */

#define DYPA06_OK					 0
#define DYPA06_ERROR				-1
#define DYPA06_ERROR_BUSY			-2
#define DYPA06_ERROR_TIMEOUT		-3
#define DYPA06_ERROR_PERIHPERAL		-4
#define DYPA06_ERROR_INVALID_PARAM	-5
#define DYPA06_ERROR_CRC			-6
#define DYPA06_ERROR_BUS			-8


/**
 * Driver Exported Functions
 */

int32_t DYPA06_RegisterBusIO(DYPA06_Object_t *pObj, DYPA06_IO_t *pIO);
int32_t DYPA06_Init(DYPA06_Object_t *pObj);
int32_t DYPA06_DeInit(DYPA06_Object_t *pObj);
int32_t DYPA06_GetCapabilities(DYPA06_Object_t *pObj, DYPA06_Capabilities_t *Capabilities);
int32_t DYPA06_Get_Init_Status(DYPA06_Object_t *pObj, uint8_t *Status);

int32_t DYPA06_DIST_Enable(DYPA06_Object_t *pObj);
int32_t DYPA06_DIST_Disable(DYPA06_Object_t *pObj);
int32_t DYPA06_DIST_GetDistance(DYPA06_Object_t *pObj, uint16_t *Value);
int32_t DYPA06_DIST_GetSamples(DYPA06_Object_t *pObj, uint8_t *Value);
int32_t DYPA06_DIST_SetSamples(DYPA06_Object_t *pObj, uint8_t Value);


/**
 * Driver Exported Variables
 */

extern DYPA06_CommonDrv_t DYPA06_COMMON_Driver;
extern DYPA06_DIST_Drv_t DYPA06_DIST_Driver;


#endif /* DRIVERS_BSP_DYPA06_DYPA06_H_ */
