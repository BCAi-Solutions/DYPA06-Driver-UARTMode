/**
 * ********************************************************************************
 * @file 		dypa06.h
 * @created-on 	June 5, 2024
 * @author 		Joshua Tucker
 * @company 	BCA Engineers PTY LTD
 * @brief		DYP-A06-v1.1  header driver file
 * ********************************************************************************
 */

/**
 * Driver Private Includes
 */

#include "dypa06.h"


/**
 * Driver Exported Variables
 */

DYPA06_CommonDrv_t DYPA06_COMMON_Driver =
{
  DYPA06_Init,
  DYPA06_DeInit,
  DYPA06_GetCapabilities
};

DYPA06_DIST_Drv_t DYPA06_DIST_Driver =
{
  DYPA06_DIST_Enable,
  DYPA06_DIST_Disable,
  DYPA06_DIST_GetDistance,
  DYPA06_DIST_GetSamples,
  DYPA06_DIST_SetSamples
};


/**
 * Driver Private Defines
 */

/* UART Triger (request) Command */
#define DYPA06_REQ_CMD 			0x01
/* Trigger (request) Command processing delay (mS) */
#define DYPA06_REQ_DLY 			50
/* Delay Between Multi Sampling Polls (mS) */
#define DYPA06_POLL_DLY			5
/* Default Number of Distance samples */
#define DEFAULT_DIST_SAMPLES	10
/* Minimum valid distance reading */
#define DYPA06_DISTANCE_MIN		250
/* Maximum invalid data polls per query */
#define MAX_VAL_ERRORS			10
/* Maximum bus failures per query */
#define MAX_BUS_ERRORS			3

/**
 * Driver Private Typedef
 */

typedef struct
{
  uint8_t 		Stx;
  uint8_t 		Distance_h;
  uint8_t 		Distance_l;
  uint8_t		Sum;
} DYPA06_Distance_Data;


/**
 * Driver Private Function Prototypes
 */

static int32_t DYPA06_Initialise(DYPA06_Object_t *pObj);
int32_t dypa06_power_on_set(DYPA06_IO_t *io, GPIO_PinState val);
int32_t dypa06_poll_distance(DYPA06_Object_t *pObj, uint16_t *val);
void dypa06_insertion_sort_16(uint16_t *arr, uint8_t n);


/**
 * Driver Private Variables
 */

uint8_t dypa06_req_cmd = DYPA06_REQ_CMD;


/**
 * Driver External Functions
 */

/**
 * @brief Register Component Bus IO operations
 * @param pObj the device pObj
 * @retval DYPA06_OK on success, otherwise device error code
 */
int32_t DYPA06_RegisterBusIO(DYPA06_Object_t *pObj, DYPA06_IO_t *pIO)
{
  int32_t ret;

  if (pObj == NULL)
	return DYPA06_ERROR;

  pObj->IO.Init			= pIO->Init;
  pObj->IO.DeInit		= pIO->DeInit;
  pObj->IO.Send			= pIO->Send;
  pObj->IO.Recv			= pIO->Recv;
  pObj->IO.EnablePort	= pIO->EnablePort;
  pObj->IO.EnablePin	= pIO->EnablePin;

  if (pObj->IO.Init != NULL)
  {
	ret = pObj->IO.Init();
  }
  else
  {
	ret = DYPA06_ERROR;
  }

  return ret;
}

/**
 * @brief  Initialize the DYPA06 sensor
 * @param  pObj the device pObj
 * @retval DYPA06_OK in case of success, an error code otherwise
 */
int32_t DYPA06_Init(DYPA06_Object_t *pObj)
{
  if (pObj->is_initialised == 0U)
  {
	if (DYPA06_Initialise(pObj) != DYPA06_OK)
	{
	  return DYPA06_ERROR;
	}
  }

  pObj->dist_samples = DEFAULT_DIST_SAMPLES;
  pObj->is_initialised = 1;

  return DYPA06_OK;
}

/**
 * @brief  Deinitialize the DYPA06 sensor
 * @param  pObj the device pObj
 * @retval DYPA06_OK in case of success, an error code otherwise
 */
int32_t DYPA06_DeInit(DYPA06_Object_t *pObj)
{
  if (pObj->is_initialised == 1U)
  {
    if (DYPA06_DIST_Disable(pObj) != DYPA06_OK)
    {
      return DYPA06_ERROR;
    }

    /* TODO: (Write DeInit function for Bus and GPIO) */

  }

  pObj->is_initialised = 0;

  return DYPA06_OK;
}

/**
 * @brief  Get DYPA06 sensor capabilities
 * @param  pObj Component object pointer
 * @param  Capabilities pointer to DYPA06 sensor capabilities
 * @retval DYPA06_OK in case of success, an error code otherwise
 */
int32_t DYPA06_GetCapabilities(DYPA06_Object_t *pObj, DYPA06_Capabilities_t *Capabilities)
{
  /* Prevent unused argument(s) compilation warning */
  (void)(pObj);

  Capabilities->Distance		= 0;
  Capabilities->DistMaxSmpls	= 255;

  return DYPA06_OK;
}

/**
 * @brief  Get the HTS221 initialization status
 * @param  pObj the device pObj
 * @param  Status 1 if initialized, 0 otherwise
 * @retval 0 in case of success, an error code otherwise
 */
int32_t DYPA06_Get_Init_Status(DYPA06_Object_t *pObj, uint8_t *Status)
{
  if (pObj == NULL)
  {
    return DYPA06_ERROR;
  }

  *Status = pObj->is_initialised;

  return DYPA06_OK;
}

/**
 * @brief  Enable the DYPA06 humidity sensor
 * @param  pObj the device pObj
 * @retval DYPA06_OK in case of success, an error code otherwise
 */
int32_t DYPA06_DIST_Enable(DYPA06_Object_t *pObj)
{
  /* Check if the component is already enabled */
  if (pObj->dist_is_enabled == 1U)
  {
    return DYPA06_OK;
  }

  /* Power on the component. */
  if (dypa06_power_on_set(&(pObj->IO), SET) != DYPA06_OK)
  {
    return DYPA06_ERROR;
  }

  pObj->dist_is_enabled = 1;

  return DYPA06_OK;
}

/**
 * @brief  Disable the DYPA06 humidity sensor
 * @param  pObj the device pObj
 * @retval DYPA06_OK in case of success, an error code otherwise
 */
int32_t DYPA06_DIST_Disable(DYPA06_Object_t *pObj)
{
  /* Check if the component is already disabled */
  if (pObj->dist_is_enabled == RESET)
  {
    return DYPA06_OK;
  }

  /* Power off the component. */
  if (dypa06_power_on_set(&(pObj->IO), SET) != DYPA06_OK)
  {
    return DYPA06_ERROR;
  }

  pObj->dist_is_enabled = 0;

  return DYPA06_OK;
}

/**
 * @brief  Get the DYPA06 Distance sensor sample count
 * @param  pObj the device pObj
 * @param  Value pointer where the sample count is written
 * @retval DYPA06_OK in case of success, an error code otherwise
 */
int32_t DYPA06_DIST_GetSamples(DYPA06_Object_t *pObj, uint8_t *Value)
{
  /* TODO: (Implement multisampling) */
  return DYPA06_OK;
}

/**
 * @brief  Set the DYPA06 Distance sensor sample count
 * @param  pObj the device pObj
 * @param  Value the sample count to be set
 * @retval DYPA06_OK in case of success, an error code otherwise
 */
int32_t DYPA06_DIST_SetSamples(DYPA06_Object_t *pObj, uint8_t Value)
{
  /* TODO: (Implement multisampling) */
  return DYPA06_OK;
}

/**
 * @brief  Get the DYPA06 distance value
 * @param  pObj the device pObj
 * @param  Value pointer where the distance value is written
 * @retval DYPA06_OK in case of success, an error code otherwise
 */
int32_t DYPA06_DIST_GetDistance(DYPA06_Object_t *pObj, uint16_t *Value)
{
  int32_t ret = DYPA06_OK;

  /* Pass-through Single Poll mode */
  if ( pObj->dist_samples == 1 )
  {
	return dypa06_poll_distance(pObj, Value);
  }

  /* Collect Samples */
  uint16_t sample;
  const uint8_t nSamples = pObj->dist_samples;
  uint16_t *distData = (uint16_t*) malloc( sizeof(uint16_t) * nSamples );
  int i = 0, countBusErr = 0, countValErr = 0;
  while( i < nSamples )
  {
	/* Delay to allow sensor refresh */
	HAL_Delay(DYPA06_REQ_DLY);

	/* Poll for Sensor Data */
	if ( dypa06_poll_distance(pObj, &sample) == DYPA06_OK )
	{
	  /* Check sample data validity */
	  if ( sample >= DYPA06_DISTANCE_MIN )
	  {
	    /* Add valid sample to Set */
		distData[i++] = sample;
	    continue;
	  }

	  /* Invalid Data */
	  if ( ++countValErr >= MAX_VAL_ERRORS )
	    return DYPA06_ERROR_PERIHPERAL;

	  continue;
	}

	/* Bus Failure */
	if ( ++countBusErr >= MAX_BUS_ERRORS )
	  return DYPA06_ERROR_BUS;

	continue;
  }

  /* Sort Sample Results */
  dypa06_insertion_sort_16(distData, pObj->dist_samples);

  /* Get Median Value */
  uint8_t median_index = ( nSamples / 2 );
  if ( ( nSamples % 2 ) == 0 )
  {
	*Value = distData[median_index];
  }
  else
  {
	*Value = ( ( distData[median_index] + distData[median_index + 1] ) / 2 );
  }

  return ret;
}

/**
 * @brief  Initialize the DYPA06 sensor
 * @param  pObj the device pObj
 * @retval DYPA06_OK in case of success, an error code otherwise
 */
static int32_t DYPA06_Initialise(DYPA06_Object_t *pObj)
{
  /* Power off Component */
  if (dypa06_power_on_set(&(pObj->IO), RESET) != DYPA06_OK)
  {
	return DYPA06_ERROR;
  }

  /* Set Default number of samples */
  if (DYPA06_DIST_SetSamples(pObj, 1U) != DYPA06_OK)
  {
	return DYPA06_ERROR;
  }

  return DYPA06_OK;
}

/**
  * @brief  Switch device on/off.[set]
  *
  * @param  io     	read / write interface definitions
  * @param  val     change the state of the chip enable pin
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t dypa06_power_on_set(DYPA06_IO_t *io, GPIO_PinState val)
{
  GPIO_PinState state;
  int32_t ret = DYPA06_OK;

  state = HAL_GPIO_ReadPin(io->EnablePort, io->EnablePin);

  if(state != val){
    state = val;
    HAL_GPIO_WritePin(io->EnablePort, io->EnablePin, state);
  }

  return ret;
}

/**
 * @brief Poll single distance measurement from DYPA06
 * @param pObj 	DYPA06 object
 * @param val 	Distance return value
 * @retval		DYPA06 Status (0 no error)
 */
int32_t dypa06_poll_distance(DYPA06_Object_t *pObj, uint16_t *val)
{
  int32_t ret;

  /* Validate Function Parameters */
  if ( pObj == NULL || val == NULL )
	return DYPA06_ERROR_INVALID_PARAM;

  DYPA06_Distance_Data data = {0};

  /* Initiate Trigger Request */
  ret = pObj->IO.Send(&dypa06_req_cmd, 1U);
  if ( ret < 1U )
	return DYPA06_ERROR_PERIHPERAL;

  /* Delay to allow sensor processing */
  HAL_Delay(DYPA06_REQ_DLY);

  /* Begin Polling */
  ret = pObj->IO.Recv(&data.Stx, 1U);
  if ( ret != 0U )
	return DYPA06_ERROR_TIMEOUT;

  /* Validate Start Byte */
  if ( data.Stx != 0xFF )
	return DYPA06_ERROR_PERIHPERAL;

  /* Poll Remaining Distance Data */
  ret = pObj->IO.Recv((uint8_t *) &data.Distance_h, 3U);
  if (ret != 0U )
	return DYPA06_ERROR_TIMEOUT;
  /* Validate Sensor Data */
  if ( ( ( data.Stx + data.Distance_h + data.Distance_l ) & 0xFF ) != data.Sum )
	return DYPA06_ERROR_CRC;

  /* Copy Distance value to Export pointer */
  *val = ( ( data.Distance_h << 8 ) | data.Distance_l );

  return DYPA06_OK;
}

/**
 * @brief	Simple Insertion Sort for 16 bit unsigned integers
 * @param arr	Array to be sorted
 * @param n		Number of elements in array (Min: 2, Max: 255)
 */
void dypa06_insertion_sort_16(uint16_t *arr, uint8_t n)
{
  /* No Elements to Sort */
  if ( n < 2 || arr == NULL )
    return;

  int key, j;
  for ( int i = 1; i < n; i++ )
  {
	key = arr[i];
	j = i - 1;
	while ( j >= 0 && arr[j] > key)
	{
	  arr[j + 1] = arr[j];
	  j -= 1;
	}
	arr[j + 1] = key;
  }
}

