/**
 *
 * @copyright &copy; 2010 - 2025, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * All rights reserved.
 *
 * Confidential or no approval for publication
 *
 */

/**
 * @file    rtt_debug.h
 * @author  foxBMS Team
 * @date    2026-01-24 (date of creation)
 * @updated 2026-01-24 (date of last update)
 * @version v1.9.0
 * @ingroup DRIVERS
 * @prefix  RTT
 *
 * @brief   Header for the driver of the RTT AFE driver
 * @details Data is received via RTT
 */

#ifndef FOXBMS__RTT_DEBUG_H_
#define FOXBMS__RTT_DEBUG_H_

/*========== Includes =======================================================*/

#include "database_cfg.h"

#include "fstd_types.h"

#include <stdbool.h>
#include <stdint.h>

/*========== Macros and Definitions =========================================*/

/** States of the state machine */
typedef enum {
    RTT_FSM_STATE_DUMMY,          /*!< dummy state - always the first state */
    RTT_FSM_STATE_HAS_NEVER_RUN,  /*!< never run state - always the second state */
    RTT_FSM_STATE_UNINITIALIZED,  /*!< uninitialized state */
    RTT_FSM_STATE_INITIALIZATION, /*!< initializing the state machine */
    RTT_FSM_STATE_RUNNING,        /*!< operational mode of the state machine */
    RTT_FSM_STATE_ERROR,          /*!< state for error processing  */
} RTT_FSM_STATES_e;

/** Substates of the state machine */
typedef enum {
    RTT_FSM_SUBSTATE_DUMMY,                                     /*!< dummy state - always the first substate */
    RTT_FSM_SUBSTATE_ENTRY,                                     /*!< entry state - always the second substate */
    RTT_FSM_SUBSTATE_INITIALIZATION_FINISH_FIRST_MEASUREMENT,   /*!< finish the first RTT measurement */
    RTT_FSM_SUBSTATE_INITIALIZATION_FIRST_MEASUREMENT_FINISHED, /*!< cleanup substate after the first RTT measurement */
    RTT_FSM_SUBSTATE_INITIALIZATION_EXIT,                       /*!< last initialization substate */
    RTT_FSM_SUBSTATE_RUNNING_READ_INPUT,                        /*!< state to read RTT input */
} RTT_FSM_SUBSTATES_e;

/** This struct contains pointer to used data buffers */
typedef struct {
    DATA_BLOCK_CELL_VOLTAGE_s *cellVoltage;             /*!< cell voltage */
    DATA_BLOCK_CELL_TEMPERATURE_s *cellTemperature;     /*!< cell temperature */
    DATA_BLOCK_BALANCING_FEEDBACK_s *balancingFeedback; /*!< balancing feedback */
    DATA_BLOCK_BALANCING_CONTROL_s *balancingControl;   /*!< balancing control */
    DATA_BLOCK_SLAVE_CONTROL_s *slaveControl;           /*!< slave control */
    DATA_BLOCK_ALL_GPIO_VOLTAGES_s *allGpioVoltages;    /*!< voltage of the slaves' GPIOs */
    DATA_BLOCK_OPEN_WIRE_s *openWire;                   /*!< open wire status */
    DATA_BLOCK_CURRENT_SENSOR_s *currentSensor;         /*!< current sensor measurement */
} RTT_DATABASE_ENTRIES_s;

/** This struct describes the state of the monitoring instance */
typedef struct {
    uint16_t timer;                       /*!< timer of the state */
    uint8_t triggerEntry;                 /*!< trigger entry of the state */
    RTT_FSM_STATES_e nextState;           /*!< next state of the FSM */
    RTT_FSM_STATES_e currentState;        /*!< current state of the FSM */
    RTT_FSM_STATES_e previousState;       /*!< previous state of the FSM */
    RTT_FSM_SUBSTATES_e nextSubstate;     /*!< next substate of the FSM */
    RTT_FSM_SUBSTATES_e currentSubstate;  /*!< current substate of the FSM */
    RTT_FSM_SUBSTATES_e previousSubstate; /*!< previous substate of the FSM */
    bool firstMeasurementFinished;        /*!< indicator if the fist measurement has been successful */
    RTT_DATABASE_ENTRIES_s data;          /*!< contains pointers to the local data buffer */
} RTT_STATE_s;

/*========== Extern Constant and Variable Declarations ======================*/

/** state of the RTT state machine */
extern RTT_STATE_s rtt_state;

/*========== Extern Function Prototypes =====================================*/

/** @brief  initialize driver */
extern STD_RETURN_TYPE_e RTT_Initialize(void);

/**
 * @brief   return whether the first measurement cycle is finished
 * @param   pState current state of the RTT driver
 * @return  true if the first measurement cycle was successfully finished,
 *          false otherwise
 */
extern bool RTT_IsFirstMeasurementCycleFinished(RTT_STATE_s *pState);

/**
 * @brief   Trigger function for the driver, called to advance the
 *          state machine
 * @param   pState current state of the RTT driver
 * @return  returns always #STD_OK
 */
extern STD_RETURN_TYPE_e RTT_TriggerAfe(RTT_STATE_s *pState);

/*========== Externalized Static Functions Prototypes (Unit Test) ===========*/
#ifdef UNITY_UNIT_TEST
#endif

#endif /* FOXBMS__RTT_DEBUG_H_ */
