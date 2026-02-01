/**
 *
 * @copyright &copy; 2010 - 2025, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * All rights reserved.
 *
 * Confidential or no approval for publication
 *
 */

/**
 * @file    rtt_debug.c
 * @author  foxBMS Team
 * @date    2026-01-24 (date of creation)
 * @updated 2026-01-24 (date of last update)
 * @version v1.9.0
 * @ingroup DRIVERS
 * @prefix  RTT
 *
 * @brief   Driver implementation for the RTT AFE
 * @details Reads voltage and temperature values via SEGGER RTT
 */

/*========== Includes =======================================================*/

#include "rtt_debug.h"

#include "battery_cell_cfg.h"
#include "battery_system_cfg.h"

#include "HL_gio.h"
#include "HL_het.h"

#include "database.h"
#include "diag.h"
#include "fstd_types.h"
#include "io.h"
#include "os.h"
#include "segger_rtt.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/*========== Macros and Definitions =========================================*/

#define LED_PORT (hetREG1)
/** Pin of HET1 that the Debug LED is connected to. */
#define LED_PIN (1u)

/**
 * state machine short time definition in #RTT_TriggerAfe calls
 * until next state is processed
 */
#define RTT_FSM_SHORT_TIME (1u)

/**
 * state machine medium time definition in #RTT_TriggerAfe calls
 * until next state/substate is processed
 */
#define RTT_FSM_MEDIUM_TIME (5u)

/**
 * state machine long time definition in #RTT_TriggerAfe calls
 * until next state/substate is processed
 */
#define RTT_FSM_LONG_TIME (10u)

/** buffer size for RTT input line */
#define RTT_INPUT_BUFFER_SIZE (512u)

/** Symbolic names to check for multiple calls of #RTT_TriggerAfe */
typedef enum {
    RTT_MULTIPLE_CALLS_NO,  /*!< no multiple calls, OK */
    RTT_MULTIPLE_CALLS_YES, /*!< multiple calls, not OK */
} RTT_CHECK_MULTIPLE_CALLS_e;

/*========== Static Constant and Variable Definitions =======================*/

/** local copies of database tables */
/**@{*/
static DATA_BLOCK_CELL_VOLTAGE_s rtt_cellVoltage             = {.header.uniqueId = DATA_BLOCK_ID_CELL_VOLTAGE_BASE};
static DATA_BLOCK_CELL_TEMPERATURE_s rtt_cellTemperature     = {.header.uniqueId = DATA_BLOCK_ID_CELL_TEMPERATURE_BASE};
static DATA_BLOCK_BALANCING_FEEDBACK_s rtt_balancingFeedback = {
    .header.uniqueId = DATA_BLOCK_ID_BALANCING_FEEDBACK_BASE};
static DATA_BLOCK_BALANCING_CONTROL_s rtt_balancingControl = {.header.uniqueId = DATA_BLOCK_ID_BALANCING_CONTROL};
static DATA_BLOCK_SLAVE_CONTROL_s rtt_slaveControl         = {.header.uniqueId = DATA_BLOCK_ID_SLAVE_CONTROL};
static DATA_BLOCK_ALL_GPIO_VOLTAGES_s rtt_allGpioVoltage   = {.header.uniqueId = DATA_BLOCK_ID_ALL_GPIO_VOLTAGES_BASE};
static DATA_BLOCK_OPEN_WIRE_s rtt_openWire                 = {.header.uniqueId = DATA_BLOCK_ID_OPEN_WIRE_BASE};
static DATA_BLOCK_CURRENT_SENSOR_s rtt_currentSensor       = {.header.uniqueId = DATA_BLOCK_ID_CURRENT_SENSOR};
/**@}*/

/*========== Extern Constant and Variable Definitions =======================*/

/** local instance of the driver-state */
RTT_STATE_s rtt_state = {
    .timer                    = 0,
    .triggerEntry             = 0,
    .nextState                = RTT_FSM_STATE_HAS_NEVER_RUN,
    .currentState             = RTT_FSM_STATE_HAS_NEVER_RUN,
    .previousState            = RTT_FSM_STATE_HAS_NEVER_RUN,
    .nextSubstate             = RTT_FSM_SUBSTATE_DUMMY,
    .currentSubstate          = RTT_FSM_SUBSTATE_DUMMY,
    .previousSubstate         = RTT_FSM_SUBSTATE_DUMMY,
    .firstMeasurementFinished = false,
    .data.allGpioVoltages     = &rtt_allGpioVoltage,
    .data.balancingControl    = &rtt_balancingControl,
    .data.balancingFeedback   = &rtt_balancingFeedback,
    .data.cellTemperature     = &rtt_cellTemperature,
    .data.cellVoltage         = &rtt_cellVoltage,
    .data.openWire            = &rtt_openWire,
    .data.slaveControl        = &rtt_slaveControl,
    .data.currentSensor       = &rtt_currentSensor,
};

/*========== Static Function Prototypes =====================================*/
static bool RTT_CheckMultipleCalls(RTT_STATE_s *pState);
static RTT_FSM_STATES_e RTT_ProcessInitializationState(RTT_STATE_s *pState);
static RTT_FSM_STATES_e RTT_ProcessRunningState(RTT_STATE_s *pState);
static void RTT_ParseInputLine(RTT_STATE_s *pState, uint8_t *pLineBuffer, uint16_t length);
static void RTT_RunStateMachine(RTT_STATE_s *pState);

/*========== Static Function Implementations ================================*/

static bool RTT_CheckMultipleCalls(RTT_STATE_s *pState) {
    FAS_ASSERT(pState != NULL_PTR);
    bool multipleCalls = false;
    OS_EnterTaskCritical();
    if (pState->triggerEntry == 0u) {
        pState->triggerEntry++;
    } else {
        multipleCalls = true; /* multiple calls */
    }
    OS_ExitTaskCritical();
    return multipleCalls;
}

static RTT_FSM_STATES_e RTT_ProcessInitializationState(RTT_STATE_s *pState) {
    FAS_ASSERT(pState != NULL_PTR);
    RTT_FSM_STATES_e nextState = RTT_FSM_STATE_INITIALIZATION; /* default: stay in state */

    switch (pState->currentSubstate) {
        case RTT_FSM_SUBSTATE_ENTRY:
            pState->nextSubstate = RTT_FSM_SUBSTATE_INITIALIZATION_FINISH_FIRST_MEASUREMENT;
            pState->timer        = RTT_FSM_SHORT_TIME;
            break;

        case RTT_FSM_SUBSTATE_INITIALIZATION_FINISH_FIRST_MEASUREMENT:
            /* Initialize database with default values so we have something valid to start with */
            DATA_WRITE_DATA(&rtt_cellVoltage, &rtt_cellTemperature);

            pState->nextSubstate = RTT_FSM_SUBSTATE_INITIALIZATION_FIRST_MEASUREMENT_FINISHED;
            pState->timer        = RTT_FSM_SHORT_TIME;
            break;

        case RTT_FSM_SUBSTATE_INITIALIZATION_FIRST_MEASUREMENT_FINISHED:
            pState->firstMeasurementFinished = true;
            pState->nextSubstate             = RTT_FSM_SUBSTATE_INITIALIZATION_EXIT;
            pState->timer                    = RTT_FSM_SHORT_TIME;
            break;

        case RTT_FSM_SUBSTATE_INITIALIZATION_EXIT:
            nextState            = RTT_FSM_STATE_RUNNING;
            pState->nextSubstate = RTT_FSM_SUBSTATE_ENTRY;
            pState->timer        = RTT_FSM_SHORT_TIME;
            break;

        default:
            /* invalid substate code */
            break;
    }
    return nextState;
}

static void RTT_ParseInputLine(RTT_STATE_s *pState, uint8_t *pLineBuffer, uint16_t length) {
    FAS_ASSERT(pState != NULL_PTR);
    FAS_ASSERT(pLineBuffer != NULL_PTR);

    if (length == 0u) {
        return;
    }

    pLineBuffer[length] = '\0'; /* Null terminate */
    char *ptr           = (char *)pLineBuffer;

    /* Parse Voltages if present */
    char *cvPtr = strstr(ptr, "CV:");
    if (cvPtr != NULL) {
        char *cvCursor = cvPtr + 3; /* Skip "CV:" */
        for (uint8_t c = 0; c < BS_NR_OF_CELL_BLOCKS_PER_STRING; c++) {
            if (*cvCursor == '\0') {
                break;
            }
            int val             = (int)strtol(cvCursor, &cvCursor, 10);
            uint8_t moduleIndex = (uint8_t)(c / BS_NR_OF_CELL_BLOCKS_PER_MODULE);
            uint8_t cellIndex   = (uint8_t)(c % BS_NR_OF_CELL_BLOCKS_PER_MODULE);
            pState->data.cellVoltage->cellVoltage_mV[0][moduleIndex][cellIndex]     = (int16_t)val;
            pState->data.cellVoltage->invalidCellVoltage[0][moduleIndex][cellIndex] = false;
            if (*cvCursor == ',') {
                cvCursor++;
            }
        }

        /* Calculate module and string voltages */
        int32_t stringVoltage_mV = 0;
        for (uint8_t m = 0u; m < BS_NR_OF_MODULES_PER_STRING; m++) {
            int32_t moduleVoltage_mV = 0;
            for (uint8_t c = 0u; c < BS_NR_OF_CELL_BLOCKS_PER_MODULE; c++) {
                moduleVoltage_mV += pState->data.cellVoltage->cellVoltage_mV[0][m][c];
            }
            pState->data.cellVoltage->moduleVoltage_mV[0][m]     = (uint32_t)moduleVoltage_mV;
            pState->data.cellVoltage->invalidModuleVoltage[0][m] = false;
            stringVoltage_mV += moduleVoltage_mV;
        }
        pState->data.cellVoltage->stringVoltage_mV[0]     = stringVoltage_mV;
        pState->data.cellVoltage->invalidStringVoltage[0] = false;
        pState->data.cellVoltage->nrValidCellVoltages[0]  = BS_NR_OF_CELL_BLOCKS_PER_STRING;

        /* Write valid flags */
        pState->data.cellVoltage->state = 0;
        DATA_WRITE_DATA(pState->data.cellVoltage);
    }

    /* Parse Temperatures if present */
    char *tempPtr = strstr((char *)pLineBuffer, "TEMP:");
    if (tempPtr != NULL) {
        char *tempCursor = tempPtr + 5; /* Skip "TEMP:" */
        for (uint8_t t = 0; t < BS_NR_OF_TEMP_SENSORS_PER_STRING; t++) {
            if (*tempCursor == '\0') {
                break;
            }
            float fVal = strtof(tempCursor, &tempCursor);
            /* Convert to deci-degC */
            uint8_t moduleIndex = (uint8_t)(t / BS_NR_OF_TEMP_SENSORS_PER_MODULE);
            uint8_t sensorIndex = (uint8_t)(t % BS_NR_OF_TEMP_SENSORS_PER_MODULE);
            pState->data.cellTemperature->cellTemperature_ddegC[0][moduleIndex][sensorIndex]  = (int16_t)(fVal * 10.0f);
            pState->data.cellTemperature->invalidCellTemperature[0][moduleIndex][sensorIndex] = false;
            if (*tempCursor == ',') {
                tempCursor++;
            }
        }
        pState->data.cellTemperature->state = 0;
        DATA_WRITE_DATA(pState->data.cellTemperature);
    }

    /* Parse Current if present */
    char *currPtr = strstr((char *)pLineBuffer, "CURR:");
    if (currPtr != NULL) {
        char *currCursor                                         = currPtr + 5; /* Skip "CURR:" */
        int iVal                                                 = (int)strtol(currCursor, &currCursor, 10);
        pState->data.currentSensor->current_mA[0]                = iVal;
        pState->data.currentSensor->newCurrent                   = 1; /* Trigger update ? */
        pState->data.currentSensor->timestampCurrentCounting[0]  = OS_GetTickCount();
        pState->data.currentSensor->timestampEnergyCounting[0]   = OS_GetTickCount();
        pState->data.currentSensor->timestampCurrent[0]          = OS_GetTickCount();
        pState->data.currentSensor->invalidCurrentMeasurement[0] = 0u;
        /* Write to DB */
        DATA_WRITE_DATA(pState->data.currentSensor);
    }
}

static RTT_FSM_STATES_e RTT_ProcessRunningState(RTT_STATE_s *pState) {
    FAS_ASSERT(pState != NULL_PTR);
    RTT_FSM_STATES_e nextState = RTT_FSM_STATE_RUNNING; /* default: stay in state */

    static uint8_t lineBuffer[RTT_INPUT_BUFFER_SIZE];
    static uint16_t bufferIndex   = 0;
    static bool rttRunningPrinted = false;

    uint8_t readByte;
    unsigned bytesRead;
    switch (pState->currentSubstate) {
        case RTT_FSM_SUBSTATE_ENTRY:
            if (rttRunningPrinted == false) {
                SEGGER_RTT_printf(0, "RTT AFE running\n");
                rttRunningPrinted = true;
            }

            pState->nextSubstate = RTT_FSM_SUBSTATE_RUNNING_READ_INPUT;
            pState->timer        = 1; /* Check frequently */
            break;

        case RTT_FSM_SUBSTATE_RUNNING_READ_INPUT:
            /*
             * Protocol: AFE_DATA,CV:v1,v2...,TEMP:t1,t2...,CURR:c\n
             * v = mV (int), t = degC (float), c = mA (int)
             */

            /* Read byte by byte to handle partial lines */

            do {
                bytesRead = SEGGER_RTT_Read(0, &readByte, 1);
                if (bytesRead > 0) {
                    IO_PinSet(&LED_PORT->DOUT, LED_PIN);
                    if (readByte == '\n' || readByte == '\r') {
                        if (bufferIndex > 0) {
                            RTT_ParseInputLine(pState, lineBuffer, bufferIndex);
                            /* Reset buffer */
                            bufferIndex = 0;
                        }
                    } else {
                        if (bufferIndex < (RTT_INPUT_BUFFER_SIZE - 1)) {
                            lineBuffer[bufferIndex++] = readByte;
                        } else {
                            /* Buffer overflow, reset */
                            bufferIndex = 0;
                        }
                    }
                }
            } while (bytesRead > 0);

            if (bufferIndex > 0u) {
                RTT_ParseInputLine(pState, lineBuffer, bufferIndex);
                bufferIndex = 0u;
            }

            pState->timer = 1; /* Check again soon */
            break;

        default:
            /* invalid substate code */
            break;
    }
    return nextState;
}

static void RTT_RunStateMachine(RTT_STATE_s *pState) {
    FAS_ASSERT(pState != NULL_PTR);

    if (pState->timer > 0u) {
        if ((--pState->timer) > 0u) {
            pState->triggerEntry--;
            return;
        }
    }

    switch (pState->currentState) {
        case RTT_FSM_STATE_HAS_NEVER_RUN:
            pState->nextState    = RTT_FSM_STATE_UNINITIALIZED;
            pState->nextSubstate = RTT_FSM_SUBSTATE_ENTRY;
            break;

        case RTT_FSM_STATE_UNINITIALIZED:
            pState->nextState    = RTT_FSM_STATE_INITIALIZATION;
            pState->nextSubstate = RTT_FSM_SUBSTATE_ENTRY;
            pState->timer        = RTT_FSM_SHORT_TIME;
            break;

        case RTT_FSM_STATE_INITIALIZATION:
            pState->nextState = RTT_ProcessInitializationState(pState);
            break;

        case RTT_FSM_STATE_RUNNING:
            pState->nextState = RTT_ProcessRunningState(pState);
            break;

        case RTT_FSM_STATE_ERROR:
            /* Stay in error state */
            break;

        default:
            /* Invalid state */
            break;
    }

    pState->currentState     = pState->nextState;
    pState->previousSubstate = pState->currentSubstate;
    pState->currentSubstate  = pState->nextSubstate;

    pState->triggerEntry--;
}

/*========== Extern Function Implementations ================================*/

extern STD_RETURN_TYPE_e RTT_Initialize(void) {
    /* Initialize RTT if not already done, though typicaly done in main or task init */
    /* SEGGER_RTT_Init(); is usually called early in ftask_cfg.c */
    return STD_OK;
}

extern bool RTT_IsFirstMeasurementCycleFinished(RTT_STATE_s *pState) {
    FAS_ASSERT(pState != NULL_PTR);
    return pState->firstMeasurementFinished;
}

extern STD_RETURN_TYPE_e RTT_TriggerAfe(RTT_STATE_s *pState) {
    FAS_ASSERT(pState != NULL_PTR);
    static bool rttTriggerPrinted = false;

    if (rttTriggerPrinted == false) {
        SEGGER_RTT_printf(0, "RTT Mock AFE(analog front end / slave board) called\n");
        rttTriggerPrinted = true;
    }

    if (RTT_CheckMultipleCalls(pState) == true) {
        return STD_NOT_OK;
    }

    RTT_RunStateMachine(pState);

    return STD_OK;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
