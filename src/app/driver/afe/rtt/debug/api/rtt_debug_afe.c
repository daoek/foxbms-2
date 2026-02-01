/**
 *
 * @copyright &copy; 2010 - 2025, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * All rights reserved.
 *
 * Confidential or no approval for publication
 *
 */

/**
 * @file    rtt_debug_afe.c
 * @author  foxBMS Team
 * @date    2026-01-24 (date of creation)
 * @updated 2026-01-24 (date of last update)
 * @version v1.9.0
 * @ingroup DRIVERS
 * @prefix  RTT
 *
 * @brief   Implementation of the RTT AFE driver API
 * @details Wrapper for AFE API
 */

/*========== Includes =======================================================*/

#include "afe.h"
#include "rtt_debug.h"

#include <stdint.h>

/*========== Macros and Definitions =========================================*/

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/

/*========== Static Function Implementations ================================*/

/*========== Extern Function Implementations ================================*/
extern STD_RETURN_TYPE_e AFE_TriggerIc(void) {
    return RTT_TriggerAfe(&rtt_state);
}

extern STD_RETURN_TYPE_e AFE_Initialize(void) {
    return RTT_Initialize();
}

extern STD_RETURN_TYPE_e AFE_StartMeasurement(void) {
    return STD_OK;
}

extern bool AFE_IsFirstMeasurementCycleFinished(void) {
    return RTT_IsFirstMeasurementCycleFinished(&rtt_state);
}

extern STD_RETURN_TYPE_e AFE_RequestTemperatureRead(uint8_t string) {
    /* parameter unused */
    (void)string;
    return STD_OK;
}

extern STD_RETURN_TYPE_e AFE_RequestBalancingFeedbackRead(uint8_t string) {
    /* parameter unused */
    (void)string;
    return STD_OK;
}

extern STD_RETURN_TYPE_e AFE_RequestEepromRead(uint8_t string) {
    /* parameter unused */
    (void)string;
    return STD_OK;
}

extern STD_RETURN_TYPE_e AFE_RequestEepromWrite(uint8_t string) {
    /* parameter unused */
    (void)string;
    return STD_OK;
}

extern STD_RETURN_TYPE_e AFE_RequestOpenWireCheck(uint8_t string) {
    /* parameter unused */
    (void)string;
    return STD_OK;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
