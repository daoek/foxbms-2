/**
 *
 * @copyright &copy; 2010 - 2025, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * We kindly request you to use one or more of the following phrases to refer to
 * foxBMS in your hardware, software, documentation or advertising materials:
 *
 * - "This product uses parts of foxBMS&reg;"
 * - "This product includes parts of foxBMS&reg;"
 * - "This product is derived from foxBMS&reg;"
 *
 */

/**
 * @file    can_cbs_tx_string-soc.c
 * @author  foxBMS Team
 * @date    2025-02-12 (date of creation)
 * @updated 2025-08-07 (date of last update)
 * @version v1.10.0
 * @ingroup DRIVERS
 * @prefix  CANTX
 *
 * @brief   CAN driver Tx callback implementation
 * @details CAN Tx callback for string SOC message
 */

/*========== Includes =======================================================*/
#include "bms.h"
/* AXIVION Next Codeline Generic-LocalInclude: 'can_cbs_tx_cyclic.h' declares
 * the prototype for the callback 'CANTX_StringSoc' */
#include "can_cbs_tx_cyclic.h"
#include "can_cfg_tx-cyclic-message-definitions.h"
#include "can_helper.h"

#include <math.h>
#include <stdint.h>

/*========== Macros and Definitions =========================================*/
#define CANTX_STRING_SOC_STRING_NUMBER (0u)

/** @{
 * defines of the SOC signals
*/
#define CANTX_SIGNAL_MINIMUM_SOC_START_BIT (3u)
#define CANTX_SIGNAL_MINIMUM_SOC_LENGTH    (9u)
#define CANTX_SIGNAL_MAXIMUM_SOC_START_BIT (10u)
#define CANTX_SIGNAL_MAXIMUM_SOC_LENGTH    (9u)
#define CANTX_SIGNAL_AVERAGE_SOC_START_BIT (17u)
#define CANTX_SIGNAL_AVERAGE_SOC_LENGTH    (9u)
#define CANTX_SIGNAL_SOH_START_BIT         (24u)
#define CANTX_SIGNAL_SOH_LENGTH            (9u)
#define CANTX_MINIMUM_VALUE_SOC_SIGNAL     (0.0f)
#define CANTX_MAXIMUM_VALUE_SOC_SIGNAL     (100.0f)
#define CANTX_FACTOR_SOC_SIGNAL            (0.2f)
/** @} */

/** @{
 * configuration of the SOC signals
*/
static const CAN_SIGNAL_TYPE_s cantx_signalMinimumStringSoc = {
    CANTX_SIGNAL_MINIMUM_SOC_START_BIT,
    CANTX_SIGNAL_MINIMUM_SOC_LENGTH,
    CANTX_FACTOR_SOC_SIGNAL,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_SOC_SIGNAL,
    CANTX_MAXIMUM_VALUE_SOC_SIGNAL};

static const CAN_SIGNAL_TYPE_s cantx_signalMaximumStringSoc = {
    CANTX_SIGNAL_MAXIMUM_SOC_START_BIT,
    CANTX_SIGNAL_MAXIMUM_SOC_LENGTH,
    CANTX_FACTOR_SOC_SIGNAL,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_SOC_SIGNAL,
    CANTX_MAXIMUM_VALUE_SOC_SIGNAL};

static const CAN_SIGNAL_TYPE_s cantx_signalAverageStringSoc = {
    CANTX_SIGNAL_AVERAGE_SOC_START_BIT,
    CANTX_SIGNAL_AVERAGE_SOC_LENGTH,
    CANTX_FACTOR_SOC_SIGNAL,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_SOC_SIGNAL,
    CANTX_MAXIMUM_VALUE_SOC_SIGNAL};

static const CAN_SIGNAL_TYPE_s cantx_signalStringSoh = {
    CANTX_SIGNAL_SOH_START_BIT,
    CANTX_SIGNAL_SOH_LENGTH,
    CANTX_FACTOR_SOC_SIGNAL,
    CAN_SIGNAL_OFFSET_0,
    CANTX_MINIMUM_VALUE_SOC_SIGNAL,
    CANTX_MAXIMUM_VALUE_SOC_SIGNAL};
/** @} */

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/
/**
 * @brief Calculates the return value for the strings minimum SOC
 * @return return value for minimum SOC
 */
static uint64_t CANTX_CalculateMinimumStringSoc(uint8_t stringNumber, const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief Calculates the return value for the strings maximum SOC
 * @return return value for maximum SOC
 */
static uint64_t CANTX_CalculateMaximumStringSoc(uint8_t stringNumber, const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief Calculates the return value for the strings average SOC
 * @return return value for average SOC
 */
static uint64_t CANTX_CalculateAverageStringSoc(uint8_t stringNumber, const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief Calculates the return value for the strings SOH
 * @return return value for SOH
 */
static uint64_t CANTX_CalculateStringSoh(uint8_t stringNumber, const CAN_SHIM_s *const kpkCanShim);

/**
 * @brief Builds the message form the data
 */
static void CANTX_BuildStringSocMessage(uint64_t *pMessageData, const CAN_SHIM_s *const kpkCanShim);

/*========== Static Function Implementations ================================*/
static uint64_t CANTX_CalculateMinimumStringSoc(uint8_t stringNumber, const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(stringNumber < BS_NR_OF_STRINGS);
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    float_t signalData = kpkCanShim->pTableSoc->minimumSoc_perc[stringNumber];
    CAN_TxPrepareSignalData(&signalData, cantx_signalMinimumStringSoc);
    return (uint64_t)signalData;
}

static uint64_t CANTX_CalculateMaximumStringSoc(uint8_t stringNumber, const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(stringNumber < BS_NR_OF_STRINGS);
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    float_t signalData = kpkCanShim->pTableSoc->maximumSoc_perc[stringNumber];
    CAN_TxPrepareSignalData(&signalData, cantx_signalMaximumStringSoc);
    return (uint64_t)signalData;
}

static uint64_t CANTX_CalculateAverageStringSoc(uint8_t stringNumber, const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(stringNumber < BS_NR_OF_STRINGS);
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    float_t signalData = kpkCanShim->pTableSoc->averageSoc_perc[stringNumber];
    CAN_TxPrepareSignalData(&signalData, cantx_signalAverageStringSoc);
    return (uint64_t)signalData;
}

static uint64_t CANTX_CalculateStringSoh(uint8_t stringNumber, const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(stringNumber < BS_NR_OF_STRINGS);
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    float_t signalData = kpkCanShim->pTableSoh->averageSoh_perc[stringNumber];
    CAN_TxPrepareSignalData(&signalData, cantx_signalStringSoh);
    return (uint64_t)signalData;
}

static void CANTX_BuildStringSocMessage(uint64_t *pMessageData, const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(pMessageData != NULL_PTR);
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    const uint8_t stringNumber = CANTX_STRING_SOC_STRING_NUMBER;
    FAS_ASSERT(stringNumber < BS_NR_OF_STRINGS);

    uint64_t data = CANTX_CalculateMinimumStringSoc(stringNumber, kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalMinimumStringSoc.bitStart,
        cantx_signalMinimumStringSoc.bitLength,
        data,
        CAN_BIG_ENDIAN);

    data = CANTX_CalculateMaximumStringSoc(stringNumber, kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalMaximumStringSoc.bitStart,
        cantx_signalMaximumStringSoc.bitLength,
        data,
        CAN_BIG_ENDIAN);

    data = CANTX_CalculateAverageStringSoc(stringNumber, kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData,
        cantx_signalAverageStringSoc.bitStart,
        cantx_signalAverageStringSoc.bitLength,
        data,
        CAN_BIG_ENDIAN);

    data = CANTX_CalculateStringSoh(stringNumber, kpkCanShim);
    CAN_TxSetMessageDataWithSignalData(
        pMessageData, cantx_signalStringSoh.bitStart, cantx_signalStringSoh.bitLength, data, CAN_BIG_ENDIAN);
}

/*========== Extern Function Implementations ================================*/
extern uint32_t CANTX_StringSoc(
    CAN_MESSAGE_PROPERTIES_s message,
    uint8_t *pCanData,
    uint8_t *pMuxId,
    const CAN_SHIM_s *const kpkCanShim) {
    FAS_ASSERT(message.id == CANTX_STRING_SOC_ID);
    FAS_ASSERT(message.idType == CANTX_STRING_SOC_ID_TYPE);
    FAS_ASSERT(message.dlc <= CAN_MAX_DLC);
    FAS_ASSERT(message.endianness == CANTX_STRING_SOC_ENDIANNESS);
    FAS_ASSERT(pCanData != NULL_PTR);
    FAS_ASSERT(pMuxId == NULL_PTR); /* pMuxId is not used here, therefore has to be NULL_PTR */
    FAS_ASSERT(kpkCanShim != NULL_PTR);

    uint64_t messageData = 0u;

    DATA_READ_DATA(kpkCanShim->pTableSoc, kpkCanShim->pTableSoh);

    CANTX_BuildStringSocMessage(&messageData, kpkCanShim);
    CAN_TxSetCanDataWithMessageData(messageData, pCanData, message.endianness);

    return 0u;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
