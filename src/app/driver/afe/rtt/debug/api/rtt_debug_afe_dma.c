/**
 *
 * @copyright &copy; 2010 - 2025, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * All rights reserved.
 *
 * Confidential or no approval for publication
 *
 */

/**
 * @file    rtt_debug_afe_dma.c
 * @author  foxBMS Team
 * @date    2026-01-24 (date of creation)
 * @updated 2026-01-24 (date of last update)
 * @version v1.9.0
 * @ingroup DRIVERS
 * @prefix  RTT
 *
 * @brief   Driver for the DMA module for the RTT AFE driver.
 * @details Dummy implementation
 */

/*========== Includes =======================================================*/
#include "afe_dma.h"

#include <stdint.h>

/*========== Macros and Definitions =========================================*/

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/

/*========== Static Function Implementations ================================*/

/*========== Extern Function Implementations ================================*/

/* Function called on DMA complete interrupts (TX and RX). */
void AFE_DmaCallback(uint8_t spiIndex) {
    /* this is a dummy implementation and not using the argument here is fine */
    (void)spiIndex;
}

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
