/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SJA1124_H_
#define SJA1124_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
/**
 * @brief The SJA1124 types
 */
#define SJA1124_SPI_READ_CMD (0x80)
#define SJA1124_SPI_WRITE_CMD (0x00)
#define SJA1124_REG_SIZE_BYTE  (1)
#define MAX_RETRIES		1000

/**
 **
 **  @brief The SJA1124 Internal Register Map.
 */
typedef enum
{
	// System control registers
	SJA1124_MODE= 0x00,
	SJA1124_PLLCFG = 0x01,
	SJA1124_INT1EN = 0x02,
	SJA1124_INT2EN = 0x03,
	SJA1124_INT3EN= 0x04,

	// System status registers
	SJA1124_INT1 = 0x10,
	SJA1124_INT2 = 0x11,
	SJA1124_INT3 = 0x12,
	SJA1124_STATUS = 0x13,

	// LIN commander controller global registers
	SJA1124_LCOM1 = 0x20,
	SJA1124_LCOM2 = 0x21,

	// LIN1 channel initialization register
	SJA1124_LIN1_CI_LCFG1 = 0x30,
	SJA1124_LIN1_CI_LCFG2 = 0x31,
	SJA1124_LIN1_CI_LITC = 0x32,
	SJA1124_LIN1_CI_LGC = 0x33,
	SJA1124_LIN1_CI_LRTC = 0x34,
	SJA1124_LIN1_CI_LFR = 0x35,
	SJA1124_LIN1_CI_LBRM = 0x36,
	SJA1124_LIN1_CI_LBRL = 0x37,
	SJA1124_LIN1_CI_LIE = 0x38,

	// LIN2 channel initialization register
	SJA1124_LIN2_CI_LCFG1 = 0x60,
	SJA1124_LIN2_CI_LCFG2 = 0x61,
	SJA1124_LIN2_CI_LITC = 0x62,
	SJA1124_LIN2_CI_LGC = 0x63,
	SJA1124_LIN2_CI_LRTC = 0x64,
	SJA1124_LIN2_CI_LFR = 0x65,
	SJA1124_LIN2_CI_LBRM = 0x66,
	SJA1124_LIN2_CI_LBRL = 0x67,
	SJA1124_LIN2_CI_LIE = 0x68,

	// LIN3 channel initialization register
	SJA1124_LIN3_CI_LCFG1 = 0x90,
	SJA1124_LIN3_CI_LCFG2 = 0x91,
	SJA1124_LIN3_CI_LITC = 0x92,
	SJA1124_LIN3_CI_LGC = 0x93,
	SJA1124_LIN3_CI_LRTC = 0x94,
	SJA1124_LIN3_CI_LFR = 0x95,
	SJA1124_LIN3_CI_LBRM = 0x96,
	SJA1124_LIN3_CI_LBRL = 0x97,
	SJA1124_LIN3_CI_LIE = 0x98,

	// LIN4 channel initialization register
	SJA1124_LIN4_CI_LCFG1 = 0xC0,
	SJA1124_LIN4_CI_LCFG2 = 0xC1,
	SJA1124_LIN4_CI_LITC = 0xC2,
	SJA1124_LIN4_CI_LGC = 0xC3,
	SJA1124_LIN4_CI_LRTC = 0xC4,
	SJA1124_LIN4_CI_LFR = 0xC5,
	SJA1124_LIN4_CI_LBRM = 0xC6,
	SJA1124_LIN4_CI_LBRL = 0xC7,
	SJA1124_LIN4_CI_LIE = 0xC8,

	// LIN1 channel send frame registers
	SJA1124_LIN1_SF_LC = 0x39,
	SJA1124_LIN1_SF_LBI = 0x3A,
	SJA1124_LIN1_SF_LBC = 0X3B,
	SJA1124_LIN1_SF_LCF = 0x3C,
	SJA1124_LIN1_SF_LBD1 = 0x3D,
	SJA1124_LIN1_SF_LBD2 = 0x3E,
	SJA1124_LIN1_SF_LBD3 = 0x3F,
	SJA1124_LIN1_SF_LBD4 = 0X40,
	SJA1124_LIN1_SF_LBD5 = 0x41,
	SJA1124_LIN1_SF_LBD6 = 0x42,
	SJA1124_LIN1_SF_LBD7 = 0x43,
	SJA1124_LIN1_SF_LBD8 = 0x44,

	// LIN2 channel send frame registers
	SJA1124_LIN2_SF_LC = 0x69,
	SJA1124_LIN2_SF_LBI = 0x6A,
	SJA1124_LIN2_SF_LBC = 0X6B,
	SJA1124_LIN2_SF_LCF = 0x6C,
	SJA1124_LIN2_SF_LBD1 = 0x6D,
	SJA1124_LIN2_SF_LBD2 = 0x6E,
	SJA1124_LIN2_SF_LBD3 = 0x6F,
	SJA1124_LIN2_SF_LBD4 = 0X70,
	SJA1124_LIN2_SF_LBD5 = 0x71,
	SJA1124_LIN2_SF_LBD6 = 0x72,
	SJA1124_LIN2_SF_LBD7 = 0x73,
	SJA1124_LIN2_SF_LBD8 = 0x74,

	// LIN3 channel send frame registers
	SJA1124_LIN3_SF_LC = 0x99,
	SJA1124_LIN3_SF_LBI = 0x9A,
	SJA1124_LIN3_SF_LBC = 0X9B,
	SJA1124_LIN3_SF_LCF = 0x9C,
	SJA1124_LIN3_SF_LBD1 = 0x9D,
	SJA1124_LIN3_SF_LBD2 = 0x9E,
	SJA1124_LIN3_SF_LBD3 = 0x9F,
	SJA1124_LIN3_SF_LBD4 = 0XA0,
	SJA1124_LIN3_SF_LBD5 = 0xA1,
	SJA1124_LIN3_SF_LBD6 = 0xA2,
	SJA1124_LIN3_SF_LBD7 = 0xA3,
	SJA1124_LIN3_SF_LBD8 = 0xA4,


	// LIN4 channel send frame registers
	SJA1124_LIN4_SF_LC = 0xC9,
	SJA1124_LIN4_SF_LBI = 0xCA,
	SJA1124_LIN4_SF_LBC = 0XCB,
	SJA1124_LIN4_SF_LCF = 0xCC,
	SJA1124_LIN4_SF_LBD1 = 0xCD,
	SJA1124_LIN4_SF_LBD2 = 0xCE,
	SJA1124_LIN4_SF_LBD3 = 0xCF,
	SJA1124_LIN4_SF_LBD4 = 0XD0,
	SJA1124_LIN4_SF_LBD5 = 0xD1,
	SJA1124_LIN4_SF_LBD6 = 0xD2,
	SJA1124_LIN4_SF_LBD7 = 0xD3,
	SJA1124_LIN4_SF_LBD8 = 0xD4,

	// LIN1 channel get status registers
	SJA1124_LIN1_GS_LSTATE = 0x4F,
	SJA1124_LIN1_GS_LES= 0x50,
	SJA1124_LIN1_GS_LS = 0X51,
	SJA1124_LIN1_GS_LCF = 0x52,
	SJA1124_LIN1_GS_LBD1 = 0x53,
	SJA1124_LIN1_GS_LBD2 = 0x54,
	SJA1124_LIN1_GS_LBD3 = 0x55,
	SJA1124_LIN1_GS_LBD4 = 0X56,
	SJA1124_LIN1_GS_LBD5 = 0x57,
	SJA1124_LIN1_GS_LBD6 = 0x58,
	SJA1124_LIN1_GS_LBD7 = 0x59,
	SJA1124_LIN1_GS_LBD8 = 0x5A,


	// LIN2 channel get status registers
	SJA1124_LIN2_GS_LSTATE = 0x7F,
	SJA1124_LIN2_GS_LES= 0x80,
	SJA1124_LIN2_GS_LS = 0X81,
	SJA1124_LIN2_GS_LCF = 0x82,
	SJA1124_LIN2_GS_LBD1 = 0x83,
	SJA1124_LIN2_GS_LBD2 = 0x84,
	SJA1124_LIN2_GS_LBD3 = 0x85,
	SJA1124_LIN2_GS_LBD4 = 0X86,
	SJA1124_LIN2_GS_LBD5 = 0x87,
	SJA1124_LIN2_GS_LBD6 = 0x88,
	SJA1124_LIN2_GS_LBD7 = 0x89,
	SJA1124_LIN2_GS_LBD8 = 0x8A,


	// LIN3 channel get status registers
	SJA1124_LIN3_GS_LSTATE = 0xAF,
	SJA1124_LIN3_GS_LES= 0xB0,
	SJA1124_LIN3_GS_LS = 0XB1,
	SJA1124_LIN3_GS_LCF = 0xB2,
	SJA1124_LIN3_GS_LBD1 = 0xB3,
	SJA1124_LIN3_GS_LBD2 = 0xB4,
	SJA1124_LIN3_GS_LBD3 = 0xB5,
	SJA1124_LIN3_GS_LBD4 = 0XB6,
	SJA1124_LIN3_GS_LBD5 = 0xB7,
	SJA1124_LIN3_GS_LBD6 = 0xB8,
	SJA1124_LIN3_GS_LBD7 = 0xB9,
	SJA1124_LIN3_GS_LBD8 = 0xBA,

	// LIN4 channel get status registers
	SJA1124_LIN4_GS_LSTATE = 0xDF,
	SJA1124_LIN4_GS_LES= 0xE0,
	SJA1124_LIN4_GS_LS = 0XE1,
	SJA1124_LIN4_GS_LCF = 0xE2,
	SJA1124_LIN4_GS_LBD1 = 0xE3,
	SJA1124_LIN4_GS_LBD2 = 0xE4,
	SJA1124_LIN4_GS_LBD3 = 0xE5,
	SJA1124_LIN4_GS_LBD4 = 0XE6,
	SJA1124_LIN4_GS_LBD5 = 0xE7,
	SJA1124_LIN4_GS_LBD6 = 0xE8,
	SJA1124_LIN4_GS_LBD7 = 0xE9,
	SJA1124_LIN4_GS_LBD8 = 0xEA,

	// LIN commander termination configuration registers
	SJA1124_LIN_CT_CFG_MCFG = 0xF0,
	SJA1124_LIN_CT_CFG_MMTPS = 0xF1,
	SJA1124_LIN_CT_CFG_MMTPSS = 0xF2,

	// Other registers
	SJA1124_OR_MTPCS = 0XFE,
	SJA1124_OR_ID = 0xFF,
}register_set;

/* variable masks */

/* register MODE */
#define SJA1124_MODE_RESET_SHIFT            (7)
#define SJA1124_MODE_RESET_MASK             (0x80u)
#define SJA1124_MODE_LOWPWRMODE_SHIFT       (0)
#define SJA1124_MODE_LOWPWRMODE_MASK        (0x01u)

/* register PLLCFG */
#define SJA1124_PLLCFG_PLLMULT_SHIFT        (0)
#define SJA1124_PLLCFG_PLLMULT_MASK         (0x0Fu)

/* register INT1EN */
#define SJA1124_INT1EN_L4WKUPIEN_SHIFT      (3)
#define SJA1124_INT1EN_L4WKUPIEN_MASK       (0x08u)
#define SJA1124_INT1EN_L3WKUPIEN_SHIFT      (2)
#define SJA1124_INT1EN_L3WKUPIEN_MASK       (0x04u)
#define SJA1124_INT1EN_L2WKUPIEN_SHIFT      (1)
#define SJA1124_INT1EN_L2WKUPIEN_MASK       (0x02u)
#define SJA1124_INT1EN_L1WKUPIEN_SHIFT      (0)
#define SJA1124_INT1EN_L1WKUPIEN_MASK       (0x01u)

/* register INT2EN */
#define SJA1124_INT2EN_OVERTMPWARNIEN_SHIFT (5)
#define SJA1124_INT2EN_OVERTMPWARNIEN_MASK  (0x20u)
#define SJA1124_INT2EN_PLLNOLOCKIEN_SHIFT   (4)
#define SJA1124_INT2EN_PLLNOLOCKIEN_MASK    (0x10u)
#define SJA1124_INT2EN_PLLINLOCKIEN_SHIFT   (3)
#define SJA1124_INT2EN_PLLINLOCKIEN_MASK    (0x08u)
#define SJA1124_INT2EN_PLLFRQFAILIEN_SHIFT  (2)
#define SJA1124_INT2EN_PLLFRQFAILIEN_MASK   (0x04u)
#define SJA1124_INT2EN_SPIERRIEN_SHIFT      (1)
#define SJA1124_INT2EN_SPIERRIEN_MASK       (0x02u)

/* register INT3EN */
#define SJA1124_INT3EN_L4CONTERRIEN_SHIFT   (7)
#define SJA1124_INT3EN_L4CONTERRIEN_MASK    (0x80u)
#define SJA1124_INT3EN_L3CONTERRIEN_SHIFT   (6)
#define SJA1124_INT3EN_L3CONTERRIEN_MASK    (0x40u)
#define SJA1124_INT3EN_L2CONTERRIEN_SHIFT   (5)
#define SJA1124_INT3EN_L2CONTERRIEN_MASK    (0x20u)
#define SJA1124_INT3EN_L1CONTERRIEN_SHIFT   (4)
#define SJA1124_INT3EN_L1CONTERRIEN_MASK    (0x10u)
#define SJA1124_INT3EN_L4CONTSTATIEN_SHIFT  (3)
#define SJA1124_INT3EN_L4CONTSTATIEN_MASK   (0x08u)
#define SJA1124_INT3EN_L3CONTSTATIEN_SHIFT  (2)
#define SJA1124_INT3EN_L3CONTSTATIEN_MASK   (0x04u)
#define SJA1124_INT3EN_L2CONTSTATIEN_SHIFT  (1)
#define SJA1124_INT3EN_L2CONTSTATIEN_MASK   (0x02u)
#define SJA1124_INT3EN_L1CONTSTATIEN_SHIFT  (0)
#define SJA1124_INT3EN_L1CONTSTATIEN_MASK   (0x01u)

/* register INT1 */
#define SJA1124_INT1_INITSTATINT_SHIFT      (7)
#define SJA1124_INT1_INITSTATINT_MASK       (0x80u)
#define SJA1124_INT1_L4WAKEUPINT_SHIFT      (3)
#define SJA1124_INT1_L4WAKEUPINT_MASK       (0x08u)
#define SJA1124_INT1_L3WAKEUPINT_SHIFT      (2)
#define SJA1124_INT1_L3WAKEUPINT_MASK       (0x04u)
#define SJA1124_INT1_L2WAKEUPINT_SHIFT      (1)
#define SJA1124_INT1_L2WAKEUPINT_MASK       (0x02u)
#define SJA1124_INT1_L1WAKEUPINT_SHIFT      (0)
#define SJA1124_INT1_L1WAKEUPINT_MASK       (0x01u)

/* register INT2 */
#define SJA1124_INT2_OVERTMPWARNINT_SHIFT   (5)
#define SJA1124_INT2_OVERTMPWARNINT_MASK    (0x20u)
#define SJA1124_INT2_PLLNOLOCKINT_SHIFT     (4)
#define SJA1124_INT2_PLLNOLOCKINT_MASK      (0x10u)
#define SJA1124_INT2_PLLINLOCKINT_SHIFT     (3)
#define SJA1124_INT2_PLLINLOCKINT_MASK      (0x08u)
#define SJA1124_INT2_PLLFRQFAILINT_SHIFT    (2)
#define SJA1124_INT2_PLLFRQFAILINT_MASK     (0x04u)
#define SJA1124_INT2_SPIERRINT_SHIFT        (1)
#define SJA1124_INT2_SPIERRINT_MASK         (0x02u)
#define SJA1124_INT2_LOWPWRREQFAILINT_SHIFT (0)
#define SJA1124_INT2_LOWPWRREQFAILINT_MASK  (0x01u)

/* register INT3 */
#define SJA1124_INT3_L4CONTERRINT_SHIFT     (7)
#define SJA1124_INT3_L4CONTERRINT_MASK      (0x80u)
#define SJA1124_INT3_L3CONTERRINT_SHIFT     (6)
#define SJA1124_INT3_L3CONTERRINT_MASK      (0x40u)
#define SJA1124_INT3_L2CONTERRINT_SHIFT     (5)
#define SJA1124_INT3_L2CONTERRINT_MASK      (0x20u)
#define SJA1124_INT3_L1CONTERRINT_SHIFT     (4)
#define SJA1124_INT3_L1CONTERRINT_MASK      (0x10u)
#define SJA1124_INT3_L4CONTSTATINT_SHIFT    (3)
#define SJA1124_INT3_L4CONTSTATINT_MASK     (0x08u)
#define SJA1124_INT3_L3CONTSTATINT_SHIFT    (2)
#define SJA1124_INT3_L3CONTSTATINT_MASK     (0x04u)
#define SJA1124_INT3_L2CONTSTATINT_SHIFT    (1)
#define SJA1124_INT3_L2CONTSTATINT_MASK     (0x02u)
#define SJA1124_INT3_L1CONTSTATINT_SHIFT    (0)
#define SJA1124_INT3_L1CONTSTATINT_MASK     (0x01u)

/* register STATUS */
#define SJA1124_STATUS_OVERTMPWARN_SHIFT    (5)
#define SJA1124_STATUS_OVERTMPWARN_MASK     (0x20u)
#define SJA1124_STATUS_PLLINLOCK_SHIFT      (3)
#define SJA1124_STATUS_PLLINLOCK_MASK       (0x08u)
#define SJA1124_STATUS_PLLINFRQFAIL_SHIFT   (2)
#define SJA1124_STATUS_PLLINFRQFAIL_MASK    (0x04u)

/* register LCOM1 */
#define SJA1124_LCOM1_L4HSMODE_SHIFT        (7)
#define SJA1124_LCOM1_L4HSMODE_MASK         (0x80u)
#define SJA1124_LCOM1_L3HSMODE_SHIFT        (6)
#define SJA1124_LCOM1_L3HSMODE_MASK         (0x40u)
#define SJA1124_LCOM1_L2HSMODE_SHIFT        (5)
#define SJA1124_LCOM1_L2HSMODE_MASK         (0x20u)
#define SJA1124_LCOM1_L1HSMODE_SHIFT        (4)
#define SJA1124_LCOM1_L1HSMODE_MASK         (0x10u)
#define SJA1124_LCOM1_L4TRANSFRAMEIN_SHIFT  (3)
#define SJA1124_LCOM1_L4TRANSFRAMEIN_MASK   (0x08u)
#define SJA1124_LCOM1_L3TRANSFRAMEIN_SHIFT  (2)
#define SJA1124_LCOM1_L3TRANSFRAMEIN_MASK   (0x04u)
#define SJA1124_LCOM1_L2TRANSFRAMEIN_SHIFT  (1)
#define SJA1124_LCOM1_L2TRANSFRAMEIN_MASK   (0x02u)
#define SJA1124_LCOM1_L1TRANSFRAMEIN_SHIFT  (0)
#define SJA1124_LCOM1_L1TRANSFRAMEIN_MASK   (0x01u)

/* register LCOM2 */
#define SJA1124_LCOM2_L4HEADTRANSREQ_SHIFT  (3)
#define SJA1124_LCOM2_L4HEADTRANSREQ_MASK   (0x08u)
#define SJA1124_LCOM2_L3HEADTRANSREQ_SHIFT  (2)
#define SJA1124_LCOM2_L3HEADTRANSREQ_MASK   (0x04u)
#define SJA1124_LCOM2_L2HEADTRANSREQ_SHIFT  (1)
#define SJA1124_LCOM2_L2HEADTRANSREQ_MASK   (0x02u)
#define SJA1124_LCOM2_L1HEADTRANSREQ_SHIFT  (0)
#define SJA1124_LCOM2_L1HEADTRANSREQ_MASK   (0x01u)

/* register LIN1CFG1 */
#define SJA1124_LIN1CFG1_CSCALCDIS_SHIFT    (7)
#define SJA1124_LIN1CFG1_CSCALHW_MASK     (0x00u)
#define SJA1124_LIN1CFG1_CSCALSW_MASK      (0x80)
#define SJA1124_LIN1CFG1_MASBREAKLEN_SHIFT  (3)
#define SJA1124_LIN1CFG1_MASBREAKLEN_MASK   (0x78u)
#define SJA1124_LIN1CFG1_SLEEP_SHIFT        (1)
#define SJA1124_LIN1CFG1_SLEEP_MASK         (0x02u)
#define SJA1124_LIN1CFG1_INIT_SHIFT         (0)
#define SJA1124_LIN1CFG1_INIT_MASK          (0x01u)

/* register LIN1CFG2 */
#define SJA1124_LIN1CFG2_TWOBITDELIM_SHIFT  (7)
#define SJA1124_LIN1CFG2_TWOBITDELIM_MASK   (0x80u)
#define SJA1124_LIN1CFG2_ONEBITDELIM_SHIFT  (7)
#define SJA1124_LIN1CFG2_ONEBITDELIM_MASK   (0x00u)
#define SJA1124_LIN1CFG2_IDLEONBITERR_SHIFT (6)
#define SJA1124_LIN1CFG2_IDLEONBITERR_MASK  (0x40u)

/* register LIN1ITC */
#define SJA1124_LIN1ITC_IDLEONTIMEOUT_SHIFT (1)
#define SJA1124_LIN1ITC_IDLEONTIMEOUT_MASK  (0x02u)

/* register LIN1GC */
#define SJA1124_LIN1GC_STOPBITCONF_SHIFT    (1)
#define SJA1124_LIN1GC_STOPBITCONF_MASK     (0x02u)
#define SJA1124_LIN1GC_SOFTRESET_SHIFT      (0)
#define SJA1124_LIN1GC_SOFTRESET_MASK       (0x01u)

/* register LIN1RTC */
#define SJA1124_LIN1RTC_RESPTIMEOUT_SHIFT   (0)
#define SJA1124_LIN1RTC_RESPTIMEOUT_MASK    (0x0Fu)

/* register LIN1FR */
#define SJA1124_LIN1FR_FRACBAUDRATE_SHIFT   (0)
#define SJA1124_LIN1FR_FRACBAUDRATE_MASK    (0x0Fu)

/* register LIN1IE */
#define SJA1124_LIN1IE_STUCKZEROIEN_SHIFT   (7)
#define SJA1124_LIN1IE_STUCKZEROIEN_MASK    (0x80u)
#define SJA1124_LIN1IE_TIMEOUTIEN_SHIFT     (6)
#define SJA1124_LIN1IE_TIMEOUTIEN_MASK      (0x40u)
#define SJA1124_LIN1IE_BITERRIEN_SHIFT      (5)
#define SJA1124_LIN1IE_BITERRIEN_MASK       (0x20u)
#define SJA1124_LIN1IE_CSERRIEN_SHIFT       (4)
#define SJA1124_LIN1IE_CSERRIEN_MASK        (0x10u)
#define SJA1124_LIN1IE_DTRCVIEN_SHIFT       (2)
#define SJA1124_LIN1IE_DTRCVIEN_MASK        (0x04u)
#define SJA1124_LIN1IE_DTTRANSIEN_SHIFT     (1)
#define SJA1124_LIN1IE_DTTRANSIEN_MASK      (0x02u)
#define SJA1124_LIN1IE_FRAMEERRIEN_SHIFT    (0)
#define SJA1124_LIN1IE_FRAMEERRIEN_MASK     (0x01u)

/* register LIN1C */
#define SJA1124_LIN1C_WAKEUPREQ_SHIFT       (4)
#define SJA1124_LIN1C_WAKEUPREQ_MASK        (0x10u)
#define SJA1124_LIN1C_DTDISCARDREQ_SHIFT    (3)
#define SJA1124_LIN1C_DTDISCARDREQ_MASK     (0x08u)
#define SJA1124_LIN1C_ABORTREQ_SHIFT        (1)
#define SJA1124_LIN1C_ABORTREQ_MASK         (0x02u)
#define SJA1124_LIN1C_HEADERTRANSREQ_SHIFT  (0)
#define SJA1124_LIN1C_HEADERTRANSREQ_MASK   (0x01u)

/* register LIN1BI */
#define SJA1124_LIN1BI_IDENTIFIER_SHIFT     (0)
#define SJA1124_LIN1BI_IDENTIFIER_MASK      (0x3Fu)

/* register LIN1BC */
#define SJA1124_LIN1BC_DTFIELDLEN_SHIFT     (2)
#define SJA1124_LIN1BC_DTFIELDLEN_MASK      (0x1Cu)
#define SJA1124_LIN1BC_DIRECTION_SHIFT      (1)
#define SJA1124_LIN1BC_DIRECTION_MASK       (0x02u)
#define SJA1124_LIN1BC_CLASSICCS_SHIFT      (0)
#define SJA1124_LIN1BC_CLASSICCS_MASK       (0x01u)

/* register LIN1STATE */
#define SJA1124_LIN1STATE_RCVBUSY_SHIFT     (7)
#define SJA1124_LIN1STATE_RCVBUSY_MASK      (0x80u)
#define SJA1124_LIN1STATE_LINSTATE_SHIFT    (0)
#define SJA1124_LIN1STATE_LINSTATE_MASK     (0x0Fu)

/* register LIN1ES */
#define SJA1124_LIN1ES_STUCKZEROFLG_SHIFT   (7)
#define SJA1124_LIN1ES_STUCKZEROFLG_MASK    (0x80u)
#define SJA1124_LIN1ES_TIMEOUTERRFLG_SHIFT  (6)
#define SJA1124_LIN1ES_TIMEOUTERRFLG_MASK   (0x40u)
#define SJA1124_LIN1ES_BITERRFLG_SHIFT      (5)
#define SJA1124_LIN1ES_BITERRFLG_MASK       (0x20u)
#define SJA1124_LIN1ES_CSERRFLG_SHIFT       (4)
#define SJA1124_LIN1ES_CSERRFLG_MASK        (0x10u)
#define SJA1124_LIN1ES_FRAMEERRFLG_SHIFT    (0)
#define SJA1124_LIN1ES_FRAMEERRFLG_MASK     (0x01u)

/* register LIN1S */
#define SJA1124_LIN1S_RCVBUFFFULLFLG_SHIFT  (6)
#define SJA1124_LIN1S_RCVBUFFFULLFLG_MASK   (0x40u)
#define SJA1124_LIN1S_RCVCOMPFLG_SHIFT      (2)
#define SJA1124_LIN1S_RCVCOMPFLG_MASK       (0x04u)
#define SJA1124_LIN1S_TRANSCOMPFLG_SHIFT    (1)
#define SJA1124_LIN1S_TRANSCOMPFLG_MASK     (0x02u)

/* register LIN2CFG1 */
#define SJA1124_LIN2CFG1_CSCALCDIS_SHIFT    (7)
#define SJA1124_LIN2CFG1_CSCALCDIS_MASK     (0x80u)
#define SJA1124_LIN2CFG1_MASBREAKLEN_SHIFT  (3)
#define SJA1124_LIN2CFG1_MASBREAKLEN_MASK   (0x78u)
#define SJA1124_LIN2CFG1_SLEEP_SHIFT        (1)
#define SJA1124_LIN2CFG1_SLEEP_MASK         (0x02u)
#define SJA1124_LIN2CFG1_INIT_SHIFT         (0)
#define SJA1124_LIN2CFG1_INIT_MASK          (0x01u)

/* register LIN2CFG2 */
#define SJA1124_LIN2CFG2_TWOBITDELIM_SHIFT  (7)
#define SJA1124_LIN2CFG2_TWOBITDELIM_MASK   (0x80u)
#define SJA1124_LIN2CFG2_IDLEONBITERR_SHIFT (6)
#define SJA1124_LIN2CFG2_IDLEONBITERR_MASK  (0x40u)

/* register LIN2ITC */
#define SJA1124_LIN2ITC_IDLEONTIMEOUT_SHIFT (1)
#define SJA1124_LIN2ITC_IDLEONTIMEOUT_MASK  (0x02u)

/* register LIN2GC */
#define SJA1124_LIN2GC_STOPBITCONF_SHIFT    (1)
#define SJA1124_LIN2GC_STOPBITCONF_MASK     (0x02u)
#define SJA1124_LIN2GC_SOFTRESET_SHIFT      (0)
#define SJA1124_LIN2GC_SOFTRESET_MASK       (0x01u)

/* register LIN2RTC */
#define SJA1124_LIN2RTC_RESPTIMEOUT_SHIFT   (0)
#define SJA1124_LIN2RTC_RESPTIMEOUT_MASK    (0x0Fu)

/* register LIN2FR */
#define SJA1124_LIN2FR_FRACBAUDRATE_SHIFT   (0)
#define SJA1124_LIN2FR_FRACBAUDRATE_MASK    (0x0Fu)

/* register LIN2IE */
#define SJA1124_LIN2IE_STUCKZEROIEN_SHIFT   (7)
#define SJA1124_LIN2IE_STUCKZEROIEN_MASK    (0x80u)
#define SJA1124_LIN2IE_TIMEOUTIEN_SHIFT     (6)
#define SJA1124_LIN2IE_TIMEOUTIEN_MASK      (0x40u)
#define SJA1124_LIN2IE_BITERRIEN_SHIFT      (5)
#define SJA1124_LIN2IE_BITERRIEN_MASK       (0x20u)
#define SJA1124_LIN2IE_CSERRIEN_SHIFT       (4)
#define SJA1124_LIN2IE_CSERRIEN_MASK        (0x10u)
#define SJA1124_LIN2IE_DTRCVIEN_SHIFT       (2)
#define SJA1124_LIN2IE_DTRCVIEN_MASK        (0x04u)
#define SJA1124_LIN2IE_DTTRANSIEN_SHIFT     (1)
#define SJA1124_LIN2IE_DTTRANSIEN_MASK      (0x02u)
#define SJA1124_LIN2IE_FRAMEERRIEN_SHIFT    (0)
#define SJA1124_LIN2IE_FRAMEERRIEN_MASK     (0x01u)

/* register LIN2C */
#define SJA1124_LIN2C_WAKEUPREQ_SHIFT       (4)
#define SJA1124_LIN2C_WAKEUPREQ_MASK        (0x10u)
#define SJA1124_LIN2C_DTDISCARDREQ_SHIFT    (3)
#define SJA1124_LIN2C_DTDISCARDREQ_MASK     (0x08u)
#define SJA1124_LIN2C_ABORTREQ_SHIFT        (1)
#define SJA1124_LIN2C_ABORTREQ_MASK         (0x02u)
#define SJA1124_LIN2C_HEADERTRANSREQ_SHIFT  (0)
#define SJA1124_LIN2C_HEADERTRANSREQ_MASK   (0x01u)

/* register LIN2BI */
#define SJA1124_LIN2BI_IDENTIFIER_SHIFT     (0)
#define SJA1124_LIN2BI_IDENTIFIER_MASK      (0x3Fu)

/* register LIN2BC */
#define SJA1124_LIN2BC_DTFIELDLEN_SHIFT     (2)
#define SJA1124_LIN2BC_DTFIELDLEN_MASK      (0x1Cu)
#define SJA1124_LIN2BC_DIRECTION_SHIFT      (1)
#define SJA1124_LIN2BC_DIRECTION_MASK       (0x02u)
#define SJA1124_LIN2BC_CLASSICCS_SHIFT      (0)
#define SJA1124_LIN2BC_CLASSICCS_MASK       (0x01u)

/* register LIN2STATE */
#define SJA1124_LIN2STATE_RCVBUSY_SHIFT     (7)
#define SJA1124_LIN2STATE_RCVBUSY_MASK      (0x80u)
#define SJA1124_LIN2STATE_LINSTATE_SHIFT    (0)
#define SJA1124_LIN2STATE_LINSTATE_MASK     (0x0Fu)

/* register LIN2ES */
#define SJA1124_LIN2ES_STUCKZEROFLG_SHIFT   (7)
#define SJA1124_LIN2ES_STUCKZEROFLG_MASK    (0x80u)
#define SJA1124_LIN2ES_TIMEOUTERRFLG_SHIFT  (6)
#define SJA1124_LIN2ES_TIMEOUTERRFLG_MASK   (0x40u)
#define SJA1124_LIN2ES_BITERRFLG_SHIFT      (5)
#define SJA1124_LIN2ES_BITERRFLG_MASK       (0x20u)
#define SJA1124_LIN2ES_CSERRFLG_SHIFT       (4)
#define SJA1124_LIN2ES_CSERRFLG_MASK        (0x10u)
#define SJA1124_LIN2ES_FRAMEERRFLG_SHIFT    (0)
#define SJA1124_LIN2ES_FRAMEERRFLG_MASK     (0x01u)

/* register LIN2S */
#define SJA1124_LIN2S_RCVBUFFFULLFLG_SHIFT  (6)
#define SJA1124_LIN2S_RCVBUFFFULLFLG_MASK   (0x40u)
#define SJA1124_LIN2S_RCVCOMPFLG_SHIFT      (2)
#define SJA1124_LIN2S_RCVCOMPFLG_MASK       (0x04u)
#define SJA1124_LIN2S_TRANSCOMPFLG_SHIFT    (1)
#define SJA1124_LIN2S_TRANSCOMPFLG_MASK     (0x02u)

/* register LIN3CFG1 */
#define SJA1124_LIN3CFG1_CSCALCDIS_SHIFT    (7)
#define SJA1124_LIN3CFG1_CSCALCDIS_MASK     (0x80u)
#define SJA1124_LIN3CFG1_MASBREAKLEN_SHIFT  (3)
#define SJA1124_LIN3CFG1_MASBREAKLEN_MASK   (0x78u)
#define SJA1124_LIN3CFG1_SLEEP_SHIFT        (1)
#define SJA1124_LIN3CFG1_SLEEP_MASK         (0x02u)
#define SJA1124_LIN3CFG1_INIT_SHIFT         (0)
#define SJA1124_LIN3CFG1_INIT_MASK          (0x01u)

/* register LIN3CFG2 */
#define SJA1124_LIN3CFG2_TWOBITDELIM_SHIFT  (7)
#define SJA1124_LIN3CFG2_TWOBITDELIM_MASK   (0x80u)
#define SJA1124_LIN3CFG2_IDLEONBITERR_SHIFT (6)
#define SJA1124_LIN3CFG2_IDLEONBITERR_MASK  (0x40u)

/* register LIN3ITC */
#define SJA1124_LIN3ITC_IDLEONTIMEOUT_SHIFT (1)
#define SJA1124_LIN3ITC_IDLEONTIMEOUT_MASK  (0x02u)

/* register LIN3GC */
#define SJA1124_LIN3GC_STOPBITCONF_SHIFT    (1)
#define SJA1124_LIN3GC_STOPBITCONF_MASK     (0x02u)
#define SJA1124_LIN3GC_SOFTRESET_SHIFT      (0)
#define SJA1124_LIN3GC_SOFTRESET_MASK       (0x01u)

/* register LIN3RTC */
#define SJA1124_LIN3RTC_RESPTIMEOUT_SHIFT   (0)
#define SJA1124_LIN3RTC_RESPTIMEOUT_MASK    (0x0Fu)

/* register LIN3FR */
#define SJA1124_LIN3FR_FRACBAUDRATE_SHIFT   (0)
#define SJA1124_LIN3FR_FRACBAUDRATE_MASK    (0x0Fu)

/* register LIN3IE */
#define SJA1124_LIN3IE_STUCKZEROIEN_SHIFT   (7)
#define SJA1124_LIN3IE_STUCKZEROIEN_MASK    (0x80u)
#define SJA1124_LIN3IE_TIMEOUTIEN_SHIFT     (6)
#define SJA1124_LIN3IE_TIMEOUTIEN_MASK      (0x40u)
#define SJA1124_LIN3IE_BITERRIEN_SHIFT      (5)
#define SJA1124_LIN3IE_BITERRIEN_MASK       (0x20u)
#define SJA1124_LIN3IE_CSERRIEN_SHIFT       (4)
#define SJA1124_LIN3IE_CSERRIEN_MASK        (0x10u)
#define SJA1124_LIN3IE_DTRCVIEN_SHIFT       (2)
#define SJA1124_LIN3IE_DTRCVIEN_MASK        (0x04u)
#define SJA1124_LIN3IE_DTTRANSIEN_SHIFT     (1)
#define SJA1124_LIN3IE_DTTRANSIEN_MASK      (0x02u)
#define SJA1124_LIN3IE_FRAMEERRIEN_SHIFT    (0)
#define SJA1124_LIN3IE_FRAMEERRIEN_MASK     (0x01u)

/* register LIN3C */
#define SJA1124_LIN3C_WAKEUPREQ_SHIFT       (4)
#define SJA1124_LIN3C_WAKEUPREQ_MASK        (0x10u)
#define SJA1124_LIN3C_DTDISCARDREQ_SHIFT    (3)
#define SJA1124_LIN3C_DTDISCARDREQ_MASK     (0x08u)
#define SJA1124_LIN3C_ABORTREQ_SHIFT        (1)
#define SJA1124_LIN3C_ABORTREQ_MASK         (0x02u)
#define SJA1124_LIN3C_HEADERTRANSREQ_SHIFT  (0)
#define SJA1124_LIN3C_HEADERTRANSREQ_MASK   (0x01u)

/* register LIN3BI */
#define SJA1124_LIN3BI_IDENTIFIER_SHIFT     (0)
#define SJA1124_LIN3BI_IDENTIFIER_MASK      (0x3Fu)

/* register LIN3BC */
#define SJA1124_LIN3BC_DTFIELDLEN_SHIFT     (2)
#define SJA1124_LIN3BC_DTFIELDLEN_MASK      (0x1Cu)
#define SJA1124_LIN3BC_DIRECTION_SHIFT      (1)
#define SJA1124_LIN3BC_DIRECTION_MASK       (0x02u)
#define SJA1124_LIN3BC_CLASSICCS_SHIFT      (0)
#define SJA1124_LIN3BC_CLASSICCS_MASK       (0x01u)

/* register LIN3STATE */
#define SJA1124_LIN3STATE_RCVBUSY_SHIFT     (7)
#define SJA1124_LIN3STATE_RCVBUSY_MASK      (0x80u)
#define SJA1124_LIN3STATE_LINSTATE_SHIFT    (0)
#define SJA1124_LIN3STATE_LINSTATE_MASK     (0x0Fu)

/* register LIN3ES */
#define SJA1124_LIN3ES_STUCKZEROFLG_SHIFT   (7)
#define SJA1124_LIN3ES_STUCKZEROFLG_MASK    (0x80u)
#define SJA1124_LIN3ES_TIMEOUTERRFLG_SHIFT  (6)
#define SJA1124_LIN3ES_TIMEOUTERRFLG_MASK   (0x40u)
#define SJA1124_LIN3ES_BITERRFLG_SHIFT      (5)
#define SJA1124_LIN3ES_BITERRFLG_MASK       (0x20u)
#define SJA1124_LIN3ES_CSERRFLG_SHIFT       (4)
#define SJA1124_LIN3ES_CSERRFLG_MASK        (0x10u)
#define SJA1124_LIN3ES_FRAMEERRFLG_SHIFT    (0)
#define SJA1124_LIN3ES_FRAMEERRFLG_MASK     (0x01u)

/* register LIN3S */
#define SJA1124_LIN3S_RCVBUFFFULLFLG_SHIFT  (6)
#define SJA1124_LIN3S_RCVBUFFFULLFLG_MASK   (0x40u)
#define SJA1124_LIN3S_RCVCOMPFLG_SHIFT      (2)
#define SJA1124_LIN3S_RCVCOMPFLG_MASK       (0x04u)
#define SJA1124_LIN3S_TRANSCOMPFLG_SHIFT    (1)
#define SJA1124_LIN3S_TRANSCOMPFLG_MASK     (0x02u)

/* register LIN4CFG1 */
#define SJA1124_LIN4CFG1_CSCALCDIS_SHIFT    (7)
#define SJA1124_LIN4CFG1_CSCALCDIS_MASK     (0x80u)
#define SJA1124_LIN4CFG1_MASBREAKLEN_SHIFT  (3)
#define SJA1124_LIN4CFG1_MASBREAKLEN_MASK   (0x78u)
#define SJA1124_LIN4CFG1_SLEEP_SHIFT        (1)
#define SJA1124_LIN4CFG1_SLEEP_MASK         (0x02u)
#define SJA1124_LIN4CFG1_INIT_SHIFT         (0)
#define SJA1124_LIN4CFG1_INIT_MASK          (0x01u)

/* register LIN4CFG2 */
#define SJA1124_LIN4CFG2_TWOBITDELIM_SHIFT  (7)
#define SJA1124_LIN4CFG2_TWOBITDELIM_MASK   (0x80u)
#define SJA1124_LIN4CFG2_IDLEONBITERR_SHIFT (6)
#define SJA1124_LIN4CFG2_IDLEONBITERR_MASK  (0x40u)

/* register LIN4ITC */
#define SJA1124_LIN4ITC_IDLEONTIMEOUT_SHIFT (1)
#define SJA1124_LIN4ITC_IDLEONTIMEOUT_MASK  (0x02u)

/* register LIN4GC */
#define SJA1124_LIN4GC_STOPBITCONF_SHIFT    (1)
#define SJA1124_LIN4GC_STOPBITCONF_MASK     (0x02u)
#define SJA1124_LIN4GC_SOFTRESET_SHIFT      (0)
#define SJA1124_LIN4GC_SOFTRESET_MASK       (0x01u)

/* register LIN4RTC */
#define SJA1124_LIN4RTC_RESPTIMEOUT_SHIFT   (0)
#define SJA1124_LIN4RTC_RESPTIMEOUT_MASK    (0x0Fu)

/* register LIN4FR */
#define SJA1124_LIN4FR_FRACBAUDRATE_SHIFT   (0)
#define SJA1124_LIN4FR_FRACBAUDRATE_MASK    (0x0Fu)

/* register LIN4IE */
#define SJA1124_LIN4IE_STUCKZEROIEN_SHIFT   (7)
#define SJA1124_LIN4IE_STUCKZEROIEN_MASK    (0x80u)
#define SJA1124_LIN4IE_TIMEOUTIEN_SHIFT     (6)
#define SJA1124_LIN4IE_TIMEOUTIEN_MASK      (0x40u)
#define SJA1124_LIN4IE_BITERRIEN_SHIFT      (5)
#define SJA1124_LIN4IE_BITERRIEN_MASK       (0x20u)
#define SJA1124_LIN4IE_CSERRIEN_SHIFT       (4)
#define SJA1124_LIN4IE_CSERRIEN_MASK        (0x10u)
#define SJA1124_LIN4IE_DTRCVIEN_SHIFT       (2)
#define SJA1124_LIN4IE_DTRCVIEN_MASK        (0x04u)
#define SJA1124_LIN4IE_DTTRANSIEN_SHIFT     (1)
#define SJA1124_LIN4IE_DTTRANSIEN_MASK      (0x02u)
#define SJA1124_LIN4IE_FRAMEERRIEN_SHIFT    (0)
#define SJA1124_LIN4IE_FRAMEERRIEN_MASK     (0x01u)

/* register LIN4C */
#define SJA1124_LIN4C_WAKEUPREQ_SHIFT       (4)
#define SJA1124_LIN4C_WAKEUPREQ_MASK        (0x10u)
#define SJA1124_LIN4C_DTDISCARDREQ_SHIFT    (3)
#define SJA1124_LIN4C_DTDISCARDREQ_MASK     (0x08u)
#define SJA1124_LIN4C_ABORTREQ_SHIFT        (1)
#define SJA1124_LIN4C_ABORTREQ_MASK         (0x02u)
#define SJA1124_LIN4C_HEADERTRANSREQ_SHIFT  (0)
#define SJA1124_LIN4C_HEADERTRANSREQ_MASK   (0x01u)

/* register LIN4BI */
#define SJA1124_LIN4BI_IDENTIFIER_SHIFT     (0)
#define SJA1124_LIN4BI_IDENTIFIER_MASK      (0x3Fu)

/* register LIN4BC */
#define SJA1124_LIN4BC_DTFIELDLEN_SHIFT     (2)
#define SJA1124_LIN4BC_DTFIELDLEN_MASK      (0x1Cu)
#define SJA1124_LIN4BC_DIRECTION_SHIFT      (1)
#define SJA1124_LIN4BC_DIRECTION_MASK       (0x02u)
#define SJA1124_LIN4BC_CLASSICCS_SHIFT      (0)
#define SJA1124_LIN4BC_CLASSICCS_MASK       (0x01u)

/* register LIN4STATE */
#define SJA1124_LIN4STATE_RCVBUSY_SHIFT     (7)
#define SJA1124_LIN4STATE_RCVBUSY_MASK      (0x80u)
#define SJA1124_LIN4STATE_LINSTATE_SHIFT    (0)
#define SJA1124_LIN4STATE_LINSTATE_MASK     (0x0Fu)

/* register LIN4ES */
#define SJA1124_LIN4ES_STUCKZEROFLG_SHIFT   (7)
#define SJA1124_LIN4ES_STUCKZEROFLG_MASK    (0x80u)
#define SJA1124_LIN4ES_TIMEOUTERRFLG_SHIFT  (6)
#define SJA1124_LIN4ES_TIMEOUTERRFLG_MASK   (0x40u)
#define SJA1124_LIN4ES_BITERRFLG_SHIFT      (5)
#define SJA1124_LIN4ES_BITERRFLG_MASK       (0x20u)
#define SJA1124_LIN4ES_CSERRFLG_SHIFT       (4)
#define SJA1124_LIN4ES_CSERRFLG_MASK        (0x10u)
#define SJA1124_LIN4ES_FRAMEERRFLG_SHIFT    (0)
#define SJA1124_LIN4ES_FRAMEERRFLG_MASK     (0x01u)

/* register LIN4S */
#define SJA1124_LIN4S_RCVBUFFFULLFLG_SHIFT  (6)
#define SJA1124_LIN4S_RCVBUFFFULLFLG_MASK   (0x40u)
#define SJA1124_LIN4S_RCVCOMPFLG_SHIFT      (2)
#define SJA1124_LIN4S_RCVCOMPFLG_MASK       (0x04u)
#define SJA1124_LIN4S_TRANSCOMPFLG_SHIFT    (1)
#define SJA1124_LIN4S_TRANSCOMPFLG_MASK     (0x02u)

/* register MCFG */
#define SJA1124_MCFG_CHAN4TERMCONF_SHIFT    (6)
#define SJA1124_MCFG_CHAN4TERMCONF_MASK     (0xC0u)
#define SJA1124_MCFG_CHAN3TERMCONF_SHIFT    (4)
#define SJA1124_MCFG_CHAN3TERMCONF_MASK     (0x30u)
#define SJA1124_MCFG_CHAN2TERMCONF_SHIFT    (2)
#define SJA1124_MCFG_CHAN2TERMCONF_MASK     (0x0Cu)
#define SJA1124_MCFG_CHAN1TERMCONF_SHIFT    (0)
#define SJA1124_MCFG_CHAN1TERMCONF_MASK     (0x03u)

/* register MMTPS */
#define SJA1124_MMTPS_FACTRESTLOCK_SHIFT    (7)
#define SJA1124_MMTPS_FACTRESTLOCK_MASK     (0x80u)
#define SJA1124_MMTPS_FACTRESTSTAT_SHIFT    (6)
#define SJA1124_MMTPS_FACTRESTSTAT_MASK     (0x40u)
#define SJA1124_MMTPS_WRTCNTSTAT_SHIFT      (0)
#define SJA1124_MMTPS_WRTCNTSTAT_MASK       (0x3Fu)

/* register ID */
#define SJA1124_ID_IDENTIFICATION_SHIFT     (0)
#define SJA1124_ID_IDENTIFICATION_MASK      (0x3Fu)

#endif /* SJA1124_H_ */