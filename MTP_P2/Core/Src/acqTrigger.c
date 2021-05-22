/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : acqTrigger.c
* Author             : MCD Tools Development
* Version            : V1.0
* Date               : 03/06/2011
* Description        : This file provides trigger mechanism for software data
*                      trace from the application.
*                      This module requires between 1K and 1.5K of program, and
*                      about 50 bytes of RAM (depends on compiler). It may be
*                      completely disconnected by suppressing the "USING_TRIGGER"
*                      compilation switch.
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include "acqTrigger.h"

#ifdef USING_TRIGGER  // Modify dataAcq.h in order to get rid of trigger

// Constants for g_internalTrigState
#define WAITING_LEVEL_BEFORE 0
#define WAITING_LEVEL_AFTER  1
#define WAITING_BUFFER_EMPTY 2
#define INTERNAL_STATE_STOPPED 3
#define INTERNAL_STATE_TRIGGED 4

// Constants for g_internalTrigMode
#define MODE_NO_TRIGGER           0 // no trigger (always trace)
#define MODE_TRIGGER_NO_PRETRIG   1
#define MODE_TRIGGER_WITH_PRETRIG 2

static volatile TraceHeaderTriggerT *g_pTriggerConf = (TraceHeaderTriggerT *)0;

static int g_internalTrigState;
static int g_internalTrigMode;
static int g_internalType;
static int g_binternalIgnoreTrig;

static unsigned char g_threshold_u8;
static signed char g_threshold_s8;
static unsigned short g_threshold_u16;
static signed short g_threshold_s16;
static unsigned long g_threshold_u32;
static signed long g_threshold_s32;
#ifdef USING_TRIGGER_ON_FLOAT
static float g_threshold_float;
#endif
#ifdef USING_TRIGGER_ON_DOUBLE
static float g_threshold_double;
#endif

// Parameter for finite acquisition mode
static int g_stopAfterRecords=0;
// Nb of calls to AcqEvaluateTrigger that returned 1 since last Start cmd
// (used in the mode "stop after N records")
static int g_nbRecords=0;

// Flag for internal buffer overflow
static int g_overflow=0;

static int bAutoRearmAfterOVF=0;

// Forward declarations
int valAboveThreshold(void);
int valBelowThreshold(void);
int valEqualsThreshold(void);
int evaluateEnd(void);
static int manageTrigEvent(void);

void AutoRearmTrigger(void) {
	bAutoRearmAfterOVF=0;
	g_internalTrigState = WAITING_LEVEL_BEFORE;
	g_pTriggerConf->state = SNP_TRC_TRIGGER_STARTED;
	g_nbRecords = 0;
}

void SetTriggerOverflow(int overflow) {
	g_overflow = overflow;
	if( (overflow == 0) && (bAutoRearmAfterOVF==1) ) {
		if ( g_binternalIgnoreTrig ) {
			// Do not immediately rearm the trigger in that case, but wait for the buffer to be completely empty
			g_internalTrigState = WAITING_BUFFER_EMPTY;
			g_pTriggerConf->state = SNP_TRC_TRIGGER_STARTED;
		}
		else {
			// It's now time to rearm the trigger after OVF stop
			AutoRearmTrigger();
		}
	}
}

// Inform the trigger module that a new record is being traced (useful for stop after N)
void AddingNewRecord(void) {
  g_nbRecords++;
}

// Interpret the trigger part of the snapshot trace header.
// To call at least once by recording session
void AcqConfigureTrigger(volatile TraceHeaderTriggerT *pTrigConfig) {
	g_pTriggerConf = pTrigConfig;

	g_internalTrigState = INTERNAL_STATE_STOPPED;
	// Process some fields once and for all for better runtime performance
	// (type conversion)
	if( pTrigConfig->mode == 0 ) {
		// No trigger configured (= immediate start + never stop)
		g_internalTrigMode = MODE_NO_TRIGGER;
	} else {
		if( pTrigConfig->nPreTrig == 0 ) {
			g_internalTrigMode = MODE_TRIGGER_NO_PRETRIG;
		} else {
			g_internalTrigMode = MODE_TRIGGER_WITH_PRETRIG;
		}
	}
	if( pTrigConfig->bIgnoreTrig == 1 ) {
		g_binternalIgnoreTrig = 1;
	}
	else {
		g_binternalIgnoreTrig = 0;
	}

	if( pTrigConfig->accessType == SNP_TRC_DATA_ACCESS_8BIT ) {
		g_internalType = SNP_TRC_DATA_ACCESS_8BIT;
		g_threshold_s8 = (signed char)g_pTriggerConf->threshold;
	}
	else if( pTrigConfig->accessType == SNP_TRC_DATA_ACCESS_16BIT ) {
		g_internalType = SNP_TRC_DATA_ACCESS_16BIT;
		g_threshold_s16 = (signed short)g_pTriggerConf->threshold;
#ifdef STM32F0XX
		// On Cortex M0, the LDRH instruction requires to be 16-bits aligned
		g_pTriggerConf->address &= 0xFFFFFFFEUL;
#endif
	}
	else if( pTrigConfig->accessType == SNP_TRC_DATA_ACCESS_32BIT ) {
		g_internalType = SNP_TRC_DATA_ACCESS_32BIT;
		g_threshold_s32 = (signed long)g_pTriggerConf->threshold;
#ifdef STM32F0XX
		// On Cortex M0, the LDR instruction requires to be 32-bits aligned
		g_pTriggerConf->address &= 0xFFFFFFFCUL;
#endif
	}
	else if( pTrigConfig->accessType == SNP_TRC_DATA_ACCESS_TRIG_U8 ) {
		g_internalType = SNP_TRC_DATA_ACCESS_TRIG_U8;
		g_threshold_u8 = (unsigned char)g_pTriggerConf->threshold;
	}
	else if( pTrigConfig->accessType == SNP_TRC_DATA_ACCESS_TRIG_U16 ) {
		g_internalType = SNP_TRC_DATA_ACCESS_TRIG_U16;
		g_threshold_u16 = (unsigned short)g_pTriggerConf->threshold;
#ifdef STM32F0XX
		// On Cortex M0, the LDRH instruction requires to be 16-bits aligned
		g_pTriggerConf->address &= 0xFFFFFFFEUL;
#endif
	}
#ifdef USING_TRIGGER_ON_FLOAT
	else if( pTrigConfig->accessType == SNP_TRC_DATA_ACCESS_TRIG_FLOAT ) {
		g_internalType = SNP_TRC_DATA_ACCESS_TRIG_FLOAT;
		g_threshold_float = *(float *)&g_pTriggerConf->threshold;
#ifdef STM32F0XX
		// On Cortex M0, the LDR instruction requires to be 32-bits aligned
		g_pTriggerConf->address &= 0xFFFFFFFCUL;
#endif
	}
#endif
#ifdef USING_TRIGGER_ON_DOUBLE
	else if( pTrigConfig->accessType == SNP_TRC_DATA_ACCESS_TRIG_DOUBLE ) {
		g_internalType = SNP_TRC_DATA_ACCESS_TRIG_DOUBLE;
		g_threshold_double = *(double *)&g_pTriggerConf->thresholdDouble;
		// Compilers commonly use LDRD instruction for loading 2 words. Which require
		// a 32-bits aligned address (Cortex M0 and M3), otherwise: Hard Fault.
		// Force the alignment for robustness purpose
		g_pTriggerConf->address &= 0xFFFFFFFCUL;
	}
#endif
	else {
		g_internalType = SNP_TRC_DATA_ACCESS_TRIG_U32;
		g_threshold_u32 = (unsigned long)g_pTriggerConf->threshold;
#ifdef STM32F0XX
		// On Cortex M0, the LDR instruction requires to be 32-bits aligned
		g_pTriggerConf->address &= 0xFFFFFFFCUL;
#endif
	}

	if( pTrigConfig->mode & SNP_TRC_TRIGGER_STOP_N_REC ) {
		g_stopAfterRecords = (int)g_pTriggerConf->stopParam;
	}
}

// Common code used twice in AcqEvaluateTrigger
static int manageTrigEvent(void)
{
	if(g_internalTrigMode != MODE_TRIGGER_WITH_PRETRIG) {
		g_pTriggerConf->state = SNP_TRC_TRIGGER_TRIGGED;
		g_internalTrigState = INTERNAL_STATE_TRIGGED;
		return TRIG_EVAL_POST_REC;
	} else {
		// In pretrig mode the SNP_TRC_TRIGGER_TRIGGED is used by the host as
		// trigger signal; so must be set only after read/write pointers are OK
		// in the shared header
		g_internalTrigState = INTERNAL_STATE_TRIGGED;
		return TRIG_EVAL_EVENT;
	}
}

// Main trigger state machine, to call periodically. Returns 1 when:
// - no trigger is configured
// - a trigger was configured, started and the condition is hit.
// Returns 0 otherwise.
int AcqEvaluateTrigger(void) {
	if( g_internalTrigMode == MODE_NO_TRIGGER ) {
		// No trigger configured => trace all
		return TRIG_EVAL_POST_REC;
	}
	if( g_internalTrigState == INTERNAL_STATE_TRIGGED ) {
		// Event already trigged: continue until end event (stop command, ...)
		return evaluateEnd();
	}
	if( g_internalTrigState == INTERNAL_STATE_STOPPED ) {
/*		if( (g_pTriggerConf->mode & SNP_TRC_TRIGGER_START_MASK) == SNP_TRC_TRIGGER_START_IMMEDIATE ) {
			g_pTriggerConf->state = SNP_TRC_TRIGGER_TRIGGED;
			g_internalTrigState = INTERNAL_STATE_TRIGGED;
			g_nbRecords = 1;
			return TRIG_EVAL_POST_REC;
		}*/
		// Check if start asked
		if( g_pTriggerConf->cmd & SNP_TRC_TRIGGER_START ) {
			g_nbRecords = 0;
			if( (g_pTriggerConf->mode & SNP_TRC_TRIGGER_START_MASK) == SNP_TRC_TRIGGER_START_IMMEDIATE ) {
				g_pTriggerConf->state = SNP_TRC_TRIGGER_TRIGGED;
				g_internalTrigState = INTERNAL_STATE_TRIGGED;
				return TRIG_EVAL_POST_REC;
			}
			// Init state variables
			if(g_internalTrigMode == MODE_TRIGGER_NO_PRETRIG) {
				g_internalTrigState = WAITING_LEVEL_BEFORE;
				g_pTriggerConf->state = SNP_TRC_TRIGGER_STARTED;
			} else {
				g_internalTrigState = WAITING_BUFFER_EMPTY;
				g_pTriggerConf->state = SNP_TRC_TRIGGER_STARTED;
			}
		} else {
			return TRIG_EVAL_NO_REC;
		}
	}
	if( g_pTriggerConf->state == SNP_TRC_TRIGGER_STARTED ) {
		// Check if stop asked
		if( g_pTriggerConf->cmd & SNP_TRC_TRIGGER_STOP ) {
			g_pTriggerConf->state = SNP_TRC_TRIGGER_STOPPED;
			g_internalTrigState = INTERNAL_STATE_STOPPED;
			return TRIG_EVAL_NO_REC;
		}

		if( g_internalTrigState == WAITING_BUFFER_EMPTY ) {
			if( IsBufferEmpty() == 1 ) {
				// Buffer is now empty; we can restart pre-triggering
				AutoRearmTrigger();
				// In this case we will return TRIG_EVAL_PRE2_REC; do not do it immediately
				// because we should evaluate the first trigger level
			} else {
				// Buffer is still not empty; must wait for it to be flushed
				return TRIG_EVAL_PRE1_REC;
			}
		}
	
		// Check against trigger condition: edge detected as 2 levels
		if( g_internalTrigState == WAITING_LEVEL_BEFORE ) {
			if( g_pTriggerConf->mode & SNP_TRC_TRIGGER_FALLING ) {
				// For falling edge detection, the level before must be above the threshold
				if( valAboveThreshold() ) {
					g_internalTrigState = WAITING_LEVEL_AFTER;
				}
			} else {
				// For rising edge detection, the level before must be below the threshold
				if( valBelowThreshold() ) {
					g_internalTrigState = WAITING_LEVEL_AFTER;
				}
			}
		}
		if( g_internalTrigState == WAITING_LEVEL_AFTER ) {
			if( g_pTriggerConf->mode & SNP_TRC_TRIGGER_FALLING ) {
				// For falling edge detection, the level after must be below the threshold
				if( valBelowThreshold() || valEqualsThreshold() ) {
					// Event occurred
					return manageTrigEvent();
				}
			} else {
				// For rising edge detection, the level after must be above the threshold
				if( valAboveThreshold() || valEqualsThreshold() ) {
					// Event occurred
					return manageTrigEvent();
				}
			}
		}
	}
	// Nothing to trace or pre-triggering
	if(g_internalTrigMode == MODE_TRIGGER_WITH_PRETRIG) {
		return TRIG_EVAL_PRE2_REC;
	} else {
		return TRIG_EVAL_NO_REC;
	}
}

int valAboveThreshold(void) {
	switch(g_internalType) {
		case SNP_TRC_DATA_ACCESS_8BIT:
			return ( (*(_FAR_DATA_ signed char *)g_pTriggerConf->address)>g_threshold_s8)?1:0;
		case SNP_TRC_DATA_ACCESS_16BIT:
			return ( (*(_FAR_DATA_ signed short *)g_pTriggerConf->address)>g_threshold_s16)?1:0;
		case SNP_TRC_DATA_ACCESS_32BIT:
			return ( (*(_FAR_DATA_ signed long *)g_pTriggerConf->address)>g_threshold_s32)?1:0;
		case SNP_TRC_DATA_ACCESS_TRIG_U8:
			return ( (*(_FAR_DATA_ unsigned char *)g_pTriggerConf->address)>g_threshold_u8)?1:0;
		case SNP_TRC_DATA_ACCESS_TRIG_U16:
			return ( (*(_FAR_DATA_ unsigned short *)g_pTriggerConf->address)>g_threshold_u16)?1:0;
		case SNP_TRC_DATA_ACCESS_TRIG_U32:
			return ( (*(_FAR_DATA_ unsigned long *)g_pTriggerConf->address)>g_threshold_u32)?1:0;
#ifdef USING_TRIGGER_ON_FLOAT
		case SNP_TRC_DATA_ACCESS_TRIG_FLOAT:
			return ( (*(_FAR_DATA_ float *)g_pTriggerConf->address)>g_threshold_float)?1:0;
#endif
#ifdef USING_TRIGGER_ON_DOUBLE
		case SNP_TRC_DATA_ACCESS_TRIG_DOUBLE:
			return ( (*(_FAR_DATA_ double *)g_pTriggerConf->address)>g_threshold_double)?1:0;
#endif
		default:
			break;
	}
	return 0;
}

int valBelowThreshold(void) {
	switch(g_internalType) {
		case SNP_TRC_DATA_ACCESS_8BIT:
			return ( (*(_FAR_DATA_ signed char *)g_pTriggerConf->address)<g_threshold_s8)?1:0;
		case SNP_TRC_DATA_ACCESS_16BIT:
			return ( (*(_FAR_DATA_ signed short *)g_pTriggerConf->address)<g_threshold_s16)?1:0;
		case SNP_TRC_DATA_ACCESS_32BIT:
			return ( (*(_FAR_DATA_ signed long *)g_pTriggerConf->address)<g_threshold_s32)?1:0;
		case SNP_TRC_DATA_ACCESS_TRIG_U8:
			return ( (*(_FAR_DATA_ unsigned char *)g_pTriggerConf->address)<g_threshold_u8)?1:0;
		case SNP_TRC_DATA_ACCESS_TRIG_U16:
			return ( (*(_FAR_DATA_ unsigned short *)g_pTriggerConf->address)<g_threshold_u16)?1:0;
		case SNP_TRC_DATA_ACCESS_TRIG_U32:
		  return ( (*(_FAR_DATA_ unsigned long *)g_pTriggerConf->address)<g_threshold_u32)?1:0;
#ifdef USING_TRIGGER_ON_FLOAT
		case SNP_TRC_DATA_ACCESS_TRIG_FLOAT:
			return ( (*(_FAR_DATA_ float *)g_pTriggerConf->address)<g_threshold_float)?1:0;
#endif
#ifdef USING_TRIGGER_ON_DOUBLE
		case SNP_TRC_DATA_ACCESS_TRIG_DOUBLE:
			return ( (*(_FAR_DATA_ double *)g_pTriggerConf->address)<g_threshold_double)?1:0;
#endif
		default:
			break;
	}
	return 0;
}

int valEqualsThreshold(void) {
	switch(g_internalType) {
		case SNP_TRC_DATA_ACCESS_8BIT:
			return ( (*(_FAR_DATA_ signed char *)g_pTriggerConf->address)==g_threshold_s8)?1:0;
		case SNP_TRC_DATA_ACCESS_16BIT:
			return ( (*(_FAR_DATA_ signed short *)g_pTriggerConf->address)==g_threshold_s16)?1:0;
		case SNP_TRC_DATA_ACCESS_32BIT:
			return ( (*(_FAR_DATA_ signed long *)g_pTriggerConf->address)==g_threshold_s32)?1:0;
		case SNP_TRC_DATA_ACCESS_TRIG_U8:
			return ( (*(_FAR_DATA_ unsigned char *)g_pTriggerConf->address)==g_threshold_u8)?1:0;
		case SNP_TRC_DATA_ACCESS_TRIG_U16:
			return ( (*(_FAR_DATA_ unsigned short *)g_pTriggerConf->address)==g_threshold_u16)?1:0;
		case SNP_TRC_DATA_ACCESS_TRIG_U32:
		  return ( (*(_FAR_DATA_ unsigned long *)g_pTriggerConf->address)==g_threshold_u32)?1:0;
#ifdef USING_TRIGGER_ON_FLOAT
		case SNP_TRC_DATA_ACCESS_TRIG_FLOAT:
			return ( (*(_FAR_DATA_ float *)g_pTriggerConf->address)==g_threshold_float)?1:0;
#endif
#ifdef USING_TRIGGER_ON_DOUBLE
		case SNP_TRC_DATA_ACCESS_TRIG_DOUBLE:
			return ( (*(_FAR_DATA_ double *)g_pTriggerConf->address)==g_threshold_double)?1:0;
#endif
		default:
			break;
	}
	return 0;
}

// To call while in TRIGGED state. Check against end conditions.
// Returns TRIG_EVAL_POST_REC if still in TRIGGED state; TRIG_EVAL_NO_REC if STOPPED.
int evaluateEnd(void) {
	// Check if stop asked
	if( g_pTriggerConf->cmd & SNP_TRC_TRIGGER_STOP ) {
		g_pTriggerConf->state = SNP_TRC_TRIGGER_STOPPED;
		g_internalTrigState = INTERNAL_STATE_STOPPED;
		return TRIG_EVAL_NO_REC;
	}
	if( (g_pTriggerConf->mode & SNP_TRC_TRIGGER_STOP_MASK) == 0 ) {
		// Never stop
		return TRIG_EVAL_POST_REC;
	}
	// Check against other stop conditions
	if( (g_pTriggerConf->mode & SNP_TRC_TRIGGER_STOP_OVF) && (g_overflow!=1) ) {
		return TRIG_EVAL_POST_REC;
	}
	if( (g_pTriggerConf->mode & SNP_TRC_TRIGGER_STOP_N_REC) && (g_nbRecords < g_stopAfterRecords) ) {
		return TRIG_EVAL_POST_REC;
	}
	// If here, a stop condition was faced. Check if autorestart
	if( (g_pTriggerConf->mode & SNP_TRC_TRIGGER_AUTO_RESTART) && (g_internalTrigMode != MODE_TRIGGER_WITH_PRETRIG) ) {
		// The auto-restart is not possible in pretrig mode, as the field g_pTriggerConf->state
		// is used for host synchronization; one must wait for host ACK before restarting
		// the pre-triggering.
		// Init state variables
		if( g_pTriggerConf->mode & SNP_TRC_TRIGGER_STOP_N_REC ) {
			if ( g_binternalIgnoreTrig ) {
				g_internalTrigState = WAITING_BUFFER_EMPTY;
			}
			else {
				// Immediately rearm the trigger in this case
				g_internalTrigState = WAITING_LEVEL_BEFORE;
			}
			g_pTriggerConf->state = SNP_TRC_TRIGGER_STARTED;
			g_nbRecords = 0;
		} else {
			// In case of OVF stop, one should stop the acquisition and wait for
			// the end of OVF before rearming
			g_pTriggerConf->cmd = 0;
			g_pTriggerConf->state = SNP_TRC_TRIGGER_STOPPED;
			g_internalTrigState = INTERNAL_STATE_STOPPED;
			bAutoRearmAfterOVF = 1;
		}
	} else {
		// In that case a user action is required to start again => clear the previous command,
		// but without clearing SNP_TRC_TRIGGER_TRIGGED (if set), because required for
		// for host synchro in preTrig mode; the state will be update after ACK from host (HostReadEvent)
		g_pTriggerConf->cmd = 0;
		g_pTriggerConf->state = g_pTriggerConf->state & ((TraceHeaderFieldT)0xFFFFFFFE); // Clear SNP_TRC_TRIGGER_STARTED
		g_internalTrigState = INTERNAL_STATE_STOPPED;
//		g_pTriggerConf->state = SNP_TRC_TRIGGER_STOPPED; // Moved into HostReadEvent
	}
	return TRIG_EVAL_NO_REC;
}

// Inform the trigger module that the host has read some data
void HostReadEvent(void) {
	if( (g_internalTrigState == INTERNAL_STATE_STOPPED) && (g_pTriggerConf->state == SNP_TRC_TRIGGER_TRIGGED) && (g_pTriggerConf->cmd == 0) ) {
		// Update the trigger state now that we are sure that the host has seen the TRIGGED state
		g_pTriggerConf->state = SNP_TRC_TRIGGER_STOPPED;
	}
}


#endif // USING_TRIGGER
