/**
\file    novatel.h
\brief   GNSS core 'c' function library: decoding/encoding NovAtel data.
\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2006-08-04

\b REFERENCES \n
- NovAtel OEM4 Command and Log Reference. www.novatel.com.

\b "LICENSE INFORMATION" \n
Copyright (c) 2007, refer to 'author' doxygen tags \n
All rights reserved. \n

Redistribution and use in source and binary forms, with or without
modification, are permitted provided the following conditions are met: \n

- Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer. \n
- Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution. \n
- The name(s) of the contributor(s) may not be used to endorse or promote 
  products derived from this software without specific prior written 
  permission. \n

THIS SOFTWARE IS PROVIDED BY THE CONTRIBUTORS ``AS IS'' AND ANY EXPRESS 
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.
*/

#ifndef _C_NOVATEL_H_
#define _C_NOVATEL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "basictypes.h"



/// brief  This enumeration is for the supported NovAtel OEM3 message types.
typedef enum
{
  NOVATELOEM3_POSB = 1,  //!< message  1, byte count = 88
  NOVATELOEM3_TM1B = 3,  //!< message  3, byte count = 52
  NOVATELOEM3_IONB = 16, //!< message 16, byte count = 76
  NOVATELOEM3_UTCB = 17, //!< message 17, byte count = 52
  NOVATELOEM3_ALMB = 18, //!< message 18, byte count = 120  
  NOVATELOEM3_REPB = 14, //!< message 14, byte count = 108
  NOVATELOEM3_RGEB = 32, //!< message 32, byte count = 32 + obs * 44   
  NOVATELOEM3_MESSAGETYPE_UNKNOWN
} NOVATELOEM3_enumMessageType;

/// brief  This enumeration is for the NovAtel OEM3 solution status indicator.
typedef enum
{
  NOVATELOEM3_SOLUTIONSTATUS_OK = 0,
  NOVATELOEM3_SOLUTIONSTATUS_INSUFFICIENT_OBS = 1,
  NOVATELOEM3_SOLUTIONSTATUS_NO_CONVERGENCE = 2,
  NOVATELOEM3_SOLUTIONSTATUS_SINGULAR = 3,
  NOVATELOEM3_SOLUTIONSTATUS_EXCEEDED_COVAR_TRACE = 4,
  NOVATELOEM3_SOLUTIONSTATUS_EXCEEDED_TEST_DIST = 5,
  NOVATELOEM3_SOLUTIONSTATUS_NOT_CONVERGED_COLD_START = 6,
  NOVATELOEM3_SOLUTIONSTATUS_EXCEEDED_COCOM = 7,
  NOVATELOEM3_SOLUTIONSTATUS_RESERVED
} NOVATELOEM3_enumSolutionStatus;

typedef struct
{
  unsigned isAntenna_OK:1; //!< This bit will be set to 1 if the antenna connection is not drawing excessive current.
  unsigned isL1PLL_OK:1;   //!< When the L1 RF downconverter passes self-test, the bit will be set to 1.
  unsigned isRAM_OK:1;     //!< When this bit is set to 1, the receiver RAM has passed the self-test requirements.
  unsigned isROM_OK:1;     //!< When this bit is set to 1, the receiver NVM test has passed the self test requirements.
  unsigned isDSP_OK:1;     //!< This bit will be set to 1 when the digital signal processors (DSP) have passed the self-test requirements.
  unsigned isL1AGC_OK:1;   //!< When set to 1, the L1AGC circuits are operating within normal range of control. Can be indicative of jamming.
  unsigned isCOM1_OK:1;    //!< When set to 1, the COM1 UART has passed the self-test requirements.
  unsigned isCOM2_OK:1;    //!< When set to 1, the COM2 UART has passed the self-test requirements.
  unsigned isWeekNotSet:1; //!< When set to 1, the week is not set. Clock jumps probable.
  unsigned noCoarseTime:1; //!< No coarse time. Clock jumps probable.
  unsigned noFineTime:1;   //!< No fine time. Clock jumps probable.
  unsigned isL1JammerDetected:1;      //!< Indicates if an L1 jammer is detected.
  unsigned isCOM1BufferOverrun:1;     //!< Indicates if the COM1 buffer is overrun. Too much data requested.
  unsigned isCOM2BufferOverrun:1;     //!< Indicates if the COM2 buffer is overrun. Too much data requested.
  unsigned isConsoleBufferOverrun:1;  //!< Indicates if the console buffer is overrun.
  unsigned isCPUOverloaded:1;         //!< Indicates if the CPU is being over-taxed.
  unsigned isAlmanacSavedInNVM:1;     //!< Indicates if the almanac is saved in non-volatile memory (12 channel cards only).
  unsigned isL2AGC_OK:1;              //!< When set to 1, the L2 AGC circuits are operating within normal range of control.
  unsigned isL2JammerDetected:1;      //!< Indicates if a jammer on L2 is detected.
  unsigned isL2PLL_OK:1;              //!< When the L2 RF downconverter passes self-test, the bit will be set to 1.
  unsigned OCXOPLL:1;                 //!< When an external oscillator is connected and the OCXOPLL bit passes self-test, the bit will be set to 1.
  unsigned SavedAlmanacNeedsUpdate:1; //!< When the almanac received is newer than the one currently stored in NVM (non-volatile memory), the bit will be set to 1.
  unsigned isAlmanacInvalid:1;        //!< Indicates if the almanac is valid.
  unsigned isPositionInvalid:1;       //!< Indicates if the position fis is valid.
  unsigned isPositionFixed:1;         //!< Indicates if a fix position command has been accepted.
  unsigned isClockModelInvalid:1;     //!< Indicates if the clock model has not stabilized.
  unsigned isClockSteeringDisabled:1; //!< Indicates if the clockadjust disable command has been accepted.
  unsigned reserved:5;                //!< reserved.
} NOVATELOEM3_bitfieldSelfTestStatus;

typedef struct
{
  unsigned State:4;                           //!< 0-L1 Idle, 1-L1 Sky Search, 2-L1 Wide FLL, 3-L1 FLL, 4-L1 PLL, 5-L1 Reacq, 6-L1 Steering, 7-L1 FLL, 8-L2 Idle, 9-L2 Pcode Alignment, 10-L2 Search, 11-L2 PLL
  unsigned Number:5;                          //!< 0-31
  unsigned isPhaseLocked:1;                   //!< A boolean to indicate if this channel phase locked. 
  unsigned isParityKnown:1;                   //!< A boolean to indicate if this channel has known parity.
  unsigned isCodeLocked:1;                    //!< A boolean to indicate if this channel is code locked.
  unsigned CorrelatorSpacing:3;               //!< 0-Not used, 1-1 chip, 2-Narrow correlator.
  unsigned SatelliteSystem:3;                 //!< 0-GPS, 1-GLONASS, 2-WAAS, 3-Pseudolite, 4-7 reserved.
  unsigned Reserved1:1;                       //!< Reserved.
  unsigned isGrouped:1;                        //!< Is this channel grouped with another channel. i.e. L1/L2 group.
  unsigned Frequency:1;                       //!< 0-L1, 1-L2
  unsigned CodeType:2;                        //!< 0-CA , 1-P, 2-P codeless, 3-reserved.
  unsigned isForwareErrorCorrectionEnabled:1; //!< Is FEC enabled.
  unsigned Reserved2:6;                       //!< Reserved.
  unsigned isExternalRange:1;                 //!< Is this an external range.
  unsigned isChannelAssignmentForced:1;       //!< Is the channel assignment forced.
} NOVATELOEM3_bitfieldTrackingStatus;

typedef struct
{
  unsigned week;     //!< week.
  double   tow;      //!< time of week [0-604800).
  unsigned nrObs;    //!< The number of observations.
  NOVATELOEM3_bitfieldSelfTestStatus status; //!< The receiver self test status. 

} NOVATELOEM3_structObservationHeader;

/// \brief  A NOVATEL OEM3 observation message container. i.e. RGEB container.
typedef struct
{
  unsigned prn;      //!< The PRN.
  double   psr;      //!< pseudorange [m].
  float    psrstd;   //!< standard deviation of the psr [m].
  double   adr;      //!< accumulated Doppler range [cycles].
  float    adrstd;   //!< standard deviation of the adr [cycles].
  float    doppler;  //!< Doppler [Hz].
  float    cno;      //!< Carrier to noise density ratio [dB-Hz].
  float    locktime; //!< locktime in PLL mode [s].
  NOVATELOEM3_bitfieldTrackingStatus status; //!< The channel tracking status.

} NOVATELOEM3_structObservation;



/// \brief  A container for REPB (raw ephemeris) data.
typedef struct
{
  unsigned prn;
  unsigned char subframe1[30];
  unsigned char subframe2[30];
  unsigned char subframe3[30];
  unsigned char reserved[2];
} NOVATELOEM3_structREPB;




/// \brief  This enumeration is for NovAtel OEM4 message types.
typedef enum 
{
  NOVATELOEM4_IONUTCB               = 8,
  NOVATELOEM4_CLOCKMODELB           = 16,
  NOVATELOEM4_RAWGPSSUBFRAMEB       = 25,
  NOVATELOEM4_CHANDEBUGB            = 32,
  NOVATELOEM4_VERSIONB              = 37,
  NOVATELOEM4_RAWEPHEMB             = 41,
  NOVATELOEM4_BESTPOSB              = 42,
  NOVATELOEM4_RANGEB                = 43,
  NOVATELOEM4_PSRPOSB               = 47,
  NOVATELOEM4_SATVISB               = 48,
  NOVATELOEM4_ALMANACB              = 73,
  NOVATELOEM4_RAWALMB               = 74,
  NOVATELOEM4_TRACKSTATB            = 83,
  NOVATELOEM4_SATSTATB              = 84,
  NOVATELOEM4_RXSTATUSB             = 93,
  NOVATELOEM4_RXSTATUSEVENTB        = 94,
  NOVATELOEM4_MATCHEDPOSB           = 96,
  NOVATELOEM4_BESTVELB              = 99,
  NOVATELOEM4_PSRVELB               = 100,
  NOVATELOEM4_TIMEB                 = 101,
  NOVATELOEM4_RANGEPNB              = 126,
  NOVATELOEM4_RXCONFIGB             = 128,
  NOVATELOEM4_RANGECMPB             = 140,
  NOVATELOEM4_RTKPOSB               = 141,
  NOVATELOEM4_NAVIGATEB             = 161,
  NOVATELOEM4_AVEPOSB               = 172,
  NOVATELOEM4_REFSTATIONB           = 175,
  NOVATELOEM4_PASSCOM1B             = 233,
  NOVATELOEM4_PASSCOM2B             = 234,
  NOVATELOEM4_PASSCOM3B             = 235,
  NOVATELOEM4_MESSAGETYPE_UNKNOWN

} NOVATELOEM4_enumMessageType;

/// \brief  This enumeration is for NovAtel OEM4 channel tracking state.
typedef enum 
{
  NOVATELOEM4_L1Idle                      = 0,
  NOVATELOEM4_L1SkySearch                 = 1,
  NOVATELOEM4_L1WideFrequencyBandPullIn   = 2,
  NOVATELOEM4_L1NarrorFrequencyBandPullIn = 3,
  NOVATELOEM4_L1PhaseLockLoop             = 4,
  NOVATELOEM4_L1ReAcquisition             = 5,
  NOVATELOEM4_L1Steerin                   = 6,
  NOVATELOEM4_L1FrequencyLockLoop         = 7,
  NOVATELOEM4_L2Idle                      = 8,
  NOVATELOEM4_L2PCodeAlignment            = 9,
  NOVATELOEM4_L2Search                    = 10,
  NOVATELOEM4_L2PhaseLockLoop             = 11

} NOVATELOEM4_enumTrackingState;

/// \brief  This enumeration is for NovAtel OEM4 correlator spacing.
typedef enum 
{
  NOVATELOEM4_ReservedSpacingA        = 0,
  NOVATELOEM4_OneChipSpacing          = 1,
  NOVATELOEM4_NarrowSpacing           = 2,
  NOVATELOEM4_ReservedSpacingB        = 3,
  NOVATELOEM4_PulseApertureCorrelator = 4,

} NOVATELOEM4_enumCorrelatorSpacing;

/// \brief  This enumeration is for NovAtel OEM4 satellite system.
typedef enum
{
  NOVATELOEM4_GPSSystem        = 0,
  NOVATELOEM4_GLONASSSystem    = 1,
  NOVATELOEM4_WAASSystem       = 2,
  NOVATELOEM4_PseudoliteSystem = 3,
  NOVATELOEM4_ReservedSystem 

} NOVATELOEM4_enumSatelliteSystem;

/// \brief  This enumeration is for NovAtel OEM4 code type.
typedef enum 
{
  NOVATELOEM4_CACode     = 0,
  NOVATELOEM4_PCode      = 1,
  NOVATELOEM4_PCodeless  = 2,  
  NOVATELOEM4_L2C        = 4,
  
  NOVATELOEM4_ReservedCodeType 
} NOVATELOEM4_enumCodeType;

/// \brief  This enumeration is for NovAtel OEM4 frequency.
typedef enum 
{
  NOVATELOEM4_L1 = 0,
  NOVATELOEM4_L2 = 1,         
  NOVATELOEM4_ReservedFrequency

} NOVATELOEM4_enumFrequency;

/// \brief  The NovAtel OEM4 tracking status.
typedef struct 
{
  NOVATELOEM4_enumTrackingState      eTrackingState;     //!< The channel tracking state. 
  unsigned                           channelNumber;      //!< The channel index.
  BOOL                               isPhaseLocked;      //!< Is the channel phase locked (TRUE/FALSE)?
  BOOL                               isParityKnown;      //!< Is the parity known (TRUE/FALSE)? if not there is a possible 1/2 cycle ambiguity. Do NOT use the adr measurement without valid parity!
  BOOL                               isCodeLocked;       //!< Is the channel code locked (TRUE/FALSE)? The pseudorange is invalid if FALSE.
  NOVATELOEM4_enumCorrelatorSpacing  eCorrelatorSpacing; //!< The correlator spacing.
  NOVATELOEM4_enumSatelliteSystem    eSatelliteSystem;   //!< The satellite sytem for this channel.
  BOOL                               isGrouped;          //!< Is this measurement grouped (TRUE/FALSE)? e.g. tracking L1 & L2 for a single satellite.
  NOVATELOEM4_enumFrequency          eFrequency;         //!< The frequency type for this channel.
  NOVATELOEM4_enumCodeType           eCodeType;          //!< The code type for this channel.
  BOOL                               isFECEnabled;       //!< Is forward error correction enable (TRUE/FALSE)?
  BOOL                               isPrimaryL1Channel; //!< Is this the primary L1 channel (TRUE/FALSE)?
  BOOL                               isHalfCycleAdded;   //!< Has a half cycle been added to the adr after the parity determination (TRUE/FALSE)?
  BOOL                               isForcedAssignment; //!< Is this channel forced to track this PRN by the user (TRUE/FALSE)?

} NOVATELOEM4_structTrackingStatus;

/// \brief  The NovAtel OEM4 time status enumeration.
typedef enum
{
  NOVATELOEM4_TIMESTATUS_UNKNOWN         = 20,  //!< Time validity is unknown.
  NOVATELOEM4_TIMESTATUS_APPROXIMATE     = 60,  //!< Time is set approximately.
  NOVATELOEM4_TIMESTATUS_COARSEADJUSTING = 80,  //!< Time is approaching coarse precision.
  NOVATELOEM4_TIMESTATUS_COARSE          = 100, //!< This time is valid to coarse precision.
  NOVATELOEM4_TIMESTATUS_COARSESTEERING  = 120, //!< Time is coarse set, and is being steered.
  NOVATELOEM4_TIMESTATUS_FREEWHEELING    = 130, //!< Position is lost, and the range bias cannot be calculated.
  NOVATELOEM4_TIMESTATUS_FINEADJUSTING   = 140, //!< Time is adjusting to fine precision.
  NOVATELOEM4_TIMESTATUS_FINE            = 160, //!< Time has fine precision.
  NOVATELOEM4_TIMESTATUS_FINESTEERING    = 180, //!< Time is fine, set and is being steered.
  NOVATELOEM4_TIMESTATUS_SATTIME         = 200  //!< Time from satellite. This is only used in logs containing satellite data such as ephemeris and almanac.

} NOVATELOEM4_enumTimeStatus;

/// \brief  The NovAtel OEM4 receiver status bitfield.
typedef struct
{
  unsigned isErrorIndicated:1;           //!< A boolean to indicate if any errors indicated by the receiver. Check the receiver status message if any error is indicated.
  unsigned isTemperatureBad:1;           //!< A boolean to indicate if there is a temperature status warning. 
  unsigned isVoltageBad:1;               //!< A boolean to indicate if there is a voltage supply status warngin.
  unsigned isAntennaNotPowered:1;        //!< A boolean to indicate if the antenna is NOT powered.
  unsigned reserved_a:1;                 //!< reserved
  unsigned isAntennaOpen:1;              //!< A boolean to indicate if the antenna is open.
  unsigned isAntennaShorted:1;           //!< A boolean to indicate if the antenna is shorted.
  unsigned isCPUOverloaded:1;            //!< A boolean to indicate if the CPU is overloaded.
  unsigned isCOM1BufferOverrun:1;        //!< A boolean to indicate if COM1 buffer is overrun.
  unsigned isCOM2BufferOverrun:1;        //!< A boolean to indicate if COM2 buffer is overrun.
  unsigned isCOM3BufferOverrun:1;        //!< A boolean to indicate if COM3 buffer is overrun.
  unsigned isUSBBufferOverrun:1;         //!< A boolean to indicate if USB buffer is overrun.
  unsigned reserved_b:3;                 //!< reserved.
  unsigned isRF1_AGC_StatusBad:1;        //!< A boolean to indicate if the RF1 AGC status is invalid.
  unsigned reserved_c:1;                 //!< reserved.
  unsigned isRF2_AGC_StatusBad:1;        //!< A boolean to indicate if the RF2 AGC status if invalid.
  unsigned isAlmanacInvalid:1;           //!< A boolean to indicate if the almanac is invalid.
  unsigned isPositionSolutionInvalid:1;  //!< A boolean to indicate if the position solution is invalid.
  unsigned isPositionFixed:1;            //!< A boolean to indicate if the user has fixed the receiver's antenna position.
  unsigned isClockSteeringDisabled:1;    //!< A boolean to indicate if clock steering is disabled.
  unsigned isClockModelInvalid:1;        //!< A boolean to indicate if the clock model is invalid.
  unsigned isExternalOcsillatorDriven:1; //!< A boolean to indicate if the receiver is driven by an external oscillator.
  unsigned isSoftwareResouceBad:1;       //!< A boolean to indicate if there is a software resource warning.
  unsigned reserved_d:4;                 //!< reserved.
  unsigned isAUX3StatusEvent:1;          //!< A boolean to indicate if an AUX3 status event has occurred.
  unsigned isAUX2StatusEvent:1;          //!< A boolean to indicate if an AUX2 status event has occurred.
  unsigned isAUX1StatusEvent:1;          //!< A boolean to indicate if an AUX1 status event has occurred.

} NOVATELOEM4_structRxStatusBitField;


/// \brief  The NovAtel OEM4 binary message header.
typedef struct 
{
  // 0xAA 0x44 0x12 sync bytes
  unsigned char                       headerLength;    //!< The header length [bytes].
  unsigned short                      messageID;       //!< The binary message ID.
  char                                messageType;     //!< The message type.
  char                                portAddress;     //!< The port address.
  unsigned short                      dataLength;      //!< The length of the data part of the message (length of entire mesage - header size - CRC size) [bytes].
  unsigned short                      sequenceNr;      //!< The sequence number.
  char                                idleTime;        //!< The processor idle time.
  NOVATELOEM4_enumTimeStatus          eTimeStatus;     //!< The quality status of the GPS time.
  unsigned short                      gpsWeek;         //!< The GPS week (0-1024+) [weeks].
  unsigned long                       gpsMilliSeconds; //!< The GPS time of week in milliseconds.
  NOVATELOEM4_structRxStatusBitField  receiverStatus;  //!< The receiver status bitfield.
  unsigned short                      reserved;        //!< A reserved value.
  unsigned short                      receiverVersion; //!< The receiver firmware version.

} NOVATELOEM4_structBinaryHeader;


/// \brief  A container for a NovAtel OEM4 observation.
typedef struct
{
  unsigned short prn;       //!< The PRN for this channel.
  unsigned short reserved;  //!< A 16 bit reserved value.
  double   psr;             //!< The pseudorange measurement [m].
  float    psrstd;          //!< The receiver estimated pseudorange measurement standard deviation [m].
  double   adr;             //!< The carrier phase or accumulated Doppler range measurement [cycles].
  float    adrstd;          //!< The receiver estimated accumulated Doppler range measurement standard deviation [cycles].
  float    doppler;         //!< The Doppler measurement for this channel [Hz].
  float    cno;             //!< The carrier to noise density ratio for this channel [dB-Hz]
  float    locktime;        //!< The number of seconds of continous phase tracking (no known cycle slips) [s].

  unsigned rawTrackingStatus;                      //!< The raw tracking status 32 bit value.
  NOVATELOEM4_structTrackingStatus trackingStatus; //!< The decoded channel tracking status information.

} NOVATELOEM4_structObservation;

/// \brief  NovAtel OEM4 Solution Status enumeration.
typedef enum
{
  NOVATELOEM4_SOLNSTATUS_SOL_COMPUTED = 0, //!< Solution computed
  NOVATELOEM4_SOLNSTATUS_INSUFFICIENT_OBS = 1, //!< Insufficient observations
  NOVATELOEM4_SOLNSTATUS_NO_CONVERGENCE = 2, //!< No convergence
  NOVATELOEM4_SOLNSTATUS_SINGULARITY = 3, //!< Singularity at parameters matrix
  NOVATELOEM4_SOLNSTATUS_COV_TRACE = 4, //!< Covariance trace exceeds maximum (trace > 1000 m)
  NOVATELOEM4_SOLNSTATUS_TEST_DIST = 5, //!< Test distance exceeded (maximum of 3 rejections if distance > 10 km)
  NOVATELOEM4_SOLNSTATUS_COLD_START = 6, //!< Not yet converged from cold start
  NOVATELOEM4_SOLNSTATUS_V_H_LIMIT = 7, //!< Height or velocity limits exceeded (in accordance with COCOM export licensing restrictions)
  NOVATELOEM4_SOLNSTATUS_VARIANCE = 8, //!< Variance exceeds limits
  NOVATELOEM4_SOLNSTATUS_RESIDUALS = 9, //!< Residuals are too large
  NOVATELOEM4_SOLNSTATUS_DELTA_POS = 10, //!< Delta position is too large
  NOVATELOEM4_SOLNSTATUS_NEGATIVE_VAR = 11, //!< Negative variance
  NOVATELOEM4_SOLNSTATUS_RESERVED = 12,
  NOVATELOEM4_SOLNSTATUS_INTEGRITY_WARNING = 13, //!< Large residuals make position unreliable
  NOVATELOEM4_SOLNSTATUS_INS_14 = 14, //!< INS solution status values - Output only when using an inertial navigation system such as NovAtel’s SPAN products. 
  NOVATELOEM4_SOLNSTATUS_INS_15 = 15, //!< INS solution status values - Output only when using an inertial navigation system such as NovAtel’s SPAN products. 
  NOVATELOEM4_SOLNSTATUS_INS_16 = 16, //!< INS solution status values - Output only when using an inertial navigation system such as NovAtel’s SPAN products. 
  NOVATELOEM4_SOLNSTATUS_INS_17 = 17, //!< INS solution status values - Output only when using an inertial navigation system such as NovAtel’s SPAN products. 
  NOVATELOEM4_SOLNSTATUS_PENDING = 18, //!< When a FIX POSITION command is entered, the receiver computes its own position and determines if the fixed position is valid. PENDING implies there are not enough satellites being tracked to verify if the FIX POSITION entered into the receiver is valid. The receiver needs to be tracking two or more GPS satellites to perform this check. Under normal conditions you should only see PENDING for a few seconds on power up before the GPS receiver has locked onto its first few satellites. If your antenna is obstructed (or not plugged in) and you have entered a FIX POSITION command, then you may see PENDING indefinitely.
  NOVATELOEM4_SOLNSTATUS_INVALID_FIX = 19, //!< The fixed position, entered using the FIX POSITION command, is not valid
  NOVATELOEM4_SOLNSTATUS_UNAUTHORIZED = 20, //!< Position type is unauthorized - HP or XP on a receiver not authorized for it
  NOVATELOEM4_SOLNSTATUS_UNKNOWN
} NOVATELOEM4_enumSolutionStatus;

/// \brief  NovAtel OEM4 Position Status enumeration.
typedef enum
{
  NOVATELOEM4_POS_STATUS_NONE = 0, //!< No solution
  NOVATELOEM4_POS_STATUS_FIXEDPOS = 1, //!< Position has been fixed by the FIX POSITION command
  NOVATELOEM4_POS_STATUS_FIXEDHEIGHT = 2, //!< Position has been fixed by the FIX HEIGHT/AUTO command
  NOVATELOEM4_POS_STATUS_RESERVED_3 = 3,
  NOVATELOEM4_POS_STATUS_RESERVED_4 = 4,
  NOVATELOEM4_POS_STATUS_RESERVED_5 = 5,
  NOVATELOEM4_POS_STATUS_RESERVED_6 = 6,
  NOVATELOEM4_POS_STATUS_RESERVED_7 = 7,
  NOVATELOEM4_POS_STATUS_DOPPLER_VELOCITY = 8, //!< Velocity computed using instantaneous Doppler
  NOVATELOEM4_POS_STATUS_RESERVED_9 = 9,
  NOVATELOEM4_POS_STATUS_RESERVED_10 = 10,
  NOVATELOEM4_POS_STATUS_RESERVED_11 = 11,
  NOVATELOEM4_POS_STATUS_RESERVED_12 = 12,
  NOVATELOEM4_POS_STATUS_RESERVED_13 = 13,
  NOVATELOEM4_POS_STATUS_RESERVED_14 = 14,
  NOVATELOEM4_POS_STATUS_RESERVED_15 = 15,
  NOVATELOEM4_POS_STATUS_SINGLE = 16, //!< Single point position
  NOVATELOEM4_POS_STATUS_PSRDIFF = 17, //!< Pseudorange differential solution
  NOVATELOEM4_POS_STATUS_WAAS = 18, //!< Solution calculated using corrections from an SBAS
  NOVATELOEM4_POS_STATUS_PROPAGATED = 19, //!< Propagated by a Kalman filter without new observations
  NOVATELOEM4_POS_STATUS_OMNISTAR = 20, //!< OmniSTAR VBS position (L1 sub-meter) a
  NOVATELOEM4_POS_STATUS_RESERVED_21 = 21,
  NOVATELOEM4_POS_STATUS_RESERVED_22 = 22,
  NOVATELOEM4_POS_STATUS_RESERVED_23 = 23,
  NOVATELOEM4_POS_STATUS_RESERVED_24 = 24,
  NOVATELOEM4_POS_STATUS_RESERVED_25 = 25,
  NOVATELOEM4_POS_STATUS_RESERVED_26 = 26,
  NOVATELOEM4_POS_STATUS_RESERVED_27 = 27,
  NOVATELOEM4_POS_STATUS_RESERVED_28 = 28,
  NOVATELOEM4_POS_STATUS_RESERVED_29 = 29,
  NOVATELOEM4_POS_STATUS_RESERVED_30 = 30,
  NOVATELOEM4_POS_STATUS_RESERVED_31 = 31,
  NOVATELOEM4_POS_STATUS_L1_FLOAT = 32, //!< Floating L1 ambiguity solution
  NOVATELOEM4_POS_STATUS_IONOFREE_FLOAT = 33, //!< Floating ionospheric-free ambiguity solution
  NOVATELOEM4_POS_STATUS_NARROW_FLOAT = 34, //!< Floating narrow-lane ambiguity solution
  NOVATELOEM4_POS_STATUS_L1_INT = 48, //!< Integer L1 ambiguity solution
  NOVATELOEM4_POS_STATUS_WIDE_INT = 49, //!< Integer wide-lane ambiguity solution
  NOVATELOEM4_POS_STATUS_NARROW_INT = 50, //!< Integer narrow-lane ambiguity solution
  NOVATELOEM4_POS_STATUS_RTK_DIRECT_INS = 51, //!< RTK status where the RTK filter is directly initialized from the INS filter *b
  NOVATELOEM4_POS_STATUS_INS_52 = 52, //!< INS calculated position types *b
  NOVATELOEM4_POS_STATUS_INS_53 = 53, //!< INS calculated position types *b
  NOVATELOEM4_POS_STATUS_INS_54 = 54, //!< INS calculated position types *b
  NOVATELOEM4_POS_STATUS_INS_55 = 55, //!< INS calculated position types *b
  NOVATELOEM4_POS_STATUS_INS_56 = 56, //!< INS calculated position types *b
  NOVATELOEM4_POS_STATUS_OMNISTAR_HP = 64, //!< OmniSTAR HP position (L1/L2 decimeter) *a
  NOVATELOEM4_POS_STATUS_OMNISTAR_XP = 65, //!< OmniSTAR XP position
  NOVATELOEM4_POS_STATUS_CDGPS = 66, //!< Position solution using CDGPS correction *a
  // *a. In addition to a NovAtel receiver with L-Band capability, a subscription to the OmniSTAR, or use of the free CDGPS, service is required. Contact NovAtel for details.
  // *b. Output only by the BESTPOS and BESTVEL logs when using an inertial navigation system such as NovAtel’s SPAN products. 
} NOVATELOEM4_enumSolutionType;

/// \brief  A container for the best position, BESTPOS, record.
typedef struct
{
  NOVATELOEM4_enumSolutionStatus solution_status;
  NOVATELOEM4_enumSolutionType solution_type;
  double latitude_in_deg; //!< [deg]
  double longitude_in_deg; //!< [deg] 
  double height_msl; //!< [m]
  float undulation; //!< [m]
  unsigned int datum_id; //!< 61 = WGS84, this is the default.
  float lat_std; //!< [m]
  float lon_std; //!< [m]
  float hgt_std; //!< [m]
  char station_id[4]; //!< Base station ID
  float diff_age; //!< Differential age [s]
  float sol_age;  //!< Solution age [s]
  unsigned char nr_obs_tracked; //!< Number of observations tracked
  unsigned char nr_GPS_L1_ranges; //!< Number of GPS L1 ranges used in computation
  unsigned char nr_GPS_L1_ranges_above_RTK_mask_angle; //!< Number of GPS L1 ranges above the RTK mask angle
  unsigned char nr_GPS_L2_ranges_above_RTK_mask_angle; //!< Number of GPS L2 ranges above the RTK mask angle
  unsigned char reserved[4];
} NOVATELOEM4_structBestPosition;

typedef enum
{
  NOVATELOEM4_CLOCK_STATUS_VALID = 0,      //!< The clock model is valid
  NOVATELOEM4_CLOCK_STATUS_CONVERGING = 1, //!< The clock model is near validity
  NOVATELOEM4_CLOCK_STATUS_ITERATING = 2,  //!< The clock model is iterating towards validity
  NOVATELOEM4_CLOCK_STATUS_INVALID = 3,    //!< The clock model is not valid
  NOVATELOEM4_CLOCK_STATUS_ERROR = 4,      //!< Clock model error
  NOVATELOEM4_CLOCK_STATUS_UNKNOWN
} NOVATELOEM4_enumClockStatus;

/// \brief  A container for the TIME message.
typedef struct
{
  NOVATELOEM4_enumClockStatus clock_status; //!< Clock model status
  double receiver_clock_offset;     //!< Receiver clock offset, in seconds from GPS time. A positive offset implies that the receiver clock is ahead of GPS time. To derive GPS time, use the following formula: GPS time = receiver time - offset
  double receiver_clock_offset_std; //!< Receiver clock offset standard deviation [s]. 
  double utc_offset;         //!< The offset of GPS time from UTC time [s], computed using almanac parameters. UTC time is GPS time plus the current UTC offset plus the receiver clock offset: UTC time = GPS time + offset + UTC offset 
  unsigned int utc_year;     //!< UTC year.
  unsigned char utc_month;   //!< UTC month [0-12]. 0 means UTC time is unknown.
  unsigned char utc_day;     //!< UTC day [0-31]. 0 means UTC time is unknown.
  unsigned char utc_hour;    //!< UTC hour [0-23].
  unsigned char utc_minute;  //!< UTC minute [0-59].
  unsigned utc_milliseconds; //!< UTC milliseconds [0-60999]. Max of 60999 when leap second is applied.
  BOOL isUTCValid;           //!< A boolean to indicate if the UTC time is valid.

} NOVATELOEM4_structTime;


/**
\brief  Find the next NovAtel OEM3 message in an open file.

Search a file (FILE*), that is already open, for the next
NovAtel OEM3 message. The user must provide a message buffer
with an associated maximum length (8192 bytes recommended ).

\author   Glenn D. MacGougan (GDM)
\date     2008-12-01
\since    2008-12-01
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM3_FindNextMessageInFile(
  FILE *fid,                       //!< A file pointer to an open file (input).
  unsigned char *message,          //!< A message buffer in which to place the message found (input/output).
  const unsigned maxMessageLength, //!< The maximum size of the message buffer (input).
  BOOL *wasEndOfFileReached,       //!< Has the end of the file been reached (output).
  BOOL *wasMessageFound,           //!< Was a valid message found (output).
  unsigned *filePosition,          //!< The file position for the start of the message found (output).
  unsigned *messageLength,         //!< The length of the entire message found and stored in the message buffer (output).
  unsigned *messageID,             //!< The message ID of the message found.
  unsigned *numberBadChecksums     //!< The number of bad checksum values found. (checksum fails or mistaken messages).
  );


/**
\brief  Find the next NovAtel OEM3 message in a data buffer.

Search a valid buffer for the next
NovAtel OEM3 message. The user must provide a message buffer
with an associated maximum length (8192 bytes recommended ).

\author   Glenn D. MacGougan (GDM)
\date     2008-12-03
\since    2008-12-03
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM3_FindNextMessageInBuffer(
  unsigned char *buffer,           //!< A pointer to a buffer containing input data.
  const unsigned bufferLength,     //!< The length of the valid data contained in the buffer.
  unsigned char *message,          //!< A message buffer in which to place the message found (input/output).
  const unsigned maxMessageLength, //!< The maximum size of the message buffer (input).
  BOOL *wasEndOfBufferReached,     //!< Has the end of the buffer been reached (output).
  BOOL *wasMessageFound,           //!< Was a valid message found (output).
  unsigned *startPosition,         //!< The index into the buffer for the start of the message found (output).
  unsigned *messageLength,         //!< The length of the entire message found and stored in the message buffer (output).
  unsigned *messageID,             //!< The message ID of the message found.
  unsigned *numberBadChecksums     //!< The number of bad checksum values found. (checksum fails or mistaken messages).
  );



/**
\brief    Decode a NovAtel OEM3 REPB message.

Given a message buffer with a complete NovAtel OEM3 REPB binary 
message, this function will decode the content into useable ephemeris
information.

\author   Glenn D. MacGougan (GDM)
\date     2008-12-01
\since    2008-12-01
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM3_DecodeREPB(
  const unsigned char *message,           //!< The message buffer containing a complete RANGEB message (input).
  const unsigned messageLength,           //!< The length of the entire message (input).
  unsigned       *prn,                    //!< The satellite PRN number.
  unsigned       *tow,                    //!< The time of week in subframe1, the time of the leading bit edge of subframe 2 [s]
  unsigned short *iodc,                   //!< 10 bit issue of data (clock), 8 LSB bits will match the iode                  []    
  unsigned char  *iode,                   //!< 8 bit  issue of data (ephemeris)                                              []
  unsigned       *toe,                    //!< reference time ephemeris (0-604800)                                           [s]
  unsigned       *toc,                    //!< reference time (clock)   (0-604800)                                           [s]      
  unsigned short *week,                   //!< 10 bit gps week 0-1023 (user must account for week rollover )                 [week]    
  unsigned char  *health,                 //!< 6 bit health parameter, 0 if healthy, unhealth othersize                      [0=healthy]    
  unsigned char  *alert_flag,             //!< 1 = URA may be worse than indicated                                           [0,1]
  unsigned char  *anti_spoof,             //!< anti-spoof flag from 0=off, 1=on                                              [0,1]    
  unsigned char  *code_on_L2,             //!< 0=reserved, 1=P code on L2, 2=C/A on L2                                       [0,1,2]
  unsigned char  *ura,                    //!< User Range Accuracy lookup code, 0 is excellent, 15 is use at own risk        [0-15], see p. 83 GPSICD200C
  unsigned char  *L2_P_data_flag,         //!< flag indicating if P is on L2 1=true                                          [0,1]
  unsigned char  *fit_interval_flag,      //!< fit interval flag (four hour interval or longer) 0=4 fours, 1=greater         [0,1]
  unsigned short *age_of_data_offset,     //!< age of data offset                                                            [s]
  double *tgd,     //!< group delay                                                                   [s]
  double *af2,     //!< polynomial clock correction coefficient (rate of clock drift)                 [s/s^2]
  double *af1,     //!< polynomial clock correction coefficient (clock drift)                         [s/s]
  double *af0,     //!< polynomial clock correction coefficient (clock bias)                          [s]    
  double *m0,      //!< mean anomaly at reference time                                                [rad]
  double *delta_n, //!< mean motion difference from computed value                                    [rad/s]
  double *ecc,     //!< eccentricity                                                                  []
  double *sqrta,   //!< square root of the semi-major axis                                            [m^(1/2)]
  double *omega0,  //!< longitude of ascending node of orbit plane at weekly epoch                    [rad]
  double *i0,      //!< inclination angle at reference time                                           [rad]
  double *w,       //!< argument of perigee                                                           [rad]
  double *omegadot,//!< rate of right ascension                                                       [rad/s]
  double *idot,    //!< rate of inclination angle                                                     [rad/s]
  double *cuc,     //!< amplitude of the cosine harmonic correction term to the argument of latitude  [rad]
  double *cus,     //!< amplitude of the sine harmonic correction term to the argument of latitude    [rad]
  double *crc,     //!< amplitude of the cosine harmonic correction term to the orbit radius          [m]
  double *crs,     //!< amplitude of the sine harmonic correction term to the orbit radius            [m]
  double *cic,     //!< amplitude of the cosine harmonic correction term to the angle of inclination  [rad]
  double *cis      //!< amplitude of the sine harmonic correction term to the angle of inclination    [rad]
  );


/**
\brief  Decode a NovAtel OEM3 RGEB message.

Given a message buffer with a complete NovAtel OEM3 RGEB binary message,
and a user provided array of observation structs, this function will 
decode the binary message into the user provided array.

\author   Glenn D. MacGougan (GDM)
\date     2008-12-01
\since    2008-12-01
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM3_DecodeRGEB(
  const unsigned char *message,            //!< The message buffer containing a complete RGEB message (input).
  const unsigned messageLength,            //!< The length of the entire message (input).
  NOVATELOEM3_structObservationHeader* obsHeader, //!< A pointer to a user provided struct with obs header info (output).
  NOVATELOEM3_structObservation* obsArray, //!< A pointer to a user provided array of NOVATELOEM3_structObservation (output).
  const unsigned char maxNrObs,            //!< The maximum number of elements in the array provided (input).
  unsigned *nrObs                          //!< The number of valid elements set in the array (output).
  );


/**
\brief  Decode a NovAtel OEM3 POSB message.

Given a message buffer with a complete NovAtel OEM3 POSB binary message.

\author   Glenn D. MacGougan (GDM)
\date     2008-12-08
\since    2008-12-08
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM3_DecodePOSB(
  const unsigned char *message,  //!< The message buffer containing a complete RGEB message (input).
  const unsigned messageLength,  //!< The length of the entire message (input).
  unsigned short* gps_week,      //!< The GPS week number [0-1023], user must account for week rollover (output).
  double* gps_tow,               //!< The GPS time of week [0-604800) (output).
  double* latitude_degs,         //!< The latitude [deg] (output).
  double* longitude_degs,        //!< The longitude [deg] (output).
  double* height_msl,            //!< The height (with respect to mean sea level) [m] (output).
  double* undulation,            //!< The geoidal undulation [m] (output).
  unsigned int* datum_id,        //!< The datum id (61 is WGS84) (output).
  double* lat_std,               //!< The estimated solution precision (1-sigma) (output).
  double* lon_std,               //!< The estimated solution precision (1-sigma) (output).
  double* hgt_std,               //!< The estimated solution precision (1-sigma) (output).
  NOVATELOEM3_enumSolutionStatus* status //!< The solution status indicator.
  );


/**
\brief  Decode a NovAtel OEM3 TM1B message.

Given a message buffer with a complete NovAtel OEM3 TM1B binary message.
This log provides the time of the GPSCard 1PPS, normally high, active low pulse (1 millisecond), where falling
edge is reference, in GPS week number and seconds into the week. The TM1A/B log follows a 1PPS pulse. It also
includes the receiver clock offset, the standard deviation of the receiver clock offset and clock model status. This
log will output at a maximum rate of 1 Hz.

\author   Glenn D. MacGougan (GDM)
\date     2008-12-08
\since    2008-12-08
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM3_DecodeTM1B(
  const unsigned char *message,  //!< The message buffer containing a complete RGEB message (input).
  const unsigned messageLength,  //!< The length of the entire message (input).
  unsigned short* gps_week,      //!< The GPS week number [0-1023], user must account for week rollover (output).
  double* gps_tow,               //!< The GPS time of week [0-604800) (output).
  double* clk_offset,            //!< The receiver clock offset [s] (output). GPSTime = receiver_time - offset.
  double* clk_offset_std,        //!< The estimated precision of the clock offset [s] at 1 sigma (output).
  double* utc_offset,            //!< The estimated difference between UTC and GPS time. UTC_time = GPS_time + utc_offset. (e.g. -13.0) (output).
  BOOL* is_clk_stabilized        //!< A boolean to indicate if the clock is stable (output).
  );



/**
\brief  Decode a NovAtel OEM3 IONB message.

Given a message buffer with a complete NovAtel OEM3 IONB binary message,
extract the alpha and beta values.

\author   Glenn D. MacGougan (GDM)
\date     2008-12-03
\since    2008-12-03
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM3_DecodeIONB(
  const unsigned char *message, //!< The message buffer containing a complete RGEB message (input).
  const unsigned messageLength, //!< The length of the entire message (input).
  double *alpha0,     //!< coefficients of a cubic equation representing the amplitude of the vertical delay [s] (output).
  double *alpha1,     //!< coefficients of a cubic equation representing the amplitude of the vertical delay [s/semi-circle] (output).
  double *alpha2,     //!< coefficients of a cubic equation representing the amplitude of the vertical delay [s/semi-circle^2] (output).
  double *alpha3,     //!< coefficients of a cubic equation representing the amplitude of the vertical delay [s/semi-circle^3] (output).
  double *beta0,      //!< coefficients of a cubic equation representing the period of the model [s] (output).
  double *beta1,      //!< coefficients of a cubic equation representing the period of the model [s/semi-circle] (output).
  double *beta2,      //!< coefficients of a cubic equation representing the period of the model [s/semi-circle^2] (output).
  double *beta3       //!< coefficients of a cubic equation representing the period of the model [s/semi-circle^3] (output).
  );


/**
\brief  Find the next NovAtel OEM4 message in an open file.

Search a file (FILE*), that is already open, for the next
NovAtel OEM4 message. The user must provide a message buffer
with an associated maximum length (8192 bytes recommended ).

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2006-11-09
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM4_FindNextMessageInFile(
  FILE *fid,                       //!< A file pointer to an open file (input).
  unsigned char *message,          //!< A message buffer in which to place the message found (input/output).
  const unsigned maxMessageLength, //!< The maximum size of the message buffer (input).
  BOOL *wasEndOfFileReached,       //!< Has the end of the file been reached (output).
  BOOL *wasMessageFound,           //!< Was a valid message found (output).
  unsigned *filePosition,          //!< The file position for the start of the message found (output).
  unsigned short *messageLength,   //!< The length of the entire message found and stored in the message buffer (output).
  unsigned short *messageID,       //!< The message ID of the message found.
  unsigned *numberBadCRC           //!< The number of bad crc values found. (crc fails or mistaken messages).
  );


/**
\brief  Find the next NovAtel OEM4 message in a buffer.

Search a valid buffer for the next
NovAtel OEM4 message. The user must provide a message buffer
with an associated maximum length (8192 bytes recommended ).

\author   Glenn D. MacGougan (GDM)
\date     2008-12-19
\since    2008-12-19
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM4_FindNextMessageInBuffer(
  unsigned char *buffer,           //!< A pointer to a buffer containing input data.
  const unsigned bufferLength,     //!< The length of the valid data contained in the buffer.
  unsigned char *message,          //!< A message buffer in which to place the message found (input/output).
  const unsigned maxMessageLength, //!< The maximum size of the message buffer (input).
  BOOL *wasEndOfBufferReached,     //!< Has the end of the buffer been reached (output).
  BOOL *wasMessageFound,           //!< Was a valid message found (output).
  unsigned *startPosition,         //!< The index into the buffer for the start of the message found (output).
  unsigned short *messageLength,   //!< The length of the entire message found and stored in the message buffer (output).
  unsigned short *messageID,       //!< The message ID of the message found.
  unsigned *numberBadCRC           //!< The number of bad crc values found. (crc fails or mistaken messages).  
  );

/**
\brief    Decode a Novatel OEM4 binary message header given a complete binary message.
\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2006-11-10
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM4_DecodeBinaryMessageHeader(
  const unsigned char *message,            //!< The message buffer containing a complete NOVATEL OEM4 binary message (input).
  const unsigned short messageLength,      //!< The length of the entire message (input).
  NOVATELOEM4_structBinaryHeader *header   //!< A pointer to a NovAtel OEM4 header information struct (output).
  );
  


/**
\brief  Decode a NovAtel OEM4 RANGEB message.

Given a message buffer with a complete NovAtel OEM4 RANGEB binary message,
and a user provided array of observation structs, this function will 
decode the binary message into the user provided array.

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2006-11-10
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM4_DecodeRANGEB(
  const unsigned char *message,            //!< The message buffer containing a complete RANGEB message (input).
  const unsigned short messageLength,      //!< The length of the entire message (input).
  NOVATELOEM4_structBinaryHeader* header,  //!< A pointer to a NovAtel OEM4 header information struct (output).
  NOVATELOEM4_structObservation* obsArray, //!< A pointer to a user provided array of struct_NOVATELOEM4_RANGE (output).
  const unsigned char maxNrObs,            //!< The maximum number of elements in the array provided (input).
  unsigned *nrObs                          //!< The number of valid elements set in the array (output).
  );


/**
\brief  Decode a NovAtel OEM4 BESTPOSB message.

Given a message buffer with a complete NovAtel OEM4 BESTPOSB binary message,
and a user provided array of observation structs, this function will 
decode the binary message.

\author   Glenn D. MacGougan (GDM)
\date     2008-12-18
\since    2008-12-18
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM4_DecodeBESTPOSB(
  const unsigned char *message,            //!< The message buffer containing a complete RANGEB message (input).
  const unsigned short messageLength,      //!< The length of the entire message (input).
  NOVATELOEM4_structBinaryHeader* header,  //!< A pointer to a NovAtel OEM4 header information struct (output).
  NOVATELOEM4_structBestPosition* bestpos  //!< A pointer to a NovAtel OEM4 best position information struct (output).
  );


/**
\brief  Decode a NovAtel OEM4 TIMEB message.

Given a message buffer with a complete NovAtel OEM4 TIMEB binary message,
and a user provided array of observation structs, this function will 
decode the binary message.

\author   Glenn D. MacGougan (GDM)
\date     2008-12-18
\since    2008-12-18
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM4_DecodeTIMEB(
  const unsigned char *message,            //!< The message buffer containing a complete RANGEB message (input).
  const unsigned short messageLength,      //!< The length of the entire message (input).
  NOVATELOEM4_structBinaryHeader* header,  //!< A pointer to a NovAtel OEM4 header information struct (output).
  NOVATELOEM4_structTime* time_data        //!< A pointer to a NovAtel OEM4 best position information struct (output).
  );




/**
\brief    Decode the raw 32 bit value that contains tracking status information.

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2006-11-10
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM4_DecodeTrackingStatus(
  const unsigned rawTrackingStatus,                 //!< The raw 32 bit tracking status value (input).
  NOVATELOEM4_structTrackingStatus *trackingStatus  //!< The decoded tracking status information (output).
  );


/**
\brief    Decode a NovAtel OEM4 RAWEPHEMB message.

Given a message buffer with a complete NovAtel OEM4 RAWEPHEMB binary 
message, this function will decode the content into useable ephemeris
information.

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2006-11-10
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM4_DecodeRAWEPHEMB(
  const unsigned char *message,           //!< The message buffer containing a complete RANGEB message (input).
  const unsigned short messageLength,     //!< The length of the entire message (input).
  NOVATELOEM4_structBinaryHeader* header, //!< A pointer to a NovAtel OEM4 header information struct (output).
  unsigned *prn,                          //!< The satellite PRN number.
  unsigned *reference_week,               //!< The reference GPS week (0-1024+) [weeks].
  unsigned *reference_time,               //!< The reference GPS time of week (0-604800) [s].
  unsigned       *tow,                    //!< The time of week in subframe1, the time of the leading bit edge of subframe 2 [s]
  unsigned short *iodc,                   //!< 10 bit issue of data (clock), 8 LSB bits will match the iode                  []    
  unsigned char  *iode,                   //!< 8 bit  issue of data (ephemeris)                                              []
  unsigned       *toe,                    //!< reference time ephemeris (0-604800)                                           [s]
  unsigned       *toc,                    //!< reference time (clock)   (0-604800)                                           [s]      
  unsigned short *week,                   //!< 10 bit gps week 0-1023 (user must account for week rollover )                 [week]    
  unsigned char  *health,                 //!< 6 bit health parameter, 0 if healthy, unhealth othersize                      [0=healthy]    
  unsigned char  *alert_flag,             //!< 1 = URA may be worse than indicated                                           [0,1]
  unsigned char  *anti_spoof,             //!< anti-spoof flag from 0=off, 1=on                                              [0,1]    
  unsigned char  *code_on_L2,             //!< 0=reserved, 1=P code on L2, 2=C/A on L2                                       [0,1,2]
  unsigned char  *ura,                    //!< User Range Accuracy lookup code, 0 is excellent, 15 is use at own risk        [0-15], see p. 83 GPSICD200C
  unsigned char  *L2_P_data_flag,         //!< flag indicating if P is on L2 1=true                                          [0,1]
  unsigned char  *fit_interval_flag,      //!< fit interval flag (four hour interval or longer) 0=4 fours, 1=greater         [0,1]
  unsigned short *age_of_data_offset,     //!< age of data offset                                                            [s]
  double *tgd,     //!< group delay                                                                   [s]
  double *af2,     //!< polynomial clock correction coefficient (rate of clock drift)                 [s/s^2]
  double *af1,     //!< polynomial clock correction coefficient (clock drift)                         [s/s]
  double *af0,     //!< polynomial clock correction coefficient (clock bias)                          [s]    
  double *m0,      //!< mean anomaly at reference time                                                [rad]
  double *delta_n, //!< mean motion difference from computed value                                    [rad/s]
  double *ecc,     //!< eccentricity                                                                  []
  double *sqrta,   //!< square root of the semi-major axis                                            [m^(1/2)]
  double *omega0,  //!< longitude of ascending node of orbit plane at weekly epoch                    [rad]
  double *i0,      //!< inclination angle at reference time                                           [rad]
  double *w,       //!< argument of perigee                                                           [rad]
  double *omegadot,//!< rate of right ascension                                                       [rad/s]
  double *idot,    //!< rate of inclination angle                                                     [rad/s]
  double *cuc,     //!< amplitude of the cosine harmonic correction term to the argument of latitude  [rad]
  double *cus,     //!< amplitude of the sine harmonic correction term to the argument of latitude    [rad]
  double *crc,     //!< amplitude of the cosine harmonic correction term to the orbit radius          [m]
  double *crs,     //!< amplitude of the sine harmonic correction term to the orbit radius            [m]
  double *cic,     //!< amplitude of the cosine harmonic correction term to the angle of inclination  [rad]
  double *cis      //!< amplitude of the sine harmonic correction term to the angle of inclination    [rad]
  );
   

/**
\brief  Decode a NovAtel OEM4 RANGEBCMPB message.

Given a message buffer with a complete NovAtel OEM4 RANGECMPB binary message,
and a user provided array of observation structs, this function will 
decode the compressed binary message into the user provided array.

\author   Ossama Al-Fanek (OA-F)
\date     March 17, 2007
\remarks  Reviewed by GDM, March 17, 2007.
\return   TRUE(1) if successful, FALSE(0) otherwise.
*/
BOOL NOVATELOEM4_DecodeRANGECMPB(
  const unsigned char *message,            //!< The message buffer containing a complete RANGEB message (input).
  const unsigned short messageLength,      //!< The length of the entire message (input).
  NOVATELOEM4_structBinaryHeader* header,  //!< A pointer to a NovAtel OEM4 header information struct (output).
  NOVATELOEM4_structObservation* obsArray, //!< A pointer to a user provided array of struct_NOVATELOEM4_RANGE (output).
  const unsigned char maxNrObs,            //!< The maximum number of elements in the array provided (input).
  unsigned *nrObs                          //!< The number of valid elements set in the array (output).
  );
                       

#ifdef __cplusplus
}
#endif


#endif // _C_NOVATEL_H_
