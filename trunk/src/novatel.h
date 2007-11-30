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

#include "basictypes.h"

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
