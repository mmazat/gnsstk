/**
\file    cycle_slip.h
\brief   GNSS core 'c' function library: Cycle slip detection.
\author  Kyle O'Keefe (KO)
\date    2008-03-14
\since   2008-03-14

\b REFERENCES \n
- 

\b "LICENSE INFORMATION" \n
Copyright (c) 2008, refer to 'author' doxygen tags \n
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

#ifndef _C_CYCLE_SLIP_H_
#define _C_CYCLE_SLIP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "basictypes.h"

/**
\brief   Use phase rate (Doppler) to detect cycle slips

\author  Kyle O'Keefe (KO)
\date    2008-03-14
\since   2008-03-14

\pre The input adr measurements must be phase locked and have valid parity.

\return  TRUE if successful, false otherwise.
*/
BOOL CYCLESLIP_CheckForCycleSlipUsingPhaseRatePrediction(
  unsigned short previous_week, //!< The previous measurement gps week [weeks].
  double   previous_tow,        //!< The previous measurement gps time of week [s].  
  double   previous_Doppler,    
  double   previous_adr,
  unsigned short current_week,  //!< The current measurement gps week [weeks].
  double   current_tow,         //!< The current measurement gps time of week [s].
  double   current_Doppler,
  double   current_adr,
  double   specifiedThresholdInCycles,
  BOOL*    wasSlipDetected,
  double*  sizeOfSlipInCycles
  );

/*
\brief  Use dual frequency phase to detect cycle slips

\author  Kyle O'Keefe (KO)
\date    2008-03-17
\since   2008-03-17

return  TRUE if successful, false otherwise.
*/

BOOL CYCLESLIP_CheckForCycleSlipUsingDualFrequencyPhase(
  double   adr_Frequency1, //!< Add comments
  double   adr_Frequency2,
  double   wavelength1,
  double   wavelength2,
  double   specifiedThresholdInCycles,
  BOOL*    wasSlipDetected,
  double*  sizeOfSlipInCycles
  );

/*
\brief  Use triple diffrence phase to detect cycle slips

\pre    Discuss what needs to be valid to use this function.

\author  Kyle O'Keefe (KO)
\date    2008-03-17
\since   2008-03-17

return  TRUE if successful, false otherwise.
*/

BOOL CYCLESLIP_CheckForCycleSlipUsingTripleDifferencePhase(
  unsigned short current_week, 
  double   current_tow,
  unsigned short previous_week, 
  double   previous_tow,  
  double   adr_reference_rx_base_sat,
  double   adr_reference_rx,
  double   adr_rover_rx_base_sat,
  double   adr_rover_rx,
  double   prev_adr_reference_rx_base_sat,
  double   prev_adr_reference_rx,
  double   prev_adr_rover_rx_base_sat,
  double   prev_adr_rover_rx,
  double   specifiedThresholdInCycles,
  BOOL*    wasSlipDetected,
  double*  sizeOfSlipInCycles
  );


#ifdef __cplusplus
}
#endif


#endif // _C_CYCLE_SLIP_H_
