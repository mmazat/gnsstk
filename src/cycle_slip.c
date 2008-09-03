/**
\file    cycle_slip.c
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
#include <math.h>
#include <stdio.h>
#include "constants.h"
#include "cycle_slip.h"
#include "gnss_error.h"

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
  )
{
  double t_prev          = 0.0;
  double t               = 0.0;
  double dt              = 0.0;
  double predicted_phase = 0.0;
  double mean_doppler    = 0.0;
  double phase_diff      = 0.0; // The difference between the predicted phase and the measured phase.

  static double max_dif  = 0.0;

  if( wasSlipDetected == NULL || sizeOfSlipInCycles == NULL )
  {
    GNSS_ERROR_MSG( "if( wasSlipDetected == NULL || sizeOfSlipInCycles == NULL )" );
    return FALSE;
  }

  t_prev = previous_week * SECONDS_IN_WEEK + previous_tow;
  t      = current_week * SECONDS_IN_WEEK + current_tow;
  dt     = t - t_prev;

  if( dt <= 0.0 )
  {
    return FALSE;
  }
  if( dt > 60.0 ) // The phase rate prediction method is not going to work well.
  {
    return TRUE;
  }

  mean_doppler    = (previous_Doppler + current_Doppler)/2.0;
  predicted_phase = previous_adr - mean_doppler * dt; // GDM_BEWARE Doppler sign convention
  phase_diff      = predicted_phase - current_adr;

  if( fabs(phase_diff) > max_dif )
    max_dif = fabs(phase_diff);

  if( fabs(phase_diff) > specifiedThresholdInCycles )
  {
    *wasSlipDetected    = TRUE; // Indicate a cycle slip has occured.
    *sizeOfSlipInCycles = phase_diff;
  }
  else
  {
    *wasSlipDetected    = FALSE; // No cycle slip detected.
  }

  return TRUE;
}

/*
\brief  Use dual frequency phase to detect cycle slips

\author Kyle O'Keefe (KO)
\date   2008-03-17
\since  2008-03-17

return  TRUE if successful, false otherwise.
*/

BOOL CYCLESLIP_CheckForCycleSlipUsingDualFrequencyPhase(
  double   adr_Frequency1,
  double   adr_Frequency2,
  double   wavelength1,
  double   wavelength2,
  double   specifiedThresholdInCycles,
  BOOL*    wasSlipDetected,
  double*  sizeOfSlipInCycles
  )
{
  double diff = 0.0;

  if( wasSlipDetected == NULL || sizeOfSlipInCycles == NULL )
  {
    GNSS_ERROR_MSG( "if( wasSlipDetected == NULL || sizeOfSlipInCycles == NULL )" );
    return FALSE;
  }

  diff = adr_Frequency1 - adr_Frequency2 * wavelength2 / wavelength1;

  if( fabs(diff) > specifiedThresholdInCycles )
  {
    // the cycle slip has occured, but it is cannot be determined either on L1 or L2
    *wasSlipDetected    = TRUE;
    *sizeOfSlipInCycles = diff;
  }
  else
  {
    *wasSlipDetected    = FALSE;
  }
  return TRUE;
}

/*
\brief  Use dual frequency phase to detect cycle slips

\author Kyle O'Keefe (KO)
\date   2008-03-29
\since  2008-03-17

return  TRUE if successful, false otherwise.
*/
BOOL CYCLESLIP_CheckForCycleSlipUsingTripleDifferencePhase(
  unsigned short current_week, 
  double   current_tow,
  unsigned short previous_week, 
  double   previous_tow,  
  double   rover_range_base_sat,
  double   prev_rover_range_base_sat,
  double   reference_range_base_sat,
  double   prev_reference_range_base_sat,  
  double   rover_range,
  double   prev_rover_range,
  double   reference_range,
  double   prev_reference_range,
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
  )
{
  double diff = 0.0;
  double current_time = 0.0;
  double prev_time = 0.0;
  double delta_time = 0.0;
  double sd_adr = 0;
  double sd_adr_basesat = 0;
  double dd_adr = 0;
  double prev_sd_adr = 0;
  double prev_sd_adr_basesat = 0;
  double prev_dd_adr = 0;

  double sd = 0;
  double sd_basesat = 0;
  double dd = 0;
  double prev_sd = 0;
  double prev_sd_basesat = 0;
  double prev_dd = 0;

  double dd_diff = 0;
  double dd_diff_prev = 0;

  if( wasSlipDetected == NULL || sizeOfSlipInCycles == NULL )
  {
    GNSS_ERROR_MSG( "if( wasSlipDetected == NULL || sizeOfSlipInCycles == NULL )" );
    return FALSE;
  }

  current_time = current_week*SECONDS_IN_WEEK + current_tow;
  prev_time = previous_week*SECONDS_IN_WEEK + previous_tow;
  delta_time = current_time - prev_time;

  if( prev_time == 0 )
  {
    wasSlipDetected     = FALSE;
    *sizeOfSlipInCycles = 0;
    return TRUE;
  }
  if( rover_range_base_sat == 0 ||
    prev_rover_range_base_sat == 0 ||
    reference_range_base_sat == 0 ||
    prev_reference_range_base_sat == 0 ||  
    rover_range == 0 ||
    prev_rover_range == 0 ||
    reference_range == 0 ||
    prev_reference_range == 0 )
  {
    wasSlipDetected     = FALSE;
    *sizeOfSlipInCycles = 0;
    return TRUE;
  }
  
  if( delta_time <= 0.0 )
  {
    GNSS_ERROR_MSG( "if( delta_time <= 0.0 )" );
    return FALSE;
  }

  if( delta_time > 60.0 )
  {
    GNSS_ERROR_MSG( "Elapsed time is too long for using the triple difference cycle slip detection method." );
    return FALSE;
  }

  // compute the value of triple difference

  sd_adr = adr_rover_rx - adr_reference_rx;
  sd_adr_basesat = adr_rover_rx_base_sat - adr_reference_rx_base_sat;
  dd_adr = sd_adr - sd_adr_basesat;

  prev_sd_adr         = prev_adr_rover_rx - prev_adr_reference_rx;
  prev_sd_adr_basesat = prev_adr_rover_rx_base_sat - prev_adr_reference_rx_base_sat;
  prev_dd_adr         = prev_sd_adr - prev_sd_adr_basesat;

  sd = rover_range - reference_range;
  sd_basesat = rover_range_base_sat - reference_range_base_sat;
  dd = sd - sd_basesat;

  prev_sd         = prev_rover_range - prev_reference_range;
  prev_sd_basesat = prev_rover_range_base_sat - prev_reference_range_base_sat;
  prev_dd         = prev_sd - prev_sd_basesat;

  dd_diff = (dd - dd_adr*GPS_WAVELENGTHL1);
  dd_diff_prev = (prev_dd - prev_dd_adr*GPS_WAVELENGTHL1); 
  diff = dd_diff - dd_diff_prev;

  if( fabs(diff) > specifiedThresholdInCycles )
  {
    // the cycle slip has occured, but it is cannot be determined either on L1 or L2
    *wasSlipDetected    = TRUE;
    *sizeOfSlipInCycles = diff;
  }
  else
  {
    wasSlipDetected     = FALSE;
    *sizeOfSlipInCycles = 0;
  }
  return TRUE;
}