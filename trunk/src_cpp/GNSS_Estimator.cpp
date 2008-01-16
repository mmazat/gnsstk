/**
\file    GNSS_Estimator.cpp
\brief   The implementation file for the Esimtator class.

\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2006-11-16

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

#include <math.h>
#include <memory.h>
#include <list>

#include "GNSS_Estimator.h"
#include "GNSS_RxData.h"
#include "constants.h"
#include "geodesy.h"
#include "gps.h"
#include "ionosphere.h"
#include "navigation.h"
#include "troposphere.h"
#include "time_conversion.h"

#define DEBUG_THE_ESTIMATOR
#define GNSS_CYCLESLIP_THREADHOLD 1.5
#define KO_SECTION

using namespace std;


namespace GNSS
{

  /// The critical values of the chi-squared distribution with an alpha of .005 with v: 01..30 (31 values).
  /// \b REFERENCE \n
  /// Walpole, R. E, R. H Myers (1993), Probability and Statistics for Engineers and 
  /// Scientists, Prentice Hall Inc. ISBN 0-02-424301-2.
  #define GNSS_CHISQUARE_00_5 { \
   0.000,  7.879, 10.597, 12.838, 14.860, 16.750, 18.548, 20.278, 21.955, 23.589, \
  25.188, 26.757, 28.300, 29.819, 31.319, 32.801, 34.267, 35.718, 37.156, 38.582, \
  39.997, 41.401, 42.796, 44.181, 45.558, 46.928, 48.290, 49.645, 50.993, 52.336, 53.672 }


  GNSS_Estimator::GNSS_Estimator()
   : m_debug(NULL)
  {    
  }


  GNSS_Estimator::~GNSS_Estimator()
  {
    if( m_debug )
    {
      fclose(m_debug);
    }
  }


  bool GNSS_Estimator::PerformLeastSquares_8StatePVT(
    GNSS_RxData *rxData,       //!< A pointer to the rover receiver data. This must be a valid pointer.
    GNSS_RxData *rxBaseData,   //!< A pointer to the reference receiver data if available. NULL if not available.
    bool &wasPositionComputed, //!< A boolean to indicate if a position solution was computed.    
    bool &wasVelocityComputed  //!< A boolean to indicate if a velocity solution was computed.    
    )
  {
    unsigned i = 0;
    unsigned j = 0;
    //unsigned k = 0;
    unsigned iter = 0;
    unsigned n = 0; 
    unsigned nrValidEph = 0; // The number of channels with valid ephemeris.
    unsigned nrP = 0;        // The number of valid pseudorange measurements. 
    unsigned nrP_base = 0;   // The number of valid pseudorange measurements (base station).
    unsigned nrD = 0;        // The number of valid Doppler measurements.
    //unsigned nrD_base = 0;   // The number of valid Doppler measurements (base station).    
    unsigned nrDifferentialPsr = 0;     // The number of valid differntial pseudorange measurements.
    unsigned nrDifferentialDoppler = 0; // the number of valid differential Doppler measurements.

    //unsigned char isGood = 0; // A helper value.

    double dtmp1 = 0;     // A temp double.
    double lat = 0;       // latitude [rad].
    double lon = 0;       // longitude [rad].
    double hgt = 0;       // height [m].
    double clk = 0;       // rx clock bias [m].
    double clkdrift = 0;  // rx clock drift [m/s].
    double vn = 0;        // rx velocity north [m/s].
    double ve = 0;        // rx velocity east [m/s].
    double vup = 0;       // rx velocity up [m/s].
    double M = 0;   // The computed meridian radius of curvature [m].
    double N = 0;   // The computed prime vertical radius of curvature [m].
    
    Matrix dx_p(4);  // The iterative update to the position, clock states,         [4  x  1].   
    Matrix dx_v(4);  // The iterative update to the velocity, clk drift states,     [4  x  1].   
    Matrix P_p(4,4); // The position, clk, state variance covariance matrix,        [4  x  4].
    Matrix P_v(4,4); // The velocity, clk drift, state variance covariance matrix,  [4  x  4].
    Matrix H_p;      // The position design matrix                                  [nP x  4].
    Matrix Ht_p;     // The position design matrix transposed                       [4  x nP].
    Matrix HtW_p;    // An intermediate result                                      [4  x nP].
    Matrix H_v;      // The velocity design matrix                                  [nD x  4].
    Matrix Ht_v;     // The velocity design matrix transposed                       [4  x nD].
    Matrix HtW_v;    // An intermediate result                                      [4  x nD].
    Matrix W_p;      // The weight matrix for the psr observations,                 [nP x nP].
    Matrix R_p;      // The variance-covariance matrix for the psr observations,    [nP x nP].
    Matrix W_v;      // The weight matrix for the Doppler observations,             [nD x nD].
    Matrix R_v;      // The variance-covariance matrix for the Doppler observations,[nD x nD].
    Matrix w_p;      // The psr misclosures vector,                                 [nP x  1].
    Matrix w_v;      // The Doppler misclosure vector,                              [nD x  1].    
    Matrix r_p;      // The psr residuals vector,                                   [nP x  1].
    Matrix r_v;      // The Doppler residuals vector,                               [nD x  1].

    double avf_p = 0; // The a-posteriori variance factor for the position solution.
    //double avf_v = 0; // The a-posteriori variance factor for the velocity solution.

    bool isGlobalTestPassed_p = false; // This indicates if the Global Reliability Test passed for the position solution.
    //bool isGlobalTestPassed_v = false; // This indicates if the Global Reliability Test passed for the velocity solution.

    bool hasRejectionOccurred_p = false; // This indicates if a pseudorange measurement was rejected.
    //bool hasRejectionOccurred_v = false; // This indicates if a Doppler measurement was rejected.

    unsigned char indexOfRejected = 0; // The index into rxData.m_ObsArray of a rejected measurement.
    
    bool result = false;

    if( rxData == NULL )
      return false;

    // Store the current input pvt as the previous pvt since we are updating.
    rxData->m_prev_pvt = rxData->m_pvt;

    lat       = rxData->m_pvt.latitude;
    lon       = rxData->m_pvt.longitude;
    hgt       = rxData->m_pvt.height;
    clk       = rxData->m_pvt.clockOffset;

    vn        = rxData->m_pvt.vn;
    ve        = rxData->m_pvt.ve;
    vup       = rxData->m_pvt.vup;
    clkdrift  = rxData->m_pvt.clockDrift;

    wasPositionComputed = false;
    wasVelocityComputed = false;
       
    // Perform very basic uniqueness check.
    n = rxData->m_nrGPSL1Obs;
    if( rxData->m_pvt.isPositionConstrained )
    {
      n += 3;
    }
    else if( rxData->m_pvt.isHeightConstrained )
    {
      n += 1;
    }
    if( n < 4 )    
    {
      return true;
    }

    // Update the receiver time information (UTC and day of year)
    result = UpdateTime( *rxData );
    if( !result )
      return false;    
    if( rxBaseData != NULL )
    {
      result = UpdateTime( *rxBaseData );
      if( !result )
        return false;    
    }

    


    result = DetermineSatellitePVT_GPSL1( rxData, rxBaseData, nrValidEph );
    if( !result )
      return false;

    result = DetermineAtmosphericCorrections_GPSL1( *rxData );
    if( !result )
      return false;
    if( rxBaseData != NULL )
    {
      result = DetermineAtmosphericCorrections_GPSL1( *rxBaseData );
      if( !result )
        return false;
    }
    
    // Check uniqueness
    n = nrValidEph;
    if( rxData->m_pvt.isPositionConstrained )
    {
      n += 3;
    }
    else if( rxData->m_pvt.isHeightConstrained )
    {
      n += 1;
    }
    if( n < 4 )    
    {
      return true;
    }
  

    // Iterate through the solution
    for( iter = 0; iter < 7; iter++ )
    {
      if( !DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( *rxData, nrP ) )
        return false;

      // Check uniqueness
      n = nrP;
      if( rxData->m_pvt.isPositionConstrained )
      {
        n += 3;
      }
      else if( rxData->m_pvt.isHeightConstrained )
      {
        n += 1;
      }
      if( n < 4 )    
      {
        return true;
      }
  
      if( rxBaseData != NULL )
      {
        if( !DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( *rxBaseData, nrP_base ) )
          return false;

        result = DetermineBetweenReceiverDifferentialIndex(
          rxData,
          rxBaseData,
          true
          );
        if( !result )
          return false;

        nrDifferentialPsr = 0;
        for( i = 0; i < rxData->m_nrValidObs; i++ )
        {
          if( rxData->m_ObsArray[i].system == GNSS_GPS &&
            rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
            rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
          {
            nrDifferentialPsr++;
          }
        }

        nrP = nrDifferentialPsr;
        n = nrP;
        if( rxData->m_pvt.isPositionConstrained )
        {
          n += 3;
        }
        else if( rxData->m_pvt.isHeightConstrained )
        {
          n += 1;
        }
        if( n < 4 )    
        {
          return true;
        }
      }
  
      result = DetermineDesignMatrixForThePositionSolution_GPSL1(
        *rxData,
        n,
        H_p );
      if( !result )
        return false;
      
      result = DetermineMeasurementWeightMatrixForThePositionSolution_GPSL1(
        *rxData,
        n,
        W_p );
      if( !result )
        return false;

      result = DetermineMeasurementVarianceCovarianceMatrixForThePositionSolution_GPSL1(
        *rxData,
        n,
        R_p );
      if( !result )
        return false;

      result = DeterminePseudorangeMisclosures_GPSL1( 
        rxData, 
        rxBaseData,
        n, 
        w_p );
      if( !result )
        return false;

      // Compute Ht_p.
      Ht_p = H_p;
      if( !Ht_p.Inplace_Transpose() )
        return false;

      // Compute HtW_p.
      if( !HtW_p.Multiply( Ht_p, W_p ) )
        return false;

      // Compute P_p.
      if( !P_p.Multiply( HtW_p, H_p ) )
        return false;
      if( !P_p.Inplace_Invert() )
        return false;

      // Compute dx_p.
      if( !dx_p.Multiply( P_p, HtW_p ) )
        return false;
      if( !dx_p.Inplace_PostMultiply( w_p ) )
        return false;

      // Update the position and clock states
      // Update height first as it is need to reduce the corrections for lat and lon.
      hgt += dx_p[2];
      clk += dx_p[3];
     
      // The corrections for lat and lon, dx_p, must be converted to [rad] from [m].
      GEODESY_ComputePrimeVerticalRadiusOfCurvature(
        GEODESY_REFERENCE_ELLIPSE_WGS84, 
        rxData->m_pvt.latitude,
        &N );
      GEODESY_ComputeMeridianRadiusOfCurvature(
        GEODESY_REFERENCE_ELLIPSE_WGS84, 
        rxData->m_pvt.latitude,
        &M );

      lat += dx_p[0] / ( M + hgt );             // convert from meters to radians.
      lon += dx_p[1] / (( N + hgt )*cos(lat));  // convert from meters to radians.

      result = rxData->UpdatePositionAndRxClock(
        lat,
        lon,
        hgt,
        clk,
        sqrt(P_p[0][0]),
        sqrt(P_p[1][1]),
        sqrt(P_p[2][2]),
        sqrt(P_p[3][3])
        );
      if( !result )
        return false;

      dtmp1 = fabs(dx_p[0]) + fabs(dx_p[1]) + fabs(dx_p[2]) + fabs(dx_p[3]);
      if( dtmp1 < 0.0001 )
      {
        if( 0 )
        {
          // Test the residuals
          result = PerformGlobalTestAndTestForMeasurementFaults( 
            *rxData,
            true,
            H_p,
            Ht_p,      
            W_p,
            R_p,
            w_p, // misclosures at convergence are the residuals.
            P_p,
            n,
            4,
            avf_p,      
            isGlobalTestPassed_p,
            hasRejectionOccurred_p,
            indexOfRejected
            );
          if( !result )
            return false;

          if( hasRejectionOccurred_p )
          {
            hasRejectionOccurred_p = false;
            j = 0; // restart the iterative loop.
            continue;
          }
          else
          {
            // converged to solution.
            break;
          }
        }
        else
        {
          // converged to solution.
          break;
        }

      }
    }
    wasPositionComputed = true;


    // compute the DOP
    Matrix Q;
    Q = H_p;
    Q.Inplace_Transpose();
    Q.Inplace_PostMultiply( H_p );
    if( !Q.Inplace_Invert() )
      return false;

    rxData->m_pvt.dop.ndop = static_cast<float>( sqrt( Q[0][0] ) );
    rxData->m_pvt.dop.edop = static_cast<float>( sqrt( Q[1][1] ) );
    rxData->m_pvt.dop.vdop = static_cast<float>( sqrt( Q[2][2] ) );
    rxData->m_pvt.dop.tdop = static_cast<float>( sqrt( Q[3][3] ) );
    rxData->m_pvt.dop.hdop = rxData->m_pvt.dop.ndop * rxData->m_pvt.dop.ndop + rxData->m_pvt.dop.edop * rxData->m_pvt.dop.edop;
    rxData->m_pvt.dop.pdop = rxData->m_pvt.dop.hdop + rxData->m_pvt.dop.vdop * rxData->m_pvt.dop.vdop;
    rxData->m_pvt.dop.gdop = rxData->m_pvt.dop.pdop + rxData->m_pvt.dop.tdop * rxData->m_pvt.dop.tdop;
    rxData->m_pvt.dop.hdop = sqrt(rxData->m_pvt.dop.hdop);
    rxData->m_pvt.dop.pdop = sqrt(rxData->m_pvt.dop.pdop);
    rxData->m_pvt.dop.gdop = sqrt(rxData->m_pvt.dop.gdop);


    //
    ////


    ////
    // Velocity

    for( iter = 0; iter < 7; iter++ )
    {
      result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1(
        *rxData,
        nrD );
      if( !result )
        return false;

      // Check uniqueness
      n = nrD;
      if( rxData->m_pvt.isPositionConstrained )
      {
        n += 3;
      }
      else if( rxData->m_pvt.isHeightConstrained )
      {
        n += 1;
      }
      if( n < 4 )    
      {
        return true;
      }

      
      if( rxBaseData != NULL )
      {
        if( !DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( *rxBaseData, nrP_base ) )
          return false;

        result = DetermineBetweenReceiverDifferentialIndex(
          rxData,
          rxBaseData,
          true
          );
        if( !result )
          return false;

        nrDifferentialDoppler = 0;
        for( i = 0; i < rxData->m_nrValidObs; i++ )
        {
          if( rxData->m_ObsArray[i].system == GNSS_GPS &&
            rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
            rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
          {
            nrDifferentialDoppler++;
          }
        }

        // Check uniqueness.
        nrD = nrDifferentialDoppler;
        n = nrD;
        if( rxData->m_pvt.isPositionConstrained )
        {
          n += 3;
        }
        else if( rxData->m_pvt.isHeightConstrained )
        {
          n += 1;
        }
        if( n < 4 )    
        {
          return true;
        }
      }


      // Need to update the rxData->m_ObsArray[i].rangerate values.
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isActive )
        {
          if( rxData->m_ObsArray[i].system == GNSS_GPS && rxData->m_ObsArray[i].freqType == GNSS_GPSL1 )
          {
            if( rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
            {
              GPS_ComputeUserToSatelliteRangeAndRangeRate(
                rxData->m_pvt.x,
                rxData->m_pvt.y,
                rxData->m_pvt.z,
                rxData->m_pvt.vx,
                rxData->m_pvt.vy,
                rxData->m_pvt.vz,
                rxData->m_ObsArray[i].satellite.x,
                rxData->m_ObsArray[i].satellite.y,
                rxData->m_ObsArray[i].satellite.z,
                rxData->m_ObsArray[i].satellite.vx,
                rxData->m_ObsArray[i].satellite.vy,
                rxData->m_ObsArray[i].satellite.vz,
                &rxData->m_ObsArray[i].range,
                &rxData->m_ObsArray[i].rangerate
                );
            }
          }
        }
      }

      result = DetermineDesignMatrixForTheVelocitySolution_GPSL1(
        *rxData,
        n,
        H_v );
      if( !result )
        return false;

      result = DetermineMeasurementWeightMatrixForTheVelocitySolution_GPSL1(
        *rxData,
        n,
        W_v );
      if( !result )
        return false;

      result = DetermineMeasurementVarianceCovarianceMatrixForTheVelocitySolution_GPSL1(
        *rxData,
        n,
        R_v );
      if( !result )
        return false;

      result = DetermineDopplerMisclosures_GPSL1(
        rxData,
        rxBaseData,
        n,
        w_v );
      if( !result )
        return false;

      // Compute Ht_v.
      Ht_v = H_v;
      if( !Ht_v.Inplace_Transpose() )
        return false;

      // Compute HtW_v.
      if( !HtW_v.Multiply( Ht_v, W_v ) )
        return false;

      // Compute P_v.
      if( !P_v.Multiply( HtW_v, H_v ) )
        return false;
      if( !P_v.Inplace_Invert() )
        return false;

      // Compute dx_v.
      if( !dx_v.Multiply( P_v, HtW_v ) )
        return false;
      if( !dx_v.Inplace_PostMultiply( w_v ) )
        return false;

      // Update the velocity and clock drift states.
      vn        += dx_v[0];
      ve        += dx_v[1];
      vup       += dx_v[2];
      clkdrift  += dx_v[3];

      result = rxData->UpdateVelocityAndClockDrift(
        vn,
        ve,
        vup,
        clkdrift,
        P_v[0][0],
        P_v[1][1],
        P_v[2][2],
        P_v[3][3] );
      if( !result )
        return false;

      dtmp1 = fabs(dx_v[0]) + fabs(dx_v[1]) + fabs(dx_v[2]) + fabs(dx_v[3]);
      if( dtmp1 < 1.0e-10 )
        break;
    }
    wasVelocityComputed = true;

    return true;
  }


  bool GNSS_Estimator::UpdateTime(
    GNSS_RxData &rxData  //!< The receiver data. The m_pvt.time struct is updated.    
    )
  {
    TIMECONV_GetUTCTimeFromGPSTime(
      rxData.m_pvt.time.gps_week,
      rxData.m_pvt.time.gps_tow,
      &rxData.m_pvt.time.utc_year,
      &rxData.m_pvt.time.utc_month,
      &rxData.m_pvt.time.utc_day,
      &rxData.m_pvt.time.utc_hour,
      &rxData.m_pvt.time.utc_minute,
      &rxData.m_pvt.time.utc_seconds
      );

    TIMECONV_GetDayOfYear(
      rxData.m_pvt.time.utc_year,
      rxData.m_pvt.time.utc_month,
      rxData.m_pvt.time.utc_day,
      &rxData.m_pvt.time.day_of_year );
      
    return true;
  }
    

  bool GNSS_Estimator::DetermineSatellitePVT_GPSL1( 
    GNSS_RxData *rxData,      //!< The pointer to the receiver data.    
    GNSS_RxData *rxBaseData,  //!< The pointer to the reference receiver data. NULL if not available.
    unsigned &nrValidEph      //!< The number of GPS L1 channels with valid ephemeris for the rover.
    )
  {
    unsigned i = 0;
    double psr = 0;
    double dtmp1 = 0;
    double dtmp2 = 0;
    bool isEphAvailable = false;
    GPS_structEphemeris eph;

    memset( &eph, 0, sizeof(eph) );

    if( rxData == NULL )
      return false;

    if( rxBaseData != NULL )
    {
      // Evaluate the satellite PVT for all channels with GPS L1 observations 
      // for the reference station.
      for( i = 0; i < rxBaseData->m_nrValidObs; i++ )
      {
        if( rxBaseData->m_ObsArray[i].flags.isActive )
        {
          if( rxBaseData->m_ObsArray[i].system == GNSS_GPS && rxBaseData->m_ObsArray[i].freqType == GNSS_GPSL1 )
          {
            // Obtain the transmit time. Use an approximate psr measurement if one is not present.
            // The true transmit time includes compensatation for the satellite clock effects.            
            // p. 88 ICD-GPS-200C      

            rxBaseData->m_ObsArray[i].week = rxBaseData->m_pvt.time.gps_week;

            // first determine rough transmit time
            if( rxBaseData->m_ObsArray[i].flags.isCodeLocked & rxBaseData->m_ObsArray[i].flags.isPsrValid )
            { 
              psr = rxBaseData->m_ObsArray[i].psr;
            }
            else
            {
              psr = 0.070*LIGHTSPEED; // 0.070 rough transmission time
            }
            // Compute the rough transmit time.
            rxBaseData->m_ObsArray[i].tow = rxBaseData->m_pvt.time.gps_tow - psr/LIGHTSPEED;

            // Adjust for week rollover.
            if( rxBaseData->m_ObsArray[i].tow < 0 )
            {
              rxBaseData->m_ObsArray[i].tow += SECONDS_IN_WEEK;
              rxBaseData->m_ObsArray[i].week = rxBaseData->m_pvt.time.gps_week - 1;
            }
            else if( rxBaseData->m_ObsArray[i].tow > SECONDS_IN_WEEK )
            {
              rxBaseData->m_ObsArray[i].tow -= SECONDS_IN_WEEK;
              rxBaseData->m_ObsArray[i].week = rxBaseData->m_pvt.time.gps_week + 1;
            }

            // Get the ephemeris.
            if( !rxBaseData->m_EphAlmArray.GetEphemeris( rxBaseData->m_ObsArray[i].id, eph, isEphAvailable ) )
              return false;
            if( !isEphAvailable )
            {
              rxBaseData->m_ObsArray[i].satellite.isValid = false;
              rxBaseData->m_ObsArray[i].flags.isEphemerisValid = false;
              continue;
            }

            // Account for week rollover if needed.
            if( eph.week < 1024 )
              eph.week += 1024;


            // Check the age of the clock information for the ephemeris.
            dtmp1 = rxBaseData->m_ObsArray[i].week*SECONDS_IN_WEEK + rxBaseData->m_ObsArray[i].tow;
            dtmp2 = eph.week*SECONDS_IN_WEEK + eph.toe;
            rxBaseData->m_ObsArray[i].satellite.ageOfEph = static_cast<int>(dtmp1 - dtmp2);
            if( rxBaseData->m_ObsArray[i].satellite.ageOfEph > static_cast<int>(rxBaseData->m_maxAgeEphemeris) )
            {
              rxBaseData->m_ObsArray[i].satellite.isValid = false;
              rxBaseData->m_ObsArray[i].flags.isEphemerisValid = false; // old ephemeris data
              continue;
            }
            else
            {
              rxBaseData->m_ObsArray[i].satellite.isValid = true;
            }

            rxBaseData->m_ObsArray[i].flags.isEphemerisValid = true;

            // Compute the satellite clock corrections, position, velocity, etc.
            GPS_ComputeSatellitePositionVelocityAzimuthElevationDoppler_BasedOnEphmerisData(
              rxBaseData->m_pvt.x,
              rxBaseData->m_pvt.y,
              rxBaseData->m_pvt.z,
              rxBaseData->m_ObsArray[i].week,
              rxBaseData->m_ObsArray[i].tow,              
              eph.week,
              eph.toe,
              eph.toc,
              eph.af0,
              eph.af1,
              eph.af2,
              eph.tgd,
              eph.m0, 
              eph.delta_n,
              eph.ecc, 
              eph.sqrta, 
              eph.omega0,
              eph.i0,    
              eph.w,     
              eph.omegadot,
              eph.idot,    
              eph.cuc,     
              eph.cus,     
              eph.crc,     
              eph.crs,     
              eph.cic,     
              eph.cis,     
              &rxBaseData->m_ObsArray[i].satellite.clk,
              &rxBaseData->m_ObsArray[i].satellite.clkdrift,
              &rxBaseData->m_ObsArray[i].satellite.x,
              &rxBaseData->m_ObsArray[i].satellite.y,
              &rxBaseData->m_ObsArray[i].satellite.z,
              &rxBaseData->m_ObsArray[i].satellite.vx,
              &rxBaseData->m_ObsArray[i].satellite.vy,
              &rxBaseData->m_ObsArray[i].satellite.vz,
              &rxBaseData->m_ObsArray[i].satellite.azimuth,
              &rxBaseData->m_ObsArray[i].satellite.elevation,
              &rxBaseData->m_ObsArray[i].satellite.doppler
              );

            rxBaseData->m_ObsArray[i].corrections.prcSatClk = static_cast<float>(rxBaseData->m_ObsArray[i].satellite.clk);
            rxBaseData->m_ObsArray[i].corrections.rrcSatClkDrift = static_cast<float>(rxBaseData->m_ObsArray[i].satellite.clkdrift);

            // Compute the reference station geometric range and range rate.
            GPS_ComputeUserToSatelliteRangeAndRangeRate(
              rxBaseData->m_pvt.x,
              rxBaseData->m_pvt.y,
              rxBaseData->m_pvt.z,
              rxBaseData->m_pvt.vx,
              rxBaseData->m_pvt.vy,
              rxBaseData->m_pvt.vz,
              rxBaseData->m_ObsArray[i].satellite.x,
              rxBaseData->m_ObsArray[i].satellite.y,
              rxBaseData->m_ObsArray[i].satellite.z,
              rxBaseData->m_ObsArray[i].satellite.vx,
              rxBaseData->m_ObsArray[i].satellite.vy,
              rxBaseData->m_ObsArray[i].satellite.vz,
              &rxBaseData->m_ObsArray[i].range,
              &rxBaseData->m_ObsArray[i].rangerate );
          }
        }
      }
    }

    // Evaluate the satellite PVT for all channesl with GPS L1 observations       
    // for the rover station using the reference station ephemeris if available.   
    nrValidEph = 0;
    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( rxData->m_ObsArray[i].flags.isActive )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS && rxData->m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          // Obtain the transmit time. Use an approximate psr measurement if one is not present.
          // The true transmit time includes compensatation for the satellite clock effects.            
          // p. 88 ICD-GPS-200C      

          rxData->m_ObsArray[i].week = rxData->m_pvt.time.gps_week;

          // first determine rough transmit time
          if( rxData->m_ObsArray[i].flags.isCodeLocked & rxData->m_ObsArray[i].flags.isPsrValid )
          { 
            psr = rxData->m_ObsArray[i].psr;
          }
          else
          {
            psr = 0.070*LIGHTSPEED; // 0.070 rough transmission time
          }
          // Compute the rough transmit time.
          rxData->m_ObsArray[i].tow = rxData->m_pvt.time.gps_tow - psr/LIGHTSPEED;

          // Adjust for week rollover.
          if( rxData->m_ObsArray[i].tow < 0 )
          {
            rxData->m_ObsArray[i].tow += SECONDS_IN_WEEK;
            rxData->m_ObsArray[i].week = rxData->m_pvt.time.gps_week - 1;
          }
          else if( rxData->m_ObsArray[i].tow > SECONDS_IN_WEEK )
          {
            rxData->m_ObsArray[i].tow -= SECONDS_IN_WEEK;
            rxData->m_ObsArray[i].week = rxData->m_pvt.time.gps_week + 1;
          }

          if( rxBaseData != NULL )
          {
            // Get the ephemeris using the reference station if available.
            if( !rxBaseData->m_EphAlmArray.GetEphemeris( rxData->m_ObsArray[i].id, eph, isEphAvailable ) )
              return false;
            if( !isEphAvailable )
            {
              // Get the ephemeris using the rover station then.
              if( !rxData->m_EphAlmArray.GetEphemeris( rxData->m_ObsArray[i].id, eph, isEphAvailable ) )
                return false;
              if( !isEphAvailable )
              {
                rxData->m_ObsArray[i].satellite.isValid = false;
                rxData->m_ObsArray[i].flags.isEphemerisValid = false; // no reference station ephemeris data
                continue;
              }
            }
          }
          else
          {
            // Get the ephemeris using the rover station then.
            if( !rxData->m_EphAlmArray.GetEphemeris( rxData->m_ObsArray[i].id, eph, isEphAvailable ) )
              return false;
            if( !isEphAvailable )
            {
              rxData->m_ObsArray[i].satellite.isValid = false;
              rxData->m_ObsArray[i].flags.isEphemerisValid = false; // no reference station ephemeris data
              continue;
            }
          }

          rxData->m_ObsArray[i].flags.isEphemerisValid = true;

          // Account for week rollover if needed.
          if( eph.week < 1024 )
            eph.week += 1024;

          // Check the age of the clock information for the ephemeris.
          dtmp1 = rxData->m_ObsArray[i].week*SECONDS_IN_WEEK + rxData->m_ObsArray[i].tow;
          dtmp2 = eph.week*SECONDS_IN_WEEK + eph.toe;
          rxData->m_ObsArray[i].satellite.ageOfEph = static_cast<int>(dtmp1 - dtmp2);
          if( rxData->m_ObsArray[i].satellite.ageOfEph > static_cast<int>(rxData->m_maxAgeEphemeris) )
          {
            rxData->m_ObsArray[i].satellite.isValid = false;
            rxData->m_ObsArray[i].flags.isEphemerisValid = false; // old ephemeris data
            continue;
          }
          else
          {
            rxData->m_ObsArray[i].satellite.isValid = true;
          }

          // Compute the satellite clock corrections, position, velocity, etc.
          GPS_ComputeSatellitePositionVelocityAzimuthElevationDoppler_BasedOnEphmerisData(
            rxData->m_pvt.x,
            rxData->m_pvt.y,
            rxData->m_pvt.z,
            rxData->m_ObsArray[i].week,
            rxData->m_ObsArray[i].tow,              
            eph.week,
            eph.toe,
            eph.toc,
            eph.af0,
            eph.af1,
            eph.af2,
            eph.tgd,
            eph.m0, 
            eph.delta_n,
            eph.ecc, 
            eph.sqrta, 
            eph.omega0,
            eph.i0,    
            eph.w,     
            eph.omegadot,
            eph.idot,    
            eph.cuc,     
            eph.cus,     
            eph.crc,     
            eph.crs,     
            eph.cic,     
            eph.cis,     
            &rxData->m_ObsArray[i].satellite.clk,
            &rxData->m_ObsArray[i].satellite.clkdrift,
            &rxData->m_ObsArray[i].satellite.x,
            &rxData->m_ObsArray[i].satellite.y,
            &rxData->m_ObsArray[i].satellite.z,
            &rxData->m_ObsArray[i].satellite.vx,
            &rxData->m_ObsArray[i].satellite.vy,
            &rxData->m_ObsArray[i].satellite.vz,
            &rxData->m_ObsArray[i].satellite.azimuth,
            &rxData->m_ObsArray[i].satellite.elevation,
            &rxData->m_ObsArray[i].satellite.doppler
            );

          rxData->m_ObsArray[i].corrections.prcSatClk = static_cast<float>(rxData->m_ObsArray[i].satellite.clk);
          rxData->m_ObsArray[i].corrections.rrcSatClkDrift = static_cast<float>(rxData->m_ObsArray[i].satellite.clkdrift);

          // Compute the rover station geometric range and range rate.
          GPS_ComputeUserToSatelliteRangeAndRangeRate(
            rxData->m_pvt.x,
            rxData->m_pvt.y,
            rxData->m_pvt.z,
            rxData->m_pvt.vx,
            rxData->m_pvt.vy,
            rxData->m_pvt.vz,
            rxData->m_ObsArray[i].satellite.x,
            rxData->m_ObsArray[i].satellite.y,
            rxData->m_ObsArray[i].satellite.z,
            rxData->m_ObsArray[i].satellite.vx,
            rxData->m_ObsArray[i].satellite.vy,
            rxData->m_ObsArray[i].satellite.vz,
            &rxData->m_ObsArray[i].range,
            &rxData->m_ObsArray[i].rangerate );

          nrValidEph++;
        }
      }
    }
    return true;
  }

  bool GNSS_Estimator::DetermineAtmosphericCorrections_GPSL1( 
    GNSS_RxData &rxData  //!< The receiver data.        
    )
  {
    unsigned i = 0;
    double zenith_dry_delay = 0;
    double zenith_wet_delay = 0;
    double dtmp1 = 0;
    double dtmp2 = 0;

    // Compute the tropospheric delays.
    TROPOSPHERE_DetermineZenithDelayValues_WAAS_Model(
      rxData.m_pvt.latitude,
      rxData.m_pvt.height,
      rxData.m_pvt.time.day_of_year,
      &zenith_dry_delay,
      &zenith_wet_delay
      );

    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( rxData.m_ObsArray[i].flags.isActive )
      {
        if( rxData.m_ObsArray[i].system == GNSS_GPS && rxData.m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          if( rxData.m_ObsArray[i].flags.isEphemerisValid )
          {
            // Always compute the tropospheric correction (it may not be applied though).
            TROPOSPHERE_GetDryAndWetDelay_UsingThe_UNBabc_MappingFunction(
              zenith_dry_delay, 
              zenith_wet_delay, 
              rxData.m_ObsArray[i].satellite.elevation,
              rxData.m_pvt.latitude,              
              rxData.m_pvt.height,
              &dtmp1,
              &dtmp2
              );
            
            rxData.m_ObsArray[i].corrections.prcTropoDry = static_cast<float>(dtmp1);
            rxData.m_ObsArray[i].corrections.prcTropoWet = static_cast<float>(dtmp2);

            if( rxData.m_klobuchar.isValid )
            {
              // Always compute the ionospheric correction (it may not be applied though).
              IONOSPHERE_GetL1KlobucharCorrection(
                rxData.m_klobuchar.alpha0,
                rxData.m_klobuchar.alpha1,
                rxData.m_klobuchar.alpha2,
                rxData.m_klobuchar.alpha3,
                rxData.m_klobuchar.beta0,
                rxData.m_klobuchar.beta1,
                rxData.m_klobuchar.beta2,
                rxData.m_klobuchar.beta3,
                rxData.m_pvt.latitude,
                rxData.m_pvt.longitude, 
                rxData.m_ObsArray[i].satellite.elevation,
                rxData.m_ObsArray[i].satellite.azimuth,
                rxData.m_pvt.time.gps_tow,
                &dtmp1
                );
              rxData.m_ObsArray[i].corrections.prcIono = static_cast<float>(dtmp1);
            }
          }
        }
      }
    }
    return true;
  }



  bool GNSS_Estimator::DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( 
    GNSS_RxData &rxData,            //!< The receiver data.
    unsigned &nrUsablePseudoranges  //!< the number of usable GPS L1 pseudorange measurements.
    )
  {
    unsigned i = 0;
    unsigned char isGood = 0;

    rxData.m_pvt.nrPsrObsAvailable = 0;
    rxData.m_pvt.nrPsrObsUsed = 0;
    rxData.m_pvt.nrPsrObsRejected = 0;

    nrUsablePseudoranges = 0;    
    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( rxData.m_ObsArray[i].flags.isActive )
      {
        if( rxData.m_ObsArray[i].system == GNSS_GPS && rxData.m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          if( rxData.m_ObsArray[i].satellite.elevation < rxData.m_elevationMask )              
          {
            rxData.m_ObsArray[i].flags.isAboveElevationMask = 0;
          }
          else
          {
            rxData.m_ObsArray[i].flags.isAboveElevationMask = 1;
          }

          if( rxData.m_ObsArray[i].cno < rxData.m_cnoMask )
          {
            rxData.m_ObsArray[i].flags.isAboveCNoMask = 0;
          }
          else
          {
            rxData.m_ObsArray[i].flags.isAboveCNoMask = 1;
          }

          if( rxData.m_ObsArray[i].locktime < rxData.m_locktimeMask )
          {
            rxData.m_ObsArray[i].flags.isAboveLockTimeMask = 0;
          }
          else
          {
            rxData.m_ObsArray[i].flags.isAboveLockTimeMask = 1;
          }

          isGood = 
            rxData.m_ObsArray[i].flags.isCodeLocked         & 
            rxData.m_ObsArray[i].flags.isPsrValid           &
            rxData.m_ObsArray[i].flags.isAboveElevationMask &
            rxData.m_ObsArray[i].flags.isAboveCNoMask       &
            rxData.m_ObsArray[i].flags.isAboveLockTimeMask  &
            rxData.m_ObsArray[i].flags.isNotUserRejected    &
            rxData.m_ObsArray[i].flags.isNotPsrRejected     &
            rxData.m_ObsArray[i].flags.isEphemerisValid;
          if( isGood )
          {
            rxData.m_ObsArray[i].flags.isPsrUsedInSolution = 1;
            nrUsablePseudoranges++;            
          }
          else
          {
            rxData.m_ObsArray[i].flags.isPsrUsedInSolution = 0;
          }
        }
      }
    }
    rxData.m_pvt.nrPsrObsAvailable = nrUsablePseudoranges;
    return true;
  }


  bool GNSS_Estimator::DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( 
    GNSS_RxData &rxData,       //!< The receiver data.
    unsigned &nrUsableDopplers //!< the number of usable GPS L1 Doppler measurements.
    )
  {
    unsigned i = 0;
    unsigned char isGood = 0;

    rxData.m_pvt.nrDopplerObsAvailable = 0;
    rxData.m_pvt.nrDopplerObsUsed = 0;
    rxData.m_pvt.nrDopplerObsRejected = 0;

    nrUsableDopplers = 0;    
    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( rxData.m_ObsArray[i].flags.isActive )
      {
        if( rxData.m_ObsArray[i].system == GNSS_GPS && rxData.m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          isGood = 
            rxData.m_ObsArray[i].flags.isCodeLocked         & 
            rxData.m_ObsArray[i].flags.isDopplerValid       &
            rxData.m_ObsArray[i].flags.isAboveElevationMask &
            rxData.m_ObsArray[i].flags.isAboveCNoMask       &
            rxData.m_ObsArray[i].flags.isAboveLockTimeMask  &
            rxData.m_ObsArray[i].flags.isNotUserRejected    &
            rxData.m_ObsArray[i].flags.isNotDopplerRejected &
            rxData.m_ObsArray[i].flags.isEphemerisValid;
          if( isGood )
          {
            rxData.m_ObsArray[i].flags.isDopplerUsedInSolution = 1;
            nrUsableDopplers++;            
          }
          else
          {
            rxData.m_ObsArray[i].flags.isDopplerUsedInSolution = 0;
          }
        }
      }
    }
    rxData.m_pvt.nrDopplerObsAvailable = nrUsableDopplers;
    return true;
  }


  bool GNSS_Estimator::DetermineUsableAdrMeasurements_GPSL1( 
    GNSS_RxData &rxData,   //!< The receiver data.
    unsigned &nrUsableAdr  //!< The number of usable GPS L1 adr measurements.
    )
  {
    unsigned i = 0;
    unsigned char isGood = 0;

    rxData.m_pvt.nrAdrObsAvailable = 0;
    rxData.m_pvt.nrAdrObsUsed = 0;
    rxData.m_pvt.nrAdrObsRejected = 0;

    // Check for cycle slips.
    rxData.CheckForCycleSlips_UsingPhaseRatePrediction( GNSS_CYCLESLIP_THREADHOLD );

    nrUsableAdr = 0;    
    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( rxData.m_ObsArray[i].flags.isActive )
      {
        if( rxData.m_ObsArray[i].system == GNSS_GPS && rxData.m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          if( rxData.m_ObsArray[i].satellite.elevation < rxData.m_elevationMask )              
            rxData.m_ObsArray[i].flags.isAboveElevationMask = 0;
          else
            rxData.m_ObsArray[i].flags.isAboveElevationMask = 1;

          if( rxData.m_ObsArray[i].cno < rxData.m_cnoMask )
            rxData.m_ObsArray[i].flags.isAboveCNoMask = 0;
          else
            rxData.m_ObsArray[i].flags.isAboveCNoMask = 1;

          if( rxData.m_ObsArray[i].locktime < rxData.m_locktimeMask )
            rxData.m_ObsArray[i].flags.isAboveLockTimeMask = 0;
          else
            rxData.m_ObsArray[i].flags.isAboveLockTimeMask = 1;

          isGood = 
            rxData.m_ObsArray[i].flags.isCodeLocked         &
            rxData.m_ObsArray[i].flags.isPhaseLocked        &
            rxData.m_ObsArray[i].flags.isParityValid        & // if not, there is a half cycle amibiguity.
            rxData.m_ObsArray[i].flags.isAdrValid           &
            rxData.m_ObsArray[i].flags.isAboveElevationMask &
            rxData.m_ObsArray[i].flags.isAboveCNoMask       &
            rxData.m_ObsArray[i].flags.isAboveLockTimeMask  &
            rxData.m_ObsArray[i].flags.isNotUserRejected    &
            rxData.m_ObsArray[i].flags.isNotAdrRejected     &
            rxData.m_ObsArray[i].flags.isEphemerisValid;
          if( isGood )
          {
            rxData.m_ObsArray[i].flags.isAdrUsedInSolution = 1;
            nrUsableAdr++;            
          }
          else
          {
            rxData.m_ObsArray[i].flags.isAdrUsedInSolution = 0;
          }
        }
      }
    }
    rxData.m_pvt.nrAdrObsAvailable = nrUsableAdr;
    return true;
  }


  bool GNSS_Estimator::DetermineDesignMatrixForThePositionSolution_GPSL1( 
    GNSS_RxData &rxData,      //!< The receiver data.
    const unsigned nrRowsInH, //!< The number of valid rows in H.
    Matrix &H                 //!< The position & rx clock design matrix [n x 4].
    )
  {
    unsigned i = 0;
    unsigned j = 0;

    // Check the dimension of H.
    if( H.GetNrRows() != nrRowsInH || H.GetNrCols() != 4 )
    {
      if( !H.Resize( nrRowsInH, 4 ) )
        return false;
    }
    // Compute the design matrix for the position solution.
    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( rxData.m_ObsArray[i].flags.isActive )
      {
        if( rxData.m_ObsArray[i].system == GNSS_GPS && rxData.m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          if( rxData.m_ObsArray[i].flags.isPsrUsedInSolution )
          {
            // Sanity index check.            
            if( j >= nrRowsInH )
              return false;
            
            NAVIGATION_ComputeDerivativesOf_Range_WithRespectToLatitudeLongitudeHeight(
              rxData.m_pvt.latitude,
              rxData.m_pvt.longitude, 
              rxData.m_pvt.height,
              rxData.m_ObsArray[i].satellite.x,
              rxData.m_ObsArray[i].satellite.y,
              rxData.m_ObsArray[i].satellite.z,
              &H[j][0],
              &H[j][1],
              &H[j][2],
              &rxData.m_ObsArray[i].range
              );
            H[j][3] = 1.0;

            // copy the row of the design matrix 
            rxData.m_ObsArray[i].H_p[0] = H[j][0];
            rxData.m_ObsArray[i].H_p[1] = H[j][1];
            rxData.m_ObsArray[i].H_p[2] = H[j][2];

            j++;
          }
        }
      }
    }

    // Constraint rows of the design matrix.
    if( rxData.m_pvt.isPositionConstrained )
    {
      if( j + 3 > nrRowsInH )
        return false;
            
      H[j][0] = 1.0; 
      H[j][1] = 0.0; 
      H[j][2] = 0.0; 
      H[j][3] = 0.0; 
      j++;
      H[j][0] = 0.0; 
      H[j][1] = 1.0; 
      H[j][2] = 0.0; 
      H[j][3] = 0.0;       
      j++;
      H[j][0] = 0.0; 
      H[j][1] = 0.0; 
      H[j][2] = 1.0; 
      H[j][3] = 0.0;       
      j++;      
    }
    else if( rxData.m_pvt.isHeightConstrained )
    {
      if( j + 1 > nrRowsInH )
        return false;
            
      H[j][0] = 0.0; 
      H[j][1] = 0.0; 
      H[j][2] = 1.0; 
      H[j][3] = 0.0;             
      j++;
    }

    return true;
  }


  bool GNSS_Estimator::DetermineMeasurementWeightMatrixForThePositionSolution_GPSL1( 
    GNSS_RxData &rxData,  //!< The receiver data.
    const unsigned n,     //!< The number of measurements and pseudo-measurements (constraints).
    Matrix &W             //!< The inverse of the pseudorange measurement variance-coraiance matrix [n x n].
    )
  {
    unsigned i = 0;
    unsigned j = 0;

    // Check the dimension of W.
    if( W.GetNrRows() != n || W.GetNrCols() != n )
    {
      if( !W.Resize( n, n ) )
        return false;
    }
    // Set the weight matrix.
    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( rxData.m_ObsArray[i].flags.isActive )
      {
        if( rxData.m_ObsArray[i].system == GNSS_GPS && rxData.m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          if( rxData.m_ObsArray[i].flags.isPsrUsedInSolution )
          {
            // Sanity index check.
            if( j >= n )
              return false;

            // Check divide by zero.
            if( rxData.m_ObsArray[i].stdev_psr == 0.0 )
              return false;

            W[j][j] = 1.0/(rxData.m_ObsArray[i].stdev_psr*rxData.m_ObsArray[i].stdev_psr);
            j++;
          }
        }
      }
    }

    // Deal with constraints.
    if( rxData.m_pvt.isPositionConstrained )
    {
      if( j + 3 > n )
        return false;
            
      W[j][j] = 1.0/(rxData.m_pvt.std_lat*rxData.m_pvt.std_lat); 
      j++;
      W[j][j] = 1.0/(rxData.m_pvt.std_lon*rxData.m_pvt.std_lon); 
      j++;
      W[j][j] = 1.0/(rxData.m_pvt.std_hgt*rxData.m_pvt.std_hgt); 
      j++;      
    }
    else if( rxData.m_pvt.isHeightConstrained )
    {
      if( j + 1 > n )
        return false;
            
      W[j][j] = 1.0/(rxData.m_pvt.std_hgt*rxData.m_pvt.std_hgt); 
      j++;
    }

    return true;
  }


  bool GNSS_Estimator::DetermineMeasurementVarianceCovarianceMatrixForThePositionSolution_GPSL1( 
    GNSS_RxData &rxData,  //!< The receiver data.
    const unsigned n,     //!< The number of measurements and pseudo-measurements (constraints).
    Matrix &R             //!< The pseudorange measurement variance-covariance matrix [n x n].
    )
  {
    unsigned i = 0;
    unsigned j = 0;

    // Check the dimension of W.
    if( R.GetNrRows() != n || R.GetNrCols() != n )
    {
      if( !R.Resize( n, n ) )
        return false;
    }
    // Set the weight matrix.
    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( rxData.m_ObsArray[i].flags.isActive )
      {
        if( rxData.m_ObsArray[i].system == GNSS_GPS && rxData.m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          if( rxData.m_ObsArray[i].flags.isPsrUsedInSolution )
          {
            // Sanity index check.
            if( j >= n )
              return false;

            // Check divide by zero.
            if( rxData.m_ObsArray[i].stdev_psr == 0.0 )
              return false;

            R[j][j] = rxData.m_ObsArray[i].stdev_psr*rxData.m_ObsArray[i].stdev_psr;
            j++;
          }
        }
      }
    }

    // Deal with constraints.
    if( rxData.m_pvt.isPositionConstrained )
    {
      if( j + 3 > n )
        return false;
            
      R[j][j] = rxData.m_pvt.std_lat*rxData.m_pvt.std_lat; 
      j++;
      R[j][j] = rxData.m_pvt.std_lon*rxData.m_pvt.std_lon; 
      j++;
      R[j][j] = rxData.m_pvt.std_hgt*rxData.m_pvt.std_hgt; 
      j++;      
    }
    else if( rxData.m_pvt.isHeightConstrained )
    {
      if( j + 1 > n )
        return false;
            
      R[j][j] = rxData.m_pvt.std_hgt*rxData.m_pvt.std_hgt; 
      j++;
    }

    return true;
  }



  bool GNSS_Estimator::DeterminePseudorangeMisclosures_GPSL1( 
    GNSS_RxData *rxData,     //!< The pointer to the receiver data.    
    GNSS_RxData *rxBaseData, //!< The pointer to the reference receiver data. NULL if not available.    
    const unsigned n,        //!< The number of misclosures.
    Matrix &w                //!< The pseudorange misclosure vector [n x 1].
    )
  {
    unsigned i = 0;
    unsigned j = 0;
    int k = 0;
    double psr_base = 0;
    double range_base = 0;
    double psr_measured = 0;
    double psr_computed = 0;

    // Check the dimension of w.
    if( w.GetNrRows() != n || w.GetNrCols() != 1 )
    {
      if( !w.Resize( n, 1 ) )
        return false;
    }

    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( rxData->m_ObsArray[i].flags.isActive )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS && rxData->m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          // allows compute the misclosure value (regardless of whether it is used in solution
          psr_measured = rxData->m_ObsArray[i].psr;

          // Add the satellite clock correction.
          psr_measured += rxData->m_ObsArray[i].satellite.clk;

          // Compensate for the ionospheric delay if indicated.
          // The corrections must be determined beforehand.
          if( rxData->m_ObsArray[i].flags.useBroadcastIonoCorrection )
          {
            // Compensate for the ionospheric delay
            psr_measured -= rxData->m_ObsArray[i].corrections.prcIono;
          }

          // Compensate for the tropospheric delay if indicated.
          // The corrections must be determined beforehand.
          if( rxData->m_ObsArray[i].flags.useTropoCorrection )
          {
            // Compensate for the tropospheric delay
            psr_measured -= rxData->m_ObsArray[i].corrections.prcTropoDry;
            psr_measured -= rxData->m_ObsArray[i].corrections.prcTropoWet;
          }

          range_base = 0;
          if( rxBaseData != NULL )
          {    
            if( rxData->m_ObsArray[i].flags.isDifferentialPsrAvailable )
            {
              k = rxData->m_ObsArray[i].index_differential;
              if( k != -1 )
              {
                psr_base  = rxBaseData->m_ObsArray[k].psr;
                psr_base += rxBaseData->m_ObsArray[k].satellite.clk;

                // Compensate for the ionospheric delay if indicated.
                // The corrections must be determined beforehand.
                if( rxBaseData->m_ObsArray[k].flags.useBroadcastIonoCorrection )
                {
                  // Compensate for the ionospheric delay
                  psr_base -= rxBaseData->m_ObsArray[k].corrections.prcIono;
                }

                // Compensate for the tropospheric delay if indicated.
                // The corrections must be determined beforehand.
                if( rxBaseData->m_ObsArray[k].flags.useTropoCorrection )
                {
                  // Compensate for the tropospheric delay
                  psr_base -= rxBaseData->m_ObsArray[k].corrections.prcTropoDry;
                  psr_base -= rxBaseData->m_ObsArray[k].corrections.prcTropoWet;
                }                  

                range_base = rxBaseData->m_ObsArray[k].range;

                // Compute the differential pseudorange.
                psr_measured -= psr_base;
              }
            }
          }

          // Calculate the computed pseudorange = geometric range + clock offset (m)
          // The range value and the clock offset must be determined beforehand.
          // If differential, range_base != 0, it is the computed psuedorange difference.
          psr_computed = rxData->m_ObsArray[i].range - range_base + rxData->m_pvt.clockOffset;

          // The misclosure is the corrected measured value minus the computed valid.
          rxData->m_ObsArray[i].psr_misclosure = psr_measured - psr_computed;            

          if( rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
          {
            // Sanity index check.
            if( j >= n )
              return false;

            w[j] = rxData->m_ObsArray[i].psr_misclosure;
            j++;
          }
        }
        else
        {
          rxData->m_ObsArray[i].psr_misclosure = 0.0;            
        }
      }
    }

    // Deal with constraints.
    if( rxData->m_pvt.isPositionConstrained )
    {
      if( j + 3 > n )
        return false;

      double N;
      double M;
      double diff;

      GEODESY_ComputePrimeVerticalRadiusOfCurvature(
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        rxData->m_prev_pvt.latitude,
        &N );
      GEODESY_ComputeMeridianRadiusOfCurvature( 
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        rxData->m_prev_pvt.latitude,
        &M );
            
      diff = (rxData->m_prev_pvt.latitude - rxData->m_pvt.latitude);
      diff *= ( M + rxData->m_prev_pvt.height ); // convert to meters.
      w[j] = diff;
      j++;
      diff = (rxData->m_prev_pvt.longitude - rxData->m_pvt.longitude );
      diff *= (N + rxData->m_prev_pvt.height)*cos(rxData->m_prev_pvt.latitude); // convert to meters.
      w[j] = diff;
      j++;
      w[j] = rxData->m_prev_pvt.height - rxData->m_pvt.height;
      j++;      
    }
    else if( rxData->m_pvt.isHeightConstrained )
    {
      if( j + 1 > n )
        return false;
            
      w[j] = rxData->m_prev_pvt.height - rxData->m_pvt.height;
      j++;
    }


    return true;
  }


  bool GNSS_Estimator::DetermineBetweenReceiverDifferentialIndex(
    GNSS_RxData *rxData,                 //!< (input) The pointer to the receiver data.    
    GNSS_RxData *rxBaseData,             //!< (input) The pointer to the reference receiver data. NULL if not available.        
    const bool setToUseOnlyDifferential  //!< (input) This indicates that only differential measurements should be used.    
    )
  {
    unsigned i = 0;
    unsigned j = 0;
    //unsigned char isGood = 0;

    // Initial the indices of the corresponding differential measurements.
    for( i = 0; i < rxData->m_nrValidObs; i++ ) 
    { 
      rxData->m_ObsArray[i].index_differential = -1; 
      rxData->m_ObsArray[i].flags.isDifferentialPsrAvailable = 0;      
      rxData->m_ObsArray[i].flags.isDifferentialAdrAvailable = 0;      
      rxData->m_ObsArray[i].flags.isDifferentialDopplerAvailable = 0;      
    }
    for( i = 0; i < rxBaseData->m_nrValidObs; i++ ) 
    { 
      rxBaseData->m_ObsArray[i].index_differential = -1; 
      rxBaseData->m_ObsArray[i].flags.isDifferentialPsrAvailable = 0;
      rxBaseData->m_ObsArray[i].flags.isDifferentialAdrAvailable = 0;
      rxBaseData->m_ObsArray[i].flags.isDifferentialDopplerAvailable = 0;
    }
    if( rxBaseData == NULL )
      return true;

    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( rxData->m_ObsArray[i].flags.isActive )
      {
        // Look for a matching differential channel in the base station data.
        for( j = 0; j < rxBaseData->m_nrValidObs; j++ )
        {
          if( rxBaseData->m_ObsArray[j].flags.isActive )
          {
            if( rxData->m_ObsArray[i].system == rxBaseData->m_ObsArray[j].system &&
              rxData->m_ObsArray[i].codeType == rxBaseData->m_ObsArray[j].codeType &&
              rxData->m_ObsArray[i].freqType == rxBaseData->m_ObsArray[j].freqType &&
              rxData->m_ObsArray[i].id       == rxBaseData->m_ObsArray[j].id )
            {
              rxData->m_ObsArray[i].index_differential     = j;
              rxBaseData->m_ObsArray[j].index_differential = i;

              if( rxData->m_ObsArray[i].flags.isPsrUsedInSolution &&
                rxBaseData->m_ObsArray[j].flags.isPsrUsedInSolution )
              {
                rxData->m_ObsArray[i].flags.isDifferentialPsrAvailable = 1;
                rxBaseData->m_ObsArray[j].flags.isDifferentialPsrAvailable = 1;
              }

              if( rxData->m_ObsArray[i].flags.isDopplerUsedInSolution &&
                rxBaseData->m_ObsArray[j].flags.isDopplerUsedInSolution )
              {
                rxData->m_ObsArray[i].flags.isDifferentialDopplerAvailable = 1;
                rxBaseData->m_ObsArray[j].flags.isDifferentialDopplerAvailable = 1;
              }

              if( rxData->m_ObsArray[i].flags.isAdrUsedInSolution &&
                rxBaseData->m_ObsArray[j].flags.isAdrUsedInSolution )
              {
                rxData->m_ObsArray[i].flags.isDifferentialAdrAvailable = 1;
                rxBaseData->m_ObsArray[j].flags.isDifferentialAdrAvailable = 1;
              }

              break;
            }
          }
        }
      }
    }

    if( setToUseOnlyDifferential )
    {
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {

        if( rxData->m_ObsArray[i].flags.isActive )
        {

          if( rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
          {
            if( !rxData->m_ObsArray[i].flags.isDifferentialPsrAvailable )
            {
              rxData->m_ObsArray[i].flags.isPsrUsedInSolution = 0;              
            }
          }

          if( rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
          {
            if( !rxData->m_ObsArray[i].flags.isDifferentialDopplerAvailable )
            {
              rxData->m_ObsArray[i].flags.isDifferentialDopplerAvailable = 0;              
            }
          }

          if( rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
          {
            if( !rxData->m_ObsArray[i].flags.isDifferentialAdrAvailable )
            {
              rxData->m_ObsArray[i].flags.isAdrUsedInSolution = 0;              
            }
          }

        }

      }
    }

    return true;
  }


  bool GNSS_Estimator::DetermineSingleDifferenceADR_Misclosures_GPSL1( 
    GNSS_RxData *rxData,     //!< The pointer to the receiver data.    
    GNSS_RxData *rxBaseData, //!< The pointer to the reference receiver data. NULL if not available.    
    const unsigned n,        //!< The number of misclosures.
    Matrix &w                //!< The adr misclosure vector [n x 1].
    )
  {
    unsigned i = 0;
    unsigned j = 0;
    int k = 0;
    double adr_base = 0;
    double range_base = 0;
    double adr_measured = 0;
    double adr_computed = 0;

    if( n == 0 )
      return true;

    // Check the dimension of w.
    if( w.GetNrRows() != n || w.GetNrCols() != 1 )
    {
      if( !w.Resize( n, 1 ) )
        return false;
    }

    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( rxData->m_ObsArray[i].flags.isActive )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS && rxData->m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          // allows computation of the misclosure value (regardless of whether it is used in solution
          adr_measured = rxData->m_ObsArray[i].adr * GPS_WAVELENGTHL1;

          // Add the satellite clock correction.
          adr_measured += rxData->m_ObsArray[i].satellite.clk;

          // Compensate for the ionospheric delay if indicated.
          // The corrections must be determined beforehand.
          if( rxData->m_ObsArray[i].flags.useBroadcastIonoCorrection )
          {
            // Compensate for the ionospheric delay
            adr_measured += rxData->m_ObsArray[i].corrections.prcIono;
          }

          // Compensate for the tropospheric delay if indicated.
          // The corrections must be determined beforehand.
          if( rxData->m_ObsArray[i].flags.useTropoCorrection )
          {
            // Compensate for the tropospheric delay
            adr_measured -= rxData->m_ObsArray[i].corrections.prcTropoDry;
            adr_measured -= rxData->m_ObsArray[i].corrections.prcTropoWet;
          }

          range_base = 0;
          if( rxBaseData != NULL )
          {    
            if( rxData->m_ObsArray[i].flags.isDifferentialAdrAvailable )
            {
              k = rxData->m_ObsArray[i].index_differential;
              if( k != -1 )
              {
                adr_base  = rxBaseData->m_ObsArray[k].adr * GPS_WAVELENGTHL1;
                adr_base += rxBaseData->m_ObsArray[k].satellite.clk;
                
                // Compensate for the ionospheric delay if indicated.
                // The corrections must be determined beforehand.
                if( rxBaseData->m_ObsArray[k].flags.useBroadcastIonoCorrection )
                {
                  // Compensate for the ionospheric delay
                  adr_base += rxBaseData->m_ObsArray[k].corrections.prcIono;
                }

                // Compensate for the tropospheric delay if indicated.
                // The corrections must be determined beforehand.
                if( rxBaseData->m_ObsArray[k].flags.useTropoCorrection )
                {
                  // Compensate for the tropospheric delay
                  adr_base -= rxBaseData->m_ObsArray[k].corrections.prcTropoDry;
                  adr_base -= rxBaseData->m_ObsArray[k].corrections.prcTropoWet;
                }                  

                range_base = rxBaseData->m_ObsArray[k].range;

                // Compute the differential adr.
                adr_measured -= adr_base;
              }
            }
          }

          // Calculate the computed adr = geometric range + clock offset (m)
          // The range value and the clock offset must be determined beforehand.
          // If differential, range_base != 0, and the ambiguity is the single differnce ambiguity [m].
          adr_computed = rxData->m_ObsArray[i].range - range_base;
          adr_computed += rxData->m_pvt.clockOffset;          
          adr_computed += rxData->m_ObsArray[i].ambiguity; 

          // The misclosure is the corrected measured value minus the computed valid.
          rxData->m_ObsArray[i].adr_misclosure = adr_measured - adr_computed;            

          if( rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
          {
            // Sanity index check.
            if( j >= n )
              return false;

            w[j] = rxData->m_ObsArray[i].adr_misclosure;
            j++;
          }
        }
        else
        {
          rxData->m_ObsArray[i].adr_misclosure = 0.0;            
        }
      }
    }
    return true;
  }

  bool GNSS_Estimator::DetermineDoubleDifferenceADR_Misclosures_GPSL1( 
    GNSS_RxData *rxData,     //!< The pointer to the receiver data.    
    GNSS_RxData *rxBaseData, //!< The pointer to the reference receiver data. NULL if not available. 
    Matrix &subB,             //!< The matrix that describes the differencing from SD to DD adr measurements
    const unsigned n,        //!< The number of DD misclosures required.
    Matrix &w                //!< The adr misclosure vector [n x 1].
    )
  {
    unsigned i = 0;
    unsigned j = 0;
    int k = 0;
    double adr_base = 0;
    double range_base = 0;
    double adr_measured = 0;
    double adr_computed = 0;

    if( n == 0 )
      return true;

    // Check the dimension of w.
    if( w.GetNrRows() != n || w.GetNrCols() != 1 )
    {
      if( !w.Resize( n, 1 ) )
        return false;
    }

    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( rxData->m_ObsArray[i].flags.isActive )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS && rxData->m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          // allows computation of the misclosure value (regardless of whether it is used in solution
          adr_measured = rxData->m_ObsArray[i].adr * GPS_WAVELENGTHL1;

          // Add the satellite clock correction.
          adr_measured += rxData->m_ObsArray[i].satellite.clk;

          // Compensate for the ionospheric delay if indicated.
          // The corrections must be determined beforehand.
          if( rxData->m_ObsArray[i].flags.useBroadcastIonoCorrection )
          {
            // Compensate for the ionospheric delay
            adr_measured += rxData->m_ObsArray[i].corrections.prcIono;
          }

          // Compensate for the tropospheric delay if indicated.
          // The corrections must be determined beforehand.
          if( rxData->m_ObsArray[i].flags.useTropoCorrection )
          {
            // Compensate for the tropospheric delay
            adr_measured -= rxData->m_ObsArray[i].corrections.prcTropoDry;
            adr_measured -= rxData->m_ObsArray[i].corrections.prcTropoWet;
          }

          range_base = 0;
          if( rxBaseData != NULL )
          {    
            if( rxData->m_ObsArray[i].flags.isDifferentialAdrAvailable )
            {
              k = rxData->m_ObsArray[i].index_differential;
              if( k != -1 )
              {
                adr_base  = rxBaseData->m_ObsArray[k].adr * GPS_WAVELENGTHL1;
                adr_base += rxBaseData->m_ObsArray[k].satellite.clk;
                
                // Compensate for the ionospheric delay if indicated.
                // The corrections must be determined beforehand.
                if( rxBaseData->m_ObsArray[k].flags.useBroadcastIonoCorrection )
                {
                  // Compensate for the ionospheric delay
                  adr_base += rxBaseData->m_ObsArray[k].corrections.prcIono;
                }

                // Compensate for the tropospheric delay if indicated.
                // The corrections must be determined beforehand.
                if( rxBaseData->m_ObsArray[k].flags.useTropoCorrection )
                {
                  // Compensate for the tropospheric delay
                  adr_base -= rxBaseData->m_ObsArray[k].corrections.prcTropoDry;
                  adr_base -= rxBaseData->m_ObsArray[k].corrections.prcTropoWet;
                }                  

                range_base = rxBaseData->m_ObsArray[k].range;

                // Compute the differential adr.
                adr_measured -= adr_base;
              }
            }
          }

          // Calculate the computed adr = geometric range + clock offset (m)
          // The range value and the clock offset must be determined beforehand.
          // If differential, range_base != 0, and the ambiguity is the single differnce ambiguity [m].
          adr_computed = rxData->m_ObsArray[i].range - range_base;
          
          //adr_computed += rxData->m_pvt.clockOffset;          
          //adr_computed += rxData->m_ObsArray[i].ambiguity; 

          // The misclosure is the corrected measured value minus the computed valid but not including rx clock or SD ambiguity.
          rxData->m_ObsArray[i].adr_misclosure_temp = adr_measured - adr_computed;            


          //if( rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
          //{
            // Sanity index check.
            //if( j >= n )
              //return false;

            //w[j] = rxData->m_ObsArray[i].adr_misclosure_temp;
            //j++;
          //}
        }
        else
        {
          rxData->m_ObsArray[i].adr_misclosure_temp = 0.0;            
        }
      }
    }  //end for each observation
    j = 0;
    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( rxData->m_ObsArray[i].flags.isActive )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS && rxData->m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          if (rxData->m_ObsArray[i].index_between_satellite_differential == -1)
            continue;
          else
          {
            rxData->m_ObsArray[i].adr_misclosure_dd = rxData->m_ObsArray[i].adr_misclosure_temp -
              rxData->m_ObsArray[rxData->m_ObsArray[i].index_between_satellite_differential].adr_misclosure_temp + rxData->m_ObsArray[i].ambiguity_dd;

            if (j > n-1)
              int crap = 1;
            w[j] = rxData->m_ObsArray[i].adr_misclosure_dd;
            j++;

          }

        }
        else
        {
          rxData->m_ObsArray[i].adr_misclosure_dd = 0.0; 
        }
      }
    }

    return true;
  }

  bool GNSS_Estimator::DetermineDopplerMisclosures_GPSL1( 
    GNSS_RxData *rxData,     //!< The pointer to the receiver data.    
    GNSS_RxData *rxBaseData, //!< The pointer to the reference receiver data. NULL if not available.    
    const unsigned n,        //!< The number of misclosures.
    Matrix &w                //!< The pseudorange misclosure vector [n x 1].
    )
  {
    unsigned i = 0;
    unsigned j = 0;
    int k = 0;
    double doppler_base = 0;
    double rangerate_base = 0;
    double doppler_measured = 0;
    double doppler_computed = 0;

    if( n == 0 )
      return true;

    // Check the dimension of w.
    if( w.GetNrRows() != n || w.GetNrCols() != 1 )
    {
      if( !w.Resize( n, 1 ) )
        return false;
    }
    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( rxData->m_ObsArray[i].flags.isActive )
      {
        rxData->m_ObsArray[i].doppler_misclosure = 0.0;            

        if( rxData->m_ObsArray[i].system == GNSS_GPS && rxData->m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          // always compute the misclosure regardless of whether it is used in solution.
          
          // Compute the pseudorange misclosures in meters/second!
          doppler_measured = rxData->m_ObsArray[i].doppler * GPS_WAVELENGTHL1;

          // Add the satellite clock drift.        
          doppler_measured += rxData->m_ObsArray[i].satellite.clkdrift;
          
          rangerate_base = 0;
          if( rxBaseData != NULL )
          {
            doppler_base = 0;
            if( rxData->m_ObsArray[i].flags.isDifferentialDopplerAvailable )
            {
              k = rxData->m_ObsArray[i].index_differential;
              if( k != -1 )
              {
                doppler_base   = rxBaseData->m_ObsArray[k].doppler * GPS_WAVELENGTHL1;
                rangerate_base = rxBaseData->m_ObsArray[k].rangerate;
              }
              doppler_measured -= doppler_base;
            }
          }

          // Calculate the computed doppler = geometric range rate + rx clock drift [m/s]
          doppler_computed = rxData->m_ObsArray[i].rangerate - rangerate_base + rxData->m_pvt.clockDrift;

          // The misclosure is the corrected measured value minus the computed valid.            
          rxData->m_ObsArray[i].doppler_misclosure = doppler_measured - doppler_computed;

          if( rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
          {
            if( j >= n )
              return false;
            
            w[j] = rxData->m_ObsArray[i].doppler_misclosure;
            j++;
          }
        }
      }
    }

    if( rxData->m_pvt.isPositionConstrained )
    {
      if( j + 3 > n )
        return false;

      w[j] = 0.0;
      j++;
      w[j] = 0.0;
      j++;
      w[j] = 0.0;
      j++;
    }
    else if( rxData->m_pvt.isHeightConstrained )
    {
      if( j + 1 > n )
        return false;
     
      w[j] = 0.0;
      j++;
    }

    return true;
  }


  bool GNSS_Estimator::DetermineDesignMatrixForTheVelocitySolution_GPSL1( 
    GNSS_RxData &rxData, //!< The receiver data.
    const unsigned n,    //!< The number of rows in the design matrix.
    Matrix &H            //!< The velocity & rx clock drift design matrix [n x 4].
    )
  {
    unsigned i = 0;
    unsigned j = 0;

    if( n == 0 )
      return true;

    // Check the dimension of H.
    if( H.GetNrRows() != n || H.GetNrCols() != 4 )
    {
      if( !H.Resize( n, 4 ) )
        return false;
    }
    // Compute the design matrix for the position solution.
    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( rxData.m_ObsArray[i].flags.isActive )
      {
        if( rxData.m_ObsArray[i].system == GNSS_GPS && rxData.m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          if( rxData.m_ObsArray[i].flags.isDopplerUsedInSolution )
          {
            // Sanity index check.            
            if( j >= n )
              return false;
            
            NAVIGATION_ComputeDerivativesOf_Range_WithRespectToLatitudeLongitudeHeight(
              rxData.m_pvt.latitude,
              rxData.m_pvt.longitude, 
              rxData.m_pvt.height,
              rxData.m_ObsArray[i].satellite.x,
              rxData.m_ObsArray[i].satellite.y,
              rxData.m_ObsArray[i].satellite.z,
              &H[j][0],
              &H[j][1],
              &H[j][2],
              &rxData.m_ObsArray[i].range
              );
            H[j][0] *= -1.0; // Doppler sign convention, NovAtel convention, increasing psr means negative Doppler
            H[j][1] *= -1.0;
            H[j][2] *= -1.0;
            H[j][3] = 1.0;

            // copy the row of the design matrix 
            rxData.m_ObsArray[i].H_v[0] = H[j][0];
            rxData.m_ObsArray[i].H_v[1] = H[j][1];
            rxData.m_ObsArray[i].H_v[2] = H[j][2];

            j++;
          }
        }
      }
    }

    if( rxData.m_pvt.isPositionConstrained )
    {
      if( j + 3 > n )
        return false;

      H[j][0] = 1.0;
      H[j][1] = 0.0;
      H[j][2] = 0.0;      
      H[j][3] = 0.0;      
      j++;
      H[j][0] = 0.0;
      H[j][1] = 1.0;
      H[j][2] = 0.0;      
      H[j][3] = 0.0;            
      j++;
      H[j][0] = 0.0;
      H[j][1] = 0.0;
      H[j][2] = 1.0;      
      H[j][3] = 0.0;            
      j++;
    }
    else if( rxData.m_pvt.isHeightConstrained )
    {
      if( j + 1 > n )
        return false;
     
      H[j][0] = 0.0;
      H[j][1] = 0.0;
      H[j][2] = 1.0;      
      H[j][3] = 0.0;            
      j++;
    }

    return true;
  }


  bool GNSS_Estimator::DetermineMeasurementWeightMatrixForTheVelocitySolution_GPSL1( 
    GNSS_RxData &rxData,  //!< The receiver data.
    const unsigned n,     //!< The number of rows in W.
    Matrix &W             //!< The inverse of the Doppler measurement variance-coraiance matrix [n x n].
    )
  {
    unsigned i = 0;
    unsigned j = 0;
    double std_ms = 0.0; // stdev in m/s

    // Check the dimension of W.
    if( W.GetNrRows() != n || W.GetNrCols() != n )
    {
      if( !W.Resize( n, n ) )
        return false;
    }
    // Set the weight matrix.
    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( rxData.m_ObsArray[i].flags.isActive )
      {
        if( rxData.m_ObsArray[i].system == GNSS_GPS && rxData.m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          if( rxData.m_ObsArray[i].flags.isDopplerUsedInSolution )
          {
            // Sanity index check.
            if( j >= n )
              return false;

            // Check divide by zero.
            if( rxData.m_ObsArray[i].stdev_doppler == 0.0 )
              return false;

            std_ms = rxData.m_ObsArray[i].stdev_doppler * GPS_WAVELENGTHL1;
            W[j][j] = 1.0/(std_ms*std_ms);            
            j++;
          }
        }
      }
    }

    if( rxData.m_pvt.isPositionConstrained )
    {
      if( j + 3 > n )
        return false;

      W[j][j] = 1.0/(1.0e-10);
      j++;
      W[j][j] = 1.0/(1.0e-10);
      j++;
      W[j][j] = 1.0/(1.0e-10);
      j++;
    }
    else if( rxData.m_pvt.isHeightConstrained )
    {
      if( j + 1 > n )
        return false;
     
      W[j][j] = 1.0/(1.0e-10);
      j++;
    }

    return true;
  }


  bool GNSS_Estimator::DetermineMeasurementVarianceCovarianceMatrixForTheVelocitySolution_GPSL1( 
    GNSS_RxData &rxData,  //!< The receiver data.
    const unsigned n,     //!< The number of rows in R.
    Matrix &R             //!< The inverse of the Doppler measurement variance-coraiance matrix [n x n].
    )
  {
    unsigned i = 0;
    unsigned j = 0;
    double std_ms = 0; // stdev in m/s

    // Check the dimension of R.
    if( R.GetNrRows() != n || R.GetNrCols() != n )
    {
      if( !R.Resize( n, n ) )
        return false;
    }
    // Set the weight matrix.
    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( rxData.m_ObsArray[i].flags.isActive )
      {
        if( rxData.m_ObsArray[i].system == GNSS_GPS && rxData.m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          if( rxData.m_ObsArray[i].flags.isDopplerUsedInSolution )
          {
            // Sanity index check.
            if( j >= n )
              return false;

            // Check divide by zero.
            if( rxData.m_ObsArray[i].stdev_doppler == 0.0 )
              return false;

            // Convert to meters/second!
            std_ms = rxData.m_ObsArray[i].stdev_doppler * GPS_WAVELENGTHL1;

            R[j][j] = std_ms*std_ms;
            j++;
          }
        }
      }
    }

    if( rxData.m_pvt.isPositionConstrained )
    {
      if( j + 3 > n )
        return false;

      R[j][j] = 1.0e-10;
      j++;
      R[j][j] = 1.0e-10;
      j++;
      R[j][j] = 1.0e-10;
      j++;
    }
    else if( rxData.m_pvt.isHeightConstrained )
    {
      if( j + 1 > n )
        return false;
     
      R[j][j] = 1.0e-10;
      j++;
    }

    return true;
  }



  /// \brief    Perform the Global Internal Reliability Test, followed by a 
  ///           search using local testing for a single measurement fault if 
  ///           the global test fails.
  ///
  /// The Global Internal Reliability Test: \n
  /// The aposteriori variance factor is computed and compared to the value 
  /// generated using Chi^2 lookup table. The aposerteriori variance factor 
  /// indicates the following: \n
  /// if( apv ~= 1 ) \n
  ///   solution and observations are well modelled. \n
  /// if( apv < 1 ) \n
  ///   observation variance matrix is pessimistic. \n
  ///   (observations are better than what you said). \n
  /// if( apv > 1 ) \n
  ///   (1) The math model incorrect, (not likely) \n
  ///   (2) The weight matrix/observation variance matrix is optimistic 
  ///      (observations are worse than what you said). \n
  ///   (3) Their is a blunder in the observations that is skewing the normality 
  ///       of the solution, and a local test for blunder my be needed. \n
  ///
  bool GNSS_Estimator::PerformGlobalTestAndTestForMeasurementFaults( 
    GNSS_RxData &rxData,           //!< The receiver data object.
    bool testPsrOrDoppler,         //!< This indicates if the psr misclosures are checked, otherwise the Doppler misclosures are checked. 
    Matrix& H,                     //!< The design matrix, H,                           [n x u].
    Matrix& Ht,                    //!< The design matrix transposed, H,                [n x u].
    Matrix& W,                     //!< The observation weight matrix, W,               [n x n].
    Matrix& R,                     //!< The observation variance-covariance matrix, R,  [n x n].
    Matrix& r,                     //!< The observation residual vector,                [n x 1].
    Matrix& P,                     //!< The state variance-covariance matrix,           [u x u].
    const unsigned char n,         //!< The number of observations, n.
    const unsigned char u,         //!< The number of unknowns, u.
    double &avf,                   //!< The computed a-posteriori variance factor is returned.
    bool &isGlobalTestPassed,      //!< This indicates if the global test passed.    
    bool &hasRejectionOccurred,    //!< This indicates if a rejection occurred. Only one measurement is flagged.
    unsigned char &indexOfRejected //!< This is the index of the rejected observation.
    )
  {
    double v = n-u; // The degree of freedom.

    unsigned i = 0;
    unsigned j = 0; 
    unsigned indexvec[GNSS_RXDATA_NR_CHANNELS]; // An array with the indices measurements that are used in solution.

    double rv = 0;           // An unsigned standardized residual value.
    double largest_rv = 0.0; // The largest unsigned standardized residual value.    
    unsigned char indexOfLargest = 0; // The index of the largest standardized residual value.  

    double chiSquaredValue = 0;                     // The chi squared test value = look up table value / degrees of freedom.
    double chiSquared005[31] = GNSS_CHISQUARE_00_5; // The chi squared look up table for alpha = 0.005.
    
    // Initial output values.
    isGlobalTestPassed = false;
    hasRejectionOccurred = false;      
    
    Matrix Cr;             // The variance-covariance matrix of the residuals [n x n].
    Matrix rT;             // The residuals vector transposed,                [1 x n]
    Matrix r_standardized; // The standardized residuals,                     [n x 1].
    Matrix tmpM;

    PrintMatToDebug( "P", P );

    i = static_cast<unsigned>(v);
    if( i == 0 )
      return true; 
    
    rT = r;
    if( !rT.Inplace_Transpose() )
      return false;

    // Compute the a-posteriori variance factor.
    tmpM = rT * W * r;
    avf = tmpM[0] / v;
    
    // Determine the chi squared test statistic value.
    i = static_cast<unsigned>(v);
    if( i < 0 || i > 30 )
      return false;
    chiSquaredValue = chiSquared005[i];
    chiSquaredValue /= v;
    
    // Perform the Global Test.
    if( avf < chiSquaredValue )
    {
      // The Global test passes. No local test will be performed.
      isGlobalTestPassed = true;
      return true;
    }

    // Check that there is enough redudancy to remove outliers.
    i = static_cast<unsigned>(v);
    if( i <= 0 )
    {
      return true;
    }

    
    ////
    // Perform the local test.

    // Determine the indices of the observations that match the
    // values in the residuals vector.
    j = 0;
    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( rxData.m_ObsArray[i].flags.isActive )
      {
        if( rxData.m_ObsArray[i].system == GNSS_GPS && rxData.m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          if( testPsrOrDoppler )
          {
            if( rxData.m_ObsArray[i].flags.isPsrUsedInSolution )
            {
              indexvec[j] = i;
              j++;
            }
          }
          else
          {
            if( rxData.m_ObsArray[i].flags.isDopplerUsedInSolution )
            {
              indexvec[j] = i;
              j++;
            }
          }
        }        
      }
    }
    
    
    // Compute the variance-covariance of the residuals.
    Cr = R - H * P * Ht;

    PrintMatToDebug( "Cr", Cr );


    // Check the diagonal of Cr for zero and negative values, an error condition.
    for( i = 0; i < Cr.GetNrRows(); i++ )
    {
      if( Cr[i][i] <= 0.0 )
      {
        return false;
      }
    }

    
    if( !r_standardized.Resize( Cr.GetNrRows(), 1) )
      return false;

    // Compute the standardized residuals, r(i) /= Cr(i,i).
    // Determine the largest standardized residual value.
    for( i = 0; i < Cr.GetNrRows(); i++ )
    {
      r_standardized[i] = r[i] / sqrt( Cr[i][i] );

      rv = fabs( r_standardized[i] );

      // don't allow the position constraint to be rejected.
      if( rxData.m_pvt.isPositionConstrained )
      {
        if( i >= Cr.GetNrRows() -3 )
          continue;
      }

      if( rv > largest_rv )
      {
        largest_rv = rv;
        indexOfLargest = i; // This is the index into r.
      }
    }
    indexOfLargest = indexvec[indexOfLargest]; // This is the index into the observation array.

    // sanity index check
    if( indexOfLargest >= rxData.m_nrValidObs )
      return false;

    PrintMatToDebug( "r", r );
    PrintMatToDebug( "r_standardized", r_standardized );
    
    // In reliability testing, alpha = 0.005, n_(1-alpha/2) = 4.57
    if( largest_rv > 4.57 ) // reject only clear blunders
    {
      // An observation will be rejected.
      hasRejectionOccurred = true;      

      if( testPsrOrDoppler ) // true is pseudorange
        rxData.m_ObsArray[indexOfLargest].flags.isNotPsrRejected = 0;
      else
        rxData.m_ObsArray[indexOfLargest].flags.isNotDopplerRejected = 0;      
    }

    return true;
  }


  bool GNSS_Estimator::ComputeTransitionMatrix_8StatePVGM(
    const double dT,  //!< The change in time since the last update [s].
    Matrix &T         //!< The transition matrix [8 x 8].
    )
  {
    const double betaVn       = 1.0/m_KF.alphaVn;       // The Gauss Markov beta for northing velocity [1/s].
    const double betaVe       = 1.0/m_KF.alphaVe;       // The Gauss Markov beta for easting velocity [1/s].
    const double betaVup      = 1.0/m_KF.alphaVup;      // The Gauss Markov beta for up velocity [1/s].
    const double betaClkDrift = 1.0/m_KF.alphaClkDrift; // The Gauss Markov beta for the clock drift [1/s].

    double eVn = 0;
    double eVe = 0;
    double eVup = 0;
    double eClkDrift = 0;

    
    if( T.GetNrRows() != 8 || T.GetNrCols() != 8 )
    {
      if( !T.Resize(8,8) )
        return false;
    }    
    
    eVn  = exp( -betaVn  * dT );
    eVe  = exp( -betaVe  * dT );
    eVup = exp( -betaVup * dT );

    eClkDrift = exp( -betaClkDrift * dT );

    T.Zero();
    
    T[0][0] = 1.0;
    T[0][3] = (1.0 - eVn) / betaVn;

    T[1][1] = 1.0;
    T[1][4] = (1.0 - eVe) / betaVe;

    T[2][2] = 1.0;
    T[2][5] = (1.0 - eVup) / betaVup;
    
    T[3][3] = eVn;
    T[4][4] = eVe;
    T[5][5] = eVup;

    T[6][6] = 1.0;
    T[6][7] = (1.0 - eClkDrift) / betaClkDrift;

    T[7][7] = eClkDrift;

    return true;
  }

  bool GNSS_Estimator::ComputeTransitionMatrix_8StatePVGM_Float(
    const double dT,  //!< The change in time since the last update [s].
    Matrix &T         //!< The transition matrix [(8 + nrAmb) x (8 + nrAmb)]. 
    )
  {
    const double betaVn       = 1.0/m_KF.alphaVn;       // The Gauss Markov beta for northing velocity [1/s].
    const double betaVe       = 1.0/m_KF.alphaVe;       // The Gauss Markov beta for easting velocity [1/s].
    const double betaVup      = 1.0/m_KF.alphaVup;      // The Gauss Markov beta for up velocity [1/s].
    const double betaClkDrift = 1.0/m_KF.alphaClkDrift; // The Gauss Markov beta for the clock drift [1/s].

    double eVn = 0;
    double eVe = 0;
    double eVup = 0;
    double eClkDrift = 0;
    const unsigned int u = 8 + static_cast<unsigned int>(m_ActiveAmbiguitiesList.size());

    T.Identity( u );
    
    eVn  = exp( -betaVn  * dT );
    eVe  = exp( -betaVe  * dT );
    eVup = exp( -betaVup * dT );

    eClkDrift = exp( -betaClkDrift * dT );

    T[0][0] = 1.0;
    T[0][3] = (1.0 - eVn) / betaVn;

    T[1][1] = 1.0;
    T[1][4] = (1.0 - eVe) / betaVe;

    T[2][2] = 1.0;
    T[2][5] = (1.0 - eVup) / betaVup;
    
    T[3][3] = eVn;
    T[4][4] = eVe;
    T[5][5] = eVup;

    T[6][6] = 1.0;
    T[6][7] = (1.0 - eClkDrift) / betaClkDrift;

    T[7][7] = eClkDrift;

    return true;
  }

  bool GNSS_Estimator::ComputeTransitionMatrix_6StatePVGM_Float(
    const double dT, //!< The change in time since the last update [s].
    Matrix &T        //!< The transition matrix [(8 + nrAmb) x (8 + nrAmb)]. 
    )
  {
    const double betaVn       = 1.0/m_KF.alphaVn;       // The Gauss Markov beta for northing velocity [1/s].
    const double betaVe       = 1.0/m_KF.alphaVe;       // The Gauss Markov beta for easting velocity [1/s].
    const double betaVup      = 1.0/m_KF.alphaVup;      // The Gauss Markov beta for up velocity [1/s].
    const double betaClkDrift = 1.0/m_KF.alphaClkDrift; // The Gauss Markov beta for the clock drift [1/s].

    //double dN = 0;
    double eVn = 0;
    double eVe = 0;
    double eVup = 0;
    //double eClkDrift = 0;
    
  	//Need to check this, when there are no dd ambiguities, T should by 6x6, when there are ambiguities, they should be added
	  const unsigned int u = 6;

    T.Redim(u,u);

    eVn  = exp( -betaVn  * dT );
    eVe  = exp( -betaVe  * dT );
    eVup = exp( -betaVup * dT );

    //eClkDrift = exp( -betaClkDrift * dT );

    T[0][0] = 1.0;
    T[0][3] = (1.0 - eVn) / betaVn;

    T[1][1] = 1.0;
    T[1][4] = (1.0 - eVe) / betaVe;

    T[2][2] = 1.0;
    T[2][5] = (1.0 - eVup) / betaVup;
    
    T[3][3] = eVn;
    T[4][4] = eVe;
    T[5][5] = eVup;

    //T[6][6] = 1.0;
    //T[6][7] = (1.0 - eClkDrift) / betaClkDrift;

    //T[7][7] = eClkDrift;

    return true;
  }



  bool GNSS_Estimator::ComputeProcessNoiseMatrix_8StatePVGM(
    const double dT,  //!< The change in time since the last update [s].
    Matrix &Q         //!< The process noise matrix [8 x 8].
    )
  {
    const double betaVn       = 1.0/m_KF.alphaVn;       // The Gauss Markov beta for northing velocity [1/s].
    const double betaVe       = 1.0/m_KF.alphaVe;       // The Gauss Markov beta for easting velocity [1/s].
    const double betaVup      = 1.0/m_KF.alphaVup;      // The Gauss Markov beta for up velocity [1/s].
    const double betaClkDrift = 1.0/m_KF.alphaClkDrift; // The Gauss Markov beta for the clock drift [1/s].

    const double qVn       = 2 * m_KF.sigmaVn  * m_KF.sigmaVn  * betaVn;  // The process noise value for northing velocity.
    const double qVe       = 2 * m_KF.sigmaVe  * m_KF.sigmaVe  * betaVe;  // The process noise value for easting  velocity.
    const double qVup      = 2 * m_KF.sigmaVup * m_KF.sigmaVup * betaVup; // The process noise value for up       velocity.
    const double qClkDrift = 2 * m_KF.sigmaClkDrift * m_KF.sigmaClkDrift * betaClkDrift; // The process noise value for clock drift.

    double eVn = 0;
    double eVe = 0;
    double eVup = 0;
    double eClkDrift = 0;
    double eVn2 = 0;
    double eVe2 = 0;
    double eVup2 = 0;
    double eClkDrift2 = 0;

    const unsigned int u = 8;

    if( Q.nrows() != u )
    {
      if( !Q.Resize( u, u ) )
      {
        return false;
      }
    }
    
    
    eVn        = exp( -betaVn  * dT );
    eVe        = exp( -betaVe  * dT );
    eVup       = exp( -betaVup * dT );
    eClkDrift  = exp( -betaClkDrift * dT );

    eVn2       = exp( -2.0 * betaVn  * dT );
    eVe2       = exp( -2.0 * betaVe  * dT );
    eVup2      = exp( -2.0 * betaVup * dT );
    eClkDrift2 = exp( -2.0 * betaClkDrift * dT );

    Q[0][0]  = qVn / (betaVn*betaVn);
    Q[0][0] *= dT - 2.0*(1.0 - eVn)/(betaVn) + (1.0 - eVn2)/(2.0*betaVn);

    Q[1][1]  = qVe / (betaVe*betaVe);
    Q[1][1] *= dT - 2.0*(1.0 - eVe)/(betaVe) + (1.0 - eVe2)/(2.0*betaVe);

    Q[2][2]  = qVn / (betaVup*betaVup);
    Q[2][2] *= dT - 2.0*(1.0 - eVup)/(betaVup) + (1.0 - eVup2)/(2.0*betaVup);

    Q[3][3]  = qVn * (1.0 - eVn2) / (2.0*betaVn);
    
    Q[4][4]  = qVe * (1.0 - eVe2) / (2.0*betaVe);

    Q[5][5]  = qVup * (1.0 - eVup2) / (2.0*betaVup);

    Q[6][6]  = qClkDrift / (betaClkDrift*betaClkDrift);
    Q[6][6] *= dT - 2.0*(1.0 - eClkDrift)/(betaClkDrift) + (1.0 - eClkDrift2)/(2.0*betaClkDrift);
    
    Q[7][7]  = qClkDrift * (1.0 - eClkDrift2) / (2.0*betaClkDrift);

    Q[0][3]  = qVn / betaVn;
    Q[0][3] *= (1.0 - eVn)/betaVn - (1.0 - eVn2)/(2.0*betaVn);
    Q[3][0]  = Q[0][3];

    Q[1][4]  = qVe / betaVe;
    Q[1][4] *= (1.0 - eVe)/betaVe - (1.0 - eVe2)/(2.0*betaVe);
    Q[4][1]  = Q[1][4];

    Q[2][5]  = qVup / betaVup;
    Q[2][5] *= (1.0 - eVup)/betaVup - (1.0 - eVup2)/(2.0*betaVup);
    Q[5][2]  = Q[2][5];

    Q[6][7]  = qClkDrift / betaClkDrift;
    Q[6][7] *= (1.0 - eClkDrift)/betaClkDrift - (1.0 - eClkDrift2)/(2.0*betaClkDrift);
    Q[7][6]  = Q[6][7];

    return true;
  }

  
  bool GNSS_Estimator::ComputeProcessNoiseMatrix_8StatePVGM_Float(
    const double dT,  //!< The change in time since the last update [s].
    Matrix &Q         //!< The process noise matrix [(8 + nrAmb) x (8 + nrAmb)].
    )
  {
    const double betaVn       = 1.0/m_KF.alphaVn;       // The Gauss Markov beta for northing velocity [1/s].
    const double betaVe       = 1.0/m_KF.alphaVe;       // The Gauss Markov beta for easting velocity [1/s].
    const double betaVup      = 1.0/m_KF.alphaVup;      // The Gauss Markov beta for up velocity [1/s].
    const double betaClkDrift = 1.0/m_KF.alphaClkDrift; // The Gauss Markov beta for the clock drift [1/s].

    const double qVn       = 2 * m_KF.sigmaVn  * m_KF.sigmaVn  * betaVn;  // The process noise value for northing velocity.
    const double qVe       = 2 * m_KF.sigmaVe  * m_KF.sigmaVe  * betaVe;  // The process noise value for easting  velocity.
    const double qVup      = 2 * m_KF.sigmaVup * m_KF.sigmaVup * betaVup; // The process noise value for up       velocity.
    const double qClkDrift = 2 * m_KF.sigmaClkDrift * m_KF.sigmaClkDrift * betaClkDrift; // The process noise value for clock drift.

    double eVn = 0;
    double eVe = 0;
    double eVup = 0;
    double eClkDrift = 0;
    double eVn2 = 0;
    double eVe2 = 0;
    double eVup2 = 0;
    double eClkDrift2 = 0;

    const unsigned int u = 8 + static_cast<unsigned int>(m_ActiveAmbiguitiesList.size());

    if( Q.nrows() != u )
    {
      if( !Q.Resize( u, u ) )
      {
        return false;
      }
    }
    
    eVn        = exp( -betaVn  * dT );
    eVe        = exp( -betaVe  * dT );
    eVup       = exp( -betaVup * dT );
    eClkDrift  = exp( -betaClkDrift * dT );

    eVn2       = exp( -2.0 * betaVn  * dT );
    eVe2       = exp( -2.0 * betaVe  * dT );
    eVup2      = exp( -2.0 * betaVup * dT );
    eClkDrift2 = exp( -2.0 * betaClkDrift * dT );

    Q[0][0]  = qVn / (betaVn*betaVn);
    Q[0][0] *= dT - 2.0*(1.0 - eVn)/(betaVn) + (1.0 - eVn2)/(2.0*betaVn);

    Q[1][1]  = qVe / (betaVe*betaVe);
    Q[1][1] *= dT - 2.0*(1.0 - eVe)/(betaVe) + (1.0 - eVe2)/(2.0*betaVe);

    Q[2][2]  = qVn / (betaVup*betaVup);
    Q[2][2] *= dT - 2.0*(1.0 - eVup)/(betaVup) + (1.0 - eVup2)/(2.0*betaVup);

    Q[3][3]  = qVn * (1.0 - eVn2) / (2.0*betaVn);
    
    Q[4][4]  = qVe * (1.0 - eVe2) / (2.0*betaVe);

    Q[5][5]  = qVup * (1.0 - eVup2) / (2.0*betaVup);

    Q[6][6]  = qClkDrift / (betaClkDrift*betaClkDrift);
    Q[6][6] *= dT - 2.0*(1.0 - eClkDrift)/(betaClkDrift) + (1.0 - eClkDrift2)/(2.0*betaClkDrift);
    
    Q[7][7]  = qClkDrift * (1.0 - eClkDrift2) / (2.0*betaClkDrift);

    Q[0][3]  = qVn / betaVn;
    Q[0][3] *= (1.0 - eVn)/betaVn - (1.0 - eVn2)/(2.0*betaVn);
    Q[3][0]  = Q[0][3];

    Q[1][4]  = qVe / betaVe;
    Q[1][4] *= (1.0 - eVe)/betaVe - (1.0 - eVe2)/(2.0*betaVe);
    Q[4][1]  = Q[1][4];

    Q[2][5]  = qVup / betaVup;
    Q[2][5] *= (1.0 - eVup)/betaVup - (1.0 - eVup2)/(2.0*betaVup);
    Q[5][2]  = Q[2][5];

    Q[6][7]  = qClkDrift / betaClkDrift;
    Q[6][7] *= (1.0 - eClkDrift)/betaClkDrift - (1.0 - eClkDrift2)/(2.0*betaClkDrift);
    Q[7][6]  = Q[6][7];

    return true;
  }

  bool GNSS_Estimator::ComputeProcessNoiseMatrix_6StatePVGM_Float(
    const double dT,  //!< The change in time since the last update [s].
    Matrix &Q         //!< The process noise matrix [(8 + nrAmb) x (8 + nrAmb)].
    )
  {
    const double betaVn       = 1.0/m_KF.alphaVn;       // The Gauss Markov beta for northing velocity [1/s].
    const double betaVe       = 1.0/m_KF.alphaVe;       // The Gauss Markov beta for easting velocity [1/s].
    const double betaVup      = 1.0/m_KF.alphaVup;      // The Gauss Markov beta for up velocity [1/s].
    const double betaClkDrift = 1.0/m_KF.alphaClkDrift; // The Gauss Markov beta for the clock drift [1/s].

    const double qVn       = 2 * m_KF.sigmaVn  * m_KF.sigmaVn  * betaVn;  // The process noise value for northing velocity.
    const double qVe       = 2 * m_KF.sigmaVe  * m_KF.sigmaVe  * betaVe;  // The process noise value for easting  velocity.
    const double qVup      = 2 * m_KF.sigmaVup * m_KF.sigmaVup * betaVup; // The process noise value for up       velocity.
    const double qClkDrift = 2 * m_KF.sigmaClkDrift * m_KF.sigmaClkDrift * betaClkDrift; // The process noise value for clock drift.

    //const double dT2 = dT*dT;
    double eVn = 0;
    double eVe = 0;
    double eVup = 0;
    //double eClkDrift = 0;
    double eVn2 = 0;
    double eVe2 = 0;
    double eVup2 = 0;
    //double eClkDrift2 = 0;

	//Same problem with dimensions. m_ActiveAmbiguitiesList.size has the be the DD size now.
    const unsigned int u = 6;

    if( Q.nrows() != u )
    {
      if( !Q.Resize( u, u ) )
      {
        return false;
      }
    }
    
    eVn        = exp( -betaVn  * dT );
    eVe        = exp( -betaVe  * dT );
    eVup       = exp( -betaVup * dT );
    //eClkDrift  = exp( -betaClkDrift * dT );

    eVn2       = exp( -2.0 * betaVn  * dT );
    eVe2       = exp( -2.0 * betaVe  * dT );
    eVup2      = exp( -2.0 * betaVup * dT );
    //eClkDrift2 = exp( -2.0 * betaClkDrift * dT );

    Q[0][0]  = qVn / (betaVn*betaVn);
    Q[0][0] *= dT - 2.0*(1.0 - eVn)/(betaVn) + (1.0 - eVn2)/(2.0*betaVn);

    Q[1][1]  = qVe / (betaVe*betaVe);
    Q[1][1] *= dT - 2.0*(1.0 - eVe)/(betaVe) + (1.0 - eVe2)/(2.0*betaVe);

    Q[2][2]  = qVn / (betaVup*betaVup);
    Q[2][2] *= dT - 2.0*(1.0 - eVup)/(betaVup) + (1.0 - eVup2)/(2.0*betaVup);

    Q[3][3]  = qVn * (1.0 - eVn2) / (2.0*betaVn);
    
    Q[4][4]  = qVe * (1.0 - eVe2) / (2.0*betaVe);

    Q[5][5]  = qVup * (1.0 - eVup2) / (2.0*betaVup);

    // GDM_HACK added the 0.009
    //Q[6][6]  = 0.3*dT + qClkDrift / (betaClkDrift*betaClkDrift);
    //Q[6][6]  = qClkDrift / (betaClkDrift*betaClkDrift);
    //Q[6][6] *= dT - 2.0*(1.0 - eClkDrift)/(betaClkDrift) + (1.0 - eClkDrift2)/(2.0*betaClkDrift);
    
    //Q[7][7]  = qClkDrift * (1.0 - eClkDrift2) / (2.0*betaClkDrift);

    Q[0][3]  = qVn / betaVn;
    Q[0][3] *= (1.0 - eVn)/betaVn - (1.0 - eVn2)/(2.0*betaVn);
    Q[3][0]  = Q[0][3];

    Q[1][4]  = qVe / betaVe;
    Q[1][4] *= (1.0 - eVe)/betaVe - (1.0 - eVe2)/(2.0*betaVe);
    Q[4][1]  = Q[1][4];

    Q[2][5]  = qVup / betaVup;
    Q[2][5] *= (1.0 - eVup)/betaVup - (1.0 - eVup2)/(2.0*betaVup);
    Q[5][2]  = Q[2][5];

    //Q[6][7]  = qClkDrift / betaClkDrift;
    //Q[6][7] *= (1.0 - eClkDrift)/betaClkDrift - (1.0 - eClkDrift2)/(2.0*betaClkDrift);
    //Q[7][6]  = Q[6][7];

    return true;
  }



  bool GNSS_Estimator::PredictAhead_8StatePVGM(
    GNSS_RxData &rxData, //!< The receiver data.
    const double dT,     //!< The change in time since the last update [s].
    Matrix &T,           //!< The transition matrix                                 [8 x 8] (output).
    Matrix &Q,           //!< The process noise matrix                              [8 x 8] (output).
    Matrix &P            //!< The state variance covariance matrix                  [8 x 8] (input/output).      
    )
  {
    double M = 0; // The meridian radius of curvature.
    double N = 0; // The prime vertical radius of curvature.
    bool result = false;
    Matrix tmpMat;
    double lat = 0;
    double h = 0;

    GEODESY_ComputeMeridianRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      rxData.m_pvt.latitude,
      &M );

    GEODESY_ComputePrimeVerticalRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      rxData.m_pvt.latitude,
      &N );

    // Get the transition Matrix
    result = ComputeTransitionMatrix_8StatePVGM( dT, T );
    if( result == false )
      return false;

    // Set the process noise Matrix
    result = ComputeProcessNoiseMatrix_8StatePVGM( dT, Q );      
    if( result == false )
      return false;

    
    ////
    // predict the states ahead    
    
    // for use of use
    lat = rxData.m_pvt.latitude;
    h   = rxData.m_pvt.height;
    
    rxData.m_pvt.latitude  += T[0][3] * rxData.m_pvt.vn / (M+h);  // for small dT, this should be: lat += dT * vn / M
    rxData.m_pvt.longitude += T[1][4] * rxData.m_pvt.ve / ((N+h)*cos(lat));  // for small dT, this should be: lon += dT * ve / ((N+h)*cos(lat))
    rxData.m_pvt.height    += T[2][5] * rxData.m_pvt.vup;  // for small dT, this should be: hgt += dT * vup    
    
    rxData.m_pvt.vn  *= T[3][3];  // for small dT, this should be vn = vn    
    rxData.m_pvt.ve  *= T[4][4];  // for small dT, this should be ve = ve
    rxData.m_pvt.vup *= T[5][5];  // for small dT, this should be vup = vup

    rxData.m_pvt.clockOffset += T[6][7] * rxData.m_pvt.clockDrift;  // for small dT, this should be: clk += dT * clkrate
    rxData.m_pvt.clockDrift  *= T[7][7];  // for small dT, this should be clkdrift = clkdrift

    //
    ////


    ////
    // predict the new state variance/covariance

    // It can be done this way:
    // P = T * P * T.transpose() + Q;
    // but the following is more efficient
    tmpMat = T;
    if( !tmpMat.Inplace_Transpose() )
      return false;

    if( !P.Inplace_PreMultiply( T ) )
      return false;

    if( !P.Inplace_PostMultiply( tmpMat ) )
      return false;

    if( !P.Inplace_Add( Q ) )
      return false;
    //
    ////

    return true;    
  }


  bool GNSS_Estimator::PredictAhead_8StatePVGM_Float(
    GNSS_RxData &rxData, //!< The receiver data.
    const double dT,     //!< The change in time since the last update [s].
    Matrix &T,           //!< The transition matrix                                 [(8 + nrAmb) x (8 + nrAmb)] (output).
    Matrix &Q,           //!< The process noise matrix                              [(8 + nrAmb) x (8 + nrAmb)] (output).
    Matrix &P            //!< The state variance covariance matrix                  [(8 + nrAmb) x (8 + nrAmb)] (input/output).      
    )
  {
    double M = 0; // The meridian radius of curvature.
    double N = 0; // The prime vertical radius of curvature.
    bool result = false;
    Matrix tmpMat;
    double lat = 0;
    double h = 0;

    GEODESY_ComputeMeridianRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      rxData.m_pvt.latitude,
      &M );

    GEODESY_ComputePrimeVerticalRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      rxData.m_pvt.latitude,
      &N );

    // Get the transition Matrix
    result = ComputeTransitionMatrix_8StatePVGM_Float( dT, T );
    if( result == false )
      return false;

    PrintMatToDebug( "T", T );

    // Set the process noise Matrix
    result = ComputeProcessNoiseMatrix_8StatePVGM_Float( dT, Q );
    if( result == false )
      return false;

    PrintMatToDebug( "Q", Q );
    
    ////
    // predict the states ahead    
    
    // for use of use
    lat = rxData.m_pvt.latitude;
    h   = rxData.m_pvt.height;
    
    rxData.m_pvt.latitude  += T[0][3] * rxData.m_pvt.vn / (M+h);  // for small dT, this should be: lat += dT * vn / M
    rxData.m_pvt.longitude += T[1][4] * rxData.m_pvt.ve / ((N+h)*cos(lat));  // for small dT, this should be: lon += dT * ve / ((N+h)*cos(lat))
    rxData.m_pvt.height    += T[2][5] * rxData.m_pvt.vup;  // for small dT, this should be: hgt += dT * vup    
    
    rxData.m_pvt.vn  *= T[3][3];  // for small dT, this should be vn = vn    
    rxData.m_pvt.ve  *= T[4][4];  // for small dT, this should be ve = ve
    rxData.m_pvt.vup *= T[5][5];  // for small dT, this should be vup = vup

    rxData.m_pvt.clockOffset += T[6][7] * rxData.m_pvt.clockDrift;  // for small dT, this should be: clk += dT * clkrate
    rxData.m_pvt.clockDrift  *= T[7][7];  // for small dT, this should be clkdrift = clkdrift

   

    //
    ////


    ////
    // predict the new state variance/covariance

    // It can be done this way:
    // P = T * P * T.transpose() + Q;
    // but the following is more efficient
    tmpMat = T;
    if( !tmpMat.Inplace_Transpose() )
      return false;

    if( !P.Inplace_PreMultiply( T ) )
      return false;

    if( !P.Inplace_PostMultiply( tmpMat ) )
      return false;

    if( !P.Inplace_Add( Q ) )
      return false;
    //
    ////

    result = rxData.UpdatePositionAndRxClock(
      rxData.m_pvt.latitude,
      rxData.m_pvt.longitude,
      rxData.m_pvt.height,
      rxData.m_pvt.clockOffset,
      sqrt( P[0][0] ),
      sqrt( P[1][1] ),
      sqrt( P[2][2] ),
      sqrt( P[6][6] )
      );
    if( !result )
      return false;   

    result = rxData.UpdateVelocityAndClockDrift(
      rxData.m_pvt.vn,
      rxData.m_pvt.ve,
      rxData.m_pvt.vup,
      rxData.m_pvt.clockDrift,
      sqrt( P[3][3] ),
      sqrt( P[4][4] ),
      sqrt( P[5][5] ),
      sqrt( P[7][7] )
      );
    if( !result )
      return false;   

    PrintMatToDebug( "P", P );

    return true;    
  }

  bool GNSS_Estimator::PredictAhead_6StatePVGM_Float(
    GNSS_RxData &rxData, //!< The receiver data.
    const double dT,     //!< The change in time since the last update [s].
    Matrix &T,           //!< The transition matrix                                 [(8 + nrAmb) x (8 + nrAmb)] (output).
    Matrix &Q,           //!< The process noise matrix                              [(8 + nrAmb) x (8 + nrAmb)] (output).
    Matrix &P            //!< The state variance covariance matrix                  [(8 + nrAmb) x (8 + nrAmb)] (input/output).      
    )
  {
    unsigned i = 0;
    unsigned j = 0;
    double M = 0; // The meridian radius of curvature.
    double N = 0; // The prime vertical radius of curvature.
    bool result = false;
    Matrix tmpMat;
    double lat = 0;
    double h = 0;

    GEODESY_ComputeMeridianRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      rxData.m_pvt.latitude,
      &M );

    GEODESY_ComputePrimeVerticalRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      rxData.m_pvt.latitude,
      &N );

    // Get the transition Matrix
    result = ComputeTransitionMatrix_6StatePVGM_Float( dT, T );
    if( result == false )
      return false;

    PrintMatToDebug( "T", T );

    // Set the process noise Matrix
    result = ComputeProcessNoiseMatrix_6StatePVGM_Float( dT, Q );
    if( result == false )
      return false;

    PrintMatToDebug( "Q", Q );
    
    ////
    // predict the states ahead    
    
    // for use of use
    lat = rxData.m_pvt.latitude;
    h   = rxData.m_pvt.height;
    
    rxData.m_pvt.latitude  += T[0][3] * rxData.m_pvt.vn / (M+h);  // for small dT, this should be: lat += dT * vn / M
    rxData.m_pvt.longitude += T[1][4] * rxData.m_pvt.ve / ((N+h)*cos(lat));  // for small dT, this should be: lon += dT * ve / ((N+h)*cos(lat))
    rxData.m_pvt.height    += T[2][5] * rxData.m_pvt.vup;  // for small dT, this should be: hgt += dT * vup    
    
    rxData.m_pvt.vn  *= T[3][3];  // for small dT, this should be vn = vn    
    rxData.m_pvt.ve  *= T[4][4];  // for small dT, this should be ve = ve
    rxData.m_pvt.vup *= T[5][5];  // for small dT, this should be vup = vup

    //
    ////


    ////
    // predict the new state variance/covariance

    // It can be done this way:
    // P = T * P * T.transpose() + Q;
    // but the following is more efficient

    if( !T.Redim( P.nrows(), P.ncols() ) )
      return false;

    for( i = 6; i < T.nrows(); i++ )
    {
      for( j = 6; j < T.ncols(); j++ )
      {
        T[i][j] = 1.0;
      }
    }

    tmpMat = T;
    if( !tmpMat.Inplace_Transpose() )
      return false;

    if( !P.Inplace_PreMultiply( T ) )
      return false;

    if( !P.Inplace_PostMultiply( tmpMat ) )
      return false;

    if( !Q.Redim( P.nrows(), P.ncols() ) )
      return false;

    if( !P.Inplace_Add( Q ) )
      return false;
    //
    ////

    PrintMatToDebug( "P", P );

    return true;    
  }






  bool GNSS_Estimator::InitializeStateVarianceCovariance_8StatePVGM(
    const double std_lat,        //!< The standard deviation uncertainty in the latitude [m].
    const double std_lon,        //!< The standard deviation uncertainty in the longitude [m]. 
    const double std_hgt,        //!< The standard deviation uncertainty in the height [m].
    const double std_vn,         //!< The standard deviation uncertainty in the northing velocity [m/s].
    const double std_ve,         //!< The standard deviation uncertainty in the easting velocity [m/s].
    const double std_vup,        //!< The standard deviation uncertainty in the up velocity [m/s].
    const double std_clk,        //!< The standard deviation uncertainty in the clock offset [m].
    const double std_clkdrift,   //!< The standard deviation uncertainty in the clock drift [m/s].    
    Matrix &P                    //!< The variance covariance of the states [8x8].
    )
  {
    if( P.GetNrRows() != 8 || P.GetNrCols() != 8 )
    {
      if( !P.Resize( 8, 8 ) )
      {
        return false;
      }
    }

    P[0][0] = std_lat*std_lat;
    P[1][1] = std_lon*std_lon;
    P[2][2] = std_hgt*std_hgt;
    P[3][3] = std_vn*std_vn;
    P[4][4] = std_ve*std_ve;
    P[5][5] = std_vup*std_vup;
    P[6][6] = std_clk*std_clk;
    P[7][7] = std_clkdrift*std_clkdrift;

    PrintMatToDebug( "P", P );

    return true;
  }

  bool GNSS_Estimator::InitializeStateVarianceCovariance_6StatePVGM(
    const double std_lat,        //!< The standard deviation uncertainty in the latitude [m].
    const double std_lon,        //!< The standard deviation uncertainty in the longitude [m]. 
    const double std_hgt,        //!< The standard deviation uncertainty in the height [m].
    const double std_vn,         //!< The standard deviation uncertainty in the northing velocity [m/s].
    const double std_ve,         //!< The standard deviation uncertainty in the easting velocity [m/s].
    const double std_vup,        //!< The standard deviation uncertainty in the up velocity [m/s].  
    Matrix &P                    //!< The variance covariance of the states [8x8].
    )
  {
    if( P.GetNrRows() != 6 || P.GetNrCols() != 6 )
    {
      if( !P.Resize( 6, 6 ) )
      {
        return false;
      }
    }

    P[0][0] = std_lat*std_lat;
    P[1][1] = std_lon*std_lon;
    P[2][2] = std_hgt*std_hgt;
    P[3][3] = std_vn*std_vn;
    P[4][4] = std_ve*std_ve;
    P[5][5] = std_vup*std_vup;
   
    PrintMatToDebug( "P", P );

    return true;
  }


  bool GNSS_Estimator::Kalman_Update_8StatePVGM(
    GNSS_RxData *rxData,      //!< A pointer to the rover receiver data. This must be a valid pointer.
    GNSS_RxData *rxBaseData,  //!< A pointer to the reference receiver data if available. NULL if not available.
    Matrix &P                 //!< The variance-covariance of the states.
    )
  {
    bool result = false;
    unsigned i = 0;
    unsigned j = 0;
    unsigned n = 0;
    unsigned nrP = 0;           // The number of valid psr.
    unsigned nrP_base = 0;      // The number of valid psr for the reference station.
    unsigned nrD = 0;           // The number of valid Doppler.
    unsigned nrD_base = 0;      // The number of valid Doppler for the reference station.
    unsigned nrValidEph = 0;    // The number of valid ephemeris.
    unsigned nrDifferentialPsr = 0;     // The number of differential psr.
    unsigned nrDifferentialDoppler = 0; // The number of differential Doppler.
    bool isDifferential = false;

    double lat = 0;
    double lon = 0;
    double hgt = 0;
    double clk = 0;
    double vn = 0;
    double ve = 0;
    double vup = 0;
    double clkdrift = 0;
    double M = 0;   // The computed meridian radius of curvature [m].
    double N = 0;   // The computed prime vertical radius of curvature [m].

    Matrix H_p;
    Matrix H_v;
    Matrix H;
    Matrix Ht;
    Matrix R_p;
    Matrix R_v;
    Matrix R;
    Matrix w_p;
    Matrix w_v;
    Matrix w;
    Matrix K;
    Matrix tmpMat;
    Matrix tmpMatP;
    Matrix dx(8);
    Matrix I;

    
    dx.Zero();
    
    // Store the current input pvt as the previous pvt since we are updating.
    rxData->m_prev_pvt = rxData->m_pvt;

    lat = rxData->m_pvt.latitude;
    lon = rxData->m_pvt.longitude;
    hgt = rxData->m_pvt.height;
    clk = rxData->m_pvt.clockOffset;
    vn  = rxData->m_pvt.vn;
    ve  = rxData->m_pvt.ve;
    vup = rxData->m_pvt.vup;
    clkdrift = rxData->m_pvt.clockDrift; 


    if( rxBaseData != NULL )
      isDifferential = true;

#ifdef CONSTRAINPOS
    rxData->m_pvt.isPositionConstrained = 1;
#else
    rxData->m_pvt.isPositionConstrained = 0;
#endif
    
    
    ////
    // Perform operations specific to the rover.

    // Update the receiver time information (UTC and day of year)
    result = UpdateTime( *rxData );
    if( !result )
      return false;    
    if( isDifferential )
    {
      result = UpdateTime( *rxBaseData );
      if( !result )
        return false;    
    }

    result = DetermineSatellitePVT_GPSL1( rxData, rxBaseData, nrValidEph );
    if( !result )
      return false;


    result = DetermineAtmosphericCorrections_GPSL1( *rxData );
    if( !result )
      return false;
    result = DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( *rxData, nrP );
    if( !result )
      return false;    
    result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( *rxData, nrD );
    if( !result )
      return false;    
    

    if( isDifferential )
    {
      result = DetermineAtmosphericCorrections_GPSL1( *rxBaseData );
      if( !result )
        return false;
      result = DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( *rxBaseData, nrP_base );
      if( !result )
        return false;
      result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( *rxBaseData, nrD_base );
      if( !result )
        return false;

      // When in differential mode, only differntial measurements will be used
      result = DetermineBetweenReceiverDifferentialIndex( 
        rxData,
        rxBaseData,
        true 
        );
      if( !result )
        return false;

       nrDifferentialPsr = 0;
        for( i = 0; i < rxData->m_nrValidObs; i++ )
        {
          if( rxData->m_ObsArray[i].system == GNSS_GPS &&
            rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
            rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
          {
            nrDifferentialPsr++;
          }
        }

#ifdef CONSTRAINPOS
      nrP = nrDifferentialPsr+3;
#else
      nrP = nrDifferentialPsr;
#endif

      nrDifferentialDoppler = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS &&
          rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
          rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
        {
          nrDifferentialDoppler++;
        }
      }

#ifdef CONSTRAINPOS
      nrD = nrDifferentialPsr+3;
#else
      nrD = nrDifferentialPsr;
#endif
    }

    result = DetermineDesignMatrixForThePositionSolution_GPSL1(
      *rxData,
      nrP,
      H_p );
    if( !result )
      return false;
    
    result = DetermineDesignMatrixForTheVelocitySolution_GPSL1(
      *rxData,
      nrD,
      H_v );
    if( !result )
      return false;


    result = DetermineMeasurementVarianceCovarianceMatrixForThePositionSolution_GPSL1(
      *rxData,
      nrP,
      R_p );
    if( !result )
      return false;


    result = DetermineMeasurementVarianceCovarianceMatrixForTheVelocitySolution_GPSL1(
      *rxData,
      nrD,
      R_v );
    if( !result )
      return false;


    result = DeterminePseudorangeMisclosures_GPSL1(
      rxData,
      rxBaseData,
      nrP,
      w_p );
    if( !result )
      return false;

    result = DetermineDopplerMisclosures_GPSL1(
      rxData,
      rxBaseData,
      nrD,
      w_v );
    if( !result )
      return false;

    // Form the combined design matrix.
    n = H_p.GetNrRows() + H_v.GetNrRows();

    result = H.Resize( n, 8 );
    if( !result )
      return false;

    for( i = 0; i < n; i++ )
    {
      if( i < nrP )
      {
        for( j = 0; j < 3; j++ )
          H[i][j] = H_p[i][j];
#ifdef CONSTRAINPOS
        if( i < nrP-3 )
          H[i][6] = 1.0;
#else
        H[i][6] = 1.0;
#endif
      }
      else
      {
        for( j = 0; j < 3; j++ )
          H[i][j+3] = H_v[i-nrP][j];
#ifdef CONSTRAINPOS
        if( i < n-3 )
          H[i][7] = 1.0;
#else
        H[i][7] = 1.0;
#endif
      }
    }
    PrintMatToDebug( "H", H );
    
    
    result = R.Resize( n, n );
    if( !result )
      return false;

    // Form the combined measurement variance-covariance matrix.
    for( i = 0; i < n; i++ )
    {
      if( i < nrP )
      {
        for( j = 0; j < nrP; j++ )
          R[i][j] = R_p[i][j];
      }
      else
      {
        for( j = 0; j < nrD; j++ )
          R[i][j+nrD] = R_v[i-nrP][j];
      }
    }
    PrintMatToDebug( "R", R );


    // Form the combined misclosure vector.
    result = w.Resize( n, 1 );
    if( !result )
      return false;
    for( i = 0; i < n; i++ )
    {
      if( i < nrP )
      {
        w[i] = w_p[i];
      }
      else
      {
        w[i] = w_v[i-nrP];
      }
    }

    
    // Form H transposed.
    Ht = H;
    result = Ht.Inplace_Transpose();
    if( !result )
      return false;


    PrintMatToDebug( "H", H );
    PrintMatToDebug( "R", R );
    PrintMatToDebug( "w", w );
    PrintMatToDebug( "P", P );

    // Compute the Kalman gain matrix.
    // K = P*Ht*(H*P*Ht+R)^-1
    // do (H*P*Ht+R)^-1 first
    tmpMat = H*P*Ht + R;
    result = tmpMat.Inplace_Invert();
    if( !result )
      return false;    
    K = P*Ht*tmpMat;
    

    // Compute the change in states due to the innovations (misclosures).
    dx = K*w;

    

    // Compute the updated state variance-covariance matrix, P.
    result = I.Resize( 8, 8 );
    if( !result )
      return false;
    result = I.Identity();
    if( !result )
      return false;

    P = (I - K * H)*P;
    
    PrintMatToDebug( "dx", dx );
    PrintMatToDebug( "P", P );
    PrintMatToDebug( "K", K );

    
    // Update the position and clock states
    // Update height first as it is need to reduce the corrections for lat and lon.
    hgt += dx[2];
    clk += dx[6];

    // The corrections for lat and lon, dx_p, must be converted to [rad] from [m].
    GEODESY_ComputePrimeVerticalRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84, 
      lat,
      &N );
    GEODESY_ComputeMeridianRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84, 
      lat,
      &M );

     lat += dx[0] / ( M + hgt );             // convert from meters to radians.
     lon += dx[1] / (( N + hgt )*cos(lat));  // convert from meters to radians.

     result = rxData->UpdatePositionAndRxClock(
       lat,
       lon,
       hgt,
       clk,
       sqrt(P[0][0]),
       sqrt(P[1][1]),
       sqrt(P[2][2]),
       sqrt(P[6][6])
       );
     if( !result )
       return false;


     // Update the velocity and clock drift states.
     vn        += dx[3];
     ve        += dx[4];
     vup       += dx[5];
     clkdrift  += dx[7];

     result = rxData->UpdateVelocityAndClockDrift(
       vn,
       ve,
       vup,
       clkdrift,
       sqrt(P[3][3]),
       sqrt(P[4][4]),
       sqrt(P[5][5]),
       sqrt(P[7][7]) );
     if( !result )
       return false;

     /*
     // GDM - recomputing the misclosures after adjustment!
    result = DeterminePseudorangeMisclosures_GPSL1(
      rxData,
      rxBaseData,
      nrP,
      w_p );
    if( !result )
      return false;

    result = DetermineDopplerMisclosures_GPSL1(
      rxData,
      rxBaseData,
      nrD,
      w_v );
    if( !result )
      return false;
      */


    return true;
  }



  bool GNSS_Estimator::Kalman_Update_6StatePVGM_FloatSolution(
    GNSS_RxData *rxData,      //!< A pointer to the rover receiver data. This must be a valid pointer.
    GNSS_RxData *rxBaseData,  //!< A pointer to the reference receiver data if available. NULL if not available.
    Matrix &P                 //!< The variance-covariance of the states.
  )
  {
    bool result = false;
    unsigned index = 0;
    unsigned i = 0;
    unsigned j = 0;
    unsigned k = 0; 
    unsigned m = 0; // counter
    unsigned n = 0;
    unsigned nrP = 0;           // The number of valid psr.
    unsigned nrP_base = 0;      // The number of valid psr for the reference station.
    unsigned nrD = 0;           // The number of valid Doppler.
    unsigned nrD_base = 0;      // The number of valid Doppler for the reference station.
    unsigned nrValidEph = 0;    // The number of valid ephemeris.
    unsigned nrUsableAdr = 0;      // The number of valid adr.
    unsigned nrUsableAdr_base = 0; // The number of valid adr for the reference station.
    unsigned nrDifferentialPsr = 0;     // The number of differential psr.
    unsigned nrDifferentialDoppler = 0; // The number of differential Doppler.
    unsigned nrDifferentialAdr = 0;     // The number of differntial adr.
	unsigned nDD = 0;
	unsigned nrPDD = 0;
	unsigned nrDDD = 0;
	unsigned nrDifferentialAdrDD = 0;
    bool isDifferential = false;
    unsigned u = 0;
    bool setToUseOnlyDifferential = true;

    double lat = 0;
    double lon = 0;
    double hgt = 0;
    double clk = 0;
    double vn = 0;
    double ve = 0;
    double vup = 0;
    double clkdrift = 0;
    double M = 0;   // The computed meridian radius of curvature [m].
    double N = 0;   // The computed prime vertical radius of curvature [m].

    Matrix H_p;
    Matrix H_v;
    Matrix H;
    Matrix r; // A vector corresponding to the diagonal elements of R.
    Matrix R_p;
    Matrix R_v;
    Matrix w_p;
    Matrix w_v;
    Matrix w_adr;
    Matrix w;
    Matrix K;
    Matrix tmpMat;
    Matrix tmpMatP;
    Matrix dx(8);
    Matrix I;

    dx.Zero();
    
    
    lat = rxData->m_pvt.latitude;
    lon = rxData->m_pvt.longitude;
    hgt = rxData->m_pvt.height;
    clk = rxData->m_pvt.clockOffset;
    vn  = rxData->m_pvt.vn;
    ve  = rxData->m_pvt.ve;
    vup = rxData->m_pvt.vup;
    clkdrift = rxData->m_pvt.clockDrift;
        
    if( rxBaseData != NULL )
      isDifferential = true;

#ifdef CONSTRAINPOS
    rxData->m_pvt.isPositionConstrained = 1;
#else
    rxData->m_pvt.isPositionConstrained = 0;
#endif
    
    
    ////
    // Perform operations specific to the rover.

    // Update the receiver time information (UTC and day of year)
    result = UpdateTime( *rxData );
    if( !result )
      return false;

    if( isDifferential )
    {
      result = UpdateTime( *rxBaseData );
      if( !result )
      {
        return false;
      }
    }

    result = DetermineSatellitePVT_GPSL1( rxData, rxBaseData, nrValidEph );
    if( !result )
    {
      return false;
    }


    result = DetermineAtmosphericCorrections_GPSL1( *rxData );
    if( !result )
    {
      return false;
    }
    result = DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( *rxData, nrP );
    if( !result )
    {
      return false;
    }
    result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( *rxData, nrD );
    if( !result )
    {
      return false;
    }
    result = DetermineUsableAdrMeasurements_GPSL1( *rxData, nrUsableAdr );
    if( !result )
    {
      return false;
    }
    
    if( isDifferential )
    {
      result = DetermineAtmosphericCorrections_GPSL1( *rxBaseData );
      if( !result )
        return false;
      result = DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( *rxBaseData, nrP_base );
      if( !result )
        return false;
      result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( *rxBaseData, nrD_base );
      if( !result )
        return false;
      result = DetermineUsableAdrMeasurements_GPSL1( *rxBaseData, nrUsableAdr_base );
      if( !result )
      {
        return false;
      }

      // When in differential mode, only differntial measurements will be used
      result = DetermineBetweenReceiverDifferentialIndex(
        rxData,
        rxBaseData,
        true
        );
      if( !result )      
        return false;

      nrDifferentialPsr = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS &&
          rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
          rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
        {
          nrDifferentialPsr++;
        }
      }
      
#ifdef CONSTRAINPOS
      nrP = nrDifferentialPsr+3;
#else
      nrP = nrDifferentialPsr;
#endif

      nrDifferentialDoppler = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS &&
          rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
          rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
        {
          nrDifferentialDoppler++;
        }
      }

#ifdef CONSTRAINPOS
      nrD = nrDifferentialDoppler+3;
#else
      nrD = nrDifferentialDoppler;
#endif

      nrDifferentialAdr = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS &&
          rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
          rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
        {
          nrDifferentialAdr++;
        }
      }

    }
    else
    {
      return false;
    }

    result = DetermineDesignMatrixForThePositionSolution_GPSL1(
      *rxData,
      nrP,
      H_p );
    if( !result )
    {
      return false;
    }
    
    result = DetermineDesignMatrixForTheVelocitySolution_GPSL1(
      *rxData,
      nrD,
      H_v );
    if( !result )
    {
      return false;
    }

    result = DeterminePseudorangeMisclosures_GPSL1(
      rxData,
      rxBaseData,
      nrP,
      w_p );
    if( !result )
    {
      return false;
    }

    result = DetermineDopplerMisclosures_GPSL1(
      rxData,
      rxBaseData,
      nrD,
      w_v );
    if( !result )
    {
      return false;
    }

    bool hasAmbiguityChangeOccurred = false;
    result = DetermineAmbiguitiesChanges(
      rxData,
      rxBaseData,
      P,
      hasAmbiguityChangeOccurred );
    if( !result )
      return false;

    if( hasAmbiguityChangeOccurred )
      int gagagagag=101;

    
    //KO Debug: We need to replace this with a DD misclosure forming function that uses the current
    //DD misclosure estimate and the B matrix to determine the new DD misclosure.
    result = DetermineSingleDifferenceADR_Misclosures_GPSL1(
      rxData,
      rxBaseData,
      nrDifferentialAdr,
      w_adr );
    if( !result )
    {
      return false;
    }

    
    // Form the combined design matrix.
    n = nrP + nrD + nrDifferentialAdr;

    if( hasAmbiguityChangeOccurred )
      int gg=99; // place breakpoint here to debug issues related to changes in ambiguities.

    result = H.Resize( n, 8 + nrDifferentialAdr );
    if( !result )
    {
      return false;
    }

    result = r.Resize( n );
    if( !result )
    {
      return false;
    }

    unsigned int iP = 0; // A counter for the pseudorange.
    unsigned int iD = 0; // A counter for the Doppler.
    unsigned int iA = 0; // A counter for the ADR.
    double stdev; // a temporary double.
    for( k = 0; k < rxData->m_nrValidObs; k++ )
    {
      if( rxData->m_ObsArray[k].flags.isActive )
      {
        if( rxData->m_ObsArray[k].flags.isPsrUsedInSolution )
        {
          H[iP][0] = rxData->m_ObsArray[k].H_p[0];
          H[iP][1] = rxData->m_ObsArray[k].H_p[1];
          H[iP][2] = rxData->m_ObsArray[k].H_p[2];
          H[iP][6] = 1.0;

          stdev = rxData->m_ObsArray[k].stdev_psr;  // KO not differential obs error (need to change)
          r[iP] = stdev*stdev;
          iP++;
        }
        if( rxData->m_ObsArray[k].flags.isDopplerUsedInSolution )
        {
          H[iD+nrP][3] = rxData->m_ObsArray[k].H_v[0];
          H[iD+nrP][4] = rxData->m_ObsArray[k].H_v[1];
          H[iD+nrP][5] = rxData->m_ObsArray[k].H_v[2];
          H[iD+nrP][7] = 1.0;
          stdev = rxData->m_ObsArray[k].stdev_doppler * GPS_WAVELENGTHL1; // Change from cycles/s to meters/s.
          r[iD+nrP] = stdev*stdev;          
          iD++;
        }
        if( rxData->m_ObsArray[k].flags.isAdrUsedInSolution )
        {          
          H[iA+nrP+nrD][0] = rxData->m_ObsArray[k].H_p[0];
          H[iA+nrP+nrD][1] = rxData->m_ObsArray[k].H_p[1];
          H[iA+nrP+nrD][2] = rxData->m_ObsArray[k].H_p[2];
          H[iA+nrP+nrD][6] = 1.0;

		  //
          //H[iA+nrP+nrD][rxData->m_ObsArray[k].index_ambiguity_state] = 1.0;

          stdev = rxData->m_ObsArray[k].stdev_adr * GPS_WAVELENGTHL1; // Change from cycles to meters.
          r[iA+nrP+nrD] = stdev*stdev;          
          iA++;
        }
      }
    }

    Matrix timeMat(1);
    timeMat[0] = rxData->m_pvt.time.gps_tow;
    PrintMatToDebug( "gpstime", timeMat );
    PrintMatToDebug( "H", H );

    
    //KO Debug: Can still use the code to form the combined code and Doppler misclosure, but then the phase
    //misclosure needs to be added after the code and Doppler w is differenced using B.

    // Form the combined misclosure vector.
    result = w.Resize( n, 1 );
    if( !result )
    {
      return false;
    }
    for( i = 0; i < n; i++ )
    {
      if( i < nrP )
      {
        w[i] = w_p[i];
      }
      else if ( i < nrP+nrD )
      {
        w[i] = w_v[i-nrP];
      }
      else
      {
        w[i] = w_adr[i-nrP-nrD];
      }
    }

    PrintMatToDebug( "w", w );
    
    
	//KO Dec 18. Now that the design matrix, covariance matrix and misclosure vector are formed for the SD case time to DD them

	if(nrP == 0) 
		nrPDD = 0;
	else
		nrPDD = nrP - 1;
	if(nrD == 0)
		nrDDD = 0;
	else
		nrDDD = nrD - 1;
	if(nrDifferentialAdr == 0)
		nrDifferentialAdrDD = 0;
	else
		nrDifferentialAdrDD = nrDifferentialAdr - 1;

	nDD = nrPDD + nrDDD + nrDifferentialAdrDD;

	
	//At this point, the B matrix exists. Now we have to apply it to H, Cl and w
	
	// This line needs to be removed, once we have a DD phase misclosure forming function that works
  // Could still use something like this to form the code and doppler misclosure sub vector
  w = m_RTKDD.B*w;
	
  
  Matrix w_adrDD;
  result = DetermineDoubleDifferenceADR_Misclosures_GPSL1(
    rxData,
    rxBaseData,
    m_RTKDD.SubB,
    nrDifferentialAdrDD,
    w_adrDD ); 

  j = 0;
  for (i = nrPDD + nrDDD; i < w.nrows(); i++)
  {
    w[i] = w_adrDD[j];
    j++;
  }

  PrintMatToDebug("w_adrDD",w_adrDD);

  PrintMatToDebug("w",w);

  H = m_RTKDD.B*H;

	//Need to remove the dt column of H, and also the dtdot columns.

	Matrix R(r.nrows(),r.nrows());
	for (i = 0; i < r.nrows(); i++)
		R[i][i] = r[i];
	R = m_RTKDD.B * R * m_RTKDD.B.Transpose();

	//At this point have DD format H, w and R matrices, as well as Transition and Process noise
	//However now we have to replace the sequential estimator with a batch one because the observations are correlated

	//Need to remove columns 6 and 7 from H and also one ambiguity column,
	//and also replace the ambiguity column for the ADR rows with the identity matrix

	H.RemoveColumn(6);
	H.RemoveColumn(6);
	if( nrDifferentialAdrDD > 0 )
	{
		H.RemoveColumn(6);
		for (i = 0; i < nrDifferentialAdrDD; i++)
			H[i + nrPDD + nrDDD][6 + i] = 1.0;
	}


	 // Compute the Kalman gain matrix.
    // K = P*Ht*(H*P*Ht+R)^-1
    // do (H*P*Ht+R)^-1 first
	tmpMat = H*m_RTKDD.P*H.Transpose() + R;
    result = tmpMat.Inplace_Invert();
    if( !result )
      return false;    
	K = m_RTKDD.P*H.Transpose()*tmpMat;
    

    // Compute the change in states due to the innovations (misclosures).
    dx = K*w;

    

    // Compute the updated state variance-covariance matrix, P.
	result = I.Resize( K.nrows(), K.nrows() );
    if( !result )
      return false;
    result = I.Identity();
    if( !result )
      return false;

    m_RTKDD.P = (I - K * H)*m_RTKDD.P;
    
    PrintMatToDebug( "dx", dx );
    PrintMatToDebug( "P", m_RTKDD.P );
    PrintMatToDebug( "K", K );

    
    // Update the position and clock states
    // Update height first as it is need to reduce the corrections for lat and lon.
    hgt += dx[2];
    //clk += dx[6];

	// The corrections for lat and lon, dx_p, must be converted to [rad] from [m].
    GEODESY_ComputePrimeVerticalRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84, 
      lat,
      &N );
    GEODESY_ComputeMeridianRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84, 
      lat,
      &M );

     lat += dx[0] / ( M + hgt );             // convert from meters to radians.
     lon += dx[1] / (( N + hgt )*cos(lat));  // convert from meters to radians.

     result = rxData->UpdatePositionAndRxClock(
       lat,
       lon,
       hgt,
       0.0,
       sqrt(m_RTKDD.P[0][0]),
       sqrt(m_RTKDD.P[1][1]),
       sqrt(m_RTKDD.P[2][2]),
       0.0
       );
     if( !result )
       return false;


     // Update the velocity and clock drift states.
     vn        += dx[3];
     ve        += dx[4];
     vup       += dx[5];
     //clkdrift  += dx[7];

     result = rxData->UpdateVelocityAndClockDrift(
       vn,
       ve,
       vup,
       0.0,
       sqrt(m_RTKDD.P[3][3]),
       sqrt(m_RTKDD.P[4][4]),
       sqrt(m_RTKDD.P[5][5]),
       0.0 );
     if( !result )
       return false;

	 //KO: Update the ambiguities: Here is where it gets tricky, 
	 //How/where were the original values set and how are they indexed?

    
    Matrix amb;
    if( nrDifferentialAdrDD > 0 )
      amb.Resize(nrDifferentialAdrDD);
       
      j = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isAdrUsedInSolution && rxData->m_ObsArray[i].index_between_satellite_differential != -1 )
        {
          rxData->m_ObsArray[i].ambiguity_dd += dx[rxData->m_ObsArray[i].index_ambiguity_state_dd];
          amb[j] = rxData->m_ObsArray[i].ambiguity_dd;
          j++;
        }
      }
    
      PrintMatToDebug( "amb", amb );

    //for( i = 0; i < rxData->m_nrValidObs; i++ )
    //{
    //  if( rxData->m_ObsArray[i].flags.isUsedInPosSolution )
    //    rxData->m_pvt.nrPsrObsUsed++;
    //  if( rxData->m_ObsArray[i].flags.isUsedInVelSolution )
    //    rxData->m_pvt.nrDopplerObsUsed++;
    //  if( rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
    //    rxData->m_pvt.nrAdrObsUsed++;
    //}

    
#ifdef DEBUG_THE_ESTIMATOR
    char supermsg[8192];
    unsigned nrBytesInBuffer = 0;
    rxData->Debug_WriteSuperMsg80CharsWide( 
      supermsg,
      8192,
      51.0916666667*DEG2RAD,
      -114.0000000000*DEG2RAD,
      1000.000,
      nrBytesInBuffer );
    printf( supermsg );
#endif
   
    return true;
  }

  bool GNSS_Estimator::FixAmbiguities()
  {
	  return true;
  }

  
  
  bool GNSS_Estimator::Kalman_Update_8StatePVGM_SequentialMode(
    GNSS_RxData *rxData,      //!< A pointer to the rover receiver data. This must be a valid pointer.
    GNSS_RxData *rxBaseData,  //!< A pointer to the reference receiver data if available. NULL if not available.
    Matrix &P                 //!< The variance-covariance of the states.
  )
  {
    bool result = false;
    unsigned index = 0;
    unsigned i = 0;
    unsigned j = 0;
    unsigned n = 0;
    unsigned nrP = 0;           // The number of valid psr.
    unsigned nrP_base = 0;      // The number of valid psr for the reference station.
    unsigned nrD = 0;           // The number of valid Doppler.
    unsigned nrD_base = 0;      // The number of valid Doppler for the reference station.
    unsigned nrValidEph = 0;    // The number of valid ephemeris.
    unsigned nrDifferentialPsr = 0;     // The number of differential psr.
    unsigned nrDifferentialDoppler = 0; // The number of differential Doppler.
    bool isDifferential = false;
    unsigned u = 0;

    double lat = 0;
    double lon = 0;
    double hgt = 0;
    double clk = 0;
    double vn = 0;
    double ve = 0;
    double vup = 0;
    double clkdrift = 0;
    double M = 0;   // The computed meridian radius of curvature [m].
    double N = 0;   // The computed prime vertical radius of curvature [m].

    Matrix H_p;
    Matrix H_v;
    Matrix H;
    Matrix R_p;
    Matrix R_v;
    Matrix R;
    Matrix w_p;
    Matrix w_v;
    Matrix w;
    Matrix K;
    Matrix tmpMat;
    Matrix tmpMatP;
    Matrix dx(8);
    Matrix I;

    dx.Zero();
    
    // Store the current input pvt as the previous pvt since we are updating.
    rxData->m_prev_pvt = rxData->m_pvt;
        
    lat = rxData->m_pvt.latitude;
    lon = rxData->m_pvt.longitude;
    hgt = rxData->m_pvt.height;
    clk = rxData->m_pvt.clockOffset;
    vn  = rxData->m_pvt.vn;
    ve  = rxData->m_pvt.ve;
    vup = rxData->m_pvt.vup;
    clkdrift = rxData->m_pvt.clockDrift;
        
    if( rxBaseData != NULL )
      isDifferential = true;

#ifdef CONSTRAINPOS
    rxData->m_pvt.isPositionConstrained = 1;
#else
    rxData->m_pvt.isPositionConstrained = 0;
#endif
    
    
    ////
    // Perform operations specific to the rover.

    // Update the receiver time information (UTC and day of year)
    result = UpdateTime( *rxData );
    if( !result )
      return false;

    if( isDifferential )
    {
      result = UpdateTime( *rxBaseData );
      if( !result )
      {
        return false;
      }
    }

    result = DetermineSatellitePVT_GPSL1( rxData, rxBaseData, nrValidEph );
    if( !result )
    {
      return false;
    }


    result = DetermineAtmosphericCorrections_GPSL1( *rxData );
    if( !result )
    {
      return false;
    }
    result = DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( *rxData, nrP );
    if( !result )
    {
    return false;
    }
    result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( *rxData, nrD );
    if( !result )
    {
      return false;
    }

    if( isDifferential )
    {
      result = DetermineAtmosphericCorrections_GPSL1( *rxBaseData );
      if( !result )
        return false;
      result = DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( *rxBaseData, nrP_base );
      if( !result )
        return false;
      result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( *rxBaseData, nrD_base );
      if( !result )
        return false;


      // When in differential mode, only differntial measurements will be used
      result = DetermineBetweenReceiverDifferentialIndex(
        rxData,
        rxBaseData,
        true 
        );
      if( !result )      
        return false;

      nrDifferentialPsr = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS &&
          rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
          rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
        {
          nrDifferentialPsr++;
        }
      }

      
#ifdef CONSTRAINPOS
      nrP = nrDifferentialPsr+3;
#else
      nrP = nrDifferentialPsr;
#endif

      nrDifferentialDoppler = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS &&
          rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
          rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
        {
          nrDifferentialDoppler++;
        }
      }

#ifdef CONSTRAINPOS
      nrD = nrDifferentialDoppler+3;
#else
      nrD = nrDifferentialDoppler;
#endif
    }

    result = DetermineDesignMatrixForThePositionSolution_GPSL1(
      *rxData,
      nrP,
      H_p );
    if( !result )
    {
      return false;
    }
    
    result = DetermineDesignMatrixForTheVelocitySolution_GPSL1(
      *rxData,
      nrD,
      H_v );
    if( !result )
    {
      return false;
    }


    result = DetermineMeasurementVarianceCovarianceMatrixForThePositionSolution_GPSL1(
      *rxData,
      nrP,
      R_p );
    if( !result )
    {
      return false;
    }


    result = DetermineMeasurementVarianceCovarianceMatrixForTheVelocitySolution_GPSL1(
      *rxData,
      nrD,
      R_v );
    if( !result )
    {
      return false;
    }


    result = DeterminePseudorangeMisclosures_GPSL1(
      rxData,
      rxBaseData,
      nrP,
      w_p );
    if( !result )
    {
      return false;
    }

    result = DetermineDopplerMisclosures_GPSL1(
      rxData,
      rxBaseData,
      nrD,
      w_v );
    if( !result )
    {
      return false;
    }

    // Form the combined design matrix.
    n = H_p.GetNrRows() + H_v.GetNrRows();

    result = H.Resize( n, 8 );
    if( !result )
    {
      return false;
    }

    for( i = 0; i < n; i++ )
    {
      if( i < nrP )
      {
        for( j = 0; j < 3; j++ )
        {
          H[i][j] = H_p[i][j];
        }
#ifdef CONSTRAINPOS
        if( i < nrP-3 )
        {
          H[i][6] = 1.0;
        }
#else
        H[i][6] = 1.0;
#endif
      }
      else
      {
        for( j = 0; j < 3; j++ )
        {
          H[i][j+3] = H_v[i-nrP][j];
        }
#ifdef CONSTRAINPOS
        if( i < n-3 )
        {
          H[i][7] = 1.0;
        }
#else
        H[i][7] = 1.0;
#endif
      }
    }
    
    
    result = R.Resize( n, n );
    if( !result )
    {
      return false;
    }

    // Form the combined measurement variance-covariance matrix.
    for( i = 0; i < n; i++ )
    {
      if( i < nrP )
      {
        for( j = 0; j < nrP; j++ )
        {
          R[i][j] = R_p[i][j];
        }
      }
      else
      {
        for( j = 0; j < nrD; j++ )
        {
          R[i][j+nrD] = R_v[i-nrP][j];
        }
      }
    }
    

    // Form the combined misclosure vector.
    result = w.Resize( n, 1 );
    if( !result )
    {
      return false;
    }
    for( i = 0; i < n; i++ )
    {
      if( i < nrP )
      {
        w[i] = w_p[i];
      }
      else
      {
        w[i] = w_v[i-nrP];
      }
    }
    
    // The corrections for lat and lon, dx_p, must be converted to [rad] from [m].
    GEODESY_ComputePrimeVerticalRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      lat,
      &N );
    GEODESY_ComputeMeridianRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      lat,
      &M );
    double dlat = M + hgt;
    double dlon = (N + hgt) * cos(lat);

    u = 8;
    // Now the sequential measurement update section
    // For each measurement
    for( index = 0; index < n; index++ )
    {
      Matrix h(1,u);  // The i'th row of the design matrix, [ux1].
      Matrix ht(u,1); // The transpose of the i'th row of the design matrix, [ux1].
      Matrix pht;     // pht = P ht, [ux1].
      Matrix C;       // C = (h P ht + R_{ii}), [1x1].
      Matrix k_i;     // The i'th kalman gain. k_i = pht/C
      Matrix D;       // D = k_i h * P.
      
      // Copy out a row of the design matrix.
      for( j = 0; j < u; j++ )
      {
        h[0][j] = H[index][j];
        ht[j][0] = H[index][j];
      }
      //PrintMatToDebug( "h", h );

      // Compute pht
      pht = P;
      if( !pht.Inplace_PostMultiply( ht ) )
        return false;
      
      // Compute C = (h P h^T + R_{ii})
      C = h * pht;
      C[0] += R[index][index];
      
      // Compute k_i
      k_i = pht / C[0];

      //PrintMatToDebug( "k_i", k_i );

      // Update the state variance-coveriance;
      D = k_i;
      if( !D.Inplace_PostMultiply( h ) )
        return false;
      if( !D.Inplace_PostMultiply( P ) )
        return false;
      P -= D;

      
      double innovation = w[index];
      k_i.Inplace_MultiplyScalar( innovation );
      dx = k_i;
      //PrintMatToDebug( "dx", dx );

      // Update the position and clock states
      // Update height first as it is need to reduce the corrections for lat and lon.
      hgt += dx[2];
      clk += dx[6];

      // The corrections for lat and lon, dx_p, must be converted to [rad] from [m].
      lat += dx[0] / dlat;  // convert from meters to radians.
      lon += dx[1] / dlon;  // convert from meters to radians.

      result = rxData->UpdatePositionAndRxClock(
        lat,
        lon,
        hgt,
        clk,
        sqrt(P[0][0]),
        sqrt(P[1][1]),
        sqrt(P[2][2]),
        sqrt(P[6][6])
        );
      if( !result )
      {
        return false;
      }

      // Update the velocity and clock drift states.
      vn        += dx[3];
      ve        += dx[4];
      vup       += dx[5];
      clkdrift  += dx[7];

      result = rxData->UpdateVelocityAndClockDrift(
        vn,
        ve,
        vup,
        clkdrift,
        sqrt(P[3][3]),
        sqrt(P[4][4]),
        sqrt(P[5][5]),
        sqrt(P[7][7]) );
      if( !result )
      {
        return false;
      }

      result = DeterminePseudorangeMisclosures_GPSL1(
        rxData,
        rxBaseData,
        nrP,
        w_p );
      if( !result )
      {
        return false;
      }

      result = DetermineDopplerMisclosures_GPSL1(
        rxData,
        rxBaseData,
        nrD,
        w_v );
      if( !result )
      {
        return false;
      }

      for( i = 0; i < n; i++ )
      {
        if( i < nrP )
        {
          w[i] = w_p[i];
        }
        else
        {
          w[i] = w_v[i-nrP];
        }
      }            
    }
    return true;
  }



  bool GNSS_Estimator::Kalman_Update_8StatePVGM_SequentialMode_FloatSolution(
    GNSS_RxData *rxData,      //!< A pointer to the rover receiver data. This must be a valid pointer.
    GNSS_RxData *rxBaseData,  //!< A pointer to the reference receiver data if available. NULL if not available.
    Matrix &P                 //!< The variance-covariance of the states.
  )
  {
    bool result = false;
    unsigned index = 0;
    unsigned i = 0;
    unsigned j = 0;
    unsigned k = 0; 
    unsigned m = 0; // counter
    unsigned n = 0;
    unsigned nrP = 0;           // The number of valid psr.
    unsigned nrP_base = 0;      // The number of valid psr for the reference station.
    unsigned nrD = 0;           // The number of valid Doppler.
    unsigned nrD_base = 0;      // The number of valid Doppler for the reference station.
    unsigned nrValidEph = 0;    // The number of valid ephemeris.
    unsigned nrUsableAdr = 0;      // The number of valid adr.
    unsigned nrUsableAdr_base = 0; // The number of valid adr for the reference station.
    unsigned nrDifferentialPsr = 0;     // The number of differential psr.
    unsigned nrDifferentialDoppler = 0; // The number of differential Doppler.
    unsigned nrDifferentialAdr = 0;     // The number of differntial adr.
    bool isDifferential = false;
    unsigned u = 0;
    bool setToUseOnlyDifferential = true;

    double lat = 0;
    double lon = 0;
    double hgt = 0;
    double clk = 0;
    double vn = 0;
    double ve = 0;
    double vup = 0;
    double clkdrift = 0;
    double M = 0;   // The computed meridian radius of curvature [m].
    double N = 0;   // The computed prime vertical radius of curvature [m].

    Matrix H_p;
    Matrix H_v;
    Matrix H;
    Matrix r; // A vector corresponding to the diagonal elements of R.
    Matrix R_p;
    Matrix R_v;
    Matrix w_p;
    Matrix w_v;
    Matrix w_adr;
    Matrix w;
    Matrix K;
    Matrix tmpMat;
    Matrix tmpMatP;
    Matrix dx(8);
    Matrix I;

    dx.Zero();
    
    
    lat = rxData->m_pvt.latitude;
    lon = rxData->m_pvt.longitude;
    hgt = rxData->m_pvt.height;
    clk = rxData->m_pvt.clockOffset;
    vn  = rxData->m_pvt.vn;
    ve  = rxData->m_pvt.ve;
    vup = rxData->m_pvt.vup;
    clkdrift = rxData->m_pvt.clockDrift;
        
    if( rxBaseData != NULL )
      isDifferential = true;

#ifdef CONSTRAINPOS
    rxData->m_pvt.isPositionConstrained = 1;
#else
    rxData->m_pvt.isPositionConstrained = 0;
#endif
    
    
    ////
    // Perform operations specific to the rover.

    // Update the receiver time information (UTC and day of year)
    result = UpdateTime( *rxData );
    if( !result )
      return false;

    if( isDifferential )
    {
      result = UpdateTime( *rxBaseData );
      if( !result )
      {
        return false;
      }
    }

    result = DetermineSatellitePVT_GPSL1( rxData, rxBaseData, nrValidEph );
    if( !result )
    {
      return false;
    }


    result = DetermineAtmosphericCorrections_GPSL1( *rxData );
    if( !result )
    {
      return false;
    }
    result = DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( *rxData, nrP );
    if( !result )
    {
      return false;
    }
    result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( *rxData, nrD );
    if( !result )
    {
      return false;
    }
    result = DetermineUsableAdrMeasurements_GPSL1( *rxData, nrUsableAdr );
    if( !result )
    {
      return false;
    }
    
    if( isDifferential )
    {
      result = DetermineAtmosphericCorrections_GPSL1( *rxBaseData );
      if( !result )
        return false;
      result = DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( *rxBaseData, nrP_base );
      if( !result )
        return false;
      result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( *rxBaseData, nrD_base );
      if( !result )
        return false;
      result = DetermineUsableAdrMeasurements_GPSL1( *rxBaseData, nrUsableAdr_base );
      if( !result )
      {
        return false;
      }

      // When in differential mode, only differntial measurements will be used
      result = DetermineBetweenReceiverDifferentialIndex(
        rxData,
        rxBaseData,
        true
        );
      if( !result )      
        return false;

      nrDifferentialPsr = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS &&
          rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
          rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
        {
          nrDifferentialPsr++;
        }
      }
      
#ifdef CONSTRAINPOS
      nrP = nrDifferentialPsr+3;
#else
      nrP = nrDifferentialPsr;
#endif

      nrDifferentialDoppler = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS &&
          rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
          rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
        {
          nrDifferentialDoppler++;
        }
      }

#ifdef CONSTRAINPOS
      nrD = nrDifferentialDoppler+3;
#else
      nrD = nrDifferentialDoppler;
#endif

      nrDifferentialAdr = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].system == GNSS_GPS &&
          rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
          rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
        {
          nrDifferentialAdr++;
        }
      }

    }
    else
    {
      return false;
    }

    result = DetermineDesignMatrixForThePositionSolution_GPSL1(
      *rxData,
      nrP,
      H_p );
    if( !result )
    {
      return false;
    }
    
    result = DetermineDesignMatrixForTheVelocitySolution_GPSL1(
      *rxData,
      nrD,
      H_v );
    if( !result )
    {
      return false;
    }

    result = DeterminePseudorangeMisclosures_GPSL1(
      rxData,
      rxBaseData,
      nrP,
      w_p );
    if( !result )
    {
      return false;
    }

    result = DetermineDopplerMisclosures_GPSL1(
      rxData,
      rxBaseData,
      nrD,
      w_v );
    if( !result )
    {
      return false;
    }

    bool hasAmbiguityChangeOccurred = false;
    result = DetermineAmbiguitiesChanges(
      rxData,
      rxBaseData,
      P,
      hasAmbiguityChangeOccurred );
    if( !result )
      return false;

    result = DetermineSingleDifferenceADR_Misclosures_GPSL1(
      rxData,
      rxBaseData,
      nrDifferentialAdr,
      w_adr );
    if( !result )
    {
      return false;
    }

    
    // Form the combined design matrix.
    n = nrP + nrD + nrDifferentialAdr;

    if( hasAmbiguityChangeOccurred )
      int gg=99; // place breakpoint here to debug issues related to changes in ambiguities.

    result = H.Resize( n, 8 + nrDifferentialAdr );
    if( !result )
    {
      return false;
    }

    result = r.Resize( n );
    if( !result )
    {
      return false;
    }

    unsigned int iP = 0; // A counter for the pseudorange.
    unsigned int iD = 0; // A counter for the Doppler.
    unsigned int iA = 0; // A counter for the ADR.
    double stdev; // a temporary double.
    for( k = 0; k < rxData->m_nrValidObs; k++ )
    {
      if( rxData->m_ObsArray[k].flags.isActive )
      {
        if( rxData->m_ObsArray[k].flags.isPsrUsedInSolution )
        {
          H[iP][0] = rxData->m_ObsArray[k].H_p[0];
          H[iP][1] = rxData->m_ObsArray[k].H_p[1];
          H[iP][2] = rxData->m_ObsArray[k].H_p[2];
          H[iP][6] = 1.0;

          stdev = rxData->m_ObsArray[k].stdev_psr;  // KO not differential obs error (need to change)
          r[iP] = stdev*stdev;
          iP++;
        }
        if( rxData->m_ObsArray[k].flags.isDopplerUsedInSolution )
        {
          H[iD+nrP][3] = rxData->m_ObsArray[k].H_v[0];
          H[iD+nrP][4] = rxData->m_ObsArray[k].H_v[1];
          H[iD+nrP][5] = rxData->m_ObsArray[k].H_v[2];
          H[iD+nrP][7] = 1.0;
          stdev = rxData->m_ObsArray[k].stdev_doppler * GPS_WAVELENGTHL1; // Change from cycles/s to meters/s.
          r[iD+nrP] = stdev*stdev;          
          iD++;
        }
        if( rxData->m_ObsArray[k].flags.isAdrUsedInSolution )
        {          
          H[iA+nrP+nrD][0] = rxData->m_ObsArray[k].H_p[0];
          H[iA+nrP+nrD][1] = rxData->m_ObsArray[k].H_p[1];
          H[iA+nrP+nrD][2] = rxData->m_ObsArray[k].H_p[2];
          H[iA+nrP+nrD][6] = 1.0;

          H[iA+nrP+nrD][rxData->m_ObsArray[k].index_ambiguity_state] = 1.0;

          stdev = rxData->m_ObsArray[k].stdev_adr * GPS_WAVELENGTHL1; // Change from cycles to meters.
          r[iA+nrP+nrD] = stdev*stdev;          
          iA++;
        }
      }
    }

    Matrix timeMat(1);
    timeMat[0] = rxData->m_pvt.time.gps_tow;
    PrintMatToDebug( "gpstime", timeMat );
    PrintMatToDebug( "H", H );

    // Form the combined misclosure vector.
    result = w.Resize( n, 1 );
    if( !result )
    {
      return false;
    }
    for( i = 0; i < n; i++ )
    {
      if( i < nrP )
      {
        w[i] = w_p[i];
      }
      else if ( i < nrP+nrD )
      {
        w[i] = w_v[i-nrP];
      }
      else
      {
        w[i] = w_adr[i-nrP-nrD];
      }
    }

    PrintMatToDebug( "w", w );
    
    // The corrections for lat and lon, dx_p, must be converted to [rad] from [m].
    GEODESY_ComputePrimeVerticalRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      lat,
      &N );
    GEODESY_ComputeMeridianRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      lat,
      &M );
    double dlat = M + hgt;
    double dlon = (N + hgt) * cos(lat);

    Matrix amb;
    if( nrDifferentialAdr > 0 )
      amb.Resize(nrDifferentialAdr);

    u = 8 + nrDifferentialAdr;
    // Now the sequential measurement update section
    // For each measurement
    for( index = 0; index < n; index++ )
    {
      Matrix h(1,u);  // The i'th row of the design matrix, [ux1].
      Matrix ht(u,1); // The transpose of the i'th row of the design matrix, [ux1].
      Matrix pht;     // pht = P ht, [ux1].
      Matrix C;       // C = (h P ht + R_{ii}), [1x1].
      Matrix k_i;     // The i'th kalman gain. k_i = pht/C
      Matrix D;       // D = k_i h * P.
      
      // Copy out a row of the design matrix.
      for( j = 0; j < u; j++ )
      {
        h[0][j] = H[index][j];
        ht[j][0] = H[index][j];
      }
      //PrintMatToDebug( "h", h );

      // Compute pht
      pht = P;
      if( !pht.Inplace_PostMultiply( ht ) )
        return false;
      
      // Compute C = (h P h^T + R_{ii})
      C = h * pht;
      C[0] += r[index];
      
      // Compute k_i
      k_i = pht / C[0];

      //PrintMatToDebug( "k_i", k_i );

      // Update the state variance-coveriance;
      D = k_i;
      if( !D.Inplace_PostMultiply( h ) )
        return false;
      if( !D.Inplace_PostMultiply( P ) )
        return false;
      P -= D;
      
      double innovation = w[index];
      k_i.Inplace_MultiplyScalar( innovation );
      dx = k_i;
      //PrintMatToDebug( "dx", dx );

      // Update the position and clock states
      // Update height first as it is need to reduce the corrections for lat and lon.
      hgt += dx[2];
      clk += dx[6];

      // The corrections for lat and lon, dx_p, must be converted to [rad] from [m].
      lat += dx[0] / dlat;  // convert from meters to radians.
      lon += dx[1] / dlon;  // convert from meters to radians.

      result = rxData->UpdatePositionAndRxClock(
        lat,
        lon,
        hgt,
        clk,
        sqrt(P[0][0]),
        sqrt(P[1][1]),
        sqrt(P[2][2]),
        sqrt(P[6][6])
        );
      if( !result )
      {
        return false;
      }

      // Update the velocity and clock drift states.
      vn        += dx[3];
      ve        += dx[4];
      vup       += dx[5];
      clkdrift  += dx[7];

      result = rxData->UpdateVelocityAndClockDrift(
        vn,
        ve,
        vup,
        clkdrift,
        sqrt(P[3][3]),
        sqrt(P[4][4]),
        sqrt(P[5][5]),
        sqrt(P[7][7]) );
      if( !result )
      {
        return false;
      }

	  // KO recompute misclosures based on the single update (theoretically could redo design matrix too, but very small).
	  result = DetermineDesignMatrixForThePositionSolution_GPSL1(
        *rxData,
        nrP,
        H_p );
      if( !result )
      {
        return false;
      }

      result = DetermineDesignMatrixForTheVelocitySolution_GPSL1(
        *rxData,
        nrD,
        H_v );
      if( !result )
      {
        return false;
      }

      result = DeterminePseudorangeMisclosures_GPSL1(
        rxData,
        rxBaseData,
        nrP,
        w_p );
      if( !result )
      {
        return false;
      }

      result = DetermineDopplerMisclosures_GPSL1(
        rxData,
        rxBaseData,
        nrD,
        w_v );
      if( !result )
      {
        return false;
      }

      j = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
        {
          rxData->m_ObsArray[i].ambiguity += dx[rxData->m_ObsArray[i].index_ambiguity_state];
          amb[j] = rxData->m_ObsArray[i].ambiguity;
          j++;
        }
      }

      result = DetermineSingleDifferenceADR_Misclosures_GPSL1(
        rxData,
        rxBaseData,
        nrDifferentialAdr,
        w_adr );

      for( i = 0; i < n; i++ )
      {
        if( i < nrP )
        {
          w[i] = w_p[i];
        }
        else if( i < nrP+nrD )
        {
          w[i] = w_v[i-nrP];
        }
        else
        {
          w[i] = w_adr[i-nrP-nrD];
        }
      }            
    }

    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
        rxData->m_pvt.nrPsrObsUsed++;
      if( rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
        rxData->m_pvt.nrDopplerObsUsed++;
      if( rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
        rxData->m_pvt.nrAdrObsUsed++;
    }

    
#ifdef DEBUG_THE_ESTIMATOR
    char supermsg[8192];
    unsigned nrBytesInBuffer = 0;
    rxData->Debug_WriteSuperMsg80CharsWide( 
      supermsg,
      8192,
      51.0916666667*DEG2RAD,
      -114.0000000000*DEG2RAD,
      1000.000,
      nrBytesInBuffer );
    printf( supermsg );
#endif
   
    return true;
  }


  bool GNSS_Estimator::PrintMatToDebug( const char *name, Matrix& M )
  {
#ifdef DEBUG_THE_ESTIMATOR

    char buffer[8192];
    if( name == NULL )
      return false;
    if( !M.PrintToBuffer( buffer, 8192, 7 ) )
      return 1;
    printf( "%s = \n%s\n", name, buffer );

    if( m_debug == NULL )
    {
      m_debug = fopen( "debug.txt", "w" );
      if( !m_debug )
      {
        return false;
      }
    }
    else
    {
      fprintf( m_debug, "%s = \n%s\n", name, buffer );      
      fflush( m_debug );
    }
#endif
    return true;
  }



  bool GNSS_Estimator::DetermineAmbiguitiesChanges( 
    GNSS_RxData *rxData,     //!< The receiver data.
    GNSS_RxData *rxBaseData, //!< The reference receiver data if any (NULL if not available).
    Matrix &P,               //!< The state variance-covariance matrix.
    bool& changeOccured 
    )
  {
    unsigned i = 0;
    unsigned j = 0;
    unsigned k = 0;
    unsigned m = 0;
    unsigned iP = 0;
    unsigned iD = 0;
    unsigned iA = 0;
    unsigned nP = 0; // The number of valid pseudoranges used in solution.
    unsigned nD = 0; // The number of valid Doppler used in solution.
    unsigned nA = 0; // The number of valid adr used in solution.
    // First look for ambiguities that are no longer active.
    bool isAmbiguityActive = false;      
    std::list<stAmbiguityInfo>::iterator iter;
    std::list<stAmbiguityInfo>::iterator check_iter;
    std::list<stAmbiguityInfo>::iterator remove_iter;
    std::list<unsigned int> remove_list;
    std::list<unsigned int>::const_iterator list_iter;

    changeOccured = false;

    // At this point the active ambiguities are those that were included in the 
    // estimation of the previous epoch. This will change if some of those ambiguites
    // are no longer included in the observations.
    for( iter = m_ActiveAmbiguitiesList.begin(); iter != m_ActiveAmbiguitiesList.end();  )
    {
      isAmbiguityActive = false;
      
      // For the active ambiguity (iter), search through the observation array to see
      // if the ambiguity is still present and if it is to be estimated.      
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( // iter->channel == rxData->m_ObsArray[i].channel && // GDM - NO CHANNEL MATCHING FOR RINEX DATA!
          iter->id        == rxData->m_ObsArray[i].id &&
          iter->system    == rxData->m_ObsArray[i].system && 
          iter->freqType  == rxData->m_ObsArray[i].freqType &&
          rxData->m_ObsArray[i].flags.isAdrUsedInSolution &&
          rxData->m_ObsArray[i].flags.isDifferentialAdrAvailable )
        {
          // This ambiguity is already actively estimated.
          isAmbiguityActive = true;
          break;
        }
      }

      if( !isAmbiguityActive )
      {
        changeOccured = true;

        // Add the state_index to the list of indices to be
        // removed from state variance covariance matrix.        
        remove_list.push_back( iter->state_index );

        remove_iter = iter;
        ++iter;
        m_ActiveAmbiguitiesList.erase( remove_iter );        
      }
      else
      {
        ++iter;
      }
    }

    // Deal with removing ambiguities from P.
    if( remove_list.size() > 0 )
    {
      unsigned int *rows = NULL;    
      unsigned int nrows = static_cast<unsigned int>(remove_list.size());
      rows = new unsigned int[nrows];
      if( rows == NULL )
      {
        return false;
      }

      i = 0;
      for( list_iter = remove_list.begin(); list_iter != remove_list.end(); ++list_iter )
      {
        rows[i] = *list_iter;
        i++;
      }

      //P.Print( "P.now.txt", 10 );
      if( !P.RemoveRowsAndColumns( nrows, rows, nrows, rows ) )
      {
        return false;
      }
      //P.Print( "P.after.txt", 10 );

      delete [] rows;      
    }

      // The indices of the ambiguities in the list are now incorrect due to the removals and must be corrected.
      // Update the ambiguities list first, then the state indices contained in rxData->m_ObsArray
      for( iter = m_ActiveAmbiguitiesList.begin(); iter != m_ActiveAmbiguitiesList.end(); ++iter )
      {
        for( list_iter = remove_list.begin(); list_iter != remove_list.end(); ++list_iter )
        {
          if( iter->state_index > (int)(*list_iter) )
          {
            iter->state_index -= 1;
          }
        }
        // Update the observation array with the new info.
        for( i = 0; i < rxData->m_nrValidObs; i++ )
        {
          if( // iter->channel == rxData->m_ObsArray[i].channel  && // GDM - NO CHANNEL MATCHING FOR RINEX DATA!
            iter->id        == rxData->m_ObsArray[i].id       &&
            iter->system    == rxData->m_ObsArray[i].system   && 
            iter->freqType  == rxData->m_ObsArray[i].freqType &&
            rxData->m_ObsArray[i].flags.isAdrUsedInSolution   &&
            rxData->m_ObsArray[i].flags.isDifferentialAdrAvailable )
          {
            rxData->m_ObsArray[i].index_ambiguity_state = iter->state_index;
            break;
          }
        }      
      }
      remove_list.clear();
    


    // Add new ambiguities if any.
    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      isAmbiguityActive = false;
      if( rxData->m_ObsArray[i].system == GNSS_GPS && rxData->m_ObsArray[i].freqType == GNSS_GPSL1 )
      {
        if( rxData->m_ObsArray[i].flags.isActive )
        {
          if( rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
          {
            // Look for this ambiguity in the ambiguities list.
            for( iter = m_ActiveAmbiguitiesList.begin(); iter != m_ActiveAmbiguitiesList.end(); ++iter )
            {
              if( // iter->channel == rxData->m_ObsArray[i].channel && // GDM - NO CHANNEL MATCHING FOR RINEX DATA!
                iter->id        == rxData->m_ObsArray[i].id      &&
                iter->system    == rxData->m_ObsArray[i].system  && 
                iter->freqType  == rxData->m_ObsArray[i].freqType &&
                rxData->m_ObsArray[i].flags.isDifferentialAdrAvailable )
              {
                // This ambiguity is already actively estimated.
                isAmbiguityActive = true;
                break;
              }
            } 

            if( !isAmbiguityActive )
            {              
              changeOccured = true;

              // Update the active amibguities list.
              stAmbiguityInfo amb_info;

              amb_info.channel     = rxData->m_ObsArray[i].channel;
              amb_info.id          = rxData->m_ObsArray[i].id;
              amb_info.system      = rxData->m_ObsArray[i].system;
              amb_info.freqType    = rxData->m_ObsArray[i].freqType;
              amb_info.state_index = P.nrows(); // This will be the index of the row and column in P for this ambiguity.

              rxData->m_ObsArray[i].index_ambiguity_state = amb_info.state_index;
              
              m_ActiveAmbiguitiesList.push_back( amb_info );

              // Add a new row and columgn to the state variance-covariance matrix.
              if( !P.Redim( P.nrows()+1, P.ncols()+1 ) )
                return false;

              // Set the initial variance of the ambiguity state [m].
              P[amb_info.state_index][amb_info.state_index] = 125; // KO Arbitrary value, to improve

              // Initialize the ambiguity state [m].
              // Compute the single difference adr measurement [m].
              double sd_adr_measured = rxData->m_ObsArray[i].adr - rxBaseData->m_ObsArray[rxData->m_ObsArray[i].index_differential].adr;
              sd_adr_measured *= GPS_WAVELENGTHL1;

              // Compute the single difference psr measurement.
              double sd_psr_measured = rxData->m_ObsArray[i].psr - rxBaseData->m_ObsArray[rxData->m_ObsArray[i].index_differential].psr;

              // Initialize the ambiguity to the difference between the single difference adr
              // and the single difference pseudorange.

              rxData->m_ObsArray[i].ambiguity =  sd_adr_measured - sd_psr_measured; // in meters!  //KO possibly replace psr with position derived range plus clock offset

              //double sd_computed = rxData->m_ObsArray[i].range - rxBaseData->m_ObsArray[rxData->m_ObsArray[i].index_differential_adr].range;
              //sd_computed += rxData->m_pvt.clockOffset;
              //rxData->m_ObsArray[i].ambiguity =  sd_adr_measured - sd_computed;
            }
          }
          else
          {
            rxData->m_ObsArray[i].index_ambiguity_state = -1;
            rxData->m_ObsArray[i].ambiguity = 0.0;
          }
        }
      }
    }

    // Check for errors.
    i = 0;
    j = 0;
    for( iter = m_ActiveAmbiguitiesList.begin(); iter != m_ActiveAmbiguitiesList.end(); ++iter )
    {
      j=0;
      for( check_iter = m_ActiveAmbiguitiesList.begin(); check_iter != m_ActiveAmbiguitiesList.end(); ++check_iter )
      {
        if( i != j )
        {
          if( check_iter->state_index == iter->state_index )
            return false;
        }
        j++;
      }
      i++;      
    }


#ifdef KO_SECTION
    //if( changeOccured ) //need this whether or not change has occured.
    {
      // KO_DEBUG  For now use the first active channel as the base satellite and differnce the others.
      int ch_index_base = -1;

      // Loop through and look for the first active channel with a 
      // valid adr (ready for use in the single difference solution).
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isActive )
        {
          if( rxData->m_ObsArray[i].system == GNSS_GPS &&
            rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
            rxData->m_ObsArray[i].flags.isAdrValid &&
            rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
          {
            ch_index_base = i;
            break;
          }
        }
      }
      if( ch_index_base == -1 )
      {
        // KO_TODO set B to 0x0
        return true;
      }

      j = 0;
      // Set the differencing indices between channels.
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        rxData->m_ObsArray[i].index_between_satellite_differential = -1; // initialize to not used.
        if( rxData->m_ObsArray[i].flags.isActive )
        {
          if( rxData->m_ObsArray[i].system == GNSS_GPS &&
            rxData->m_ObsArray[i].freqType == GNSS_GPSL1 &&
            rxData->m_ObsArray[i].flags.isAdrValid &&
            rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
          {
            if( i != ch_index_base )
            {
              rxData->m_ObsArray[i].index_between_satellite_differential = ch_index_base;
              //KYLE CHANGE 11JAN2008 CHECK THIS
              rxData->m_ObsArray[i].index_ambiguity_state_dd = 6 + j;
              j++;
            }
          }
        }
      }

      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        rxData->m_ObsArray[i].index_psr_B = -1;
        if( rxData->m_ObsArray[i].flags.isActive )
        {
          if( rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
          {
            rxData->m_ObsArray[i].index_psr_B = nP;
            nP++;
          }          
        }
      }

      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        rxData->m_ObsArray[i].index_Doppler_B = -1;
        if( rxData->m_ObsArray[i].flags.isActive )
        {
          if( rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
          {
            rxData->m_ObsArray[i].index_Doppler_B = nP + nD;
            nD++;
          }          
        }
      }

      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        rxData->m_ObsArray[i].index_adr_B = -1;
        if( rxData->m_ObsArray[i].flags.isActive )
        {
          if( rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
          {
            rxData->m_ObsArray[i].index_adr_B = nP + nD + nA;
            nA++;
          }
        }
      }

      unsigned n=0;
      
      j = 0;
      if( nP > 0 )
      {
        n += nP - 1;
        j++;
      }
      if( nD > 0 )
      {
        n += nD - 1;
        j++;
      }
      if( nA > 0 )
      {
        n += nA - 1;
        j++;
      }

      // Dimension B and set to zero.
      if( !m_RTKDD.B.Resize( n, n+j ) )
        return false;

      // Form the double difference operator matrix.
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isActive )
        {
          if( rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
          {
            if( rxData->m_ObsArray[i].index_between_satellite_differential != -1 )
            {
              if( rxData->m_ObsArray[ rxData->m_ObsArray[i].index_between_satellite_differential ].index_psr_B == -1 )
              {
                return false;
              }
              m_RTKDD.B[iP][ rxData->m_ObsArray[ rxData->m_ObsArray[i].index_between_satellite_differential ].index_psr_B ] = -1.0;
              m_RTKDD.B[iP][ rxData->m_ObsArray[i].index_psr_B ] = 1.0;
              iP++;
            }
          }
        }
      }


      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isActive )
        {      
          if( rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
          {
            if( rxData->m_ObsArray[i].index_between_satellite_differential != -1 )
            {
              if( rxData->m_ObsArray[rxData->m_ObsArray[i].index_between_satellite_differential].index_Doppler_B == -1 )
              {
                return false;
              }
              m_RTKDD.B[iD+iP][ rxData->m_ObsArray[rxData->m_ObsArray[i].index_between_satellite_differential].index_Doppler_B ] = -1.0;
              m_RTKDD.B[iD+iP][ rxData->m_ObsArray[i].index_Doppler_B ] = 1.0;
              iD++;
            }            
          }
        }
      }

      if( !m_RTKDD.B.isEmpty() )
        m_RTKDD.prevB    = m_RTKDD.B;
      if( !m_RTKDD.SubB.isEmpty() )
        m_RTKDD.prevSubB = m_RTKDD.SubB;

      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isActive )
        {      
          if( rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
          {
            if( rxData->m_ObsArray[i].index_between_satellite_differential != -1 )
            {
              if( rxData->m_ObsArray[rxData->m_ObsArray[i].index_between_satellite_differential].index_adr_B == -1 )
              {
                return false;
              }
              m_RTKDD.B[iA+iP+iD][ rxData->m_ObsArray[rxData->m_ObsArray[i].index_between_satellite_differential].index_adr_B ] = -1.0;
              m_RTKDD.B[iA+iP+iD][ rxData->m_ObsArray[i].index_adr_B ] = 1.0;
              
              
              //Assign an initial value to the DD ambiguity based on the initial value of the SD ambiguity set above
              // Initialize the ambiguity state [m].
              // Compute the single difference adr measurement [m].
              double sd_adr_current = rxData->m_ObsArray[i].adr - rxBaseData->m_ObsArray[rxData->m_ObsArray[i].index_differential].adr;
              sd_adr_current *= GPS_WAVELENGTHL1;
              double sd_psr_current = rxData->m_ObsArray[i].psr - rxBaseData->m_ObsArray[rxData->m_ObsArray[i].index_differential].psr;
              
              double sd_ambiguity_current =  sd_adr_current - sd_psr_current; // in meters!  //KO possibly replace psr with position derived range plus clock offset

              // Initialize the ambiguity state [m].
              // Compute the single difference adr measurement [m].
              double sd_adr_base = rxData->m_ObsArray[rxData->m_ObsArray[i].index_between_satellite_differential].adr - rxBaseData->m_ObsArray[rxData->m_ObsArray[rxData->m_ObsArray[i].index_between_satellite_differential].index_differential].adr;
              sd_adr_base *= GPS_WAVELENGTHL1;
              
              double sd_psr_base = rxData->m_ObsArray[rxData->m_ObsArray[i].index_between_satellite_differential].psr - rxBaseData->m_ObsArray[rxData->m_ObsArray[rxData->m_ObsArray[i].index_between_satellite_differential].index_differential].psr;
              double sd_ambiguity_base =  sd_adr_base - sd_psr_base;

              // Initial double difference ambiugity is the difference difference between the adr and phase of the 4 observations involved in the DD
              rxData->m_ObsArray[i].ambiguity_dd = sd_ambiguity_current - sd_ambiguity_base;
              iA++;
            }            
          }
        }
      }
      if( iA < 1 )
      {
        // KO_TODO anything else?
        return true;
      }

      PrintMatToDebug( "B", m_RTKDD.B );

      if( !m_RTKDD.SubB.Resize(iA,iA+1)  )
        return false;

      for( i = 0; i < iA; i++ )
      {
        for( j = 0; j < iA+1; j++ )
        {
          m_RTKDD.SubB[i][j] = m_RTKDD.B[i+iP+iD][j+nP+nD];
        }
      }

      PrintMatToDebug( "subB", m_RTKDD.SubB );

      if( m_RTKDD.prevSubB.isEmpty() )
      {
        // Just build dd P
        Matrix subP(nA,nA);
        for( i = 0; i < nA; i++ )
        {
          for( j = 0; j < nA; j++ )
          {          
            subP[i][j] = P[i+6][j+6];            
          }
        }

        m_RTKDD.P = P;
        P.Redim( 6, 6 );
        P.Redim( 6+iA, 6+iA );
        
        subP = m_RTKDD.SubB*subP*m_RTKDD.SubB.T();
        for( i = 0; i < iA; i++ )
        {
          for( j = 0; j < iA; j++ )
          {          
            P[i+6][j+6] = subP[i][j];
          }
        }       
        PrintMatToDebug( "subP", subP );
      }
      
      PrintMatToDebug( "P", m_RTKDD.P );

      //m_RTKDD.P = 



      /*
      for( iter = m_ActiveAmbiguitiesList.begin(); iter != m_ActiveAmbiguitiesList.end(); ++iter )
      {
        if( rxData->m_ObsArray[iter->channel].index_between_satellite_differential != -1 )
        {

        }
      }
      */
      



      // Form the double differnce state variance covariance matrix.

    }
#endif // KO_SECTION

    return true;
  }
      




} // end namespace GNSS



