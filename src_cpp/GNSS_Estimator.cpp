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

#include "gnss_error.h"
#include "GNSS_Estimator.h"
#include "GNSS_RxData.h"
#include "constants.h"
#include "geodesy.h"
#include "gps.h"
#include "ionosphere.h"
#include "navigation.h"
#include "troposphere.h"
#include "time_conversion.h"

//#define DEBUG_THE_ESTIMATOR
#define GNSS_CYCLESLIP_THREADHOLD 3
//#define KO_SECTION

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
   : m_debug(NULL), m_FilterType(GNSS_FILTER_TYPE_INVALID)
  {    
  }


  GNSS_Estimator::~GNSS_Estimator()
  {
    if( m_debug )
    {
      fclose(m_debug);
    }
  }

  bool GNSS_Estimator::InitializeStateVarianceCovarianceFromLeastSquares_RTK(
    Matrix &pos_P, //!< The variance covariance of the position and clock states from least squares, state order: latitude, longitude, height, clock ofset [4x4].
    Matrix &vel_P  //!< The variance covariance of the velocity and clock drift states from least squares, state order: latitude rate, longitude rate, height rate, clock drift [4x4].
    )
  {
    unsigned i = 0;
    unsigned j = 0;

    if( pos_P.nrows() != 4 || pos_P.ncols() != 4 )
    {
      GNSS_ERROR_MSG( "if( pos_P.nrows() != 4 || pos_P.ncols() != 4 )" );
      return false;
    }

    if( m_FilterType == GNSS_FILTER_TYPE_EKF || m_FilterType == GNSS_FILTER_TYPE_RTK8 )
    {
      if( vel_P.nrows() != 4 || vel_P.ncols() != 4 )      
      {
        // velocity is unknown, so we'll indicate that with large variance.
        if( !vel_P.Identity( 4 ) )
        {
          GNSS_ERROR_MSG( "if( !vel_P.Identity( 4 ) )" );
          return false;
        }
        if( !vel_P.Inplace_AddScalar( 10000.0 ) )
        {
          GNSS_ERROR_MSG( "if( !vel_P.Inplace_AddScalar( 10000.0 ) )" );
          return false;
        }
      }
      if( !m_RTK.P.Resize(8,8) )
      {
        GNSS_ERROR_MSG( "if( !m_RTK.Resize(8,8) )" )
        return false;
      }
      for( i = 0; i < 4; i++ )
      {
        for( j = 0; j < 4; j++ )
        {
          m_RTK.P[i][j]     = pos_P[i][j];
          m_RTK.P[i+4][j+4] = vel_P[i][j];
        }
      }
    }
    else
    {
      m_RTK.P = pos_P;
    }
    return true;
  }

  

  bool GNSS_Estimator::DealWithClockJumps(
    GNSS_RxData *rxData,      //!< A pointer to the rover receiver data. This must be a valid pointer.
    GNSS_RxData *rxBaseData   //!< A pointer to the reference receiver data if available. NULL if not available.
    )
  {
    unsigned i = 0;
    if( rxData == NULL )
    {
      GNSS_ERROR_MSG( "rxData == NULL" );
      return false;
    }
    
    if( rxData->m_msJumpDetected_Positive )
    {
      rxData->m_pvt.clockOffset += ONE_MS_IN_M;

      if( m_FilterType == GNSS_FILTER_TYPE_RTK4 )
      {
        m_RTK.P[3][3] += 10000.0;

        for( i = 4; i < m_RTK.P.nrows(); i++ )
          m_RTK.P[i][i] += 10000.0;
      }
      else if( m_FilterType == GNSS_FILTER_TYPE_RTK8 )
      {
        m_RTK.P[6][6] += 10000.0;
      }
      else if( m_FilterType == GNSS_FILTER_TYPE_EKF )
      {
        m_EKF.P[6][6] += 10000.0;
      }
    }
    else if( rxData->m_msJumpDetected_Negative )
    {
      rxData->m_pvt.clockOffset -= ONE_MS_IN_M;

      if( m_FilterType == GNSS_FILTER_TYPE_RTK4 )
      {
        m_RTK.P[3][3] += 10000.0;

        for( i = 4; i < m_RTK.P.nrows(); i++ )
          m_RTK.P[i][i] += 10000.0;
      }
      else if( m_FilterType == GNSS_FILTER_TYPE_RTK8 )
      {
        m_RTK.P[6][6] += 10000.0;
      }
      else if( m_FilterType == GNSS_FILTER_TYPE_EKF )
      {
        m_EKF.P[6][6] += 10000.0;
      }
    }
    else if( rxData->m_clockJumpDetected ) 
    {
      // A large arbitrary clock jump was detected.
      // compensate for it as best as possible.
      rxData->m_pvt.clockOffset += rxData->m_clockJump; 

      if( m_FilterType == GNSS_FILTER_TYPE_RTK4 )
      {
        m_RTK.P[3][3] += 10000.0;

        for( i = 4; i < m_RTK.P.nrows(); i++ )
          m_RTK.P[i][i] += 10000.0;
      }
      else if( m_FilterType == GNSS_FILTER_TYPE_RTK8 )
      {
        m_RTK.P[6][6] += 10000.0;
      }
      else if( m_FilterType == GNSS_FILTER_TYPE_EKF )
      {
        m_EKF.P[6][6] += 10000.0;
      }
    }

    if( rxBaseData )
    {
      if( rxBaseData->m_msJumpDetected_Positive )
      {
        rxData->m_pvt.clockOffset -= ONE_MS_IN_M;

        if( m_FilterType == GNSS_FILTER_TYPE_RTK4 )
        {
          m_RTK.P[3][3] += 10000.0;

          for( i = 4; i < m_RTK.P.nrows(); i++ )
            m_RTK.P[i][i] += 10000.0;
        }
        else if( m_FilterType == GNSS_FILTER_TYPE_RTK8 )
        {
          m_RTK.P[6][6] += 10000.0;
        }
        else if( m_FilterType == GNSS_FILTER_TYPE_EKF )
        {
          m_EKF.P[6][6] += 10000.0;
        }
      }
      else if( rxBaseData->m_msJumpDetected_Negative )
      {
        rxData->m_pvt.clockOffset += ONE_MS_IN_M;

        if( m_FilterType == GNSS_FILTER_TYPE_RTK4 )
        {
          m_RTK.P[3][3] += 10000.0;

          for( i = 4; i < m_RTK.P.nrows(); i++ )
            m_RTK.P[i][i] += 10000.0;
        }
        else if( m_FilterType == GNSS_FILTER_TYPE_RTK8 )
        {
          m_RTK.P[6][6] += 10000.0;
        }
        else if( m_FilterType == GNSS_FILTER_TYPE_EKF )
        {
          m_EKF.P[6][6] += 10000.0;
        }
      }
      else if( rxBaseData->m_clockJumpDetected ) 
      {
        // A large arbitrary clock jump was detected on the base station receiver.
        // compensate for it as best as possible.
        rxData->m_pvt.clockOffset -= rxBaseData->m_clockJump;

        if( m_FilterType == GNSS_FILTER_TYPE_RTK4 )
        {
          m_RTK.P[3][3] += 10000.0;

          for( i = 4; i < m_RTK.P.nrows(); i++ )
            m_RTK.P[i][i] += 10000.0;
        }
        else if( m_FilterType == GNSS_FILTER_TYPE_RTK8 )
        {
          m_RTK.P[6][6] += 10000.0;
        }
        else if( m_FilterType == GNSS_FILTER_TYPE_EKF )
        {
          m_EKF.P[6][6] += 10000.0;
        }
      }
    }

    return true;
  }


  bool GNSS_Estimator::PerformLeastSquares(
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
    unsigned nrD_base = 0;   // The number of valid Doppler measurements (base station).    
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
    double stdev = 0.0; // A temporary double for getting standard deviation values.
    double speed = 0.0; // The speed estimate [m/s].
    
    Matrix Ht_p;     // The position design matrix transposed                       [4  x nP].
    Matrix HtW_p;    // An intermediate result                                      [4  x nP].
    Matrix Ht_v;     // The velocity design matrix transposed                       [4  x nD].
    Matrix HtW_v;    // An intermediate result                                      [4  x nD].
    //Matrix r_p;      // The psr residuals vector,                                   [nP x  1].
    //Matrix r_v;      // The Doppler residuals vector,                               [nD x  1].

    double avf_p = 0; // The a-posteriori variance factor for the position solution.
    double avf_v = 0; // The a-posteriori variance factor for the velocity solution.

    bool isGlobalTestPassed_p = false; // This indicates if the Global Reliability Test passed for the position solution.
    bool isGlobalTestPassed_v = false; // This indicates if the Global Reliability Test passed for the velocity solution.

    bool hasRejectionOccurred_p = false; // This indicates if a pseudorange measurement was rejected.
    bool hasRejectionOccurred_v = false; // This indicates if a Doppler measurement was rejected.

    unsigned char indexOfRejected = 0; // The index into rxData.m_ObsArray of a rejected measurement.
    
    bool result = false;

    if( rxData == NULL )
    {
      GNSS_ERROR_MSG( "rxData == NULL" );
      return false;
    }

    // Compensate the clock offset clock jumps to reduce iterations.
    if( !DealWithClockJumps( rxData, rxBaseData ) )    
    {
      GNSS_ERROR_MSG( "DealWithClockJumps returned false" );
      return false;    
    }

    if( !rxData->CheckForCycleSlips_UsingPhaseRatePrediction( GNSS_CYCLESLIP_THREADHOLD ) )
    {
      GNSS_ERROR_MSG( "CheckForCycleSlips_UsingPhaseRatePrediction returned false" );
      return false;    
    }
    if( rxBaseData )
    {
      if( !rxBaseData->CheckForCycleSlips_UsingPhaseRatePrediction( GNSS_CYCLESLIP_THREADHOLD ) )
      {
        GNSS_ERROR_MSG( "CheckForCycleSlips_UsingPhaseRatePrediction returned false" );
        return false;    
      }
    }
   
    if( m_FilterType == GNSS_FILTER_TYPE_LSQ )
    {
      // Store the current input pvt as the previous pvt since we are updating.
      rxData->m_prev_pvt = rxData->m_pvt;
    }

    lat       = rxData->m_pvt_lsq.latitude;
    lon       = rxData->m_pvt_lsq.longitude;
    hgt       = rxData->m_pvt_lsq.height;
    clk       = rxData->m_pvt_lsq.clockOffset;

    vn        = rxData->m_pvt_lsq.vn;
    ve        = rxData->m_pvt_lsq.ve;
    vup       = rxData->m_pvt_lsq.vup;
    clkdrift  = rxData->m_pvt_lsq.clockDrift;

    wasPositionComputed = false;
    wasVelocityComputed = false;
        
    // Perform very basic uniqueness check.
    n = rxData->m_nrGPSL1Obs;
    if( rxData->m_pvt_lsq.isPositionConstrained )
    {
      n += 3;
    }
    else if( rxData->m_pvt_lsq.isHeightConstrained )
    {
      n += 1;
    }
    if( n < 4 )    
    {
      return true;
    }

    // Update the receiver time information (UTC and day of year)
    result = UpdateTime( rxData->m_pvt_lsq );
    if( !result )
    {
      GNSS_ERROR_MSG( "UpdateTime returned false." );
      return false;    
    }
    if( rxBaseData != NULL )
    {
      result = UpdateTime( rxBaseData->m_pvt_lsq );
      if( !result )
      {
        GNSS_ERROR_MSG( "UpdateTime returned false." );
        return false;    
      }
    }

    result = DetermineSatellitePVT_GPSL1( rxData, rxBaseData, nrValidEph, true );
    if( !result )
    {
      GNSS_ERROR_MSG( "DetermineSatellitePVT_GPSL1 returned false." );
      return false;
    }

    result = DetermineAtmosphericCorrections_GPSL1( *rxData, true );
    if( !result )
    {
      GNSS_ERROR_MSG( "DetermineAtmosphericCorrections_GPSL1 returned false." );
      return false;
    }
    if( rxBaseData != NULL )
    {
      result = DetermineAtmosphericCorrections_GPSL1( *rxBaseData, false );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineAtmosphericCorrections_GPSL1 returned false." );
        return false;
      }
    }
    
    // Check uniqueness
    n = nrValidEph;
    if( rxData->m_pvt_lsq.isPositionConstrained )
    {
      n += 3;
    }
    else if( rxData->m_pvt_lsq.isHeightConstrained )
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
      if( !DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1(
        *rxData, 
        rxData->m_pvt_lsq.nrPsrObsUsed, 
        rxData->m_pvt_lsq.nrPsrObsAvailable, 
        rxData->m_pvt_lsq.nrPsrObsRejected ) )
      {
        GNSS_ERROR_MSG( "DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1 returned false." );
        return false;
      }
      nrP = rxData->m_pvt_lsq.nrPsrObsUsed;

      // Check uniqueness
      n = nrP;
      if( rxData->m_pvt_lsq.isPositionConstrained )
      {
        n += 3;
      }
      else if( rxData->m_pvt_lsq.isHeightConstrained )
      {
        n += 1;
      }
      if( n < 4 )    
      {
        return true;
      }
  
      if( rxBaseData != NULL )
      {
        if( !DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1(
          *rxBaseData, 
          rxBaseData->m_pvt.nrPsrObsUsed, 
          rxBaseData->m_pvt.nrPsrObsAvailable, 
          rxBaseData->m_pvt.nrPsrObsRejected ) )
        {
          GNSS_ERROR_MSG( "DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1 returned false." );
          return false;
        }
        nrP_base = rxBaseData->m_pvt.nrPsrObsUsed;

        result = DetermineBetweenReceiverDifferentialIndex(
          rxData,
          rxBaseData,
          true
          );
        if( !result )
        {
          GNSS_ERROR_MSG( "DetermineBetweenReceiverDifferentialIndex returned false" );
          return false;
        }

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

        // Check uniqueness
        nrP = nrDifferentialPsr;
        n = nrP;
        if( rxData->m_pvt_lsq.isPositionConstrained )
        {
          n += 3;
        }
        else if( rxData->m_pvt_lsq.isHeightConstrained )
        {
          n += 1;
        }
        if( n < 4 )    
        {
          return true;
        }
      }
  
#ifdef GDM_UWB_RANGE_HACK
      if( rxData->m_UWB.isValidForThisEpoch )
        n++;
#endif

      // Form H from the pseudoranges.
      if( !m_posLSQ.H.Resize( n, 4 ) )
      {
        GNSS_ERROR_MSG( "if( !m_posLSQ.H.Resize( n, 4 ) )" );
        return false;
      }
      result = DetermineDesignMatrixElements_GPSL1_Psr( *rxData, true );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineDesignMatrixElements_GPSL1_Psr returned false." );
        return false;
      }
      j = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isActive && rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
        {
          m_posLSQ.H[j][0] = rxData->m_ObsArray[i].H_p[0];
          m_posLSQ.H[j][1] = rxData->m_ObsArray[i].H_p[1];
          m_posLSQ.H[j][2] = rxData->m_ObsArray[i].H_p[2];
          if( rxData->m_ObsArray[i].system == GNSS_UWBSystem )
            m_posLSQ.H[j][3] = 0.0; // no clock with UWB ranges
          else
            m_posLSQ.H[j][3] = 1.0;
          j++;          
        }
      }    
      // Add constraints to H if any.
      if( rxData->m_pvt_lsq.isPositionConstrained )
      {
        m_posLSQ.H[j][0] = 1.0; // latitude constraint
        j++;
        m_posLSQ.H[j][1] = 1.0; // longitude constraint
        j++;
        m_posLSQ.H[j][2] = 1.0; // height constraint
        j++;
      }
      else if( rxData->m_pvt_lsq.isClockConstrained )
      {
        m_posLSQ.H[j][2] = 1.0;
        j++;
      }
      PrintMatToDebug( "LSQ Position H", m_posLSQ.H, 3 );
      

      // Form w from the pseudoranges.
      if( !m_posLSQ.w.Resize( n ) )
      {
        GNSS_ERROR_MSG( "if( !m_posLSQ.w.Resize( n ) )" );
        return false;      
      }
      result = DeterminePseudorangeMisclosures_GPSL1( rxData, rxBaseData, true );
      if( !result )      
      {
        GNSS_ERROR_MSG( "DeterminePseudorangeMisclosures_GPSL1 returned false." );
        return false;
      }
      j = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isActive && rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
        {
          m_posLSQ.w[j] = rxData->m_ObsArray[i].psr_misclosure_lsq;
          j++;
        }
      }            
      // Add constraints to w if any.      
      if( rxData->m_pvt_lsq.isPositionConstrained )
      {
        double w_lat = 0.0;
        double w_lon = 0.0;
        double w_hgt = 0.0;
        result = DeterminePositionConstraintMisclosures( rxData, w_lat, w_lon, w_hgt );
        if( !result )
        {
          GNSS_ERROR_MSG( "DeterminePositionConstraintMisclosures returned false." );
          return false;
        }
        m_posLSQ.w[j] = w_lat;
        j++;
        m_posLSQ.w[j] = w_lon;
        j++;
        m_posLSQ.w[j] = w_hgt;
        j++;
      }
      else if( rxData->m_pvt_lsq.isHeightConstrained )
      {
        double w_hgt = 0.0;       
        result = DetermineHeightConstraintMisclosures( rxData, w_hgt );
        if( !result )
        {
          GNSS_ERROR_MSG( "DetermineHeightConstraintMisclosures returned false." );
          return false;
        }
        m_posLSQ.w[j] = w_hgt;
        j++;
      }
      PrintMatToDebug( "LSQ pseudorange misclosures", m_posLSQ.w, 3 );
      
      // Form R, the combined measurement variance-covariance matrix.
      if( !m_posLSQ.R.Resize( n, n ) )
      {
        GNSS_ERROR_MSG( "if( !m_posLSQ.R.Resize( n, n ) )" );
        return false;
      }
      j = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add pseudoranges
      {
        if( rxData->m_ObsArray[i].flags.isActive &&
          rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
        {
          m_posLSQ.R[j][j] = rxData->m_ObsArray[i].stdev_psr*rxData->m_ObsArray[i].stdev_psr;
          j++;
        }
      }
      // Deal with constraints.
      if( rxData->m_pvt_lsq.isPositionConstrained )
      {
        m_posLSQ.R[j][j] = rxData->m_pvt_lsq.std_lat*rxData->m_pvt_lsq.std_lat; 
        j++;
        m_posLSQ.R[j][j] = rxData->m_pvt_lsq.std_lon*rxData->m_pvt_lsq.std_lon; 
        j++;
        m_posLSQ.R[j][j] = rxData->m_pvt_lsq.std_hgt*rxData->m_pvt_lsq.std_hgt; 
        j++; 
      }
      else if( rxData->m_pvt_lsq.isHeightConstrained )
      {
        m_posLSQ.R[j][j] = rxData->m_pvt_lsq.std_hgt*rxData->m_pvt_lsq.std_hgt; 
        j++;
       }
      PrintMatToDebug( "LSQ Position R", m_posLSQ.R, 2 );


      if( !m_posLSQ.W.Resize( n, n ) )
      {
        GNSS_ERROR_MSG( "if( !m_posLSQ.W.Resize( n, n ) )" );
        return false;
      }
      for( i = 0; i < n; i++ )
      {
        if( m_posLSQ.R[i][i] == 0.0 )
        {
          GNSS_ERROR_MSG( "m_posLSQ.R[i][i] == 0.0" );
          return false;
        }
        m_posLSQ.W[i][i] = 1.0/m_posLSQ.R[i][i];
      }
      PrintMatToDebug( "LSQ Position W", m_posLSQ.W, 3 );

      
      // Compute Ht_p.
      Ht_p = m_posLSQ.H;
      if( !Ht_p.Inplace_Transpose() )
      {
        GNSS_ERROR_MSG( "if( !Ht_p.Inplace_Transpose() )" );
        return false;
      }

      // Compute HtW_p.
      if( !HtW_p.Multiply( Ht_p, m_posLSQ.W ) )
      {
        GNSS_ERROR_MSG( "if( !HtW_p.Multiply( Ht_p, m_posLSQ.W ) )" );
        return false;
      }

      // Compute P_p.
      if( !m_posLSQ.P.Multiply( HtW_p, m_posLSQ.H ) )
      {
        GNSS_ERROR_MSG( "if( !m_posLSQ.P.Multiply( HtW_p, m_posLSQ.H ) )" );
        return false;
      }
      if( !m_posLSQ.P.Inplace_Invert() )
      {
        GNSS_ERROR_MSG( "if( !m_posLSQ.P.Inplace_Invert() )" );
        return false;
      }
      PrintMatToDebug( "LSQ Position P", m_posLSQ.P, 3 );

      // Compute dx_p.
      if( !m_posLSQ.dx.Multiply( m_posLSQ.P, HtW_p ) )
      {
        GNSS_ERROR_MSG( "if( !m_posLSQ.dx.Multiply( m_posLSQ.P, HtW_p ) )" );
        return false;
      }
      if( !m_posLSQ.dx.Inplace_PostMultiply( m_posLSQ.w ) )
      {
        GNSS_ERROR_MSG( "if( !m_posLSQ.dx.Inplace_PostMultiply( m_posLSQ.w ) )" );
        return false;
      }

      // Update the position and clock states
      // Update height first as it is need to reduce the corrections for lat and lon.
      hgt += m_posLSQ.dx[2];
      clk += m_posLSQ.dx[3];
     
      // The corrections for lat and lon, dx_p, must be converted to [rad] from [m].
      GEODESY_ComputePrimeVerticalRadiusOfCurvature(
        GEODESY_REFERENCE_ELLIPSE_WGS84, 
        rxData->m_pvt_lsq.latitude,
        &N );
      GEODESY_ComputeMeridianRadiusOfCurvature(
        GEODESY_REFERENCE_ELLIPSE_WGS84, 
        rxData->m_pvt_lsq.latitude,
        &M );

      lat += m_posLSQ.dx[0] / ( M + hgt );             // convert from meters to radians.
      lon += m_posLSQ.dx[1] / (( N + hgt )*cos(lat));  // convert from meters to radians.

      result = rxData->UpdatePositionAndRxClock(
        rxData->m_pvt_lsq,
        lat,
        lon,
        hgt,
        clk,
        sqrt(m_posLSQ.P[0][0]),
        sqrt(m_posLSQ.P[1][1]),
        sqrt(m_posLSQ.P[2][2]),
        sqrt(m_posLSQ.P[3][3])
        );
      if( !result )
      {
        GNSS_ERROR_MSG( "UpdatePositionAndRxClock returned false." );
        return false;
      }

      dtmp1 = fabs(m_posLSQ.dx[0]) + fabs(m_posLSQ.dx[1]) + fabs(m_posLSQ.dx[2]) + fabs(m_posLSQ.dx[3]);
      if( dtmp1 < 0.0001 )
      {
        if( !rxData->m_pvt_lsq.isPositionConstrained )
        {
          // Test the residuals
          result = PerformGlobalTestAndTestForMeasurementFaults( 
            *rxData,
            true,
            m_posLSQ.H,
            Ht_p,      
            m_posLSQ.W,
            m_posLSQ.R,
            m_posLSQ.w, // misclosures at convergence are the residuals.
            m_posLSQ.P,
            n,
            4,
            avf_p,      
            isGlobalTestPassed_p,
            hasRejectionOccurred_p,
            indexOfRejected
            );
          if( !result )
          {
            GNSS_ERROR_MSG( "PerformGlobalTestAndTestForMeasurementFaults returned false." );
            return false;
          }

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

    // Compute the DOP values.
    if( !ComputeDOP( rxData, true ) )
    {
      GNSS_ERROR_MSG( "ComputeDOP returned false." );
      return false;
    }

    Matrix wtWw;
    m_posLSQ.n = n;
    m_posLSQ.u = 4;
    if( m_posLSQ.n > m_posLSQ.u )
    {
      wtWw = m_posLSQ.w;
      if( !wtWw.Inplace_transpose() )
      {
        GNSS_ERROR_MSG( "if( !wtWw.Inplace_transpose() )" );
        return false;
      }
      if( !wtWw.Inplace_PostMultiply(m_posLSQ.W) )
      {
        GNSS_ERROR_MSG( "if( !wtWw.Inplace_PostMultiply(m_posLSQ.W) )" );
        return false;
      }
      if( !wtWw.Inplace_PostMultiply(m_posLSQ.w) )
      {
        GNSS_ERROR_MSG( "if( !wtWw.Inplace_PostMultiply(m_posLSQ.w) )" );
        return false;
      }
      m_posLSQ.apvf = wtWw[0]/((double)m_posLSQ.n-m_posLSQ.u);
      m_posLSQ.sqrt_apvf = sqrt(m_posLSQ.apvf);

      m_posLSQ.w.GetStats_RMS( m_posLSQ.rms_residual );    
    }
    else
    {
      m_posLSQ.apvf = 0.0;
      m_posLSQ.sqrt_apvf = 0.0;
      m_posLSQ.rms_residual = 0;
    }
    rxData->m_pvt_lsq.pos_apvf = m_posLSQ.apvf;


    if( m_FilterType == GNSS_FILTER_TYPE_LSQ )
    {
      // If the filter type is least squares, store the least squares solution in the main pvt struct.
      // This copy may occur twice if the velocity solution is valid. It is needed here to ensure
      // the position solution gets copied.
      rxData->m_pvt = rxData->m_pvt_lsq;
    }

    ////
    // Velocity

    for( iter = 0; iter < 7; iter++ )
    {
      result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( 
        *rxData, 
        rxData->m_pvt_lsq.nrDopplerObsUsed,
        rxData->m_pvt_lsq.nrDopplerObsAvailable,
        rxData->m_pvt_lsq.nrDopplerObsRejected
        );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1 returned false." );
        return false;
      }
      nrD = rxData->m_pvt_lsq.nrDopplerObsUsed;

      // Check uniqueness
      n = nrD;
      if( rxData->m_pvt_lsq.isPositionConstrained )
      {
        n += 3;
      }
      else if( rxData->m_pvt_lsq.isHeightConstrained )
      {
        n += 1;
      }
      if( n < 4 )    
      {
        return true;
      }

      
      if( rxBaseData != NULL )
      {
        result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( 
          *rxBaseData, 
          rxBaseData->m_pvt.nrDopplerObsUsed,
          rxBaseData->m_pvt.nrDopplerObsAvailable,
          rxBaseData->m_pvt.nrDopplerObsRejected
          );
        if( !result )
        {
          GNSS_ERROR_MSG( "DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1 returned false." );
          return false;
        }
        nrD_base = rxBaseData->m_pvt.nrDopplerObsUsed;

        result = DetermineBetweenReceiverDifferentialIndex( rxData, rxBaseData, true );
        if( !result )
        {
          GNSS_ERROR_MSG( "DetermineBetweenReceiverDifferentialIndex returned false." );
          return false;
        }

        nrDifferentialDoppler = 0;
        for( i = 0; i < rxData->m_nrValidObs; i++ )
        {
          if( rxData->m_ObsArray[i].flags.isActive &&            
            rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
          {
            nrDifferentialDoppler++;
          }
        }

        // Check uniqueness.
        nrD = nrDifferentialDoppler;
        n = nrD;
        if( rxData->m_pvt_lsq.isPositionConstrained )
        {
          n += 3;
        }
        else if( rxData->m_pvt_lsq.isHeightConstrained )
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
        if( rxData->m_ObsArray[i].flags.isActive &&
          rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
        {
          GPS_ComputeUserToSatelliteRangeAndRangeRate(
            rxData->m_pvt_lsq.x,
            rxData->m_pvt_lsq.y,
            rxData->m_pvt_lsq.z,
            rxData->m_pvt_lsq.vx,
            rxData->m_pvt_lsq.vy,
            rxData->m_pvt_lsq.vz,
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

      // Form the design matrix for the velocity solution.
      if( !m_velLSQ.H.Resize( n, 4 ) )
      {
        GNSS_ERROR_MSG( "if( !m_velLSQ.H.Resize( n, 4 ) )" );
        return false;
      }
      result = DetermineDesignMatrixElements_GPSL1_Doppler( *rxData, true );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineDesignMatrixElements_GPSL1_Doppler returned false." );
        return false;
      }
      j = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isActive &&
          rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )        
        {
          m_velLSQ.H[j][0] = rxData->m_ObsArray[i].H_v[0];
          m_velLSQ.H[j][1] = rxData->m_ObsArray[i].H_v[1];
          m_velLSQ.H[j][2] = rxData->m_ObsArray[i].H_v[2];
          m_velLSQ.H[j][3] = 1.0;
          j++;
        }
      }
      if( rxData->m_pvt_lsq.isPositionConstrained )
      {
        m_velLSQ.H[j][0] = 1.0;
        j++;
        m_velLSQ.H[j][1] = 1.0;
        j++;
        m_velLSQ.H[j][2] = 1.0;
        j++;
      }
      else if ( rxData->m_pvt_lsq.isHeightConstrained )
      {
        m_velLSQ.H[j][2] = 1.0;
        j++;
      }
      PrintMatToDebug( "LSQ Velocity H", m_velLSQ.H, 3 );
     
      // Form R, the combined measurement variance-covariance matrix.
      if( !m_velLSQ.R.Resize( n, n ) )
      {
        GNSS_ERROR_MSG( "if( !m_velLSQ.R.Resize( n, n ) )" );
        return false;
      }
      j = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add Dopplers
      {
        if( rxData->m_ObsArray[i].flags.isActive &&
          rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
        {
          stdev = rxData->m_ObsArray[i].stdev_doppler * GPS_WAVELENGTHL1; // Change from cycles/s to meters/s.
          m_velLSQ.R[j][j] = stdev*stdev;
          j++;
        }
      }
      // Deal with constraints.
      if( rxData->m_pvt_lsq.isPositionConstrained )
      {
        m_velLSQ.R[j][j] = 1.0e-10; 
        j++;
        m_velLSQ.R[j][j] = 1.0e-10; 
        j++;
        m_velLSQ.R[j][j] = 1.0e-10;      
        j++;      
      }
      else if( rxData->m_pvt_lsq.isHeightConstrained )
      {
        m_velLSQ.R[j][j] = 1.0e-10;      
        j++;   
      }
      PrintMatToDebug( "LSQ Velocity R", m_velLSQ.R, 3 );

      if( !m_velLSQ.W.Resize( n, n ) )
      {
        GNSS_ERROR_MSG( "if( !m_velLSQ.W.Resize( n, n ) )" );
        return false;
      }
      for( i = 0; i < n; i++ )
      {
        if( m_velLSQ.R[i][i] == 0.0 )
        {
          GNSS_ERROR_MSG( "m_velLSQ.R[i][i] == 0.0" );
          return false;
        }
        m_velLSQ.W[i][i] = 1.0/m_velLSQ.R[i][i];
      }

      // Form the misclosure vector for the velocity solution.
      if( !m_velLSQ.w.Resize( n ) )
      {
        GNSS_ERROR_MSG( "if( !m_velLSQ.w.Resize( n ) )" );
        return false;
      }
      result = DetermineDopplerMisclosures_GPSL1( rxData, rxBaseData, true );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineDopplerMisclosures_GPSL1 returned false." );
        return false;
      }
      j = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isActive &&
          rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )          
        {
          m_velLSQ.w[j] = rxData->m_ObsArray[i].doppler_misclosure;
          j++;
        }
      }
      if( rxData->m_pvt_lsq.isPositionConstrained )
      {
        m_velLSQ.w[j] = 0.0;
        j++;
        m_velLSQ.w[j] = 0.0;
        j++;
        m_velLSQ.w[j] = 0.0;
        j++;
      }
      else if ( rxData->m_pvt_lsq.isHeightConstrained )
      {
        m_velLSQ.w[j] = 0.0;
        j++;
      }            
      PrintMatToDebug( "LSQ Velocity w", m_velLSQ.w, 3 );

      // Compute Ht_v.
      Ht_v = m_velLSQ.H;
      if( !Ht_v.Inplace_Transpose() )
      {
        GNSS_ERROR_MSG( "if( !Ht_v.Inplace_Transpose() )" );
        return false;
      }

      // Compute HtW_v.
      if( !HtW_v.Multiply( Ht_v, m_velLSQ.W ) )
      {
        GNSS_ERROR_MSG( "if( !HtW_v.Multiply( Ht_v, m_velLSQ.W ) )" );
        return false;
      }

      // Compute P_v.
      if( !m_velLSQ.P.Multiply( HtW_v, m_velLSQ.H ) )
      {
        GNSS_ERROR_MSG( "if( !m_velLSQ.P.Multiply( HtW_v, m_velLSQ.H ) )" );
        return false;
      }
      if( !m_velLSQ.P.Inplace_Invert() )
      {
        GNSS_ERROR_MSG( "if( !m_velLSQ.P.Inplace_Invert() )" );
        return false;
      }

      // Compute dx_v.
      if( !m_velLSQ.dx.Multiply( m_velLSQ.P, HtW_v ) )
      {
        GNSS_ERROR_MSG( "if( !m_velLSQ.dx.Multiply( m_velLSQ.P, HtW_v ) )" );
        return false;
      }
      if( !m_velLSQ.dx.Inplace_PostMultiply( m_velLSQ.w ) )
      {
        GNSS_ERROR_MSG( "if( !m_velLSQ.dx.Inplace_PostMultiply( m_velLSQ.w ) )" );
        return false;
      }

      // Update the velocity and clock drift states.
      vn        += m_velLSQ.dx[0];
      ve        += m_velLSQ.dx[1];
      vup       += m_velLSQ.dx[2];
      clkdrift  += m_velLSQ.dx[3];

      result = rxData->UpdateVelocityAndClockDrift(
        rxData->m_pvt_lsq,
        vn,
        ve,
        vup,
        clkdrift,
        m_velLSQ.P[0][0],
        m_velLSQ.P[1][1],
        m_velLSQ.P[2][2],
        m_velLSQ.P[3][3] );
      if( !result )
      {
        GNSS_ERROR_MSG( "rxData->UpdateVelocityAndClockDrift returned false." );
        return false;
      }

      dtmp1 = fabs(m_velLSQ.dx[0]) + fabs(m_velLSQ.dx[1]) + fabs(m_velLSQ.dx[2]) + fabs(m_velLSQ.dx[3]);
      if( dtmp1 < 1.0e-10 )
      {
        if( !rxData->m_pvt_lsq.isPositionConstrained )
        {
          // Test the residuals
          result = PerformGlobalTestAndTestForMeasurementFaults( 
            *rxData,
            false,
            m_velLSQ.H,
            Ht_v,      
            m_velLSQ.W,
            m_velLSQ.R,
            m_velLSQ.w, // misclosures at convergence are the residuals.
            m_velLSQ.P,
            n,
            4,
            avf_v,      
            isGlobalTestPassed_v,
            hasRejectionOccurred_v,
            indexOfRejected
            );
          if( !result )
          {
            GNSS_ERROR_MSG( "PerformGlobalTestAndTestForMeasurementFaults returned false." );
            return false;
          }

          if( hasRejectionOccurred_v )
          {
            hasRejectionOccurred_v = false;
            j = 0; // restart the iterative loop.
            continue;
          }
          else
          {
            // converged to solution.
            break;
          }
        }
      }
    }
    wasVelocityComputed = true;

    // Store the residuals (misclosures at the end of the iteration process) specific for least squares.
    // This is done if LSQ is running parallel to another filter and the LSQ residuals are of interest.
    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( rxData->m_ObsArray[i].flags.isActive )
      {
        rxData->m_ObsArray[i].doppler_misclosure_lsq = rxData->m_ObsArray[i].doppler_misclosure;        
      }
    }
    
    speed = sqrt( vn*vn + ve*ve + vup*vup );
    if( speed > 515.0 ) // COCOM limts
    {
      GNSS_ERROR_MSG( "Speed exceeded 515 m/s." );
    }      

    // Compute the a-posteriori variance factor.
    m_velLSQ.n = n;
    m_velLSQ.u = 4;
    if( m_velLSQ.n > m_velLSQ.u )
    {
      wtWw = m_velLSQ.w;
      if( !wtWw.Inplace_transpose() )
      {
        GNSS_ERROR_MSG( "if( !wtWw.Inplace_transpose() )" );
        return false;
      }
      if( !wtWw.Inplace_PostMultiply(m_velLSQ.W) )
      {
        GNSS_ERROR_MSG( "if( !wtWw.Inplace_PostMultiply(m_velLSQ.W) )" );
        return false;
      }
      if( !wtWw.Inplace_PostMultiply(m_velLSQ.w) )
      {
        GNSS_ERROR_MSG( "if( !wtWw.Inplace_PostMultiply(m_velLSQ.w) )" );
        return false;
      }
      m_velLSQ.apvf = wtWw[0]/((double)m_velLSQ.n-m_velLSQ.u);
      m_velLSQ.sqrt_apvf = sqrt(m_velLSQ.apvf);

      m_velLSQ.w.GetStats_RMS( m_velLSQ.rms_residual );
    }
    else
    {
      m_velLSQ.apvf = 0.0;
      m_velLSQ.sqrt_apvf = 0.0;
      m_velLSQ.rms_residual = 0.0;
    }
    rxData->m_pvt_lsq.vel_apvf = m_velLSQ.apvf;

    if( m_FilterType == GNSS_FILTER_TYPE_LSQ )
    {
      // If the filter type is least squares, store the least squares solution in the main pvt struct.
      rxData->m_pvt = rxData->m_pvt_lsq;
    }
    
    return true;
  }


  bool GNSS_Estimator::UpdateTime(
    GNSS_structPVT& pvt // The position, velocity, and time information struct.
    )
  {
    BOOL result;
    result = TIMECONV_GetUTCTimeFromGPSTime(
      pvt.time.gps_week,
      pvt.time.gps_tow,
      &pvt.time.utc_year,
      &pvt.time.utc_month,
      &pvt.time.utc_day,
      &pvt.time.utc_hour,
      &pvt.time.utc_minute,
      &pvt.time.utc_seconds
      );
    if( result == FALSE )
    {
      GNSS_ERROR_MSG( "TIMECONV_GetUTCTimeFromGPSTime returned FALSE." );
      return false;
    }

    result = TIMECONV_GetDayOfYear(
      pvt.time.utc_year,
      pvt.time.utc_month,
      pvt.time.utc_day,
      &pvt.time.day_of_year );
    if( result == FALSE )
    {
      GNSS_ERROR_MSG( "TIMECONV_GetDayOfYear returned FALSE." );
      return false;
    }
      
    return true;
  }
    

  bool GNSS_Estimator::DetermineSatellitePVT_GPSL1( 
    GNSS_RxData *rxData,       //!< The pointer to the receiver data.    
    GNSS_RxData *rxBaseData,   //!< The pointer to the reference receiver data. NULL if not available.
    unsigned &nrValidEph,      //!< The number of GPS L1 channels with valid ephemeris for the rover.
    const bool isLeastSquares  //!< A boolean to indicate if the rover position and velocity values are from least squares rxData->m_pvt_lsq or from rxData->m_pvt.
    )
  {
    unsigned i = 0;
    double psr = 0;
    double dtmp1 = 0;
    double dtmp2 = 0;
    bool isEphAvailable = false;
    GPS_structEphemeris eph;
    double x = 0;   // The rover receiver positon ECEF.
    double y = 0;   // The rover receiver positon ECEF.
    double z = 0;   // The rover receiver positon ECEF.
    double vx = 0;  // The rover receiver velocity ECEF.
    double vy = 0;  // The rover receiver velocity ECEF.
    double vz = 0;  // The rover receiver velocity ECEF.
    
    memset( &eph, 0, sizeof(eph) );

    if( rxData == NULL )
    {
      GNSS_ERROR_MSG( "rxData == NULL" );
      return false;
    }

    if( isLeastSquares )
    {
      x = rxData->m_pvt_lsq.x;
      y = rxData->m_pvt_lsq.y;
      z = rxData->m_pvt_lsq.z;
      vx = rxData->m_pvt_lsq.vx;
      vy = rxData->m_pvt_lsq.vy;
      vz = rxData->m_pvt_lsq.vz;
    }
    else
    {
      x = rxData->m_pvt.x;
      y = rxData->m_pvt.y;
      z = rxData->m_pvt.z;
      vx = rxData->m_pvt.vx;
      vy = rxData->m_pvt.vy;
      vz = rxData->m_pvt.vz;
    }

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
            {
              GNSS_ERROR_MSG( "rxBaseData->m_EphAlmArray.GetEphemeris returned false." );
              return false;
            }
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

    // Evaluate the satellite PVT for all channels with GPS L1 observations       
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
            {
              GNSS_ERROR_MSG( "rxBaseData->m_EphAlmArray.GetEphemeris returned false." );
              return false;
            }
            if( !isEphAvailable )
            {
              // Get the ephemeris using the rover station then.
              if( !rxData->m_EphAlmArray.GetEphemeris( rxData->m_ObsArray[i].id, eph, isEphAvailable ) )
              {
                GNSS_ERROR_MSG( "rxData->m_EphAlmArray.GetEphemeris returned false." );
                return false;
              }              
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
            {
              GNSS_ERROR_MSG( "rxData->m_EphAlmArray.GetEphemeris returned false." );
              return false;
            }
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
            x,
            y,
            z,
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
            x,
            y,
            z,
            vx,
            vy,
            vz,
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
#ifdef GDM_UWB_RANGE_HACK
        else if( rxData->m_ObsArray[i].system == GNSS_UWBSystem )
        {
           // Compute the rover station geometric range and range rate.
          GPS_ComputeUserToSatelliteRangeAndRangeRate(
            x,
            y,
            z,
            vx,
            vy,
            vz,
            rxData->m_ObsArray[i].satellite.x,
            rxData->m_ObsArray[i].satellite.y,
            rxData->m_ObsArray[i].satellite.z,
            rxData->m_ObsArray[i].satellite.vx,
            rxData->m_ObsArray[i].satellite.vy,
            rxData->m_ObsArray[i].satellite.vz,
            &rxData->m_ObsArray[i].range,
            &rxData->m_ObsArray[i].rangerate );
          rxBaseData->m_ObsArray[i].rangerate = 0.0;
        }
#endif
      }
    }
    return true;
  }

  bool GNSS_Estimator::DetermineAtmosphericCorrections_GPSL1( 
    GNSS_RxData &rxData,       //!< The receiver data.
    const bool isLeastSquares  //!< A boolean to indicate if the rover position and velocity values are from least squares rxData->m_pvt_lsq or from rxData->m_pvt.)
    )
  {
    unsigned i = 0;
    double zenith_dry_delay = 0;
    double zenith_wet_delay = 0;
    double dtmp1 = 0;
    double dtmp2 = 0;
    BOOL result;

    double lat = 0;
    double lon = 0;
    double hgt = 0;
    double tow = 0;
    unsigned short day_of_year = 0;

    if( isLeastSquares )
    {
      lat = rxData.m_pvt_lsq.latitude;
      lon = rxData.m_pvt_lsq.longitude;
      hgt = rxData.m_pvt_lsq.height;
      day_of_year = rxData.m_pvt_lsq.time.day_of_year;
      tow = rxData.m_pvt_lsq.time.gps_tow;
    }
    else
    {
      lat = rxData.m_pvt.latitude;
      lon = rxData.m_pvt.longitude;
      hgt = rxData.m_pvt.height;
      day_of_year = rxData.m_pvt.time.day_of_year;
      tow = rxData.m_pvt.time.gps_tow;
    }

    // Compute the tropospheric delays.
    TROPOSPHERE_DetermineZenithDelayValues_WAAS_Model(
      lat,
      hgt,
      day_of_year,
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
              lat,              
              hgt,
              &dtmp1,
              &dtmp2
              );
            
            rxData.m_ObsArray[i].corrections.prcTropoDry = static_cast<float>(dtmp1);
            rxData.m_ObsArray[i].corrections.prcTropoWet = static_cast<float>(dtmp2);

            if( rxData.m_klobuchar.isValid )
            {
              // Always compute the ionospheric correction (it may not be applied though).
              result = IONOSPHERE_GetL1KlobucharCorrection(
                rxData.m_klobuchar.alpha0,
                rxData.m_klobuchar.alpha1,
                rxData.m_klobuchar.alpha2,
                rxData.m_klobuchar.alpha3,
                rxData.m_klobuchar.beta0,
                rxData.m_klobuchar.beta1,
                rxData.m_klobuchar.beta2,
                rxData.m_klobuchar.beta3,
                lat,
                lon, 
                rxData.m_ObsArray[i].satellite.elevation,
                rxData.m_ObsArray[i].satellite.azimuth,
                tow,
                &dtmp1
                );
              if( result == FALSE )
              {
                GNSS_ERROR_MSG( "IONOSPHERE_GetL1KlobucharCorrection returned FALSE." );
                return false;
              }
              rxData.m_ObsArray[i].corrections.prcIono = static_cast<float>(dtmp1);
            }
          }
        }
      }
    }
    return true;
  }



  bool GNSS_Estimator::DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( 
    GNSS_RxData &rxData,                 //!< The receiver data.
    unsigned char &nrUsablePseudoranges, //!< The number of usable GPS L1 pseudorange measurements.
    unsigned char &nrPsrObsAvailable,    //!< The number of psr measurements available for use.
    unsigned char &nrPsrObsRejected      //!< The number of psr measurements flagged as rejected.
    )
  {
    unsigned i = 0;
    unsigned char isGood = 0;

    nrPsrObsAvailable = 0;
    nrPsrObsRejected = 0;
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

          if( rxData.m_ObsArray[i].flags.isCodeLocked & 
            rxData.m_ObsArray[i].flags.isPsrValid )
          {
            nrPsrObsAvailable++;
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
            if( !rxData.m_ObsArray[i].flags.isNotUserRejected || !rxData.m_ObsArray[i].flags.isNotPsrRejected )
            {
              nrPsrObsRejected++;
            }            
            rxData.m_ObsArray[i].flags.isPsrUsedInSolution = 0;
          }
        }
#ifdef GDM_UWB_RANGE_HACK
        else if( rxData.m_ObsArray[i].system == GNSS_UWBSystem )
        {
          nrPsrObsAvailable++;
          nrUsablePseudoranges++;
        }
#endif
      }
    }
    return true;
  }


  bool GNSS_Estimator::DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( 
    GNSS_RxData &rxData,                   //!< The receiver data.
    unsigned char &nrUsableDopplers,       //!< The number of usable GPS L1 Doppler measurements.
    unsigned char &nrDopplerObsAvailable,  //!< The number of psr measurements available for use.
    unsigned char &nrDopplerObsRejected    //!< The number of psr measurements flagged as rejected.
    )
  {
    unsigned i = 0;
    unsigned char isGood = 0;

    nrDopplerObsAvailable = 0;
    nrDopplerObsRejected = 0;
    nrUsableDopplers = 0;    

    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( rxData.m_ObsArray[i].flags.isActive )
      {
        if( rxData.m_ObsArray[i].system == GNSS_GPS && rxData.m_ObsArray[i].freqType == GNSS_GPSL1 )
        {
          if( !rxData.m_ObsArray[i].flags.isPsrUsedInSolution )
            rxData.m_ObsArray[i].flags.isDopplerValid = false; // GDM - not optimal but safe to do this.

          if( rxData.m_ObsArray[i].flags.isCodeLocked & 
            rxData.m_ObsArray[i].flags.isDopplerValid )
          {
            nrDopplerObsAvailable++;
          }
            
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
            if( !rxData.m_ObsArray[i].flags.isNotUserRejected || !rxData.m_ObsArray[i].flags.isNotDopplerRejected )
            {
              nrDopplerObsRejected++;
            }
            rxData.m_ObsArray[i].flags.isDopplerUsedInSolution = 0;
          }
        }
      }
    }
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

          if( rxData.m_ObsArray[i].flags.isCodeLocked &
            rxData.m_ObsArray[i].flags.isPhaseLocked  &
            rxData.m_ObsArray[i].flags.isParityValid  & // if not, there is a half cycle amibiguity.
            rxData.m_ObsArray[i].flags.isAdrValid )
          {
            rxData.m_pvt.nrAdrObsAvailable++;
          }            

          isGood = 
            rxData.m_ObsArray[i].flags.isCodeLocked          &
            rxData.m_ObsArray[i].flags.isPhaseLocked         &
            rxData.m_ObsArray[i].flags.isParityValid         & // if not, there is a half cycle amibiguity.
            rxData.m_ObsArray[i].flags.isAdrValid            &
            rxData.m_ObsArray[i].flags.isAboveElevationMask  &
            rxData.m_ObsArray[i].flags.isAboveCNoMask        &
            rxData.m_ObsArray[i].flags.isAboveLockTimeMask   &
            rxData.m_ObsArray[i].flags.isNotUserRejected     &
            rxData.m_ObsArray[i].flags.isNotAdrRejected      &
            rxData.m_ObsArray[i].flags.isEphemerisValid;
          if( isGood )
          {
            // Measurements with cycle slip flags are still used. However, their variance is reset to a large value.
            rxData.m_ObsArray[i].flags.isAdrUsedInSolution = 1;
            rxData.m_pvt.nrAdrObsUsed++;            
          }
          else
          {
            if( !rxData.m_ObsArray[i].flags.isNotUserRejected || !rxData.m_ObsArray[i].flags.isNotAdrRejected )
              rxData.m_pvt.nrAdrObsRejected++;
            
            rxData.m_ObsArray[i].flags.isAdrUsedInSolution = 0;
          }
        }
      }
    }
    nrUsableAdr = rxData.m_pvt.nrAdrObsUsed;
    return true;
  }

  bool GNSS_Estimator::DetermineDesignMatrixElement_GPSL1_Psr( 
    GNSS_RxData &rxData,       //!< The receiver data.
    const unsigned int index,  //!< The index of the observation i.e. rxData.m_ObsArray[index].
    const bool isLeastSquares  //!< A boolean to indicate if the rover position and velocity values are from least squares rxData->m_pvt_lsq or from rxData->m_pvt.
    )
  {
    double lat = 0;
    double lon = 0;
    double hgt = 0;

    // Check the index.
    if( index < 0 || index >= GNSS_RXDATA_NR_CHANNELS )
    {
      GNSS_ERROR_MSG( "if( index < 0 || index >= GNSS_RXDATA_NR_CHANNELS )" );
      return false;
    }

    if( isLeastSquares )
    {
      lat = rxData.m_pvt_lsq.latitude;
      lon = rxData.m_pvt_lsq.longitude;
      hgt = rxData.m_pvt_lsq.height;
    }
    else
    {
      lat = rxData.m_pvt.latitude;
      lon = rxData.m_pvt.longitude;
      hgt = rxData.m_pvt.height;
    }

    // Compute the design matrix for the position solution.
    if( rxData.m_ObsArray[index].flags.isActive )
    {
      if( rxData.m_ObsArray[index].system == GNSS_GPS && rxData.m_ObsArray[index].freqType == GNSS_GPSL1 )
      {
        if( rxData.m_ObsArray[index].flags.isPsrUsedInSolution )
        {
          NAVIGATION_ComputeDerivativesOf_Range_WithRespectToLatitudeLongitudeHeight(
            lat,
            lon,
            hgt,
            rxData.m_ObsArray[index].satellite.x,
            rxData.m_ObsArray[index].satellite.y,
            rxData.m_ObsArray[index].satellite.z,
            &(rxData.m_ObsArray[index].H_p[0]),
            &(rxData.m_ObsArray[index].H_p[1]),
            &(rxData.m_ObsArray[index].H_p[2]),
            &rxData.m_ObsArray[index].range
            );
        }
      }
#ifdef GDM_UWB_RANGE_HACK
      else if( rxData.m_ObsArray[index].system == GNSS_UWBSystem && rxData.m_ObsArray[index].flags.isActive )
      {
        NAVIGATION_ComputeDerivativesOf_Range_WithRespectToLatitudeLongitudeHeight(
          lat,
          lon,
          hgt,
          rxData.m_ObsArray[index].satellite.x,
          rxData.m_ObsArray[index].satellite.y,
          rxData.m_ObsArray[index].satellite.z,
          &(rxData.m_ObsArray[index].H_p[0]),
          &(rxData.m_ObsArray[index].H_p[1]),
          &(rxData.m_ObsArray[index].H_p[2]),
          &rxData.m_ObsArray[index].range
          );
        rxData.m_ObsArray[index].flags.isPsrUsedInSolution = true;
      }
#endif      
    }
    return true;
  }


  bool GNSS_Estimator::DetermineDesignMatrixElement_GPSL1_Adr( GNSS_RxData &rxData, const unsigned int index )
  {
    // Check the index.
    if( index < 0 || index >= GNSS_RXDATA_NR_CHANNELS )
    {
      GNSS_ERROR_MSG( "if( index < 0 || index >= GNSS_RXDATA_NR_CHANNELS )" );
      return false;
    }

    // Compute the design matrix for the position solution.
    if( rxData.m_ObsArray[index].flags.isActive )
    {
      if( rxData.m_ObsArray[index].system == GNSS_GPS && rxData.m_ObsArray[index].freqType == GNSS_GPSL1 )
      {
        if( rxData.m_ObsArray[index].flags.isAdrUsedInSolution )
        {
          NAVIGATION_ComputeDerivativesOf_Range_WithRespectToLatitudeLongitudeHeight(
            rxData.m_pvt.latitude,
            rxData.m_pvt.longitude, 
            rxData.m_pvt.height,
            rxData.m_ObsArray[index].satellite.x,
            rxData.m_ObsArray[index].satellite.y,
            rxData.m_ObsArray[index].satellite.z,
            &(rxData.m_ObsArray[index].H_a[0]),
            &(rxData.m_ObsArray[index].H_a[1]),
            &(rxData.m_ObsArray[index].H_a[2]),
            &rxData.m_ObsArray[index].range
            );
        }
      }
    }
    return true;
  }


  bool GNSS_Estimator::DetermineDesignMatrixElements_GPSL1_Psr( 
    GNSS_RxData &rxData,       //!< The receiver data.
    const bool isLeastSquares  //!< A boolean to indicate if the rover position and velocity values are from least squares rxData->m_pvt_lsq or from rxData->m_pvt.)
    )
  {
    unsigned i = 0;

    // Compute the design matrix for the position solution.
    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( DetermineDesignMatrixElement_GPSL1_Psr( rxData, i, isLeastSquares ) == false )
      {
        GNSS_ERROR_MSG( "DetermineDesignMatrixElement_GPSL1_Psr returned false." );
        return false;
      }
    }
    return true;
  }


  bool GNSS_Estimator::DeterminePseudorangeMisclosure_GPSL1( 
    GNSS_RxData *rxData,       //!< The pointer to the receiver data.    
    const unsigned int index,  //!< The index of the observation in the receiver data.
    GNSS_RxData *rxBaseData,   //!< The pointer to the reference receiver data. NULL if not available.    
    const bool isLeastSquares  //!< A boolean to indicate if the rover position and velocity values are from least squares rxData->m_pvt_lsq or from rxData->m_pvt.
    )
  {
    int k = 0;
    double psr_base = 0;
    double range_base = 0;
    double psr_measured = 0;
    double psr_computed = 0;
    double clock_offset = 0;

    // Check the index.
    if( index < 0 || index >= GNSS_RXDATA_NR_CHANNELS )
    {
      GNSS_ERROR_MSG( "if( index < 0 || index >= GNSS_RXDATA_NR_CHANNELS )" );
      return false;
    }
      
    // Check the pointers.
    if( rxData == NULL )
    {
      GNSS_ERROR_MSG( "if( rxData == NULL )" );
      return false;
    }

    if( rxData->m_ObsArray[index].flags.isActive )
    {
      if( rxData->m_ObsArray[index].system == GNSS_GPS && rxData->m_ObsArray[index].freqType == GNSS_GPSL1 )
      {
        if( isLeastSquares )
        {
          clock_offset = rxData->m_pvt_lsq.clockOffset;
        }
        else
        {
          clock_offset = rxData->m_pvt.clockOffset;
        }

        // allows compute the misclosure value (regardless of whether it is used in solution
        psr_measured = rxData->m_ObsArray[index].psr;

        // Add the satellite clock correction.
        psr_measured += rxData->m_ObsArray[index].satellite.clk;

        // Compensate for the ionospheric delay if indicated.
        // The corrections must be determined beforehand.
        if( rxData->m_ObsArray[index].flags.useBroadcastIonoCorrection )
        {
          // Compensate for the ionospheric delay
          psr_measured -= rxData->m_ObsArray[index].corrections.prcIono;
        }

        // Compensate for the tropospheric delay if indicated.
        // The corrections must be determined beforehand.
        if( rxData->m_ObsArray[index].flags.useTropoCorrection )
        {
          // Compensate for the tropospheric delay
          psr_measured -= rxData->m_ObsArray[index].corrections.prcTropoDry;
          psr_measured -= rxData->m_ObsArray[index].corrections.prcTropoWet;
        }

        range_base = 0;
        if( rxBaseData != NULL )
        {    
          if( rxData->m_ObsArray[index].flags.isDifferentialPsrAvailable )
          {
            k = rxData->m_ObsArray[index].index_differential;
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
            else
            {
              // This misclosure cannot be computed since no differential measurements is available
              // and a differential clock offset is computed.
              rxData->m_ObsArray[index].psr_misclosure = 0.0;
              return true;
            }
          }
        }

        // Calculate the computed pseudorange = geometric range + clock offset (m)
        // The range value and the clock offset must be determined beforehand.
        // If differential, range_base != 0, it is the computed psuedorange difference.
        psr_computed = rxData->m_ObsArray[index].range - range_base + clock_offset;

        // The misclosure is the corrected measured value minus the computed valid.
        if( isLeastSquares )
        {
          rxData->m_ObsArray[index].psr_misclosure_lsq = psr_measured - psr_computed;
        }
        else
        {
          rxData->m_ObsArray[index].psr_misclosure = psr_measured - psr_computed;            
        }
      }
#ifdef GDM_UWB_RANGE_HACK
      else if( rxData->m_ObsArray[index].system == GNSS_UWBSystem && rxData->m_ObsArray[index].flags.isActive )
      {
        psr_measured = rxData->m_ObsArray[index].psr;
        
        // Calculate the computed uwbrange = geometric range only (m)
        psr_computed = rxData->m_ObsArray[index].range;

        // The misclosure is the corrected measured value minus the computed valid.
        if( isLeastSquares )
        {
          rxData->m_ObsArray[index].psr_misclosure_lsq = psr_measured - psr_computed;
        }
        else
        {
          rxData->m_ObsArray[index].psr_misclosure = psr_measured - psr_computed;            
        }
        
        rxData->m_ObsArray[index].flags.isPsrUsedInSolution = true; // GDM_HACK
      }
#endif
      else
      {
        if( isLeastSquares )
        {
          rxData->m_ObsArray[index].psr_misclosure_lsq = 0.0;            
        }
        else
        {
          rxData->m_ObsArray[index].psr_misclosure = 0.0;            
        }
      }
    }
    return true;
  }

 
  bool GNSS_Estimator::DeterminePseudorangeMisclosures_GPSL1( 
    GNSS_RxData *rxData,       //!< The pointer to the receiver data.    
    GNSS_RxData *rxBaseData,   //!< The pointer to the reference receiver data. NULL if not available.    
    const bool isLeastSquares  //!< A boolean to indicate if the rover position and velocity values are from least squares rxData->m_pvt_lsq or from rxData->m_pvt.
    )
  {
    unsigned i = 0;
    
    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( DeterminePseudorangeMisclosure_GPSL1( rxData, i, rxBaseData, isLeastSquares ) == false )
      {
        GNSS_ERROR_MSG( "DeterminePseudorangeMisclosure_GPSL1 returned false." );
        return false;
      }
    }
    return true;
  }


  bool GNSS_Estimator::DeterminePositionConstraintMisclosures( 
    GNSS_RxData *rxData, //!< The rover receiver data.
    double &w_lat,       //!< The computed latitude constraint misclosure [m].
    double &w_lon,       //!< The computed longitude constraint misclosure [m].
    double &w_hgt        //!< The computed height constraint misclosure [m].
    )
  {
    double N;
    double M;
    double diff;

    if( rxData == NULL )
    {
      GNSS_ERROR_MSG( "if( rxData == NULL )" );
      return false;
    }

    if( rxData->m_pvt.isPositionConstrained )
    {
      GEODESY_ComputePrimeVerticalRadiusOfCurvature(
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        rxData->m_prev_pvt.latitude,
        &N );
      GEODESY_ComputeMeridianRadiusOfCurvature( 
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        rxData->m_prev_pvt.latitude,
        &M );
            
      // Constraining to the previuos solution.
      diff = (rxData->m_prev_pvt.latitude - rxData->m_pvt.latitude); 
      diff *= ( M + rxData->m_prev_pvt.height ); // convert to meters.
      w_lat = diff;
      diff = (rxData->m_prev_pvt.longitude - rxData->m_pvt.longitude );
      diff *= (N + rxData->m_prev_pvt.height)*cos(rxData->m_prev_pvt.latitude); // convert to meters.
      w_lon = diff;
      w_hgt = rxData->m_prev_pvt.height - rxData->m_pvt.height;
    }
    return true;
  }

  bool GNSS_Estimator::DetermineHeightConstraintMisclosures( 
    GNSS_RxData *rxData, //!< The rover receiver data.
    double &w_hgt        //!< The computed height constraint misclosure [m].
    )
  {
    if( rxData->m_pvt.isPositionConstrained )
    {
      GNSS_ERROR_MSG( "Already position constrained." );
      return false;
    }
    else if( rxData->m_pvt.isHeightConstrained )
    {
      w_hgt = rxData->m_prev_pvt.height - rxData->m_pvt.height;      
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

    if( rxData == NULL || rxBaseData == NULL )
    {
      GNSS_ERROR_MSG( "if( rxData == NULL || rxBaseData == NULL )" );
      return false;
    }

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


  bool GNSS_Estimator::DetermineSingleDifferenceADR_Misclosure_GPSL1( 
    GNSS_RxData *rxData,      //!< The pointer to the receiver data.    
    const unsigned int index, //!< The index of the observation in the receiver data.
    GNSS_RxData *rxBaseData   //!< The pointer to the reference receiver data. NULL if not available.    
    )
  {
    unsigned i = 0;
    int k = 0;
    double adr_base = 0;
    double range_base = 0;
    double adr_measured = 0;
    double adr_computed = 0;

    // Check the index.
    if( index < 0 || index >= GNSS_RXDATA_NR_CHANNELS )
    {
      GNSS_ERROR_MSG( "if( index < 0 || index >= GNSS_RXDATA_NR_CHANNELS )" );
      return false;
    }
    
    // Check the pointers.
    if( rxData == NULL || rxBaseData == NULL )
    {
      GNSS_ERROR_MSG( "if( rxData == NULL || rxBaseData == NULL )" );
      return false;
    }

    if( rxData->m_ObsArray[index].flags.isActive )
    {
      if( rxData->m_ObsArray[index].system == GNSS_GPS && rxData->m_ObsArray[index].freqType == GNSS_GPSL1 )
      {
        // allows computation of the misclosure value (regardless of whether it is used in solution
        adr_measured = rxData->m_ObsArray[index].adr * GPS_WAVELENGTHL1;

        // Add the satellite clock correction.
        adr_measured += rxData->m_ObsArray[index].satellite.clk;

        // Compensate for the ionospheric delay if indicated.
        // The corrections must be determined beforehand.
        if( rxData->m_ObsArray[index].flags.useBroadcastIonoCorrection )
        {
          // Compensate for the ionospheric delay
          adr_measured += rxData->m_ObsArray[index].corrections.prcIono;
        }

        // Compensate for the tropospheric delay if indicated.
        // The corrections must be determined beforehand.
        if( rxData->m_ObsArray[index].flags.useTropoCorrection )
        {
          // Compensate for the tropospheric delay
          adr_measured -= rxData->m_ObsArray[index].corrections.prcTropoDry;
          adr_measured -= rxData->m_ObsArray[index].corrections.prcTropoWet;
        }

        range_base = 0;
        if( rxBaseData != NULL )
        {    
          if( rxData->m_ObsArray[index].flags.isDifferentialAdrAvailable )
          {
            k = rxData->m_ObsArray[index].index_differential;
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
            else
            {
              // This misclosure cannot be computed since no differential measurements is available
              // and a differential clock offset is computed.
              rxData->m_ObsArray[index].adr_misclosure = 0.0;
              return true;
            }
          }
        }

        // Calculate the computed adr = geometric range + clock offset (m)
        // The range value and the clock offset must be determined beforehand.
        // If differential, range_base != 0, and the ambiguity is the single differnce ambiguity [m].
        adr_computed = rxData->m_ObsArray[index].range - range_base;
        adr_computed += rxData->m_pvt.clockOffset;          
        adr_computed += rxData->m_ObsArray[index].ambiguity; 

        // The misclosure is the corrected measured value minus the computed valid.
        rxData->m_ObsArray[index].adr_misclosure = adr_measured - adr_computed;            
      }
      else
      {
        rxData->m_ObsArray[index].adr_misclosure = 0.0;            
      }
    }    
    return true;
  }



  bool GNSS_Estimator::DetermineSingleDifferenceADR_Misclosures_GPSL1( 
    GNSS_RxData *rxData,     //!< The pointer to the receiver data.    
    GNSS_RxData *rxBaseData  //!< The pointer to the reference receiver data. NULL if not available.    
    )
  {
    unsigned i = 0;

    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( DetermineSingleDifferenceADR_Misclosure_GPSL1(rxData, i, rxBaseData) == false )
      {
        GNSS_ERROR_MSG( "DetermineSingleDifferenceADR_Misclosure_GPSL1 returned false." );
        return false;
      }
    }
    return true;
  }


  bool GNSS_Estimator::DetermineDoubleDifferenceADR_Misclosures_GPSL1( 
    GNSS_RxData *rxData,     //!< The pointer to the receiver data.    
    GNSS_RxData *rxBaseData, //!< The pointer to the reference receiver data. NULL if not available. 
    Matrix &subB,            //!< The matrix that describes the differencing from SD to DD adr measurements
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
      {
        GNSS_ERROR_MSG( "if( !w.Resize( n, 1 ) )" );
        return false;
      }
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
              rxData->m_ObsArray[rxData->m_ObsArray[i].index_between_satellite_differential].adr_misclosure_temp + 
              rxData->m_ObsArray[i].ambiguity_dd;

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


  bool GNSS_Estimator::DetermineDopplerMisclosure_GPSL1( 
    GNSS_RxData *rxData,       //!< The pointer to the receiver data.    
    const unsigned int index,  //!< The index of the observation in the receiver data.
    GNSS_RxData *rxBaseData,   //!< The pointer to the reference receiver data. NULL if not available.    
    const bool isLeastSquares  //!< A boolean to indicate if the rover position and velocity values are from least squares rxData->m_pvt_lsq or from rxData->m_pvt.
    )
  {
    int k = 0;
    double doppler_base = 0;
    double rangerate_base = 0;
    double doppler_measured = 0;
    double doppler_computed = 0;
    double clkdrift = 0;

    // Check the index.
    if( index < 0 || index >= GNSS_RXDATA_NR_CHANNELS )
    {
      GNSS_ERROR_MSG( "if( index < 0 || index >= GNSS_RXDATA_NR_CHANNELS )" );
      return false;
    }
    
    // Check the pointers.
    if( rxData == NULL )
    {
      GNSS_ERROR_MSG( "if( rxData == NULL )" );
      return false;
    }

    if( rxData->m_ObsArray[index].flags.isActive )
    {
      rxData->m_ObsArray[index].doppler_misclosure = 0.0;            

      if( rxData->m_ObsArray[index].system == GNSS_GPS && rxData->m_ObsArray[index].freqType == GNSS_GPSL1 )
      {
        if( isLeastSquares )
        {
          clkdrift = rxData->m_pvt_lsq.clockDrift;
        }
        else
        {
          clkdrift = rxData->m_pvt.clockDrift;
        }

        // always compute the misclosure regardless of whether it is used in solution.

        // Compute the pseudorange misclosures in meters/second!
        doppler_measured = rxData->m_ObsArray[index].doppler * GPS_WAVELENGTHL1;

        // Add the satellite clock drift.        
        doppler_measured += rxData->m_ObsArray[index].satellite.clkdrift;

        rangerate_base = 0;
        if( rxBaseData != NULL )
        {
          doppler_base = 0;
          if( rxData->m_ObsArray[index].flags.isDifferentialDopplerAvailable )
          {
            k = rxData->m_ObsArray[index].index_differential;
            if( k != -1 )
            {
              doppler_base   = rxBaseData->m_ObsArray[k].doppler * GPS_WAVELENGTHL1;
              rangerate_base = rxBaseData->m_ObsArray[k].rangerate;
            }            
            else
            {
              // This misclosure cannot be computed since no differential measurements is available
              // and a differential clock offset is computed.
              rxData->m_ObsArray[index].doppler_misclosure = 0.0;
              return true;
            }
            doppler_measured -= doppler_base;
          }
        }

        // Calculate the computed doppler = geometric range rate + rx clock drift [m/s]
        doppler_computed = rxData->m_ObsArray[index].rangerate - rangerate_base + clkdrift;

        // The misclosure is the corrected measured value minus the computed valid.            
        rxData->m_ObsArray[index].doppler_misclosure = doppler_measured - doppler_computed;
      }
    }
    return true;
  }

  bool GNSS_Estimator::DetermineDopplerMisclosures_GPSL1( 
    GNSS_RxData *rxData,       //!< The pointer to the receiver data.    
    GNSS_RxData *rxBaseData,   //!< The pointer to the reference receiver data. NULL if not available.    
    const bool isLeastSquares  //!< A boolean to indicate if the rover position and velocity values are from least squares rxData->m_pvt_lsq or from rxData->m_pvt.
    )
  {
    unsigned i = 0;

    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( DetermineDopplerMisclosure_GPSL1( rxData, i, rxBaseData, isLeastSquares ) == false )
      {
        GNSS_ERROR_MSG( "DetermineDopplerMisclosure_GPSL1 returned false." );
        return false;
      }
    }
    return true;
  }


  bool GNSS_Estimator::DetermineDesignMatrixElement_GPSL1_Doppler( 
    GNSS_RxData &rxData,      //!< The receiver data.
    const unsigned int index, //!< The index of the observation in the receiver data.
    const bool isLeastSquares  //!< A boolean to indicate if the rover position and velocity values are from least squares rxData->m_pvt_lsq or from rxData->m_pvt.
    )
  {
    double lat = 0;
    double lon = 0;
    double hgt = 0;

    // Check the index.
    if( index < 0 || index >= GNSS_RXDATA_NR_CHANNELS )
    {
      GNSS_ERROR_MSG( "if( index < 0 || index >= GNSS_RXDATA_NR_CHANNELS )" );
      return false;
    }

    if( isLeastSquares )
    {
      lat = rxData.m_pvt_lsq.latitude;
      lon = rxData.m_pvt_lsq.longitude;
      hgt = rxData.m_pvt_lsq.height;
    }
    else
    {
      lat = rxData.m_pvt.latitude;
      lon = rxData.m_pvt.longitude;
      hgt = rxData.m_pvt.height;
    }
    
    // Compute the design matrix row for the velocity solution.
    if( rxData.m_ObsArray[index].flags.isActive )
    {
      if( rxData.m_ObsArray[index].system == GNSS_GPS && rxData.m_ObsArray[index].freqType == GNSS_GPSL1 )
      {
        if( rxData.m_ObsArray[index].flags.isDopplerUsedInSolution )
        {
          NAVIGATION_ComputeDerivativesOf_Range_WithRespectToLatitudeLongitudeHeight(
            lat,
            lon, 
            hgt,
            rxData.m_ObsArray[index].satellite.x,
            rxData.m_ObsArray[index].satellite.y,
            rxData.m_ObsArray[index].satellite.z,
            &(rxData.m_ObsArray[index].H_v[0]),
            &(rxData.m_ObsArray[index].H_v[1]),
            &(rxData.m_ObsArray[index].H_v[2]),
            &rxData.m_ObsArray[index].range
            );
          rxData.m_ObsArray[index].H_v[0] *= -1.0; // Doppler sign convention, NovAtel convention, increasing psr means negative Doppler
          rxData.m_ObsArray[index].H_v[1] *= -1.0;
          rxData.m_ObsArray[index].H_v[2] *= -1.0;
        }
      }
    }    
    return true;
  }


  bool GNSS_Estimator::DetermineDesignMatrixElements_GPSL1_Doppler( 
    GNSS_RxData &rxData,       //!< The receiver data.
    const bool isLeastSquares  //!< A boolean to indicate if the rover position and velocity values are from least squares rxData->m_pvt_lsq or from rxData->m_pvt.
    )
  {
    unsigned i = 0;
    // Compute the design matrix for the position solution.
    for( i = 0; i < rxData.m_nrValidObs; i++ )
    {
      if( DetermineDesignMatrixElement_GPSL1_Doppler( rxData, i, isLeastSquares ) == false )
      {
        GNSS_ERROR_MSG( "DetermineDesignMatrixElement_GPSL1_Doppler returned false." );
        return false;
      }
    }
    return true;
  }


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

    i = static_cast<unsigned>(v);
    if( i == 0 )
      return true; 
    
    rT = r;
    if( !rT.Inplace_Transpose() )
    {
      GNSS_ERROR_MSG( "if( !rT.Inplace_Transpose() )" );
      return false;
    }

    // Compute the a-posteriori variance factor.
    tmpM = rT * W * r;
    avf = tmpM[0] / v;
    
    // Determine the chi squared test statistic value.
    i = static_cast<unsigned>(v);
    if( i < 0 || i > 30 )
    {
      GNSS_ERROR_MSG( "if( i < 0 || i > 30 )" );
      return false;
    }
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

    PrintMatToDebug( "Cr", Cr, 3 );


    // Check the diagonal of Cr for zero and negative values, an error condition.
    for( i = 0; i < Cr.GetNrRows(); i++ )
    {
      if( Cr[i][i] <= 0.0 )
      {
        GNSS_ERROR_MSG( "if( Cr[i][i] <= 0.0 )" );
        return false;
      }
    }

    
    if( !r_standardized.Resize( Cr.GetNrRows(), 1) )
    {
      GNSS_ERROR_MSG( "if( !r_standardized.Resize( Cr.GetNrRows(), 1) )" );
      return false;
    }

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
    {
      GNSS_ERROR_MSG( "if( indexOfLargest >= rxData.m_nrValidObs )" );
      return false;
    }

    PrintMatToDebug( "r", r, 3 );
    PrintMatToDebug( "r_standardized", r_standardized, 3 );
    
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


  bool GNSS_Estimator::ComputeTransitionMatrix_EKF(
    const double dT  //!< The change in time since the last update [s].
    )
  {
    const double betaVn       = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaVn;       // The Gauss Markov beta for northing velocity [1/s].
    const double betaVe       = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaVe;       // The Gauss Markov beta for easting velocity [1/s].
    const double betaVup      = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaVup;      // The Gauss Markov beta for up velocity [1/s].
    const double betaClkDrift = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaClkDrift; // The Gauss Markov beta for the clock drift [1/s].

    double eVn = 0;
    double eVe = 0;
    double eVup = 0;
    double eClkDrift = 0;

    
    if( m_EKF.T.GetNrRows() != 8 || m_EKF.T.GetNrCols() != 8 )
    {
      if( !m_EKF.T.Resize(8,8) )
      {
        GNSS_ERROR_MSG( "if( !m_EKF.T.Resize(8,8) )" );
        return false;
      }
    }    
    
    eVn  = exp( -betaVn  * dT );
    eVe  = exp( -betaVe  * dT );
    eVup = exp( -betaVup * dT );

    eClkDrift = exp( -betaClkDrift * dT );

    m_EKF.T.Zero();
    
    m_EKF.T[0][0] = 1.0;
    m_EKF.T[0][3] = (1.0 - eVn) / betaVn;

    m_EKF.T[1][1] = 1.0;
    m_EKF.T[1][4] = (1.0 - eVe) / betaVe;

    m_EKF.T[2][2] = 1.0;
    m_EKF.T[2][5] = (1.0 - eVup) / betaVup;
    
    m_EKF.T[3][3] = eVn;
    m_EKF.T[4][4] = eVe;
    m_EKF.T[5][5] = eVup;

    m_EKF.T[6][6] = 1.0;
    m_EKF.T[6][7] = -1.0*(1.0 - eClkDrift) / betaClkDrift; // GDM - multiply by -1 required due to Doppler convention

    m_EKF.T[7][7] = eClkDrift;

    return true;
  }

  bool GNSS_Estimator::ComputeTransitionMatrix_RTK(
    const double dT  //!< The change in time since the last update [s].    
    )
  {
    if( m_FilterType == GNSS_FILTER_TYPE_EKF || m_FilterType == GNSS_FILTER_TYPE_RTK8 )
    {
      const double betaVn       = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaVn;       // The Gauss Markov beta for northing velocity [1/s].
      const double betaVe       = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaVe;       // The Gauss Markov beta for easting velocity [1/s].
      const double betaVup      = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaVup;      // The Gauss Markov beta for up velocity [1/s].
      const double betaClkDrift = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaClkDrift; // The Gauss Markov beta for the clock drift [1/s].

      double eVn = 0;
      double eVe = 0;
      double eVup = 0;
      double eClkDrift = 0;
      const unsigned int u = 8 + static_cast<unsigned int>(m_ActiveAmbiguitiesList.size());

      m_RTK.T.Identity( u );

      eVn  = exp( -betaVn  * dT );
      eVe  = exp( -betaVe  * dT );
      eVup = exp( -betaVup * dT );

      eClkDrift = exp( -betaClkDrift * dT );

      m_RTK.T[0][0] = 1.0;
      m_RTK.T[0][3] = (1.0 - eVn) / betaVn;

      m_RTK.T[1][1] = 1.0;
      m_RTK.T[1][4] = (1.0 - eVe) / betaVe;

      m_RTK.T[2][2] = 1.0;
      m_RTK.T[2][5] = (1.0 - eVup) / betaVup;

      m_RTK.T[3][3] = eVn;
      m_RTK.T[4][4] = eVe;
      m_RTK.T[5][5] = eVup;

      m_RTK.T[6][6] = 1.0;
      m_RTK.T[6][7] = -1.0 * (1.0 - eClkDrift) / betaClkDrift; // GDM - multiply by -1 required due to Doppler convention

      m_RTK.T[7][7] = eClkDrift;

      return true;
    }
    else if( m_FilterType == GNSS_FILTER_TYPE_RTK4 )
    {
      const unsigned int u = 4 + static_cast<unsigned int>(m_ActiveAmbiguitiesList.size());
      m_RTK.T.Identity( u );
      return true;
    }
    else
    {
      GNSS_ERROR_MSG( "Only EKF, RTK4, and RTK8 supported" );
      return false;
    }    
  }

  bool GNSS_Estimator::ComputeProcessNoiseMatrix_EKF(
    const double dT  //!< The change in time since the last update [s].
    )
  {
    const double betaVn       = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaVn;       // The Gauss Markov beta for northing velocity [1/s].
    const double betaVe       = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaVe;       // The Gauss Markov beta for easting velocity [1/s].
    const double betaVup      = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaVup;      // The Gauss Markov beta for up velocity [1/s].
    const double betaClkDrift = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaClkDrift; // The Gauss Markov beta for the clock drift [1/s].

    const double qVn       = 2 * m_FirstOrderGaussMarkovKalmanModel.sigmaVn  * m_FirstOrderGaussMarkovKalmanModel.sigmaVn  * betaVn;  // The process noise value for northing velocity.
    const double qVe       = 2 * m_FirstOrderGaussMarkovKalmanModel.sigmaVe  * m_FirstOrderGaussMarkovKalmanModel.sigmaVe  * betaVe;  // The process noise value for easting  velocity.
    const double qVup      = 2 * m_FirstOrderGaussMarkovKalmanModel.sigmaVup * m_FirstOrderGaussMarkovKalmanModel.sigmaVup * betaVup; // The process noise value for up       velocity.
    const double qClkDrift = 2 * m_FirstOrderGaussMarkovKalmanModel.sigmaClkDrift * m_FirstOrderGaussMarkovKalmanModel.sigmaClkDrift * betaClkDrift; // The process noise value for clock drift.

    double eVn = 0;
    double eVe = 0;
    double eVup = 0;
    double eClkDrift = 0;
    double eVn2 = 0;
    double eVe2 = 0;
    double eVup2 = 0;
    double eClkDrift2 = 0;

    const unsigned int u = 8;

    if( m_EKF.Q.nrows() != u )
    {
      if( !m_EKF.Q.Resize( u, u ) )
      {
        GNSS_ERROR_MSG( "if( !m_EKF.Q.Resize( u, u ) )" );
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

    m_EKF.Q[0][0]  = qVn / (betaVn*betaVn);
    m_EKF.Q[0][0] *= dT - 2.0*(1.0 - eVn)/(betaVn) + (1.0 - eVn2)/(2.0*betaVn);

    m_EKF.Q[1][1]  = qVe / (betaVe*betaVe);
    m_EKF.Q[1][1] *= dT - 2.0*(1.0 - eVe)/(betaVe) + (1.0 - eVe2)/(2.0*betaVe);

    m_EKF.Q[2][2]  = qVn / (betaVup*betaVup);
    m_EKF.Q[2][2] *= dT - 2.0*(1.0 - eVup)/(betaVup) + (1.0 - eVup2)/(2.0*betaVup);

    m_EKF.Q[3][3]  = qVn * (1.0 - eVn2) / (2.0*betaVn);
    
    m_EKF.Q[4][4]  = qVe * (1.0 - eVe2) / (2.0*betaVe);

    m_EKF.Q[5][5]  = qVup * (1.0 - eVup2) / (2.0*betaVup);

    m_EKF.Q[6][6]  = qClkDrift / (betaClkDrift*betaClkDrift);
    m_EKF.Q[6][6] *= dT - 2.0*(1.0 - eClkDrift)/(betaClkDrift) + (1.0 - eClkDrift2)/(2.0*betaClkDrift);
    
    m_EKF.Q[7][7]  = qClkDrift * (1.0 - eClkDrift2) / (2.0*betaClkDrift);

    m_EKF.Q[0][3]  = qVn / betaVn;
    m_EKF.Q[0][3] *= (1.0 - eVn)/betaVn - (1.0 - eVn2)/(2.0*betaVn);
    m_EKF.Q[3][0]  = m_EKF.Q[0][3];

    m_EKF.Q[1][4]  = qVe / betaVe;
    m_EKF.Q[1][4] *= (1.0 - eVe)/betaVe - (1.0 - eVe2)/(2.0*betaVe);
    m_EKF.Q[4][1]  = m_EKF.Q[1][4];

    m_EKF.Q[2][5]  = qVup / betaVup;
    m_EKF.Q[2][5] *= (1.0 - eVup)/betaVup - (1.0 - eVup2)/(2.0*betaVup);
    m_EKF.Q[5][2]  = m_EKF.Q[2][5];

    m_EKF.Q[6][7]  = qClkDrift / betaClkDrift;
    m_EKF.Q[6][7] *= (1.0 - eClkDrift)/betaClkDrift - (1.0 - eClkDrift2)/(2.0*betaClkDrift);
    m_EKF.Q[7][6]  = m_EKF.Q[6][7];

    return true;
  }

  bool GNSS_Estimator::ComputeProcessNoiseMatrix_RTK(
    const double dT  //!< The change in time since the last update [s].
    )
  {
    if( m_FilterType == GNSS_FILTER_TYPE_EKF || m_FilterType == GNSS_FILTER_TYPE_RTK8 )
    {
      const double betaVn       = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaVn;       // The Gauss Markov beta for northing velocity [1/s].
      const double betaVe       = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaVe;       // The Gauss Markov beta for easting velocity [1/s].
      const double betaVup      = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaVup;      // The Gauss Markov beta for up velocity [1/s].
      const double betaClkDrift = 1.0/m_FirstOrderGaussMarkovKalmanModel.alphaClkDrift; // The Gauss Markov beta for the clock drift [1/s].

      const double qVn       = 2 * m_FirstOrderGaussMarkovKalmanModel.sigmaVn  * m_FirstOrderGaussMarkovKalmanModel.sigmaVn  * betaVn;  // The process noise value for northing velocity.
      const double qVe       = 2 * m_FirstOrderGaussMarkovKalmanModel.sigmaVe  * m_FirstOrderGaussMarkovKalmanModel.sigmaVe  * betaVe;  // The process noise value for easting  velocity.
      const double qVup      = 2 * m_FirstOrderGaussMarkovKalmanModel.sigmaVup * m_FirstOrderGaussMarkovKalmanModel.sigmaVup * betaVup; // The process noise value for up       velocity.
      const double qClkDrift = 2 * m_FirstOrderGaussMarkovKalmanModel.sigmaClkDrift * m_FirstOrderGaussMarkovKalmanModel.sigmaClkDrift * betaClkDrift; // The process noise value for clock drift.

      double eVn = 0;
      double eVe = 0;
      double eVup = 0;
      double eClkDrift = 0;
      double eVn2 = 0;
      double eVe2 = 0;
      double eVup2 = 0;
      double eClkDrift2 = 0;

      const unsigned int u = 8 + static_cast<unsigned int>(m_ActiveAmbiguitiesList.size());

      if( m_RTK.Q.nrows() != u )
      {
        if( !m_RTK.Q.Resize( u, u ) )
        {
          GNSS_ERROR_MSG( "if( !m_RTK.Q.Resize( u, u ) )" );
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

      m_RTK.Q[0][0]  = qVn / (betaVn*betaVn);
      m_RTK.Q[0][0] *= dT - 2.0*(1.0 - eVn)/(betaVn) + (1.0 - eVn2)/(2.0*betaVn);

      m_RTK.Q[1][1]  = qVe / (betaVe*betaVe);
      m_RTK.Q[1][1] *= dT - 2.0*(1.0 - eVe)/(betaVe) + (1.0 - eVe2)/(2.0*betaVe);

      m_RTK.Q[2][2]  = qVn / (betaVup*betaVup);
      m_RTK.Q[2][2] *= dT - 2.0*(1.0 - eVup)/(betaVup) + (1.0 - eVup2)/(2.0*betaVup);

      m_RTK.Q[3][3]  = qVn * (1.0 - eVn2) / (2.0*betaVn);

      m_RTK.Q[4][4]  = qVe * (1.0 - eVe2) / (2.0*betaVe);

      m_RTK.Q[5][5]  = qVup * (1.0 - eVup2) / (2.0*betaVup);

      m_RTK.Q[6][6]  = qClkDrift / (betaClkDrift*betaClkDrift);
      m_RTK.Q[6][6] *= dT - 2.0*(1.0 - eClkDrift)/(betaClkDrift) + (1.0 - eClkDrift2)/(2.0*betaClkDrift);

      m_RTK.Q[7][7]  = qClkDrift * (1.0 - eClkDrift2) / (2.0*betaClkDrift);

      m_RTK.Q[0][3]  = qVn / betaVn;
      m_RTK.Q[0][3] *= (1.0 - eVn)/betaVn - (1.0 - eVn2)/(2.0*betaVn);
      m_RTK.Q[3][0]  = m_RTK.Q[0][3];

      m_RTK.Q[1][4]  = qVe / betaVe;
      m_RTK.Q[1][4] *= (1.0 - eVe)/betaVe - (1.0 - eVe2)/(2.0*betaVe);
      m_RTK.Q[4][1]  = m_RTK.Q[1][4];

      m_RTK.Q[2][5]  = qVup / betaVup;
      m_RTK.Q[2][5] *= (1.0 - eVup)/betaVup - (1.0 - eVup2)/(2.0*betaVup);
      m_RTK.Q[5][2]  = m_RTK.Q[2][5];

      m_RTK.Q[6][7]  = qClkDrift / betaClkDrift;
      m_RTK.Q[6][7] *= (1.0 - eClkDrift)/betaClkDrift - (1.0 - eClkDrift2)/(2.0*betaClkDrift);
      m_RTK.Q[7][6]  = m_RTK.Q[6][7];

      return true;
    }
    else if( m_FilterType == GNSS_FILTER_TYPE_RTK4 )
    {
      // This is a random walk model for the latitude, longitude, height and clock offset states.
      const double qLat = dT * m_FourStateRandomWalkKalmanModel.sigmaNorth * m_FourStateRandomWalkKalmanModel.sigmaNorth; // The process noise value for northing.
      const double qLon = dT * m_FourStateRandomWalkKalmanModel.sigmaEast  * m_FourStateRandomWalkKalmanModel.sigmaEast;  // The process noise value for easting.
      const double qHgt = dT * m_FourStateRandomWalkKalmanModel.sigmaUp    * m_FourStateRandomWalkKalmanModel.sigmaUp;    // The process noise value for up.
      const double qClk = dT * m_FourStateRandomWalkKalmanModel.sigmaClock * m_FourStateRandomWalkKalmanModel.sigmaClock; // The process noise value for clock offset.

      const unsigned int u = 4 + static_cast<unsigned int>(m_ActiveAmbiguitiesList.size());

      if( m_RTK.Q.nrows() != u )
      {
        if( !m_RTK.Q.Resize( u, u ) )
        {
          GNSS_ERROR_MSG( "if( !m_RTK.Q.Resize( u, u ) )" );
          return false;
        }
      }

      m_RTK.Q[0][0]  = qLat;
      m_RTK.Q[1][1]  = qLon;
      m_RTK.Q[2][2]  = qHgt;
      m_RTK.Q[3][3]  = qClk;

      return true;
    }
    else
    {
      GNSS_ERROR_MSG( "Only EKF, RTK4, and RTK8 supported." );
      return false;
    }
  }


  bool GNSS_Estimator::PredictAhead_EKF(
    GNSS_RxData &rxData, //!< The receiver data.
    const double dT      //!< The change in time since the last update [s].
    )
  {
    double M = 0; // The meridian radius of curvature.
    double N = 0; // The prime vertical radius of curvature.
    bool result = false;
    Matrix tmpMat;
    double lat = 0;
    double h = 0;

    if( dT == 0.0 )
      return true;

    GEODESY_ComputeMeridianRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      rxData.m_pvt.latitude,
      &M );

    GEODESY_ComputePrimeVerticalRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      rxData.m_pvt.latitude,
      &N );

    // Get the transition Matrix
    result = ComputeTransitionMatrix_EKF( dT );
    if( result == false )    
    {
      GNSS_ERROR_MSG( "ComputeTransitionMatrix_EKF returned false." );
      return false;
    }

    // Set the process noise Matrix
    result = ComputeProcessNoiseMatrix_EKF( dT );      
    if( result == false )
    {
      GNSS_ERROR_MSG( "ComputeProcessNoiseMatrix_EKF returned false." );
      return false;
    }

    
    ////
    // predict the states ahead    
    
    // for use of use
    lat = rxData.m_pvt.latitude;
    h   = rxData.m_pvt.height;
    
    rxData.m_pvt.latitude  += m_EKF.T[0][3] * rxData.m_pvt.vn / (M+h);  // for small dT, this should be: lat += dT * vn / M
    rxData.m_pvt.longitude += m_EKF.T[1][4] * rxData.m_pvt.ve / ((N+h)*cos(lat));  // for small dT, this should be: lon += dT * ve / ((N+h)*cos(lat))
    rxData.m_pvt.height    += m_EKF.T[2][5] * rxData.m_pvt.vup;  // for small dT, this should be: hgt += dT * vup    
    
    rxData.m_pvt.vn  *= m_EKF.T[3][3];  // for small dT, this should be vn = vn    
    rxData.m_pvt.ve  *= m_EKF.T[4][4];  // for small dT, this should be ve = ve
    rxData.m_pvt.vup *= m_EKF.T[5][5];  // for small dT, this should be vup = vup

    rxData.m_pvt.clockOffset += m_EKF.T[6][7] * rxData.m_pvt.clockDrift;  // for small dT, this should be: clk += dT * clkrate
    rxData.m_pvt.clockDrift  *= m_EKF.T[7][7];  // for small dT, this should be clkdrift = clkdrift

    //
    ////


    ////
    // predict the new state variance/covariance

    // It can be done this way:
    // P = T * P * T.transpose() + Q;
    // but the following is more efficient
    tmpMat = m_EKF.T;
    if( !tmpMat.Inplace_Transpose() )
    {
      GNSS_ERROR_MSG( "if( !tmpMat.Inplace_Transpose() )" );
      return false;
    }

    if( !m_EKF.P.Inplace_PreMultiply( m_EKF.T ) )
    {
      GNSS_ERROR_MSG( "if( !m_EKF.P.Inplace_PreMultiply( m_EKF.T ) )" );
      return false;
    }

    if( !m_EKF.P.Inplace_PostMultiply( tmpMat ) )
    {
      GNSS_ERROR_MSG( "if( !m_EKF.P.Inplace_PostMultiply( tmpMat ) )" );
      return false;
    }

    if( !m_EKF.P.Inplace_Add( m_EKF.Q ) )
    {
      GNSS_ERROR_MSG( "if( !m_EKF.P.Inplace_Add( m_EKF.Q ) )" );
      return false;
    }
    //
    ////

    return true;    
  }

  bool GNSS_Estimator::PredictAhead_RTK(
    GNSS_RxData &rxData, //!< The receiver data.
    const double dT      //!< The change in time since the last update [s].    
    )
  {
    double M = 0; // The meridian radius of curvature.
    double N = 0; // The prime vertical radius of curvature.
    bool result = false;
    Matrix tmpMat;
    double lat = 0;
    double h = 0;
    double clkvar = 0; // The variance of the clock offset state.

    if( dT == 0.0 )
      return true;

    if( m_FilterType == GNSS_FILTER_TYPE_RTK8 )
    {
      GEODESY_ComputeMeridianRadiusOfCurvature(
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        rxData.m_pvt.latitude,
        &M );

      GEODESY_ComputePrimeVerticalRadiusOfCurvature(
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        rxData.m_pvt.latitude,
        &N );
    }

    // Get the transition Matrix
    result = ComputeTransitionMatrix_RTK( dT );
    if( result == false )
    {
      GNSS_ERROR_MSG( "ComputeTransitionMatrix_RTK returned false." );
      return false;
    }
    PrintMatToDebug( "m_RTK.T", m_RTK.T, 3 );

    // Set the process noise Matrix
    result = ComputeProcessNoiseMatrix_RTK( dT );
    if( result == false )
    {
      GNSS_ERROR_MSG( "ComputeProcessNoiseMatrix_RTK returned false." );
      return false;
    }
    PrintMatToDebug( "m_RTK.Q", m_RTK.Q, 3 );
    
    ////
    // predict the states ahead
    //
    if( m_FilterType == GNSS_FILTER_TYPE_RTK8 )
    {
      lat = rxData.m_pvt.latitude;
      h   = rxData.m_pvt.height;
    
      rxData.m_pvt.latitude  += m_RTK.T[0][3] * rxData.m_pvt.vn / (M+h);  // for small dT, this should be: lat += dT * vn / M
      rxData.m_pvt.longitude += m_RTK.T[1][4] * rxData.m_pvt.ve / ((N+h)*cos(lat));  // for small dT, this should be: lon += dT * ve / ((N+h)*cos(lat))
      rxData.m_pvt.height    += m_RTK.T[2][5] * rxData.m_pvt.vup;  // for small dT, this should be: hgt += dT * vup    
    
      rxData.m_pvt.vn  *= m_RTK.T[3][3];  // for small dT, this should be vn = vn    
      rxData.m_pvt.ve  *= m_RTK.T[4][4];  // for small dT, this should be ve = ve
      rxData.m_pvt.vup *= m_RTK.T[5][5];  // for small dT, this should be vup = vup

      rxData.m_pvt.clockOffset += m_RTK.T[6][7] * rxData.m_pvt.clockDrift;  // for small dT, this should be: clk += dT * clkrate
      rxData.m_pvt.clockDrift  *= m_RTK.T[7][7];  // for small dT, this should be clkdrift = clkdrift
    }
    else
    {
      // No change in states, with random walk 4 state model, T = Identity.
    }
    ////

    ////
    // predict the new state variance/covariance

    // It can be done this way in the code:
    // P = T * P * T.transpose() + Q;
    // but the following is more efficient
    if( !tmpMat.Copy( m_RTK.T ) )
    {
      GNSS_ERROR_MSG( "if( !tmpMat.Copy( T ) )" );
      return false;
    }
    if( !tmpMat.Inplace_Transpose() )
    {
      GNSS_ERROR_MSG( "if( !tmpMat.Inplace_Transpose() )" );
      return false;
    }
    if( !m_RTK.P.Inplace_PreMultiply( m_RTK.T ) )
    {
      GNSS_ERROR_MSG( "if( !m_RTK.P.Inplace_PreMultiply( T ) )" );
      return false;
    }
    if( !m_RTK.P.Inplace_PostMultiply( tmpMat ) )
    {
      GNSS_ERROR_MSG( "if( !m_RTK.P.Inplace_PostMultiply( tmpMat ) )" );
      return false;
    }

    if( !m_RTK.P.Inplace_Add( m_RTK.Q ) )
    {
      GNSS_ERROR_MSG( "if( !m_RTK.P.Inplace_Add( m_RTK.Q ) )" );
      return false;
    }
    PrintMatToDebug( "m_RTK.P", m_RTK.P, 3 );
    //
    ////

    if( m_FilterType == GNSS_FILTER_TYPE_RTK8 )
    {
      clkvar = m_RTK.P[6][6];
    }
    else
    {
      clkvar = m_RTK.P[3][3];
    }

    result = rxData.UpdatePositionAndRxClock(
      rxData.m_pvt,
      rxData.m_pvt.latitude,
      rxData.m_pvt.longitude,
      rxData.m_pvt.height,
      rxData.m_pvt.clockOffset,
      sqrt( m_RTK.P[0][0] ),
      sqrt( m_RTK.P[1][1] ),
      sqrt( m_RTK.P[2][2] ),
      sqrt( clkvar )
      );
    if( !result )
    {
      GNSS_ERROR_MSG( "rxData.UpdatePositionAndRxClock returned false." );
      return false;   
    }

    if( m_FilterType == GNSS_FILTER_TYPE_RTK8 )
    {
      result = rxData.UpdateVelocityAndClockDrift(
        rxData.m_pvt,
        rxData.m_pvt.vn,
        rxData.m_pvt.ve,
        rxData.m_pvt.vup,
        rxData.m_pvt.clockDrift,
        sqrt( m_RTK.P[3][3] ),
        sqrt( m_RTK.P[4][4] ),
        sqrt( m_RTK.P[5][5] ),
        sqrt( m_RTK.P[7][7] )
        );
      if( !result )
      {
        GNSS_ERROR_MSG( "rxData.UpdateVelocityAndClockDrift returned false." );
        return false;   
      }
    }
    
    return true;    
  }

  bool GNSS_Estimator::InitializeStateVarianceCovariance_EKF(
    Matrix &pos_P, //!< The variance covariance of the position and clock states from least squares, state order: latitude, longitude, height, clock ofset [4x4].
    Matrix &vel_P  //!< The variance covariance of the velocity and clock drift states from least squares, state order: latitude rate, longitude rate, height rate, clock drift [4x4].      
    )
  {
    unsigned i = 0;
    unsigned j = 0;

    if( pos_P.nrows() != 4 || pos_P.ncols() != 4 )
    {
      GNSS_ERROR_MSG( "if( pos_P.nrows() != 4 || pos_P.ncols() != 4 )" );
      return false;
    }

    if( m_FilterType == GNSS_FILTER_TYPE_EKF )
    {
      if( vel_P.nrows() != 4 || vel_P.ncols() != 4 )      
      {
        // velocity is unknown, so we'll indicate that with large variance.
        if( !vel_P.Identity( 4 ) )
        {
          GNSS_ERROR_MSG( "if( !vel_P.Identity( 4 ) )" );
          return false;
        }
        if( !vel_P.Inplace_AddScalar( 10000.0 ) )
        {
          GNSS_ERROR_MSG( "if( !vel_P.Inplace_AddScalar( 10000.0 ) )" );
          return false;
        }
      }
      if( !m_EKF.P.Resize(8,8) )
      {
        GNSS_ERROR_MSG( "if( !m_RTK.Resize(8,8) )" )
        return false;
      }
      for( i = 0; i < 4; i++ )
      {
        for( j = 0; j < 4; j++ )
        {
          m_EKF.P[i][j]     = pos_P[i][j];
          m_EKF.P[i+4][j+4] = vel_P[i][j];
        }
      }
    }
    else
    {
      GNSS_ERROR_MSG( "Unexpected. Only EKF supported." );
      return false;
    }
    return true;
  }


  bool GNSS_Estimator::Kalman_Update_EKF(
    GNSS_RxData *rxData,     //!< A pointer to the rover receiver data. This must be a valid pointer.
    GNSS_RxData *rxBaseData  //!< A pointer to the reference receiver data if available. NULL if not available.
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
    double stdev = 0; // A temporary double for standard deviation.

    Matrix Ht;
    Matrix tmpMat;
    Matrix PHt;
    Matrix I; // Identity

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

    ////
    // Perform operations specific to the rover.

    // Update the receiver time information (UTC and day of year)
    result = UpdateTime( rxData->m_pvt );
    if( !result )
    {
      GNSS_ERROR_MSG( "UpdateTime returned false." );
      return false;    
    }
    if( isDifferential )
    {
      result = UpdateTime( rxBaseData->m_pvt );
      if( !result )
      {
        GNSS_ERROR_MSG( "UpdateTime returned false." );
        return false;    
      }
    }

    result = DetermineSatellitePVT_GPSL1( rxData, rxBaseData, nrValidEph, false );
    if( !result )
    {
      GNSS_ERROR_MSG( "DetermineSatellitePVT_GPSL1 returned false." );
      return false;
    }
    result = DetermineAtmosphericCorrections_GPSL1( *rxData, false );
    if( !result )
    {
      GNSS_ERROR_MSG( "DetermineAtmosphericCorrections_GPSL1 returned false." );
      return false;
    }
    
    if( !DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1(
      *rxData, 
      rxData->m_pvt.nrPsrObsUsed, 
      rxData->m_pvt.nrPsrObsAvailable, 
      rxData->m_pvt.nrPsrObsRejected ) )
    {
      GNSS_ERROR_MSG( "DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1 returned false." );
      return false;
    }
    nrP = rxData->m_pvt.nrPsrObsUsed;

    result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( 
      *rxData, 
      rxData->m_pvt.nrDopplerObsUsed,
      rxData->m_pvt.nrDopplerObsAvailable,
      rxData->m_pvt.nrDopplerObsRejected
      );
    if( !result )
    {
      GNSS_ERROR_MSG( "DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1 returned false." );
      return false;
    }
    nrD = rxData->m_pvt.nrDopplerObsUsed;

    n = nrP + nrD;
    if( rxData->m_pvt.isPositionConstrained )
      n += 6;
    else if( rxData->m_pvt.isHeightConstrained )
      n += 2;

    if( isDifferential )
    {
      result = DetermineAtmosphericCorrections_GPSL1( *rxBaseData, false );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineAtmosphericCorrections_GPSL1 returned false." );
        return false;
      }
      
      if( !DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1(
        *rxBaseData, 
        rxBaseData->m_pvt.nrPsrObsUsed, 
        rxBaseData->m_pvt.nrPsrObsAvailable, 
        rxBaseData->m_pvt.nrPsrObsRejected ) )
      {
        GNSS_ERROR_MSG( "DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1 returned false." );
        return false;
      }
      nrP_base = rxBaseData->m_pvt.nrPsrObsUsed;

      
      result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( 
        *rxBaseData, 
        rxBaseData->m_pvt.nrDopplerObsUsed,
        rxBaseData->m_pvt.nrDopplerObsAvailable,
        rxBaseData->m_pvt.nrDopplerObsRejected
        );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1 returned false." );
        return false;
      }
      nrD_base = rxBaseData->m_pvt.nrDopplerObsUsed;

      // When in differential mode, only differntial measurements will be used
      result = DetermineBetweenReceiverDifferentialIndex( 
        rxData,
        rxBaseData,
        true 
        );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineBetweenReceiverDifferentialIndex returned false." );
        return false;
      }

      nrDifferentialPsr = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isActive &&
          rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
        {
          nrDifferentialPsr++;
        }
      }

      nrDifferentialDoppler = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( !rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
          rxData->m_ObsArray[i].flags.isDopplerUsedInSolution = false; // GDM - force Doppler and code coupled.

        if( rxData->m_ObsArray[i].flags.isActive &&          
          rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
        {
          nrDifferentialDoppler++;
        }
      }

      n = nrDifferentialPsr + nrDifferentialDoppler;
      if( rxData->m_pvt.isPositionConstrained )
        n += 6;
      else if( rxData->m_pvt.isHeightConstrained )
        n += 2;
    }

#ifdef GDM_UWB_RANGE_HACK
    if( rxData->m_UWB.isValidForThisEpoch )
      n++; // The UWB range appears as a fake psueodrange.
#endif

    // Build the design matrix, H.
    result = DetermineDesignMatrixElements_GPSL1_Psr( *rxData, false );
    if( !result )
    {
      GNSS_ERROR_MSG( "DetermineDesignMatrixElements_GPSL1_Psr returned false." );
      return false;
    }
    result = DetermineDesignMatrixElements_GPSL1_Doppler( *rxData, false );
    if( !result )
    {
      GNSS_ERROR_MSG( "DetermineDesignMatrixElements_GPSL1_Doppler returned false." );
      return false;
    }
    if( !m_EKF.H.Resize( n, 8 ) ) // n = nrPseudoranges + nrDopplers + nrConstraints.
    {
      GNSS_ERROR_MSG( "if( !m_EKF.H.Resize( n, 8 ) )" );
      return false;
    }
    j = 0;
    for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add pseudoranges
    {
      if( rxData->m_ObsArray[i].flags.isActive &&
        rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
      {
        m_EKF.H[j][0] = rxData->m_ObsArray[i].H_p[0];
        m_EKF.H[j][1] = rxData->m_ObsArray[i].H_p[1];
        m_EKF.H[j][2] = rxData->m_ObsArray[i].H_p[2];
        if( rxData->m_ObsArray[i].system == GNSS_UWBSystem )
          m_EKF.H[j][6] = 0.0;
        else
          m_EKF.H[j][6] = 1.0;
        j++;
      }
    }
    for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add Dopplers
    {
      if( rxData->m_ObsArray[i].flags.isActive &&
        rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
      {
        m_EKF.H[j][3] = rxData->m_ObsArray[i].H_v[0];
        m_EKF.H[j][4] = rxData->m_ObsArray[i].H_v[1];
        m_EKF.H[j][5] = rxData->m_ObsArray[i].H_v[2];
        m_EKF.H[j][7] = 1.0;
        j++;
      }
    }
    if( rxData->m_pvt.isPositionConstrained )
    {
      m_EKF.H[j][0] = 1.0;
      j++;
      m_EKF.H[j][1] = 1.0;
      j++;
      m_EKF.H[j][2] = 1.0;
      j++;

      m_EKF.H[j][3] = 1.0;
      j++;
      m_EKF.H[j][4] = 1.0;
      j++;
      m_EKF.H[j][5] = 1.0;
      j++;
    }
    else if( rxData->m_pvt.isHeightConstrained )
    {
      m_EKF.H[j][2] = 1.0;
      j++;
      m_EKF.H[j][5] = 1.0;
      j++;
    }
    PrintMatToDebug( "EKF H", m_EKF.H );


    // Form the misclosure vector.
    if( !m_EKF.w.Resize( n ) )
    {
      GNSS_ERROR_MSG( "if( !m_EKF.w.Resize( n ) )" );
      return false;    
    }
    result = DeterminePseudorangeMisclosures_GPSL1( rxData, rxBaseData, false );
    if( !result )
    {
      GNSS_ERROR_MSG( "DeterminePseudorangeMisclosures_GPSL1 returned false." );
      return false;
    }    
    result = DetermineDopplerMisclosures_GPSL1( rxData, rxBaseData, false );
    if( !result )
    {
      GNSS_ERROR_MSG( "DetermineDopplerMisclosures_GPSL1 returned false." );
      return false;
    }    
    j = 0;
    for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add pseudoranges
    {
      if( rxData->m_ObsArray[i].flags.isActive &&
        rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
      {
        m_EKF.w[j] = rxData->m_ObsArray[i].psr_misclosure;
        j++;
      }
    }
    for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add Dopplers
    {
      if( rxData->m_ObsArray[i].flags.isActive &&
        rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
      {
        m_EKF.w[j] = rxData->m_ObsArray[i].doppler_misclosure;
        j++;
      }
    }
    // Add constraints to w if any.      
    if( rxData->m_pvt.isPositionConstrained )
    {
      double w_lat = 0.0;
      double w_lon = 0.0;
      double w_hgt = 0.0;
      result = DeterminePositionConstraintMisclosures( rxData, w_lat, w_lon, w_hgt );
      if( !result )
      {
        GNSS_ERROR_MSG( "DeterminePositionConstraintMisclosures returned false." );
        return false;
      }
      m_EKF.w[j] = w_lat;
      j++;
      m_EKF.w[j] = w_lon;
      j++;
      m_EKF.w[j] = w_hgt;
      j++;
      m_EKF.w[j] = 0.0;
      j++;
      m_EKF.w[j] = 0.0;
      j++;
      m_EKF.w[j] = 0.0;
      j++;
    }
    else if( rxData->m_pvt.isHeightConstrained )
    {
      double w_hgt = 0.0;       
      result = DetermineHeightConstraintMisclosures( rxData, w_hgt );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineHeightConstraintMisclosures returned false." );
        return false;
      }
      m_EKF.w[j] = w_hgt;
      j++;
      m_EKF.w[j] = 0.0;
      j++;
    }
    PrintMatToDebug( "EKF w", m_EKF.w );

    // Form R, the combined measurement variance-covariance matrix.
    if( !m_EKF.R.Resize( n, n ) )
    {
      GNSS_ERROR_MSG( "if( !m_EKF.R.Resize( n, n ) )" );
      return false;
    }
    j = 0;
    for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add pseudoranges
    {
      if( rxData->m_ObsArray[i].flags.isActive &&
        rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
      {
        m_EKF.R[j][j] = rxData->m_ObsArray[i].stdev_psr*rxData->m_ObsArray[i].stdev_psr;
        j++;
      }
    }
    for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add Dopplers
    {
      if( rxData->m_ObsArray[i].flags.isActive &&
        rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
      {
        stdev = rxData->m_ObsArray[i].stdev_doppler * GPS_WAVELENGTHL1;
        m_EKF.R[j][j] = stdev*stdev;
        j++;
      }
    }
    // Deal with constraints.
    if( rxData->m_pvt.isPositionConstrained )
    {
      m_EKF.R[j][j] = rxData->m_pvt.std_lat*rxData->m_pvt.std_lat; 
      j++;
      m_EKF.R[j][j] = rxData->m_pvt.std_lon*rxData->m_pvt.std_lon; 
      j++;
      m_EKF.R[j][j] = rxData->m_pvt.std_hgt*rxData->m_pvt.std_hgt; 
      j++; 
      m_EKF.R[j][j] = 0.0; 
      j++;
      m_EKF.R[j][j] = 0.0; 
      j++;
      m_EKF.R[j][j] = 0.0;      
      j++;      
    }
    else if( rxData->m_pvt.isHeightConstrained )
    {
      m_EKF.R[j][j] = rxData->m_pvt.std_hgt*rxData->m_pvt.std_hgt; 
      j++;
      m_EKF.R[j][j] = 0.0;      
      j++;   
    }
    PrintMatToDebug( "EKF R", m_EKF.R, 2 );


    // Form H transposed.
    if( !Ht.Copy( m_EKF.H ) )
    {
      GNSS_ERROR_MSG( "if( !Ht.Copy( m_EKF.H ) )" );
      return false;
    }
    if( !Ht.Inplace_Transpose() )
    {    
      GNSS_ERROR_MSG( "if( !Ht.Inplace_Transpose() )" );
      return false;
    }

    PrintMatToDebug( "EKF P", m_EKF.P, 2 );

    // Compute the Kalman gain matrix.
    // K = P*Ht*(H*P*Ht+R)^-1
    // 1. PHt = P*Ht
    // 2. tmpMat = (H*PHt+R)^-1 
    // 3. K = PHt*tmpMat
    if( !PHt.Copy( m_EKF.P ) )
    {
      GNSS_ERROR_MSG( "if( !PHt.Copy( m_EKF.P ) )" );
      return false;
    }
    if( !PHt.Inplace_PostMultiply( Ht ) )
    {
      GNSS_ERROR_MSG( "if( !PHt.Inplace_PostMultiply( Ht ) )" );
      return false;
    }
    if( !tmpMat.Copy( m_EKF.H ) )
    {
      GNSS_ERROR_MSG( "if( !tmpMat.Copy( m_EKF.H ) )" );
      return false;
    }
    if( !tmpMat.Inplace_PostMultiply( PHt ) )
    {
      GNSS_ERROR_MSG( "if( !tmpMat.Inplace_PostMultiply( PHt ) )" );
      return false;
    }
    if( !tmpMat.Inplace_Add( m_EKF.R ) )
    {
      GNSS_ERROR_MSG( "if( !tmpMat.Inplace_Add( m_EKF.R ) )" );
      return false;
    }
    if( !tmpMat.Inplace_Invert() )
    {
      GNSS_ERROR_MSG( "if( !tmpMat.Inplace_Invert() )" );
      return false;    
    }
    if( !m_EKF.K.Copy( PHt ) )
    {
      GNSS_ERROR_MSG( "if( !m_EKF.K.Copy( PHt ) )" );
      return false;    
    }
    if( !m_EKF.K.Inplace_PostMultiply( tmpMat ) )
    {
      GNSS_ERROR_MSG( "if( !m_EKF.K.Inplace_PostMultiply( tmpMat ) )" );
      return false;    
    }
    
    // Compute the change in states due to the innovations (misclosures).
    if( !m_EKF.dx.Copy( m_EKF.K ) )
    {
      GNSS_ERROR_MSG( "if( !m_EKF.dx.Copy( m_EKF.K ) )" );
      return false;    
    }
    if( !m_EKF.dx.Inplace_PostMultiply( m_EKF.w ) )
    {
      GNSS_ERROR_MSG( "if( !m_EKF.dx.Inplace_PostMultiply( m_EKF.w ) )" );
      return false;    
    }
    
    // Compute the updated state variance-covariance matrix, P.
    if( !I.Identity(8) )
    {
      GNSS_ERROR_MSG( "if( !I.Identity(8) )" );    
      return false;
    }
    
    // P = (I - K*H)*P
    if( !tmpMat.Copy( m_EKF.K ) )
    {
      GNSS_ERROR_MSG( "if( !tmpMat.Copy( m_EKF.K ) )" );
      return false;
    }
    if( !tmpMat.Inplace_PostMultiply( m_EKF.H ) )
    {
      GNSS_ERROR_MSG( "if( !tmpMat.Inplace_PostMultiply( m_EKF.H ) )" );
      return false;
    }
    if( !I.Inplace_Subtract( tmpMat ) )
    {
      GNSS_ERROR_MSG( "if( !I.Inplace_Subtract( tmpMat ) )" );
      return false;
    }
    if( !m_EKF.P.Inplace_PreMultiply( I ) )
    {
      GNSS_ERROR_MSG( "if( !m_EKF.P.Inplace_PreMultiply( tmpMat ) )" );
      return false;
    }
    
    PrintMatToDebug( "EKF dx", m_EKF.dx, 3 );
    PrintMatToDebug( "EKF P", m_EKF.P, 2 );


    // Update the position and clock states
    // Update height first as it is need to reduce the corrections for lat and lon.
    hgt += m_EKF.dx[2];
    clk += m_EKF.dx[6];

    // The corrections for lat and lon, dx_p, must be converted to [rad] from [m].
    GEODESY_ComputePrimeVerticalRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84, 
      lat,
      &N );
    GEODESY_ComputeMeridianRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84, 
      lat,
      &M );

    lat += m_EKF.dx[0] / ( M + hgt );             // convert from meters to radians.
    lon += m_EKF.dx[1] / (( N + hgt )*cos(lat));  // convert from meters to radians.

    result = rxData->UpdatePositionAndRxClock(
      rxData->m_pvt,
      lat,
      lon,
      hgt,
      clk,
      sqrt(m_EKF.P[0][0]),
      sqrt(m_EKF.P[1][1]),
      sqrt(m_EKF.P[2][2]),
      sqrt(m_EKF.P[6][6])
      );
    if( !result )
    {
      GNSS_ERROR_MSG( "rxData->UpdatePositionAndRxClock returned false." );
      return false;
    }


    // Update the velocity and clock drift states.
    vn        += m_EKF.dx[3];
    ve        += m_EKF.dx[4];
    vup       += m_EKF.dx[5];
    clkdrift  += m_EKF.dx[7];

    result = rxData->UpdateVelocityAndClockDrift(
      rxData->m_pvt,
      vn,
      ve,
      vup,
      clkdrift,
      sqrt(m_EKF.P[3][3]),
      sqrt(m_EKF.P[4][4]),
      sqrt(m_EKF.P[5][5]),
      sqrt(m_EKF.P[7][7]) );
    if( !result )
    {
      GNSS_ERROR_MSG( "rxData->UpdateVelocityAndClockDrift returned false." );
      return false;
    }

    // Compute the DOP values.
    if( !ComputeDOP( rxData, false ) )
    {
      GNSS_ERROR_MSG( "ComputeDOP returned false." );
      return false;
    }

    return true;
  }

  bool GNSS_Estimator::Kalman_Update_RTK(
    GNSS_RxData *rxData,      //!< A pointer to the rover receiver data. This must be a valid pointer.
    GNSS_RxData *rxBaseData   //!< A pointer to the reference receiver data if available. NULL if not available.    
  )
  {
    bool result = false;
    unsigned index = 0;
    unsigned i = 0;
    unsigned j = 0;
    unsigned k = 0; 
    unsigned m = 0; // counter
    unsigned n = 0;             // The totla number of usable observations.
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
    bool hasAmbiguityChangeOccurred = false;
    bool isEightStateModel = false;

    struct stIndex
    {
      GNSS_enumMeasurementType type; // The observation type.
      unsigned intoRxData; // The index into observation array of the rxData object.
      int intoH;           // The index (row) of the design matrix corresponding to this observation. -1 if invalid. 

      stIndex() : type(GNSS_INVALID_MEASUREMENT),intoRxData(0),intoH(-1) {}      
    };

    stIndex smart_index[GNSS_RXDATA_MAX_OBS]; // An array of indices

    double lat = 0;
    double lon = 0;
    double hgt = 0;
    double clk = 0;
    double clkvar = 0; // The variance of the clk state.
    double vn = 0;
    double ve = 0;
    double vup = 0;
    double clkdrift = 0;
    double M = 0;   // The computed meridian radius of curvature [m].
    double N = 0;   // The computed prime vertical radius of curvature [m].
    double dlat = 0;  // A temporary value.
    double dlon = 0;  // A temporary value.
    double stdev = 0; // A temporary value.
    double innovation = 0; // A single innovation value.

    Matrix w_adr;
    Matrix tmpMat;
    Matrix I;

    Matrix amb; // A vector of the amibiguties for easy viewing while debugging.

    Matrix h;    // The i'th row of the design matrix, [1xu].
    Matrix ht;   // The transpose of the i'th row of the design matrix, [ux1].
    Matrix pht;  // pht = P ht, [ux1].
    Matrix C;    // C = (h P ht + R_{ii}), [1x1].
    Matrix k_i;  // The i'th kalman gain. k_i = pht/C
    Matrix D;    // D = k_i h * P.      

    double w_lat = 0.0; // constraint misclosure value
    double w_lon = 0.0; // constraint misclosure value
    double w_hgt = 0.0; // constraint misclosure value


    if( m_FilterType == GNSS_FILTER_TYPE_EKF || m_FilterType == GNSS_FILTER_TYPE_RTK8 )
    {
      isEightStateModel = true;
    }
    else if( m_FilterType == GNSS_FILTER_TYPE_RTK4 )
    {
      isEightStateModel = false;
    }
    else
    {
      GNSS_ERROR_MSG( "Only EKF, RTK4, and RTK8 are supported" );
      return false;
    }

    // Compensate the clock offset for pure 1 ms clock jumps.
    if( !DealWithClockJumps( rxData, rxBaseData ) )
    {
      GNSS_ERROR_MSG( "DealWithClockJumps returned false." );
      return false;
    }
   
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

    ////
    // Perform operations specific to the rover.

    // Update the receiver time information (UTC and day of year)
    result = UpdateTime( rxData->m_pvt );
    if( !result )
    {
      GNSS_ERROR_MSG( "UpdateTime returned false." );
      return false;
    }

    if( isDifferential )
    {
      result = UpdateTime( rxBaseData->m_pvt );
      if( !result )
      {
        GNSS_ERROR_MSG( "UpdateTime returned false." );
        return false;
      }
    }

    result = DetermineSatellitePVT_GPSL1( rxData, rxBaseData, nrValidEph, false );
    if( !result )
    {
      GNSS_ERROR_MSG( "DetermineSatellitePVT_GPSL1 returned false." );
      return false;
    }
    result = DetermineAtmosphericCorrections_GPSL1( *rxData, false );
    if( !result )
    {
      GNSS_ERROR_MSG( "DetermineAtmosphericCorrections_GPSL1 returned false." );
      return false;
    }

    if( !DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1(
      *rxData, 
      rxData->m_pvt.nrPsrObsUsed, 
      rxData->m_pvt.nrPsrObsAvailable, 
      rxData->m_pvt.nrPsrObsRejected ) )
    {
      GNSS_ERROR_MSG( "DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1 returned false." );
      return false;
    }
    nrP = rxData->m_pvt.nrPsrObsUsed;

    if( isEightStateModel )
    {
      result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( 
        *rxData, 
        rxData->m_pvt.nrDopplerObsUsed,
        rxData->m_pvt.nrDopplerObsAvailable,
        rxData->m_pvt.nrDopplerObsRejected
        );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1 returned false." );
        return false;
      }
      nrD = rxData->m_pvt.nrDopplerObsUsed;
    }
    if( m_FilterType == GNSS_FILTER_TYPE_RTK4 || m_FilterType == GNSS_FILTER_TYPE_RTK8 )
    {
      result = DetermineUsableAdrMeasurements_GPSL1( *rxData, nrUsableAdr );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineUsableAdrMeasurements_GPSL1 returned false." );
        return false;
      }
    }

    if( isDifferential )
    {
      result = DetermineAtmosphericCorrections_GPSL1( *rxBaseData, false );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineAtmosphericCorrections_GPSL1 returned false." );
        return false;
      }

      if( !DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1(
        *rxBaseData, 
        rxBaseData->m_pvt.nrPsrObsUsed, 
        rxBaseData->m_pvt.nrPsrObsAvailable, 
        rxBaseData->m_pvt.nrPsrObsRejected ) )
      {
        GNSS_ERROR_MSG( "DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1 returned false." );
        return false;
      }
      nrP_base = rxBaseData->m_pvt.nrPsrObsUsed;

      if( isEightStateModel )
      {
        result = DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( 
          *rxBaseData, 
          rxBaseData->m_pvt.nrDopplerObsUsed,
          rxBaseData->m_pvt.nrDopplerObsAvailable,
          rxBaseData->m_pvt.nrDopplerObsRejected
          );
        if( !result )
        {
          GNSS_ERROR_MSG( "DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1 returned false." );
          return false;
        }
        nrD_base = rxBaseData->m_pvt.nrDopplerObsUsed;
      }
      if( m_FilterType == GNSS_FILTER_TYPE_RTK4 || m_FilterType == GNSS_FILTER_TYPE_RTK8 )
      {
        result = DetermineUsableAdrMeasurements_GPSL1( *rxBaseData, nrUsableAdr_base );
        if( !result )
        {
          GNSS_ERROR_MSG( "DetermineUsableAdrMeasurements_GPSL1 returned false." );
          return false;
        }
      }

      // When in differential mode, only differntial measurements will be used
      result = DetermineBetweenReceiverDifferentialIndex(
        rxData,
        rxBaseData,
        true
        );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineBetweenReceiverDifferentialIndex returned false." );
        return false;
      }

      nrDifferentialPsr = 0;
      for( i = 0; i < rxData->m_nrValidObs; i++ )
      {
        if( rxData->m_ObsArray[i].flags.isActive &&          
          rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
        {
          nrDifferentialPsr++;
        }
      }
      
      nrDifferentialDoppler = 0;        
      if( isEightStateModel )
      {
        for( i = 0; i < rxData->m_nrValidObs; i++ )
        {
          if( !rxData->m_ObsArray[i].flags.isPsrUsedInSolution ) // GDM_HACK ensure valid Doppler's here for now.
            rxData->m_ObsArray[i].flags.isDopplerUsedInSolution = false;
  
          if( rxData->m_ObsArray[i].flags.isActive &&                    
            rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
          {
            nrDifferentialDoppler++;
          }
        }
      }

      nrDifferentialAdr = 0;
      if( m_FilterType == GNSS_FILTER_TYPE_RTK4 || m_FilterType == GNSS_FILTER_TYPE_RTK8 )
      {
        for( i = 0; i < rxData->m_nrValidObs; i++ )
        {
          if( rxData->m_ObsArray[i].flags.isActive &&                    
            rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
          {
            nrDifferentialAdr++;
          }
        }
      }
    }
    else
    {
      GNSS_ERROR_MSG( "Only differential mode is supported." );
      return false;
    }

    n = nrDifferentialPsr + nrDifferentialDoppler + nrDifferentialAdr;

    if( isEightStateModel )
    {     
      if( rxData->m_pvt.isPositionConstrained )
      {
        if( isEightStateModel )
        {     
          n+=6;
        }
        else
        {
          n+=3;
        }
      }
      else if( rxData->m_pvt.isHeightConstrained )
      {
        if( isEightStateModel )
        {            
          n+=2;
        }
        else
        {
          n+=1;
        }
      }
    }

#ifdef GDM_UWB_RANGE_HACK
    if( rxData->m_UWB.isValidForThisEpoch )
      n++; // The observation appears as a fake pseudorange.
#endif

    if( m_FilterType == GNSS_FILTER_TYPE_RTK4 || m_FilterType == GNSS_FILTER_TYPE_RTK8 )
    {
      // Determine if there are any changes to the ambiguities being estimated.
      result = DetermineAmbiguitiesChanges(
        rxData,
        rxBaseData,
        m_RTK.P,
        isEightStateModel,
        hasAmbiguityChangeOccurred );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineAmbiguitiesChanges returned false." );
        return false;
      }

      if( hasAmbiguityChangeOccurred )
        int gg=99; // place breakpoint here to debug issues related to changes in ambiguities.
    }


    // Form smart_index 
    j = 0;
    // Deal with constraints first.
    if( rxData->m_pvt.isPositionConstrained )
    {
      smart_index[j].type = GNSS_LAT_CONSTRAINT;
      smart_index[j].intoRxData = 0; // GDM_TODO, use -1 here?
      smart_index[j].intoH = j;
      j++;
      smart_index[j].type = GNSS_LON_CONSTRAINT;
      smart_index[j].intoRxData = 0; // GDM_TODO, use -1 here?
      smart_index[j].intoH = j;
      j++;
      smart_index[j].type = GNSS_HGT_CONSTRAINT;
      smart_index[j].intoH = j;
      j++;
      if( isEightStateModel )
      {
        smart_index[j].type = GNSS_VN_CONSTRAINT;
        smart_index[j].intoRxData = 0; // GDM_TODO, use -1 here?
        smart_index[j].intoH = j;
        j++;
        smart_index[j].type = GNSS_VE_CONSTRAINT;
        smart_index[j].intoRxData = 0; // GDM_TODO, use -1 here?
        smart_index[j].intoH = j;
        j++;
        smart_index[j].type = GNSS_VUP_CONSTRAINT;
        smart_index[j].intoRxData = 0; // GDM_TODO, use -1 here?
        smart_index[j].intoH = j;
        j++;
      }
    }
    else if( rxData->m_pvt.isHeightConstrained )
    {
      smart_index[j].type = GNSS_HGT_CONSTRAINT;
      smart_index[j].intoRxData = 0; // GDM_TODO, use -1 here?
      smart_index[j].intoH = j;
      j++;
      if( isEightStateModel )
      {      
        smart_index[j].type = GNSS_VUP_CONSTRAINT;
        smart_index[j].intoRxData = 0; // GDM_TODO, use -1 here?
        smart_index[j].intoH = j;
        j++;
      }
    }
    for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add pseudoranges
    {
      if( rxData->m_ObsArray[i].flags.isActive &&
        rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
      {
        smart_index[j].type = GNSS_PSR_MEASUREMENT;
        smart_index[j].intoRxData = i;
        smart_index[j].intoH = j;
        j++;
      }
    }
    if( isEightStateModel )
    {                    
      for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add Dopplers
      {
        if( rxData->m_ObsArray[i].flags.isActive &&
          rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
        {
          smart_index[j].type = GNSS_DOPPLER_MEASUREMENT;
          smart_index[j].intoRxData = i;
          smart_index[j].intoH = j;
          j++;
        }
      }
    }
    if( m_FilterType == GNSS_FILTER_TYPE_RTK4 || m_FilterType == GNSS_FILTER_TYPE_RTK8 )
    {
      for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add ADR
      {
        if( rxData->m_ObsArray[i].flags.isActive &&
          rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
        {
          smart_index[j].type = GNSS_ADR_MEASUREMENT;
          smart_index[j].intoRxData = i;
          smart_index[j].intoH = j;
          j++;
        }
      } 
    }
    
    // Form r, the combined measurement variance-covariance matrix diagonal.
    if( !m_RTK.r.Resize( n ) )
    {
      GNSS_ERROR_MSG( "if( !m_RTK.r.Resize( n ) )" );
      return false;
    }
    j = 0;
    // Deal with constraints first.
    if( rxData->m_pvt.isPositionConstrained )
    {
      m_RTK.r[j] = rxData->m_pvt.std_lat*rxData->m_pvt.std_lat; 
      j++;
      m_RTK.r[j] = rxData->m_pvt.std_lon*rxData->m_pvt.std_lon; 
      j++;
      m_RTK.r[j] = rxData->m_pvt.std_hgt*rxData->m_pvt.std_hgt; 
      j++; 
      if( isEightStateModel )
      {                    
        m_RTK.r[j] = 1E-5; 
        j++;
        m_RTK.r[j] = 1E-5; 
        j++;
        m_RTK.r[j] = 1E-5;      
        j++;      
      }
    }
    else if( rxData->m_pvt.isHeightConstrained )
    {
      m_RTK.r[j] = rxData->m_pvt.std_hgt*rxData->m_pvt.std_hgt; 
      j++;
      if( isEightStateModel )
      {                          
        m_RTK.r[j] = 1E-5;
        j++;   
      }
    }
    for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add pseudoranges
    {
      if( rxData->m_ObsArray[i].flags.isActive &&
        rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
      {
        m_RTK.r[j] = rxData->m_ObsArray[i].stdev_psr*rxData->m_ObsArray[i].stdev_psr;
        j++;
      }
    }
    if( isEightStateModel )
    {                    
      for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add Dopplers
      {
        if( rxData->m_ObsArray[i].flags.isActive &&
          rxData->m_ObsArray[i].flags.isDopplerUsedInSolution )
        {
          stdev = rxData->m_ObsArray[i].stdev_doppler * GPS_WAVELENGTHL1;
          m_RTK.r[j] = stdev*stdev;
          j++;
        }
      }
    }
    if( m_FilterType == GNSS_FILTER_TYPE_RTK4 || m_FilterType == GNSS_FILTER_TYPE_RTK8 )
    {    
      for( i = 0; i < rxData->m_nrValidObs; i++ ) // Add ADR
      {
        if( rxData->m_ObsArray[i].flags.isActive &&
          rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
        {
          stdev = rxData->m_ObsArray[i].stdev_adr * GPS_WAVELENGTHL1;
          double elevation = rxData->m_ObsArray[i].satellite.elevation*RAD2DEG;

          // GDM_HACK scaling the stdev for adr when below 30 degrees elevation.
          // This really shouldn't be handled here.
          if( elevation < 5.0 )
          {
            stdev *= 10.0;
          }
          else if( elevation < 10.0 )
          {
            stdev *= 5.0;
          }
          else if( elevation < 20.0 )
          {
            stdev *= 3.0;
          }

          m_RTK.r[j] = stdev*stdev;
          j++;
        }
      }
    }
    PrintMatToDebug( "RTK r", m_RTK.r );
    

    // The corrections for lat and lon, dx_p, must be converted to [rad] from [m].
    GEODESY_ComputePrimeVerticalRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      lat,
      &N );
    GEODESY_ComputeMeridianRadiusOfCurvature(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      lat,
      &M );
    dlat = M + hgt;
    dlon = (N + hgt) * cos(lat);

    
    if( nrDifferentialAdr > 0 )
      amb.Resize(nrDifferentialAdr,2);

    if( isEightStateModel )
    {
      u = 8 + nrDifferentialAdr;
    }
    else
    {
      u = 4 + nrDifferentialAdr;
    }

    if( !m_RTK.H.Resize( n, u ) )
    {
      GNSS_ERROR_MSG( "if( !m_RTK.H.Resize( n, u ) )" );
      return false;    
    }

    if( !m_RTK.w.Resize( n ) )
    {
      GNSS_ERROR_MSG( "if( !m_RTK.w.Resize( n ) )" );
      return false;    
    }

    if( !h.Resize( 1, u ) )
    {
      GNSS_ERROR_MSG( "if( !h.Resize( 1, u ) )" );
      return false;    
    }
    if( !ht.Resize( u, 1 ) )
    {
      GNSS_ERROR_MSG( "if( !ht.Resize( u, 1 ) )" );
      return false;    
    }
    
    // Determine constraint misclosures if applicable.
    if( rxData->m_pvt.isPositionConstrained )
    {
      result = DeterminePositionConstraintMisclosures( rxData, w_lat, w_lon, w_hgt );
      if( !result )
      {
        GNSS_ERROR_MSG( "DeterminePositionConstraintMisclosures returned false." );
        return false;
      }
    }
    else if( rxData->m_pvt.isHeightConstrained )
    {
      result = DetermineHeightConstraintMisclosures( rxData, w_hgt );
      if( !result )
      {
        GNSS_ERROR_MSG( "DetermineHeightConstraintMisclosures returned false." );
        return false;
      }
    }

    PrintMatToDebug( "RTK T", m_RTK.T, 3 );
    PrintMatToDebug( "RTK Q", m_RTK.Q, 3 );
    PrintMatToDebug( "RTK P", m_RTK.P, 3 );
    

    // Now the sequential measurement update section
    // For each measurement
    for( index = 0; index < n; index++ )
    {
      switch( smart_index[index].type )
      {
      case GNSS_LAT_CONSTRAINT:
        {
          m_RTK.H[index][0] = 1.0;
          m_RTK.w[index] = w_lat;
          break;
        }
      case GNSS_LON_CONSTRAINT:
        {
          m_RTK.H[index][1] = 1.0;
          m_RTK.w[index] = w_lon;
          break;
        }
      case GNSS_HGT_CONSTRAINT:
        {
          m_RTK.H[index][2] = 1.0;
          m_RTK.w[index] = w_hgt;
          break;
        }
      case GNSS_VN_CONSTRAINT:
        {
          if( !isEightStateModel )
          { 
            GNSS_ERROR_MSG( "Cannot constrain velocity with the 4 state RTK model." );
            return false;
          }            
          m_RTK.H[index][3] = 1.0;
          m_RTK.w[index] = 0;
          break;
        }
      case GNSS_VE_CONSTRAINT:
        {
          if( !isEightStateModel )
          { 
            GNSS_ERROR_MSG( "Cannot constrain velocity with the 4 state RTK model." );
            return false;
          }            
          m_RTK.H[index][4] = 1.0;
          m_RTK.w[index] = 0;
          break;
        }
      case GNSS_VUP_CONSTRAINT:
        {
          if( !isEightStateModel )
          { 
            GNSS_ERROR_MSG( "Cannot constrain velocity with the 4 state RTK model." );
            return false;
          }            
          m_RTK.H[index][5] = 1.0;
          m_RTK.w[index] = 0;
          break;
        }
      case GNSS_PSR_MEASUREMENT:
        {
          if( rxData->m_ObsArray[smart_index[index].intoRxData].flags.isActive &&
            rxData->m_ObsArray[smart_index[index].intoRxData].flags.isPsrUsedInSolution )
          {
            result = DetermineDesignMatrixElement_GPSL1_Psr( *rxData, smart_index[index].intoRxData, false );
            if( !result )
            {
              GNSS_ERROR_MSG( "DetermineDesignMatrixElement_GPSL1_Psr returned false." );
              return false;
            }

            m_RTK.H[index][0] = rxData->m_ObsArray[smart_index[index].intoRxData].H_p[0];
            m_RTK.H[index][1] = rxData->m_ObsArray[smart_index[index].intoRxData].H_p[1];
            m_RTK.H[index][2] = rxData->m_ObsArray[smart_index[index].intoRxData].H_p[2];

            if( isEightStateModel )
            {                        
              if( rxData->m_ObsArray[smart_index[index].intoRxData].system == GNSS_UWBSystem )
                m_RTK.H[index][6] = 0.0;
              else
                m_RTK.H[index][6] = 1.0;
            }
            else
            {
              if( rxData->m_ObsArray[smart_index[index].intoRxData].system == GNSS_UWBSystem )
                m_RTK.H[index][3] = 0.0;
              else
                m_RTK.H[index][3] = 1.0;
            }

            result = DeterminePseudorangeMisclosure_GPSL1( rxData, smart_index[index].intoRxData, rxBaseData, false );
            if( !result )
            {
              GNSS_ERROR_MSG( "DeterminePseudorangeMisclosure_GPSL1 returned false." );
              return false;      
            }

            m_RTK.w[index] = rxData->m_ObsArray[smart_index[index].intoRxData].psr_misclosure;
          }        
          else
          {
            GNSS_ERROR_MSG( "unexpected" );
            return false;
          }
          break;
        }
      case GNSS_DOPPLER_MEASUREMENT:
        {
          if( !isEightStateModel )
          { 
            GNSS_ERROR_MSG( "Doppler measurements cannot be used with the 4 state RTK model." );
            return false;
          }
          
          if( rxData->m_ObsArray[smart_index[index].intoRxData].flags.isActive &&
            rxData->m_ObsArray[smart_index[index].intoRxData].flags.isDopplerUsedInSolution )
          {
            result = DetermineDesignMatrixElement_GPSL1_Doppler( *rxData, smart_index[index].intoRxData, false );
            if( !result )
            {
              GNSS_ERROR_MSG( "DetermineDesignMatrixElement_GPSL1_Doppler returned false." );
              return false;      
            }

            m_RTK.H[index][3] = rxData->m_ObsArray[smart_index[index].intoRxData].H_v[0];
            m_RTK.H[index][4] = rxData->m_ObsArray[smart_index[index].intoRxData].H_v[1];
            m_RTK.H[index][5] = rxData->m_ObsArray[smart_index[index].intoRxData].H_v[2];
            m_RTK.H[index][7] = 1.0;        

            result = DetermineDopplerMisclosure_GPSL1( rxData, smart_index[index].intoRxData, rxBaseData, false );
            if( !result )
            {
              GNSS_ERROR_MSG( "DetermineDopplerMisclosure_GPSL1 returned false." );
              return false;      
            }

            m_RTK.w[index] = rxData->m_ObsArray[smart_index[index].intoRxData].doppler_misclosure;
          }
          else
          {
            GNSS_ERROR_MSG( "unexpected" );
            return false;
          }
          break;
        }
      case GNSS_ADR_MEASUREMENT:
        {
          if( !(m_FilterType == GNSS_FILTER_TYPE_RTK4 || m_FilterType == GNSS_FILTER_TYPE_RTK8) )
          {
            GNSS_ERROR_MSG( "Only RTK4 and RTK8 filter modes allowed." );
            return false;            
          }
          if( rxData->m_ObsArray[smart_index[index].intoRxData].flags.isActive &&
            rxData->m_ObsArray[smart_index[index].intoRxData].flags.isAdrUsedInSolution )
          {
            // GDM - It is important that the design matrix row is recomputed for the ADR measurement 
            // even though it is very close to the values in the design matrix row for the associated 
            // PSR measurement.
            result = DetermineDesignMatrixElement_GPSL1_Adr( *rxData, smart_index[index].intoRxData );
            if( !result )
            {
              GNSS_ERROR_MSG( "DetermineDesignMatrixElement_GPSL1_Psr returned false." );
              return false;      
            }

            m_RTK.H[index][0] = rxData->m_ObsArray[smart_index[index].intoRxData].H_a[0];
            m_RTK.H[index][1] = rxData->m_ObsArray[smart_index[index].intoRxData].H_a[1];
            m_RTK.H[index][2] = rxData->m_ObsArray[smart_index[index].intoRxData].H_a[2];
            if( isEightStateModel )
            {           
              m_RTK.H[index][6] = 1.0;
            }
            else
            {
              m_RTK.H[index][3] = 1.0;
            }
            m_RTK.H[index][rxData->m_ObsArray[smart_index[index].intoRxData].index_ambiguity_state] = 1.0;          

            result = DetermineSingleDifferenceADR_Misclosure_GPSL1( rxData, smart_index[index].intoRxData, rxBaseData );
            if( !result )
            {
              GNSS_ERROR_MSG( "DetermineSingleDifferenceADR_Misclosure_GPSL1 returned false." );
              return false;      
            }

            m_RTK.w[index] = rxData->m_ObsArray[smart_index[index].intoRxData].adr_misclosure;
          }
          else
          {
            GNSS_ERROR_MSG( "unexpected" );
            return false;
          }
          break;
        }
      default:
        {
          GNSS_ERROR_MSG( "Default case unexpected." );
          return false;
          break;
        }
      }

      PrintMatToDebug( "RTK H", m_RTK.H, 3 );
      PrintMatToDebug( "RTK w", m_RTK.w, 3 );

      // Copy out the row of the design matrix.
      for( j = 0; j < u; j++ )
      {
        h[0][j] = m_RTK.H[index][j];
        ht[j][0] = m_RTK.H[index][j];
      }
      //PrintMatToDebug( "h", h );

      // Compute pht
      if( !pht.Copy( m_RTK.P ) )
      {
        GNSS_ERROR_MSG( "if( !pht.Copy( m_RTK.P ) )" );
        return false;
      }
      if( !pht.Inplace_PostMultiply( ht ) )
      {
        GNSS_ERROR_MSG( "if( !pht.Inplace_PostMultiply( ht ) )" );
        return false;
      }
      
      // Compute C = (h P h^T + R_{ii})
      if( !C.Copy( h ) )
      {
        GNSS_ERROR_MSG( "if( !C.Copy( h ) )" );
        return false;
      }
      if( !C.Inplace_PostMultiply( pht ) )
      {
        GNSS_ERROR_MSG( "if( !C.Inplace_PostMultiply( pht ) )" );
        return false;
      }
      if( !C.Inplace_AddScalar( m_RTK.r[index] ) )
      {
        GNSS_ERROR_MSG( "if( !C.Inplace_AddScalar( m_RTK.r[index] ) )" );
        return false;
      }
      
      // Compute k_i
      double c0 = C[0];
      if( c0 == 0.0 )
      {
        GNSS_ERROR_MSG( "Unexpected divide by zero." );
        return false;
      }
      if( !k_i.Copy( pht ) )
      {
        GNSS_ERROR_MSG( "if( !k_i.Copy( pht ) )" );
        return false;
      }
      if( !k_i.Inplace_DivideScalar( c0 ) )
      {
        GNSS_ERROR_MSG( "if( !k_i.Inplace_DivideScalar( c0 ) )" );
        return false;
      }
      
      //PrintMatToDebug( "k_i", k_i );

      // Update the state variance-coveriance;
      if( !D.Copy( k_i ) )
      {
        GNSS_ERROR_MSG( "if( !D.Copy( k_i ) )" );
        return false;
      }
      if( !D.Inplace_PostMultiply( h ) )
      {
        GNSS_ERROR_MSG( "if( !D.Inplace_PostMultiply( h ) )" );
        return false;
      }
      if( !D.Inplace_PostMultiply( m_RTK.P ) )
      {
        GNSS_ERROR_MSG( "if( !D.Inplace_PostMultiply( m_RTK.P ) )" );
        return false;
      }
      if( !m_RTK.P.Inplace_Subtract( D ) )
      {
        GNSS_ERROR_MSG( "if( !m_RTK.P.Inplace_Subtract( D ) )" );
        return false;
      }      

      PrintMatToDebug( "RTK P", m_RTK.P, 2 );
      
      innovation = m_RTK.w[index];
      if( !k_i.Inplace_MultiplyScalar( innovation ) )
      {
        GNSS_ERROR_MSG( "if( !k_i.Inplace_MultiplyScalar( innovation ) )" );
        return false;
      }
      if( !m_RTK.dx.Copy( k_i ) )
      {
        GNSS_ERROR_MSG( "if( !m_RTK.dx.Copy( k_i ) )" );
        return false;
      }
      PrintMatToDebug( "RTK dx", m_RTK.dx, 3 );

      // Update the position and clock states
      // Update height first as it is need to reduce the corrections for lat and lon.
      hgt += m_RTK.dx[2];
      if( isEightStateModel )
      {
        clk += m_RTK.dx[6];
      }
      else
      {
        clk += m_RTK.dx[3];
      }

      // The corrections for lat and lon, dx_p, must be converted to [rad] from [m].
      lat += m_RTK.dx[0] / dlat;  // convert from meters to radians.
      lon += m_RTK.dx[1] / dlon;  // convert from meters to radians.

      // Check for negative variance in P.
      for( i = 0; i < m_RTK.P.nrows(); i++ )
      {
        if( m_RTK.P[i][i] < 0.0 )
        {
          GNSS_ERROR_MSG( "if( m_RTK.P[i][i] < 0.0 )" );
          return false;
        }
      }

      if( isEightStateModel )
      {
        clkvar = sqrt(m_RTK.P[6][6]);
      }
      else
      {
        clkvar = sqrt(m_RTK.P[3][3]);
      }
       
      result = rxData->UpdatePositionAndRxClock(
        rxData->m_pvt,
        lat,
        lon,
        hgt,
        clk,
        sqrt(m_RTK.P[0][0]),
        sqrt(m_RTK.P[1][1]),
        sqrt(m_RTK.P[2][2]),
        sqrt( clkvar )
        );
      if( !result )
      {
        GNSS_ERROR_MSG( "rxData->UpdatePositionAndRxClock returned false." );
        return false;
      }

      if( isEightStateModel )
      {
        // Update the velocity and clock drift states.
        vn        += m_RTK.dx[3];
        ve        += m_RTK.dx[4];
        vup       += m_RTK.dx[5];
        clkdrift  += m_RTK.dx[7];
      
        result = rxData->UpdateVelocityAndClockDrift(
          rxData->m_pvt,
          vn,
          ve,
          vup,
          clkdrift,
          sqrt(m_RTK.P[3][3]),
          sqrt(m_RTK.P[4][4]),
          sqrt(m_RTK.P[5][5]),
          sqrt(m_RTK.P[7][7]) );
        if( !result )
        {
          GNSS_ERROR_MSG( "rxData->UpdateVelocityAndClockDrift returned false." );
          return false;
        }
      }

      if( m_FilterType == GNSS_FILTER_TYPE_RTK4 || m_FilterType == GNSS_FILTER_TYPE_RTK8 )
      {
        // Update the ambiguities
        j = 0;
        for( i = 0; i < rxData->m_nrValidObs; i++ )
        {
          if( rxData->m_ObsArray[i].flags.isActive && rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
          {
            int state_index = rxData->m_ObsArray[i].index_ambiguity_state;
            if( state_index < 0 )
            {
              GNSS_ERROR_MSG( "if( state_index < 0 )" );
              return false;
            }
            double delta_amb = m_RTK.dx[state_index];
            rxData->m_ObsArray[i].ambiguity += delta_amb;
            amb[j][0] = rxData->m_ObsArray[i].id;
            amb[j][1] = rxData->m_ObsArray[i].ambiguity;
            j++;
          }
        }
        PrintMatToDebug( "amb", amb );
      }
    }

    // All measurments have been including in the update!

    // Compute the DOP values.
    if( !ComputeDOP( rxData, false ) )
    {
      GNSS_ERROR_MSG( "ComputeDOP returned false." );
      return false;
    }

#ifdef DEBUG_THE_ESTIMATOR
    char supermsg[8192];
    unsigned nrBytesInBuffer = 0;
    rxData->Debug_WriteSuperMsg80CharsWide( 
      supermsg,
      8192,
      rxData->m_datum_pvt.latitude,
      rxData->m_datum_pvt.longitude,
      rxData->m_datum_pvt.height,
      nrBytesInBuffer );
    printf( supermsg );
#endif
   
    return true;
  }


  bool GNSS_Estimator::ComputeDOP( 
    GNSS_RxData *rxData,       //!< The receiver data.
    const bool isLeastSquares  //!< A boolean to indicate if the rover position and velocity values are from least squares rxData->m_pvt_lsq or from rxData->m_pvt.
    )      
  {
    unsigned i = 0;
    unsigned j = 0;
    unsigned nrP = 0;
    unsigned n = 0;

    Matrix H;
    Matrix Q;    

    if( rxData == NULL )
    {
      GNSS_ERROR_MSG( "if( rxData == NULL )" );
      return false;
    }
    
    // Compute DOP
    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( rxData->m_ObsArray[i].flags.isActive && rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
        nrP++;
    }
    n = nrP;
    if( rxData->m_pvt.isPositionConstrained )
    {
      n+=3;
    }    
    else if( rxData->m_pvt.isHeightConstrained )
    {
      n+=1;
    }
    H.Redim( n, 4 );
    j = 0;
    for( i = 0; i < rxData->m_nrValidObs; i++ ) 
    {
      if( rxData->m_ObsArray[i].flags.isActive && rxData->m_ObsArray[i].flags.isPsrUsedInSolution )
      {
        H[j][0] = rxData->m_ObsArray[i].H_p[0];
        H[j][1] = rxData->m_ObsArray[i].H_p[1];
        H[j][2] = rxData->m_ObsArray[i].H_p[2];
        if( rxData->m_ObsArray[i].system == GNSS_UWBSystem )
          H[j][3] = 0.0;
        else
          H[j][3] = 1.0;
        j++;
      }
    }
    if( rxData->m_pvt.isPositionConstrained )
    {
      H[j][0] = 1.0;
      j++;
      H[j][1] = 1.0;
      j++;
      H[j][2] = 1.0;
      j++;
    }    
    else if( rxData->m_pvt.isHeightConstrained )
    {
      H[j][2] = 1.0;
      j++;
    }
    Q = H;
    Q.Inplace_Transpose();
    Q.Inplace_PostMultiply( H );
    if( !Q.Inplace_Invert() )
    {
      GNSS_ERROR_MSG( "if( !Q.Inplace_Invert() )" );
      return false;
    }
    
    if( isLeastSquares )
    {
      rxData->m_pvt_lsq.dop.ndop = static_cast<float>( sqrt( Q[0][0] ) );
      rxData->m_pvt_lsq.dop.edop = static_cast<float>( sqrt( Q[1][1] ) );
      rxData->m_pvt_lsq.dop.vdop = static_cast<float>( sqrt( Q[2][2] ) );
      rxData->m_pvt_lsq.dop.tdop = static_cast<float>( sqrt( Q[3][3] ) );
      rxData->m_pvt_lsq.dop.hdop = rxData->m_pvt_lsq.dop.ndop * rxData->m_pvt_lsq.dop.ndop + rxData->m_pvt_lsq.dop.edop * rxData->m_pvt_lsq.dop.edop;
      rxData->m_pvt_lsq.dop.pdop = rxData->m_pvt_lsq.dop.hdop + rxData->m_pvt_lsq.dop.vdop * rxData->m_pvt_lsq.dop.vdop;
      rxData->m_pvt_lsq.dop.gdop = rxData->m_pvt_lsq.dop.pdop + rxData->m_pvt_lsq.dop.tdop * rxData->m_pvt_lsq.dop.tdop;
      rxData->m_pvt_lsq.dop.hdop = sqrt(rxData->m_pvt_lsq.dop.hdop);
      rxData->m_pvt_lsq.dop.pdop = sqrt(rxData->m_pvt_lsq.dop.pdop);
      rxData->m_pvt_lsq.dop.gdop = sqrt(rxData->m_pvt_lsq.dop.gdop);
    }
    else
    {
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
    }    
    return true;
  }


  bool GNSS_Estimator::PrintMatToDebug( const char *name, Matrix& M, const unsigned precision )
  {
#ifdef DEBUG_THE_ESTIMATOR
    FILE* fid = NULL;    
    char buffer[8192];

    if( name == NULL )
      return false;
    if( !M.PrintToBuffer( buffer, 8192, precision ) )
      return 1;
    printf( "%s = \n%s\n", name, buffer );

    if( m_debug == NULL )
    {
      m_debug = fopen( "debug.txt", "w" );
      if( !m_debug )
      {
        return false;
      }
      fprintf( m_debug, "%s = \n%s\n", name, buffer );      
      fflush( m_debug );
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
    const bool isEightStateModel, //!< A boolean indicating if the velocity and clock drift states are included.
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

    double sd_adr_measured = 0;
    double sd_psr_measured = 0;
    double sd_dif = 0;
                  
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
        GNSS_ERROR_MSG( "if( rows == NULL )" );
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
        GNSS_ERROR_MSG( "if( !P.RemoveRowsAndColumns( nrows, rows, nrows, rows ) )" );
        return false;
      }
      //P.Print( "P.after.txt", 10 );

      delete [] rows;      
    }

    if( remove_list.size() > 0 )
    {
      // The state indices are in order of the ambiguity list.
      j = 0;
      for( iter = m_ActiveAmbiguitiesList.begin(); iter != m_ActiveAmbiguitiesList.end(); ++iter )
      {
        if( isEightStateModel )
        {
          iter->state_index = 8+j;
        }
        else
        {
          iter->state_index = 4+j;
        }
        j++;
      }
    }

    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( rxData->m_ObsArray[i].flags.isActive && rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
      {
        // find the new state index.
        for( iter = m_ActiveAmbiguitiesList.begin(); iter != m_ActiveAmbiguitiesList.end(); ++iter )
        {
          if( // iter->channel == rxData->m_ObsArray[i].channel && // GDM - NO CHANNEL MATCHING FOR RINEX DATA!
            iter->id        == rxData->m_ObsArray[i].id &&
            iter->system    == rxData->m_ObsArray[i].system && 
            iter->freqType  == rxData->m_ObsArray[i].freqType )
          {
            rxData->m_ObsArray[i].index_ambiguity_state = iter->state_index;
          }
        }
      }
    }
    remove_list.clear();

    // Deal with cycle slips
    for( i = 0; i < rxData->m_nrValidObs; i++ )
    {
      if( rxData->m_ObsArray[i].flags.isActive && rxData->m_ObsArray[i].flags.isAdrUsedInSolution )
      {
        for( iter = m_ActiveAmbiguitiesList.begin(); iter != m_ActiveAmbiguitiesList.end(); ++iter )
        {
          if( // iter->channel == rxData->m_ObsArray[i].channel && // GDM - NO CHANNEL MATCHING FOR RINEX DATA!
            iter->id        == rxData->m_ObsArray[i].id &&
            iter->system    == rxData->m_ObsArray[i].system && 
            iter->freqType  == rxData->m_ObsArray[i].freqType )
          {
            if( !rxData->m_ObsArray[i].flags.isNoCycleSlipDetected )
            {
              // Set the row and column to zero and the reset the diagonal variance value for this ambiguity.
              for( j = 0; j < P.nrows(); j++ )
              {
                if( j == iter->state_index )
                {
                  // Set the initial variance of the ambiguity state [m].
                  P[j][j] = 1000.0; // KO Arbitrary value, to improve

                  // Initialize the ambiguity state [m].
                  // Compute the single difference adr measurement [m].
                  sd_adr_measured = rxData->m_ObsArray[i].adr - rxBaseData->m_ObsArray[rxData->m_ObsArray[i].index_differential].adr;
                  sd_adr_measured *= GPS_WAVELENGTHL1;

                  // Compute the single difference psr measurement.
                  sd_psr_measured = rxData->m_ObsArray[i].psr - rxBaseData->m_ObsArray[rxData->m_ObsArray[i].index_differential].psr;

                  // Initialize the ambiguity to the difference between the single difference adr
                  // and the single difference pseudorange.

                  sd_dif = sd_adr_measured - sd_psr_measured;
                  rxData->m_ObsArray[i].ambiguity =  sd_dif; // in meters!  //KO possibly replace psr with position derived range plus clock offset                  
                }
                else
                {
                  P[j][iter->state_index] = 0.0;
                  P[iter->state_index][j] = 0.0;
                }
              }
            }
          }
        }
      }
    }

    
    
    
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
              {
                GNSS_ERROR_MSG( "if( !P.Redim( P.nrows()+1, P.ncols()+1 ) )" );
                return false;
              }

              // Set the initial variance of the ambiguity state [m].
              P[amb_info.state_index][amb_info.state_index] = 1000.0; // KO Arbitrary value, to improve

              // Initialize the ambiguity state [m].
              // Compute the single difference adr measurement [m].
              sd_adr_measured = rxData->m_ObsArray[i].adr - rxBaseData->m_ObsArray[rxData->m_ObsArray[i].index_differential].adr;
              sd_adr_measured *= GPS_WAVELENGTHL1;

              // Compute the single difference psr measurement.
              sd_psr_measured = rxData->m_ObsArray[i].psr - rxBaseData->m_ObsArray[rxData->m_ObsArray[i].index_differential].psr;

              // Initialize the ambiguity to the difference between the single difference adr
              // and the single difference pseudorange.

              sd_dif = sd_adr_measured - sd_psr_measured;
              rxData->m_ObsArray[i].ambiguity =  sd_dif; // in meters!  //KO possibly replace psr with position derived range plus clock offset

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
          {
            GNSS_ERROR_MSG( "if( check_iter->state_index == iter->state_index )" );
            return false;
          }
        }
        j++;
      }
      i++;      
    }

    return true;
  }
      




} // end namespace GNSS



