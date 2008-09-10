/**
\file    GNSS_main.cpp
\brief   The main() for the Essential GNSS Project Post Processing software.
\author  Glenn D. MacGougan (GDM)
\date    2007-11-28
\since   2006-11-13

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

#include <stdio.h>
#include <math.h>
#include "gnss_error.h"
#include "constants.h"
#include "geodesy.h"
#include "GNSS_RxData.h"
#include "GNSS_Estimator.h"
#include "GNSS_OptionFile.h"

//#define _CRT_SECURE_NO_DEPRECATE
#ifndef WIN32
#define _CRT_SECURE_NO_DEPRECATE
#endif

using namespace GNSS;

//#define EXTRACTSVDATA 


bool OutputPVT(
  FILE* fid,            //!< The output file. This must already be open.
  GNSS_RxData& rxData,  //!< The receiver data.
  const double datumLatitudeRads,  //!< Used to compute a position difference.
  const double datumLongitudeRads, //!< Used to compute a position difference.
  const double datumHeight,         //!< Used to compute a position difference.
  bool isStatic //!< Indicates if the rover receiver is in static mode.
  );

bool OutputPVT(
  Matrix &PVT,
  GNSS_RxData& rxData,  //!< The receiver data.
  const double datumLatitudeRads,  //!< Used to compute a position difference.
  const double datumLongitudeRads, //!< Used to compute a position difference.
  const double datumHeight,         //!< Used to compute a position difference.
  bool isStatic //!< Indicates if the rover receiver is in static mode.
  );

bool OutputObservationData( 
  Matrix* ptrObsMatrixArray,  //!< An array of n Matrices. e.g. {Matrix ObsMatrixArray[32]; Matrix* ptrObsMatrixArray = ObsMatrixArray;} for index by id (prn)
  const unsigned n,           //!< The number of Obs Matrices in the array. e.g. 32.
  GNSS_RxData* rxData, 
  bool isRover,
  GNSS_Estimator::GNSS_FilterType filterType
  );

/// \brief    Try to sychronize the base and rover measurement sources.
/// \return   true if successful, false if error.
bool GetNextSetOfSynchronousMeasurements( 
  GNSS_RxData &rxDataBase,  //!< The base station receiver data.
  bool &endOfStreamBase,    //!< A boolean that indicates if the end of the data has been reached for the base station.
  GNSS_RxData &rxData, //!< The rover station receiver data.
  bool &endOfStreamRover,   //!< A boolean that indicates if the end of the data has been reached for the rover station.
  bool &isSynchronized      //!< A boolean to indicate if the base and rover are synchronized.
  );


int main( int argc, char* argv[] )
{
  GNSS_RxData rxDataBase;
  GNSS_RxData rxData;  
  GNSS_Estimator Estimator;
  bool isValidPath;
  bool result;
  bool endOfStreamBase = false;
  bool endOfStreamRover = false;
  bool isSynchronized = false;
  bool isEightStateModel = false; // A boolean indicating if the velocity and clock drift states are included as well as the position and clock offset states in the RTK model.

  char msg[256];
  char supermsg[8192];
  unsigned nrBytesInBuffer = 0;

  supermsg[0] = 0;


  unsigned i = 0;
  unsigned j = 0;
  //unsigned k = 0;

  double dT = 0.0;
  double time = 0.0;
  double time_prev = 0.0;
  
  double start_time = 0.0;
  double end_time = 604800.0;

  bool wasPositionComputed = false;
  bool wasVelocityComputed = false;

  FILE *fid = NULL; 
  FILE *fid_pvt = NULL;
  FILE *fid_obs = NULL;
  
  GNSS_OptionFile opt;

  std::string OptionFilePath;

  bool useLSQ = true;
  bool useEKF = false;
  bool useRTK = false;
  bool useTripleDiff = false;
  bool isFilterInitialized = false;

  bool skipEpochForTripleDiff = true;
  
  bool isAtFirstEpoch = true;

  char fname[24];

  double lsq_accuracy = 0;
  double opt_accuracy = 0;          

  Matrix PVT;
  Matrix SatObs[33]; // GPS L1, C/A code, observation information, index by prn, SatObs[0] is empty

  try
  {
    printf( "\nUSAGE: EGNSS <option file path>\n" );
    if( argc != 2 )
    {      
      sprintf( msg, "Invalid arguments: you must specify the option file.", fname );
      GNSS_ERROR_MSG( msg );
      return 1;
    }
    OptionFilePath = argv[1];

    if( !opt.ReadAndInterpretOptions( OptionFilePath ) )
    {
      GNSS_ERROR_MSG( "Invalid option file." );
      return 1;
    }

    rxData.SetDefaultMeasurementStdev_GPSL1(
      opt.m_Rover.stdev_GPSL1_psr,
      opt.m_Rover.stdev_GPSL1_doppler,
      opt.m_Rover.stdev_GPSL1_adr 
      );

    rxDataBase.SetDefaultMeasurementStdev_GPSL1(
      opt.m_Reference.stdev_GPSL1_psr,
      opt.m_Reference.stdev_GPSL1_doppler,
      opt.m_Reference.stdev_GPSL1_adr 
      );

    // Set the start time.
    start_time = opt.m_StartTime.GPSWeek*SECONDS_IN_WEEK + opt.m_StartTime.GPSTimeOfWeek;

    // Set the end time.
    end_time = opt.m_EndTime.GPSWeek*SECONDS_IN_WEEK + opt.m_EndTime.GPSTimeOfWeek;

    if( opt.m_ProcessingMethod == "LSQ" )
    {
      useLSQ = true; // least squares is used for the first epoch
      useEKF = false;
      useRTK = false;
      Estimator.m_FilterType = GNSS_Estimator::GNSS_FILTER_TYPE_LSQ;
      isFilterInitialized = true;
    }
    else if( opt.m_ProcessingMethod == "EKF" )
    {
      useLSQ = true; // least squares is used for the first epoch
      useEKF = true;
      useRTK = false;
      Estimator.m_FilterType = GNSS_Estimator::GNSS_FILTER_TYPE_EKF;
    }
    else if( opt.m_ProcessingMethod == "RTK4" )
    {
      useLSQ = true; // least squares is used for the first epoch
      useEKF = false;
      useRTK  = true;
      isEightStateModel = false;
      Estimator.m_FilterType = GNSS_Estimator::GNSS_FILTER_TYPE_RTK4;
    }
    else if( opt.m_ProcessingMethod == "RTK8" )
    {
      useLSQ = true; // least squares is used for the first epoch
      useEKF = false;
      useRTK  = true;
      isEightStateModel = true;
      Estimator.m_FilterType = GNSS_Estimator::GNSS_FILTER_TYPE_RTK8;
    }
    else if( opt.m_ProcessingMethod == "TRIPLEDIFF" )
    {
      useLSQ = true; // least squares is used for the first epoch
      useTripleDiff = true;
      Estimator.m_FilterType = GNSS_Estimator::GNSS_FILTER_TYPE_TRIPLEDIFF;
    }
    else 
    {
      GNSS_ERROR_MSG( "Unexpected." );
      return 1;      
    }

    if( opt.m_ProcessingMethod == "RTK4" )
    {
      Estimator.m_FourStateRandomWalkKalmanModel.sigmaNorth = opt.m_KalmanOptions.RTK4_sigmaNorth;
      Estimator.m_FourStateRandomWalkKalmanModel.sigmaEast = opt.m_KalmanOptions.RTK4_sigmaEast;
      Estimator.m_FourStateRandomWalkKalmanModel.sigmaUp = opt.m_KalmanOptions.RTK4_sigmaUp;
      Estimator.m_FourStateRandomWalkKalmanModel.sigmaClock = opt.m_KalmanOptions.RTK4_sigmaClock;
    }
    else if( opt.m_ProcessingMethod == "EKF" || opt.m_ProcessingMethod == "RTK8" )
    {
      Estimator.m_FirstOrderGaussMarkovKalmanModel.alphaVn       = opt.m_KalmanOptions.alphaVn;
      Estimator.m_FirstOrderGaussMarkovKalmanModel.alphaVe       = opt.m_KalmanOptions.alphaVe;
      Estimator.m_FirstOrderGaussMarkovKalmanModel.alphaVup      = opt.m_KalmanOptions.alphaVup;
      Estimator.m_FirstOrderGaussMarkovKalmanModel.alphaClkDrift = opt.m_KalmanOptions.alphaClkDrift;
      Estimator.m_FirstOrderGaussMarkovKalmanModel.sigmaVn       = opt.m_KalmanOptions.sigmaVn;
      Estimator.m_FirstOrderGaussMarkovKalmanModel.sigmaVe       = opt.m_KalmanOptions.sigmaVe;
      Estimator.m_FirstOrderGaussMarkovKalmanModel.sigmaVup      = opt.m_KalmanOptions.sigmaVup;
      Estimator.m_FirstOrderGaussMarkovKalmanModel.sigmaClkDrift = opt.m_KalmanOptions.sigmaClkDrift;
    }

    if( opt.m_Reference.isValid )
    {
      if( opt.m_RINEXNavDataPath.length() != 0 )
      {
        result = rxDataBase.Initialize( opt.m_Reference.DataPath.c_str(), isValidPath, opt.m_Reference.DataType, opt.m_RINEXNavDataPath.c_str() );
      }
      else
      {
        result = rxDataBase.Initialize( opt.m_Reference.DataPath.c_str(), isValidPath, opt.m_Reference.DataType, NULL );
      }
      if( !result )
      {
        GNSS_ERROR_MSG( "Failed to initialize the reference receiver object." );        
        return 1;
      }

      if( opt.m_klobuchar.isValid )
      {
        rxDataBase.m_klobuchar = opt.m_klobuchar;
        if( !opt.m_Reference.useIono )
          rxDataBase.m_DisableIonoCorrection = true;
      }
      else
      {
        rxDataBase.m_DisableIonoCorrection = true;
      }
      rxDataBase.m_elevationMask = opt.m_elevationMask*DEG2RAD;
      rxDataBase.m_locktimeMask  = opt.m_locktimeMask;
      rxDataBase.m_cnoMask       = opt.m_cnoMask;

      if( !opt.m_Reference.useTropo )
      {
        rxDataBase.m_DisableTropoCorrection = true;
      }

      result = rxDataBase.SetInitialPVT(
        opt.m_Reference.latitudeRads,
        opt.m_Reference.longitudeRads,
        opt.m_Reference.height,
        0.0, 0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0, 0.0,  // The reference position is fixed.
        0.0, 0.0, 0.0,
        100.0,
        10.0 ); // The reference position is fixed but the clock is still unknown.
      if( !result )
      {
        GNSS_ERROR_MSG( "Failed to initialize the reference receiver object." );
        return 1;
      }
    }
    if( !opt.m_Rover.isValid )
    {
      GNSS_ERROR_MSG( "Rover options are not valid." );
      return 1;
    }

    if( !opt.m_Reference.isValid && opt.m_RINEXNavDataPath.length() != 0 )
    {
      // Stand-Alone mode
      result = rxData.Initialize( opt.m_Rover.DataPath.c_str(), isValidPath, opt.m_Rover.DataType, opt.m_RINEXNavDataPath.c_str() );
    }
    else
    {
      result = rxData.Initialize( opt.m_Rover.DataPath.c_str(), isValidPath, opt.m_Rover.DataType, NULL );
    }
    if( !result )
    {
      GNSS_ERROR_MSG( "Failed to initialize the rover receiver object." );
      return 1;
    }
    if( opt.m_klobuchar.isValid )
    {
      rxData.m_klobuchar = opt.m_klobuchar;
      if( !opt.m_Reference.useIono )
        rxData.m_DisableIonoCorrection = true;
    }
    else
    {
      rxData.m_DisableIonoCorrection = true;
    }
    rxData.m_elevationMask = opt.m_elevationMask*DEG2RAD;
    rxData.m_locktimeMask  = opt.m_locktimeMask;
    rxData.m_cnoMask       = opt.m_cnoMask;

    if( !opt.m_Rover.useTropo )
    {
      rxData.m_DisableTropoCorrection = true;
    }
    
    result = rxData.SetInitialPVT(
      opt.m_Rover.latitudeRads,
      opt.m_Rover.longitudeRads,
      opt.m_Rover.height,
      0.0, 0.0, 0.0,
      0.0, 0.0, 
      opt.m_Rover.uncertaintyLatitudeOneSigma, 
      opt.m_Rover.uncertaintyLongitudeOneSigma, 
      opt.m_Rover.uncertaintyHeightOneSigma, 
      1.0, 1.0, 1.0,
      1000.0,
      1.0e9 );
    if( !result )
    {
      GNSS_ERROR_MSG( "Failed to initialize the rover receiver object." );
      return 1; 
    }

    // GDM_HACK
    // For comparing the computed rover position
    if( opt.m_RoverDatum.isValid )
    {
      result = rxData.SetDatumPVT(
        opt.m_RoverDatum.latitudeRads,
        opt.m_RoverDatum.longitudeRads,
        opt.m_RoverDatum.height );
      if( !result )
      {
        GNSS_ERROR_MSG( "Failed to initialize the rover receiver object's datum point." );
        return 1; 
      }
    }

#ifdef GDM_UWB_RANGE_HACK
    if( !opt.m_UWBFilePath.empty() )
    {
      // GDM - Load the UWB range data.
      result = rxData.EnableAndLoadUWBData( 
        opt.m_UWBFilePath.c_str(), 
        opt.m_UWB_a.id,
        opt.m_UWB_a.x, 
        opt.m_UWB_a.y,
        opt.m_UWB_a.z,
        opt.m_UWB_b.id,
        opt.m_UWB_b.x,
        opt.m_UWB_b.y,
        opt.m_UWB_b.z,
        opt.m_UWB_c.id,
        opt.m_UWB_c.x,
        opt.m_UWB_c.y,
        opt.m_UWB_c.z,
        true );
      if( !result )
      {
        GNSS_ERROR_MSG( "Failed to load the UWB data." );
        return 1; 
      }
    }
#endif

    while( !endOfStreamRover )
    {
      if( !isAtFirstEpoch ) 
      {
        time_prev = rxData.m_pvt.time.gps_week*SECONDS_IN_WEEK + rxData.m_pvt.time.gps_tow;
      }

      if( opt.m_Reference.isValid )
      {
        result = GetNextSetOfSynchronousMeasurements( 
          rxDataBase,
          endOfStreamBase,
          rxData,
          endOfStreamRover,
          isSynchronized );
        if( !result )
        {
          GNSS_ERROR_MSG( "Unable to load next set of measurements." );
          return -1;
        }
      }
      else
      {
        result = rxData.LoadNext( endOfStreamRover );
        if( !result )
        {
          GNSS_ERROR_MSG( "Unable to load next set of measurements." );
          return -1;
        }
      }

      if( endOfStreamRover )
        break;

      if( opt.m_Reference.isValid )
      {
        if( endOfStreamBase )
          break;
      }

      if( rxData.m_nrValidObs == 0 )
        continue;

      // Check that the processing time is within the processing interval.
      time = rxData.m_pvt.time.gps_week*SECONDS_IN_WEEK + rxData.m_pvt.time.gps_tow;
      if( time < start_time )
        continue;
      if( time > end_time )
        break;


      // GDM_DEBUG breakpoint times
      if( rxData.m_pvt.time.gps_tow > 320039 )
        int gaa = 99;

      if( rxData.m_pvt.time.gps_tow > 320414 )
        int ggga = 99;

      if( isAtFirstEpoch )
      {
        isAtFirstEpoch = false;
        dT = 0.0;
      }
      else
      {
        dT = time - time_prev;      
        if( dT < 0.0 )
        {
          // should never happen
          GNSS_ERROR_MSG( "dT is negative." );
          return 1; 
        }        
      }

      if( !opt.m_UseDopplerMeasurements )
      {
        // Disable all the Doppler measurements on the rover.
        for( i = 0; i < rxData.m_nrValidObs; i++ )
        {
          rxData.m_ObsArray[i].flags.isDopplerValid = false;
          rxData.m_ObsArray[i].flags.isDopplerUsedInSolution = false;          
        }

        if( opt.m_Reference.isValid )
        {
          // Disable all the Doppler measurements on the base station.
          for( i = 0; i < rxDataBase.m_nrValidObs; i++ )
          {
            rxDataBase.m_ObsArray[i].flags.isDopplerValid = false;
            rxDataBase.m_ObsArray[i].flags.isDopplerUsedInSolution = false;            
          }
        }          
      }
        
      // exclude satellites as indicated in the option file
      for( j = 0; j < opt.m_Rover.nrSatsToExclude; j++ )
      {
        for( i = 0; i < rxData.m_nrValidObs; i++ )
        {
          if( rxData.m_ObsArray[i].id == opt.m_Rover.satsToExclude[j] )
          {
            rxData.m_ObsArray[i].flags.isNotUserRejected = 0;
          }
        }
      } 

      if( opt.m_Reference.isValid )
      {
        // exclude satellites as indicated in the option file
        for( j = 0; j < opt.m_Reference.nrSatsToExclude; j++ )
        {
          for( i = 0; i < rxDataBase.m_nrValidObs; i++ )
          {
            if( rxDataBase.m_ObsArray[i].id == opt.m_Reference.satsToExclude[j] )
            {
              rxDataBase.m_ObsArray[i].flags.isNotUserRejected = 0;
            }
          }
        } 
      }

      if( opt.m_RoverIsStatic )
      {
        // Enable cycle slip detection using the triple difference method.
        rxData.m_isStatic = true;
      }

      // Enable constraints if any.
      if( opt.m_isPositionFixed )
      {
        rxData.m_pvt_lsq.isHeightConstrained = false;
        rxData.m_pvt.isHeightConstrained = false;        
        rxData.m_pvt_lsq.isPositionFixed = true;
        rxData.m_pvt.isPositionFixed = true;
        
        // Enable cycle slip detection using the triple difference method.
        rxData.m_isStatic = true;
      }
      else if( opt.m_isHeightConstrained )
      {
        rxData.m_pvt_lsq.isHeightConstrained = true;
        rxData.m_heightConstraint = opt.m_Rover.height;
        rxData.m_heightConstraintStdev = opt.m_Rover.uncertaintyHeightOneSigma;
        rxData.m_pvt.isHeightConstrained = true;
        rxData.m_pvt_lsq.isPositionFixed = false;
        rxData.m_pvt.isPositionFixed = false;
      }

      // Always perfom least squares.
      if( opt.m_Reference.isValid )
      {
        result = Estimator.PerformLeastSquares(
          &rxData,
          &rxDataBase,
          wasPositionComputed,
          wasVelocityComputed );
        if( !result )
        {
          sprintf( msg, "%.1Lf %d LSQ returned false\n", rxData.m_pvt_lsq.time.gps_tow, rxData.m_pvt_lsq.time.gps_tow );
          GNSS_ERROR_MSG( msg );
          return 2;
        }        
      }
      else
      {
        result = Estimator.PerformLeastSquares(
          &rxData,
          NULL,
          wasPositionComputed,
          wasVelocityComputed );
        if( !result )
        {
          sprintf( msg, "%.1Lf %d LSQ returned false\n", rxData.m_pvt_lsq.time.gps_tow, rxData.m_pvt_lsq.time.gps_tow );
          GNSS_ERROR_MSG( msg );
          return 2;
        }
      }

      if( !isFilterInitialized )
      {
        // Is there a 'valid' least squares position estimate?
        if( !( wasPositionComputed && rxData.m_pvt_lsq.didGlobalTestPassForPosition ) )
        {
          continue;
        }
        else
        {
          if( opt.m_isPositionFixed )
          {
            result = rxData.UpdatePositionAndRxClock(
              rxData.m_pvt,
              opt.m_Rover.latitudeRads,
              opt.m_Rover.longitudeRads,
              opt.m_Rover.height,
              rxData.m_pvt.clockOffset,
              opt.m_Rover.uncertaintyLatitudeOneSigma, 
              opt.m_Rover.uncertaintyLongitudeOneSigma, 
              opt.m_Rover.uncertaintyHeightOneSigma, 
              rxData.m_pvt.std_clk
              );

            // The least squares covariance matrix is still used to seed the filter.
            Estimator.m_posLSQ.P.Zero();
            Estimator.m_posLSQ.P[0][0] = opt.m_Rover.uncertaintyLatitudeOneSigma*opt.m_Rover.uncertaintyLatitudeOneSigma;
            Estimator.m_posLSQ.P[1][1] = opt.m_Rover.uncertaintyLongitudeOneSigma*opt.m_Rover.uncertaintyLongitudeOneSigma;
            Estimator.m_posLSQ.P[2][2] = opt.m_Rover.uncertaintyHeightOneSigma*opt.m_Rover.uncertaintyHeightOneSigma;
            Estimator.m_posLSQ.P[3][3] = 2.0*rxData.m_pvt.std_clk*rxData.m_pvt.std_clk;
          }
          else
          {
            // Seed with either least squares or the option file solution
            // whichever is more precise

            lsq_accuracy  = rxData.m_pvt_lsq.std_lat*rxData.m_pvt_lsq.std_lat;
            lsq_accuracy += rxData.m_pvt_lsq.std_lon*rxData.m_pvt_lsq.std_lon;
            lsq_accuracy += rxData.m_pvt_lsq.std_hgt*rxData.m_pvt_lsq.std_hgt;
            lsq_accuracy  = sqrt(lsq_accuracy);
            opt_accuracy  = opt.m_Rover.uncertaintyLatitudeOneSigma*opt.m_Rover.uncertaintyLatitudeOneSigma;
            opt_accuracy += opt.m_Rover.uncertaintyLongitudeOneSigma*opt.m_Rover.uncertaintyLongitudeOneSigma;
            opt_accuracy += opt.m_Rover.uncertaintyHeightOneSigma*opt.m_Rover.uncertaintyHeightOneSigma;
            opt_accuracy  = sqrt(opt_accuracy);

            if( opt_accuracy <= lsq_accuracy )
            {
              result = rxData.UpdatePositionAndRxClock(
                rxData.m_pvt,
                opt.m_Rover.latitudeRads,
                opt.m_Rover.longitudeRads,
                opt.m_Rover.height,
                rxData.m_pvt.clockOffset,
                opt.m_Rover.uncertaintyLatitudeOneSigma, 
                opt.m_Rover.uncertaintyLongitudeOneSigma, 
                opt.m_Rover.uncertaintyHeightOneSigma, 
                rxData.m_pvt.std_clk
                );

              Estimator.m_posLSQ.P.Zero();
              Estimator.m_posLSQ.P[0][0] = opt.m_Rover.uncertaintyLatitudeOneSigma*opt.m_Rover.uncertaintyLatitudeOneSigma;
              Estimator.m_posLSQ.P[1][1] = opt.m_Rover.uncertaintyLongitudeOneSigma*opt.m_Rover.uncertaintyLongitudeOneSigma;
              Estimator.m_posLSQ.P[2][2] = opt.m_Rover.uncertaintyHeightOneSigma*opt.m_Rover.uncertaintyHeightOneSigma;
              Estimator.m_posLSQ.P[3][3] = 2.0*rxData.m_pvt.std_clk*rxData.m_pvt.std_clk;
            }
            else
            {
              // Seed the filter with the least squares solution.
              rxData.m_pvt = rxData.m_pvt_lsq;
            }
          }
            
          if( useRTK )
          {
            result = Estimator.InitializeStateVarianceCovarianceFromLeastSquares_RTK(
              Estimator.m_posLSQ.P,
              Estimator.m_velLSQ.P
              );
            if( !result )
            {
              GNSS_ERROR_MSG( "Estimator.InitializeStateVarianceCovarianceFromLeastSquares_RTK returned false." );
              return 1;
            }
          }
          else if( useEKF )
          {            
            result = Estimator.InitializeStateVarianceCovariance_EKF(
              Estimator.m_posLSQ.P,
              Estimator.m_velLSQ.P
              );
            if( !result )
            {
              GNSS_ERROR_MSG( "Estimator.InitializeStateVarianceCovariance_EKF returned false." );
              return 1;
            }
          }
          else if( useTripleDiff )
          {            
            skipEpochForTripleDiff = true;

            // Estimator.m_posLSQ.P contains either the least squares covariance info
            // or the option file specified covariance information.
            Estimator.m_TD.Cx.Resize(3,3);
            if( lsq_accuracy < opt_accuracy )
            {
              Estimator.m_TD.Cx[0][0] = 6.0*Estimator.m_posLSQ.P[0][0];
              Estimator.m_TD.Cx[1][1] = 6.0*Estimator.m_posLSQ.P[1][1];
              Estimator.m_TD.Cx[2][2] = 6.0*Estimator.m_posLSQ.P[2][2];
            }
            else
            {
              Estimator.m_TD.Cx[0][0] = Estimator.m_posLSQ.P[0][0];
              Estimator.m_TD.Cx[1][1] = Estimator.m_posLSQ.P[1][1];
              Estimator.m_TD.Cx[2][2] = Estimator.m_posLSQ.P[2][2];
            }
          }
          else
          {
            GNSS_ERROR_MSG( "Unexpected." );
            return 1;
          }
          isFilterInitialized = true;
        }
      }

      if( useTripleDiff )
      {
        if( !skipEpochForTripleDiff )
        {
          // copy the velocity and clock drift information to the triple difference solution
          rxData.m_pvt.clockOffset = rxData.m_pvt_lsq.clockOffset;
          rxData.m_pvt.std_clk = rxData.m_pvt_lsq.std_clk;

          rxData.UpdateVelocityAndClockDrift(
            rxData.m_pvt, 
            rxData.m_pvt_lsq.vn, 
            rxData.m_pvt_lsq.ve, 
            rxData.m_pvt_lsq.vup, 
            rxData.m_pvt_lsq.clockDrift,
            rxData.m_pvt_lsq.std_vn,
            rxData.m_pvt_lsq.std_ve,
            rxData.m_pvt_lsq.std_vup,
            rxData.m_pvt_lsq.std_clkdrift );

          rxData.m_pvt.nrDopplerObsAvailable = rxData.m_pvt_lsq.nrDopplerObsAvailable;
          rxData.m_pvt.nrDopplerObsRejected = rxData.m_pvt_lsq.nrDopplerObsRejected;
          rxData.m_pvt.nrDopplerObsUsed = rxData.m_pvt_lsq.nrDopplerObsUsed;
          
          result = Estimator.EstimateTripleDifferenceSolution(
            &rxData,
            &rxDataBase,
            wasPositionComputed
            );
          skipEpochForTripleDiff = true;
        }
        else
        {
          skipEpochForTripleDiff = false;
        }
      }
      else if( useEKF )
      {
        result = Estimator.PredictAhead_EKF(
          rxData,
          dT
          );
        if( !result )
        {
          GNSS_ERROR_MSG( "Estimator.PredictAhead_EKF returned false." );
          return 1;
        }

        if( opt.m_Reference.isValid )
        {
          result = Estimator.Kalman_Update_EKF(
            &rxData,
            &rxDataBase
            );
          if( !result )
          {
            GNSS_ERROR_MSG( "Estimator.Kalman_Update_EKF returned false." );
            return -1;
          }
        }
        else
        {
          result = Estimator.Kalman_Update_EKF(
            &rxData,
            NULL
            );
          if( !result )
          {
            GNSS_ERROR_MSG( "Estimator.Kalman_Update_EKF returned false." );
            return -1;
          }
        }
      }
      else if( useRTK )
      {
        if( Estimator.m_FilterType == GNSS_Estimator::GNSS_FILTER_TYPE_RTK4 )
        {
          // copy the velocity and clock drift information to the rtk solution
          rxData.UpdateVelocityAndClockDrift(
            rxData.m_pvt, 
            rxData.m_pvt_lsq.vn, 
            rxData.m_pvt_lsq.ve, 
            rxData.m_pvt_lsq.vup, 
            rxData.m_pvt_lsq.clockDrift,
            rxData.m_pvt_lsq.std_vn,
            rxData.m_pvt_lsq.std_ve,
            rxData.m_pvt_lsq.std_vup,
            rxData.m_pvt_lsq.std_clkdrift );

          rxData.m_pvt.nrDopplerObsAvailable = rxData.m_pvt_lsq.nrDopplerObsAvailable;
          rxData.m_pvt.nrDopplerObsRejected = rxData.m_pvt_lsq.nrDopplerObsRejected;
          rxData.m_pvt.nrDopplerObsUsed = rxData.m_pvt_lsq.nrDopplerObsUsed;

          // Bump the clock state using the least squares solution
          // to reduce innovation values for the pseudoranges if
          // the clock is not really being filtered.
          if( opt.m_KalmanOptions.RTK4_sigmaClock > 50.0 )
          {
            if( rxData.m_pvt_lsq.std_clk < 50.0 )
            {
              rxData.m_pvt.clockOffset = rxData.m_pvt_lsq.clockOffset;
              Estimator.m_RTK.P[3][3] = rxData.m_pvt_lsq.std_clk*rxData.m_pvt_lsq.std_clk;
            }
          }
        }

        result = Estimator.PredictAhead_RTK( rxData, dT );
        if( !result )
        {
          GNSS_ERROR_MSG( "Estimator.PredictAhead_RTK returned false." );
          return 1;
        }

        if( opt.m_Reference.isValid )
        {
          result = Estimator.Kalman_Update_RTK( &rxData, &rxDataBase );
          if( !result )
          {
            GNSS_ERROR_MSG( "Estimator.Kalman_Update_RTK returned false." );
            return 1;
          }
        }
        else
        {
          GNSS_ERROR_MSG( "Only differential supported." );
          return 1;
        }
      }	       


      /*
      printf( "%12.3lf %5d %d %d %d \n", 
        rxData.m_pvt.time.gps_tow, 
        rxData.m_pvt.time.gps_week,         
        rxData.m_pvt.nrPsrObsUsed,
        rxData.m_pvt.nrDopplerObsUsed,
        rxData.m_pvt.nrAdrObsUsed );    
        */

      /*
      char supermsg[8192];
      unsigned nrBytesInBuffer;
      rxData.Debug_WriteSuperMsg80CharsWide( 
      supermsg,
      8192,
      51.0916666667*DEG2RAD,
      -114.0000000000*DEG2RAD,
      1000.000,
      nrBytesInBuffer );

      printf( supermsg );
      */

      
      rxData.m_prev_pvt = rxData.m_pvt;
      rxDataBase.m_prev_pvt = rxDataBase.m_pvt;

      if( !opt.m_RoverDatum.isValid )
      {
        opt.m_RoverDatum.latitudeRads = rxData.m_pvt.latitude;
        opt.m_RoverDatum.longitudeRads = rxData.m_pvt.longitude;
        opt.m_RoverDatum.height = rxData.m_pvt.height;
        opt.m_RoverDatum.isValid = true;
      }
      // Output the PVT results.
      if( !OutputPVT( PVT, rxData, opt.m_RoverDatum.latitudeRads, opt.m_RoverDatum.longitudeRads, opt.m_RoverDatum.height, opt.m_RoverIsStatic ) )
      {
        sprintf( msg, "%.3Lf %d OutputPVT Failed\n", rxData.m_pvt.time.gps_tow, rxData.m_pvt.time.gps_week );
        GNSS_ERROR_MSG( msg );
        return 1;
      }

      if( !OutputObservationData( SatObs, 32, &rxData, true, Estimator.m_FilterType ) )
      {
        sprintf( msg, "%.3Lf %d OutputObservationData returned false.\n", rxData.m_pvt.time.gps_tow, rxData.m_pvt.time.gps_week );
        GNSS_ERROR_MSG( msg );
        return 1;
      }
    }    
  }
  catch( MatrixException& matrixException )
  {
    printf( "%s", matrixException.GetExceptionMessage().c_str() );
  }
  catch ( ... )
  {
    printf( "\nCaught unknown exception\n" );
  }

  if( !PVT.isEmpty() )
  {
    if( !PVT.Inplace_Transpose() )
    {
      GNSS_ERROR_MSG( "if( !PVT.Inplace_Transpose() )" );
      return 1;
    }

    // Open the PVT output file
    fid_pvt = fopen( "pvt.csv", "w" );
    if( !fid_pvt )
    {
      printf( "Please close pvt.csv and type GO: " );
      gets( msg );
      fid_pvt = fopen( "pvt.csv", "w" );
      if( !fid_pvt )
      {
        sprintf( msg, "Unable to open pvt.csv." );
        GNSS_ERROR_MSG( msg );
        return 1;
      }
    }
    fprintf( fid_pvt, "GPS time of week(s), GPS week, latitude (deg), longitude (deg), height (m), Velocity North (m/s), Velocity East (m/s), Velocity Up (m/s), Ground Speed (km/hr), Clock Offset (m), Clock Drift (m/s), " );
    if( opt.m_RoverIsStatic )
      fprintf( fid_pvt, "Error North (m), Error East (m), Error Up (m),");
    else
      fprintf( fid_pvt, "Northing (m), Easting (m), Up (m),");
    fprintf( fid_pvt, "STDEV latitude (m), STDEV longitude (m), STDEV height (m), STDEV Velocity North (m/s), STDEV Velocity East (m/s), STDEV Velocity Up (m/s), STDEV Clock Offset (m), STDEV Clock Drift (m/s), " );
    fprintf( fid_pvt, "NR PSR Available, NR PSR Used, NR Doppler Available, NR Doppler Used, NR ADR Available, NR ADR Used," );
    fprintf( fid_pvt, "NDOP,EDOP,VDOP,HDOP,PDOP,TDOP,GDOP,");    
    fprintf( fid_pvt, "LSQ APVF Position, LSQ Pos Global Test, APVF Velocity, LSQ Vel Global Test, FixedSoln latitude (deg), FixedSoln longitude (deg), FixedSoln height (m),");    
    if( opt.m_RoverIsStatic )
      fprintf( fid_pvt, "FixedSoln ErrN (m), FixedSoln ErrE (m), FixedSoln ErrUp (m),");
    else
      fprintf( fid_pvt, "FixedSoln Northing (m), FixedSoln Easting (m), FixedSoln Up (m),");
    fprintf( fid_pvt, "ambiguity ratio, P(a_check=a)\n");    
    
    fclose( fid_pvt );

    if( !PVT.PrintDelimited( "pvt.csv", 12, ',', true )  )
    {
      GNSS_ERROR_MSG( "if( !PVT.PrintDelimited( \"pvt.csv\", 12, ',', true )  )" );
      return 1;
    }
  }

  for( i = 1; i <= 32; i++ )
  {
    if( !SatObs[i].isEmpty() )
    {
      sprintf( fname, "obs_%02d.csv", i );
      fid_obs = fopen( fname, "w" );
      if( !fid_obs )
      {
        printf( "Please close %s and type GO: ", fname );
        gets( msg );
        fid_obs = fopen( fname, "w" );
        if( !fid_obs )
        {
          sprintf( msg, "Unable to open %s.", fname );
          GNSS_ERROR_MSG( msg );
          return 1;
        }
      }

      fprintf( fid_obs, "GPS time of week(s),GPS week,ID,Channel," );
      fprintf( fid_obs, "System,Code Type,Frequency Type," );
      fprintf( fid_obs, "Elevation (deg), Azimuth (deg)," );
      fprintf( fid_obs, "PSR (m), ADR (cycles), PSR-ADR (m), Doppler (Hz),C/No (dB-Hz),Lock Time (s)," );
      fprintf( fid_obs, "isActive,isCodeLocked,isPhaseLocked,isParityValid,isPsrValid,isAdrValid,isDopplerValid,isGrouped,isAutoAssigned,isCarrierSmoothed," );
      fprintf( fid_obs, "isEphemerisValid,isAlmanacValid,isAboveElevationMask,isAboveCNoMask,isAboveLockTimeMask,isNotUserRejected,isNotPsrRejected,isNotAdrRejected,isNotDopplerRejected,isNoCycleSlipDetected," );
      fprintf( fid_obs, "isPsrUsedInSolution,isDopplerUsedInSolution,isAdrUsedInSolution,isDifferentialPsrAvailable,isDifferentialDopplerAvailable,isDifferentialAdrAvailable,useTropoCorrection,useBroadcastIonoCorrection,isBaseSatellite," );
      fprintf( fid_obs, "stdev PSR (m),stdev adr (cycles),stdev Doppler (Hz)," );
      fprintf( fid_obs, "PSR misclosure (m), Doppler misclosure (m/s), ADR misclosure (m)," );
      fprintf( fid_obs, "SD_ambiguity (m), DD_ambiguity (m), DD_fixed_ambiguity (cycles), SD ADR residual (m), DD ADR residual (m), DD ADR residual fixed (m)\n" );
      fclose( fid_obs );

      if( !SatObs[i].Inplace_Transpose() )
      {
        GNSS_ERROR_MSG( "if( !SatObs[i].Inplace_Transpose() )" );
        return 1;
      }
      if( !SatObs[i].PrintDelimited( fname, 12, ',', true )  )
      {
        GNSS_ERROR_MSG( "if( !SatObs[i].PrintDelimited( fname, 12, ',', true )  )" );
        return 1;
      }
    }
  }

  return 0;
}


bool OutputPVT(
  Matrix &PVT,
  GNSS_RxData& rxData,  //!< The receiver data.
  const double datumLatitudeRads,  //!< Used to compute a position difference.
  const double datumLongitudeRads, //!< Used to compute a position difference.
  const double datumHeight,         //!< Used to compute a position difference.
  bool isStatic //!< Indicates if the rover receiver is in static mode.
  )
{
  static bool once = true;
  BOOL result;
  double northing = 0.0;
  double easting = 0.0;
  double up = 0.0;
  unsigned i = 0;
  const unsigned nr_items = 48;
  Matrix data(nr_items,1);

  result = GEODESY_ComputePositionDifference(
    GEODESY_REFERENCE_ELLIPSE_WGS84,
    datumLatitudeRads,
    datumLongitudeRads,
    datumHeight,
    rxData.m_pvt.latitude,
    rxData.m_pvt.longitude,
    rxData.m_pvt.height,
    &northing,
    &easting,
    &up );

  if( result == FALSE )
  {
    GNSS_ERROR_MSG( "GEODESY_ComputePositionDifference returned FALSE." );
    return false;
  }

  data[i] = rxData.m_pvt.time.gps_tow; i++;
  data[i] = rxData.m_pvt.time.gps_week; i++;
  data[i] = rxData.m_pvt.latitudeDegs; i++;
  data[i] = rxData.m_pvt.longitudeDegs; i++;
  data[i] = rxData.m_pvt.height; i++;
  data[i] = rxData.m_pvt.vn; i++;
  data[i] = rxData.m_pvt.ve; i++;
  data[i] = rxData.m_pvt.vup; i++;
  data[i] = sqrt( rxData.m_pvt.vn*rxData.m_pvt.vn + rxData.m_pvt.ve*rxData.m_pvt.ve)*3.6; i++;
  data[i] = rxData.m_pvt.clockOffset; i++;
  data[i] = rxData.m_pvt.clockDrift; i++;

  data[i] = northing; i++;
  data[i] = easting; i++;
  data[i] = up; i++;

  data[i] = rxData.m_pvt.std_lat; i++;
  data[i] = rxData.m_pvt.std_lon; i++;
  data[i] = rxData.m_pvt.std_hgt; i++;
  data[i] = rxData.m_pvt.std_vn; i++;
  data[i] = rxData.m_pvt.std_ve; i++;
  data[i] = rxData.m_pvt.std_vup; i++;
  data[i] = rxData.m_pvt.std_clk; i++;
  data[i] = rxData.m_pvt.std_clkdrift; i++;

  data[i] = rxData.m_pvt.nrPsrObsAvailable; i++;
  data[i] = rxData.m_pvt.nrPsrObsUsed; i++;
  data[i] = rxData.m_pvt.nrDopplerObsAvailable; i++;
  data[i] = rxData.m_pvt.nrDopplerObsUsed; i++;
  data[i] = rxData.m_pvt.nrAdrObsAvailable; i++;
  data[i] = rxData.m_pvt.nrAdrObsUsed; i++;

  data[i] = rxData.m_pvt.dop.ndop; i++;
  data[i] = rxData.m_pvt.dop.edop; i++;
  data[i] = rxData.m_pvt.dop.vdop; i++;
  data[i] = rxData.m_pvt.dop.hdop; i++;
  data[i] = rxData.m_pvt.dop.pdop; i++;
  data[i] = rxData.m_pvt.dop.tdop; i++;
  data[i] = rxData.m_pvt.dop.gdop; i++;
    
  data[i] = rxData.m_pvt_lsq.pos_apvf; i++;
  data[i] = rxData.m_pvt_lsq.didGlobalTestPassForPosition; i++;
  data[i] = rxData.m_pvt_lsq.vel_apvf; i++;
  data[i] = rxData.m_pvt_lsq.didGlobalTestPassForVelocity; i++;

  data[i] = rxData.m_pvt_fixed.latitudeDegs; i++;
  data[i] = rxData.m_pvt_fixed.longitudeDegs; i++;
  data[i] = rxData.m_pvt_fixed.height; i++;  

  if( rxData.m_pvt_fixed.latitude == 0.0 &&
    rxData.m_pvt_fixed.longitude == 0.0 &&
    rxData.m_pvt_fixed.height == 0.0 )
  {
    northing = 0;
    easting = 0;
    up = 0;
  }
  else
  {
    result = GEODESY_ComputePositionDifference(
      GEODESY_REFERENCE_ELLIPSE_WGS84,
      datumLatitudeRads,
      datumLongitudeRads,
      datumHeight,
      rxData.m_pvt_fixed.latitude,
      rxData.m_pvt_fixed.longitude,
      rxData.m_pvt_fixed.height,
      &northing,
      &easting,
      &up );
    if( result == FALSE )
    {
      GNSS_ERROR_MSG( "GEODESY_ComputePositionDifference returned FALSE." );
      return false;
    }
  }

  data[i] = northing; i++;
  data[i] = easting; i++;
  data[i] = up; i++;

  data[i] = rxData.m_ambiguity_validation_ratio; i++;  
  data[i] = rxData.m_probability_of_correct_ambiguities; i++;  
  data[i] = rxData.m_norm; i++;

  result = PVT.Concatonate( data );
  if( !result )
  {
    GNSS_ERROR_MSG( "result = PVT.Concatonate( data );" );
    return true;
  }
  return true;
}


bool OutputObservationData( 
  Matrix* ptrObsMatrixArray,  //!< An array of n Matrices. e.g. {Matrix ObsMatrixArray[32]; Matrix* ptrObsMatrixArray = ObsMatrixArray;} for index by id (prn)
  const unsigned n,           //!< The number of Obs Matrices in the array. e.g. 32.
  GNSS_RxData* rxData, 
  bool isRover,
  GNSS_Estimator::GNSS_FilterType filterType
  )
{
  unsigned i = 0;
  unsigned j = 0;
  unsigned index = 0;
  bool result = false;  
  Matrix* ptr = ptrObsMatrixArray;

  const unsigned nr_items = 56;
  Matrix data(nr_items,1);

  if( ptr == NULL )
    return false;

  if( n != 32 )
    return false;

  for( i = 0; i < rxData->m_nrValidObs; i++ )
  {
    if( rxData->m_ObsArray[i].codeType != GNSS_CACode ||
      rxData->m_ObsArray[i].freqType != GNSS_GPSL1 ||
      rxData->m_ObsArray[i].system != GNSS_GPS )
    {
      continue;
    }

    index = rxData->m_ObsArray[i].id;
    if( index > n )
      return false;

    j = 0;
    
    data[j] = rxData->m_pvt.time.gps_tow;
    j++; data[j] = rxData->m_pvt.time.gps_week;
    j++; data[j] = rxData->m_ObsArray[i].id;
    j++; data[j] = rxData->m_ObsArray[i].channel;

    j++; data[j] = rxData->m_ObsArray[i].system; 
    j++; data[j] = rxData->m_ObsArray[i].codeType;
    j++; data[j] = rxData->m_ObsArray[i].freqType;

    j++; data[j] = rxData->m_ObsArray[i].satellite.elevation*RAD2DEG;
    j++; data[j] = rxData->m_ObsArray[i].satellite.azimuth*RAD2DEG;

    j++; data[j] = rxData->m_ObsArray[i].psr;
    j++; data[j] = rxData->m_ObsArray[i].adr;
    j++; data[j] = rxData->m_ObsArray[i].psr-(rxData->m_ObsArray[i].adr*GPS_WAVELENGTHL1);
    j++; data[j] = rxData->m_ObsArray[i].doppler;
    j++; data[j] = rxData->m_ObsArray[i].cno;
    j++; data[j] = rxData->m_ObsArray[i].locktime;


    j++; data[j] = rxData->m_ObsArray[i].flags.isActive;                //!< This flag indicates that the channel is active for use. If this is not set, no other flags are valid for use.
    j++; data[j] = rxData->m_ObsArray[i].flags.isCodeLocked;            //!< Indicates if the code tracking is locked.
    j++; data[j] = rxData->m_ObsArray[i].flags.isPhaseLocked;           //!< Indicates if the phase tracking is locked.
    j++; data[j] = rxData->m_ObsArray[i].flags.isParityValid;           //!< Indicates if the phase parity if valid.      
    j++; data[j] = rxData->m_ObsArray[i].flags.isPsrValid;              //!< Indicates if the pseudorange valid for use.
    j++; data[j] = rxData->m_ObsArray[i].flags.isAdrValid;              //!< Indicates if the ADR is valid for use.
    j++; data[j] = rxData->m_ObsArray[i].flags.isDopplerValid;          //!< Indicates if the Doppler if valid for use.
    j++; data[j] = rxData->m_ObsArray[i].flags.isGrouped;               //!< Indicates if this channel has another associated channel. eg. L1 and L2 measurements.
    j++; data[j] = rxData->m_ObsArray[i].flags.isAutoAssigned;          //!< Indicates if the channel was receiver assigned (otherwise, the user forced this channel assignment).
    j++; data[j] = rxData->m_ObsArray[i].flags.isCarrierSmoothed;       //!< Indicates if the pseudorange has carrier smoothing enabled.

    j++; data[j] = rxData->m_ObsArray[i].flags.isEphemerisValid;        //!< Indicates if this channel has valid associated ephemeris information. 
    j++; data[j] = rxData->m_ObsArray[i].flags.isAlmanacValid;          //!< Indicates if this channel has valid associated almanac information.
    j++; data[j] = rxData->m_ObsArray[i].flags.isAboveElevationMask;    //!< Indicates if the satellite tracked is above the elevation mask.    
    j++; data[j] = rxData->m_ObsArray[i].flags.isAboveCNoMask;          //!< Indciates if the channel's C/No is above a threshold value.
    j++; data[j] = rxData->m_ObsArray[i].flags.isAboveLockTimeMask;     //!< Indicates if the channel's locktime is above a treshold value.
    j++; data[j] = rxData->m_ObsArray[i].flags.isNotUserRejected;       //!< Indicates if the user has not forced the rejection of this channel or PRN.
    j++; data[j] = rxData->m_ObsArray[i].flags.isNotPsrRejected;        //!< Indicates if the pseudorange was not rejetced (ie Fault Detection and Exclusion).
    j++; data[j] = rxData->m_ObsArray[i].flags.isNotAdrRejected;        //!< Indicates if the ADR was not rejetced (ie Fault Detection and Exclusion).
    j++; data[j] = rxData->m_ObsArray[i].flags.isNotDopplerRejected;    //!< Indicates if the Doppler was not rejected (ie Fault Detection and Exclusion).
    j++; data[j] = rxData->m_ObsArray[i].flags.isNoCycleSlipDetected;   //!< Indicates that no cycle slip has occurred at this epoch.

    j++; data[j] = rxData->m_ObsArray[i].flags.isPsrUsedInSolution;            //!< Indicates if some part (pseudorange) of this channel's measurement was used in the position solution.
    j++; data[j] = rxData->m_ObsArray[i].flags.isDopplerUsedInSolution;        //!< Indicates if some part (Doppler) of this channel's measurement was used in the velocity solution.
    j++; data[j] = rxData->m_ObsArray[i].flags.isAdrUsedInSolution;            //!< Indicates if the the ADR is used in the solution.
    j++; data[j] = rxData->m_ObsArray[i].flags.isDifferentialPsrAvailable;     //!< Indicates if a matching pseudrange observation is available from another receiver.
    j++; data[j] = rxData->m_ObsArray[i].flags.isDifferentialDopplerAvailable; //!< Indicates if a matching Doppler observation is available from another receiver.
    j++; data[j] = rxData->m_ObsArray[i].flags.isDifferentialAdrAvailable;     //!< Indicates if a matching ADR observation is available from another receiver.
    j++; data[j] = rxData->m_ObsArray[i].flags.useTropoCorrection;             //!< Indicates that the tropospheric correction should be applied.
    j++; data[j] = rxData->m_ObsArray[i].flags.useBroadcastIonoCorrection;     //!< Indicates that the broadcast ionospheric correction should be applied.
    j++; data[j] = rxData->m_ObsArray[i].flags.isBaseSatellite;
    
    j++; data[j] = rxData->m_ObsArray[i].stdev_psr;         //!< The estimated pseudorange measurement standard deviation [m].
    j++; data[j] = rxData->m_ObsArray[i].stdev_adr;         //!< The estimated accumulated Doppler range measurement standard deviation [cycles].
    j++; data[j] = rxData->m_ObsArray[i].stdev_doppler;     //!< The estimated Doppler measurement standard deviation [Hz].

    if( filterType == GNSS_Estimator::GNSS_FILTER_TYPE_LSQ )
    {
      j++; data[j] = rxData->m_ObsArray[i].psr_misclosure_lsq;     //!< The measured psr minus the computed psr estimate [m].
      j++; data[j] = rxData->m_ObsArray[i].doppler_misclosure_lsq; //!< The measured Doppler minus the computed Doppler estimate [m].
    }
    else
    {
      j++; data[j] = rxData->m_ObsArray[i].psr_misclosure;     //!< The measured psr minus the computed psr estimate [m].
      j++; data[j] = rxData->m_ObsArray[i].doppler_misclosure; //!< The measured Doppler minus the computed Doppler estimate [m].
    }
    j++; data[j] = rxData->m_ObsArray[i].adr_misclosure;     //!< The measured ADR minus the computed ADR estimate [m]. This is the between receiver differential adr misclosure.

    j++; data[j] = rxData->m_ObsArray[i].sd_ambiguity;          //!< The estimated single difference float ambiguity [m].
    j++; data[j] = rxData->m_ObsArray[i].dd_ambiguity;       //!< The estimated double difference float ambiguity [m].
    j++; data[j] = rxData->m_ObsArray[i].dd_ambiguity_fixed; //!< The estimated double difference fixed ambiguity [m].

    j++; data[j] = rxData->m_ObsArray[i].adr_residual_sd; //!< The single difference adr residual [m].
    j++; data[j] = rxData->m_ObsArray[i].adr_residual_dd; //!< The double difference adr residual [m].
    j++; data[j] = rxData->m_ObsArray[i].adr_residual_dd_fixed; //!< The double difference adr residual based on the fixed solution [m].


    /*
    fprintf( fid, "GPS time of week(s),GPS week, ID,Channel," );
    fprintf( fid, "System,Code Type,Frequency Type," );
    fprintf( fid, "Elevation (deg), Azimuth (deg)," );
    fprintf( fid, "PSR (m), ADR (cycles), PSR-ADR (m), Doppler (Hz),C/No (dB-Hz),Lock Time (s)," );
    fprintf( fid, "isActive,isCodeLocked,isPhaseLocked,isParityValid,isPsrValid,isAdrValid,isDopplerValid,isGrouped,isAutoAssigned,isCarrierSmoothed," );
    fprintf( fid, "isEphemerisValid,isAlmanacValid,isAboveElevationMask,isAboveCNoMask,isAboveLockTimeMask,isNotUserRejected,isNotPsrRejected,isNotAdrRejected,isNotDopplerRejected,isNoCycleSlipDetected," );
    fprintf( fid, "isPsrUsedInSolution,isDopplerUsedInSolution,isAdrUsedInSolution,isDifferentialPsrAvailable,isDifferentialDopplerAvailable,isDifferentialAdrAvailable,useTropoCorrection,useBroadcastIonoCorrection,isBaseSatellite," );
    fprintf( fid, "stdev PSR (m),stdev adr (cycles),stdev Doppler (Hz)," );
    fprintf( fid, "PSR residual (m), Doppler residual (m/s), ADR residual (m)," );
    fprintf( fid, "SD_ambiguity, DD_ambiguity, DD_fixed_ambiguity\n" );
    fclose( fid );
    */

    result = ptr[index].Concatonate( data );
    if( !result )
    {
      GNSS_ERROR_MSG( "result = ptr[index].Concatonate( data );" );
      return false;
    }  
  }

  
  return true;
}


bool GetNextSetOfSynchronousMeasurements( 
  GNSS_RxData &rxDataBase,  //!< The base station receiver data.
  bool &endOfStreamBase,    //!< A boolean that indicates if the end of the data has been reached for the base station.
  GNSS_RxData &rxData,      //!< The rover station receiver data.
  bool &endOfStreamRover,   //!< A boolean that indicates if the end of the data has been reached for the rover station.
  bool &isSynchronized      //!< A boolean to indicate if the base and rover are synchronized.
  )
{
  bool result = false;
  double timeBase = 0;
  double timeRover = 0;
  double timeDiff = 0;

  isSynchronized = false;

  result = rxDataBase.LoadNext( endOfStreamBase );
  if( !result )
  {
    GNSS_ERROR_MSG( "rxDataBase.LoadNext returned false." );
    return false;
  }
  if( endOfStreamBase )
    return true;

  result = rxData.LoadNext( endOfStreamRover );
  if( !result )
  {
    GNSS_ERROR_MSG( "rxData.LoadNext returned false." );
    return false;
  }

  if( endOfStreamRover )
    return true;

  while( !endOfStreamBase && !endOfStreamRover )
  {
    timeBase  = rxDataBase.m_pvt.time.gps_week*SECONDS_IN_WEEK  + rxDataBase.m_pvt.time.gps_tow;
    timeRover = rxData.m_pvt.time.gps_week*SECONDS_IN_WEEK + rxData.m_pvt.time.gps_tow;
    timeDiff = timeBase - timeRover;

    if( fabs(timeDiff) < 0.020 ) // must match to 20 ms
    {
      isSynchronized = true;
      break; // synchronized
    }

    if( timeDiff < 0 )
    {
      // The rover time is ahead of the base time.
      result = rxDataBase.LoadNext( endOfStreamRover );
      if( !result )
      {
        GNSS_ERROR_MSG( "rxDataBase.LoadNext returned false." );
        return false;
      }
    }
    else
    {
      result = rxData.LoadNext( endOfStreamRover );
      if( !result )
      {
        GNSS_ERROR_MSG( "rxData.LoadNext returned false." );
        return false;
      }
    }
  }

//#define GDM_HACK_ADD_PSR_BLUNDER 1
#ifdef GDM_HACK_ADD_PSR_BLUNDER  
  for( unsigned i = 0; i < rxData.m_nrValidObs; i++ )
  {
    if( rxData.m_ObsArray[i].id == 9 )
    {
      rxData.m_ObsArray[i].psr += 50;
    }
  }
#endif

//#define GDM_HACK_ADD_ADR_BLUNDER
#ifdef GDM_HACK_ADD_ADR_BLUNDER 
  if( rxData.m_pvt.time.gps_tow > 30 && rxData.m_pvt.time.gps_tow < 12345 )
  {
  for( unsigned i = 0; i < rxData.m_nrValidObs; i++ )
  {
    if( rxData.m_ObsArray[i].id == 12 )
    {
      rxData.m_ObsArray[i].adr += 10.75;
    }
  }
  }
#endif

  return true;
}









