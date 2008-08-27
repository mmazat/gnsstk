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

bool OutputObservationData( GNSS_RxData* rxData, bool isRover );

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
  
  GNSS_OptionFile opt;

  std::string OptionFilePath;

  bool useLSQ = true;
  bool useEKF = false;
  bool useRTK = false;
  bool isFilterInitialized = false;
  
  bool isAtFirstEpoch = true;

  char fname[24];

  Matrix PVT;

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

    // Delete any GPSL1_svXX.csv files in the working directory.
    for( i = 0; i < 32; i++ )
    {
      sprintf( fname, "GPSL1_PRN%02d.csv", i );
      fid = fopen( fname, "r" );
      if( fid )
      {
        fclose(fid);
        remove( fname );
        fid = NULL;
      }
      sprintf( fname, "GPSL1_PRN%02d_BASE.csv", i );
      fid = fopen( fname, "r" );
      if( fid )
      {
        fclose(fid);
        remove( fname );
        fid = NULL;
      }
    }

    
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
      result = rxData.EnableAndLoadUWBData( opt.m_UWBFilePath.c_str(), opt.m_Reference.x, opt.m_Reference.y, opt.m_Reference.z, true );
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
      if( rxData.m_pvt.time.gps_tow > 319273 )
        int ggg = 99;

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

      // Enable constraints if any.
      if( opt.m_isPositionConstrained )
      {
        rxData.m_pvt.isPositionConstrained = true;
      }
      else if( opt.m_isHeightConstrained )
      {
        rxData.m_pvt.isHeightConstrained = true;
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
        // Is there a valid least squares position estimate?
        if( !wasPositionComputed )
        {
          continue;
        }
        else
        {
          // Seed the filter with the least squares solution.
          rxData.m_pvt = rxData.m_pvt_lsq;
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
        else
        {
          GNSS_ERROR_MSG( "Unexpected." );
          return 1;
        }
        isFilterInitialized = true;
      }
      
      if( useEKF )
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

      /*
      if( !OutputObservationData( &rxData, true ) )
      {
        sprintf( msg, "%.3Lf %d OutputObservationData Failed\n", rxData.m_pvt.time.gps_tow, rxData.m_pvt.time.gps_week );
        GNSS_ERROR_MSG( msg );
        return 1;
      }
      if( !OutputObservationData( &rxDataBase, false ) )
      {
        sprintf( msg, "%.3Lf %d OutputObservationData Failed\n", rxData.m_pvt.time.gps_tow, rxData.m_pvt.time.gps_week );
        GNSS_ERROR_MSG( msg );
        return 1;
      }
      */
    }

    if( !PVT.Inplace_Transpose() )
    {
      GNSS_ERROR_MSG( "if( !PVT.Inplace_Transpose() )" );
      return 1;
    }
    
    // Open the PVT output file
    fid_pvt = fopen( "pvt.csv", "w" );
    if( !fid_pvt )
    {
      sprintf( msg, "Unable to open pvt.csv." );
      GNSS_ERROR_MSG( msg );
      return 1;
    }
    fprintf( fid_pvt, "GPS time of week(s), GPS week, latitude (deg), longitude (deg), height (m), Velocity North (m/s), Velocity East (m/s), Velocity Up (m/s), Ground Speed (km/hr), Clock Offset (m), Clock Drift (m/s), " );
    if( opt.m_RoverIsStatic )
      fprintf( fid_pvt, "Error North (m), Error East (m), Error Up (m),");
    else
      fprintf( fid_pvt, "Northing (m), Easting (m), Up (m),");
    fprintf( fid_pvt, "STDEV latitude (m), STDEV longitude (m), STDEV height (m), STDEV Velocity North (m/s), STDEV Velocity East (m/s), STDEV Velocity Up (m/s), STDEV Clock Offset (m), STDEV Clock Drift (m/s), " );
    fprintf( fid_pvt, "NR PSR Available, NR PSR Used, NR Doppler Available, NR Doppler Used, NR ADR Available, NR ADR Used," );
    fprintf( fid_pvt, "NDOP,EDOP,VDOP,HDOP,PDOP,TDOP,GDOP,");    
    fprintf( fid_pvt, "APVF Position, APVF Velocity\n");    
    fclose( fid_pvt );

    if( !PVT.PrintDelimited( "pvt.csv", 12, ',', true )  )
    {
      GNSS_ERROR_MSG( "if( !PVT.PrintDelimited( \"pvt.csv\", 12, ',', true )  )" );
      return 1;
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

  return 0;
}


bool OutputPVT(
  FILE* fid,            //!< The output file. This must already be open.
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

  if( fid == NULL )
  {
    GNSS_ERROR_MSG( "if( fid == NULL )" );
    return false;
  }

  if( once )
  {
    fprintf( fid, "GPS time of week(s), GPS week, latitude (deg), longitude (deg), height (m), Velocity North (m/s), Velocity East (m/s), Velocity Up (m/s), Ground Speed (km/hr), Clock Offset (m), Clock Drift (m/s), " );
    if( isStatic )
      fprintf( fid, "Error North (m), Error East (m), Error Up (m),");
    else
      fprintf( fid, "Northing (m), Easting (m), Up (m),");
    fprintf( fid, "STDEV latitude (m), STDEV longitude (m), STDEV height (m), STDEV Velocity North (m/s), STDEV Velocity East (m/s), STDEV Velocity Up (m/s), STDEV Clock Offset (m), STDEV Clock Drift (m/s), " );
    fprintf( fid, "NR PSR Available, NR PSR Used, NR Doppler Available, NR Doppler Used, NR ADR Available, NR ADR Used," );
    fprintf( fid, "NDOP,EDOP,VDOP,HDOP,PDOP,TDOP,GDOP,");    
    fprintf( fid, "APVF Position, APVF Velocity, latitude_fixed (deg), longitude_fixed (deg), height_fixed (m), Error_N_fixed, Error_E_fixed, Error_Up_fixed \n");    
    once = false;
  }

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
  
  fprintf( fid, "%.10g,%d,%.13g,%.14g,%.8g,%.7g,%.7g,%.7g,%.7g,%.13g,%.9g,",
        rxData.m_pvt.time.gps_tow,
        rxData.m_pvt.time.gps_week,
        rxData.m_pvt.latitudeDegs,
        rxData.m_pvt.longitudeDegs,
        rxData.m_pvt.height,
        rxData.m_pvt.vn,
        rxData.m_pvt.ve,
        rxData.m_pvt.vup,
        sqrt( rxData.m_pvt.vn*rxData.m_pvt.vn + rxData.m_pvt.ve*rxData.m_pvt.ve)*3.6,
        rxData.m_pvt.clockOffset,
        rxData.m_pvt.clockDrift
        );        

  fprintf( fid, "%.8g,%.8g,%.8g,", northing, easting, up );
  

  fprintf( fid, "%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,",
        rxData.m_pvt.std_lat,
        rxData.m_pvt.std_lon,
        rxData.m_pvt.std_hgt,
        rxData.m_pvt.std_vn,
        rxData.m_pvt.std_ve,
        rxData.m_pvt.std_vup,
        rxData.m_pvt.std_clk,
        rxData.m_pvt.std_clkdrift
        );        

  fprintf( fid, "%d,%d,%d,%d,%d,%d,",
    rxData.m_pvt.nrPsrObsAvailable,
    rxData.m_pvt.nrPsrObsUsed,
    rxData.m_pvt.nrDopplerObsAvailable,
    rxData.m_pvt.nrDopplerObsUsed,
    rxData.m_pvt.nrAdrObsAvailable,
    rxData.m_pvt.nrAdrObsUsed    
    );

  fprintf( fid, "%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,%.8g,",
    rxData.m_pvt.dop.ndop,
    rxData.m_pvt.dop.edop,
    rxData.m_pvt.dop.vdop,
    rxData.m_pvt.dop.hdop,
    rxData.m_pvt.dop.pdop,
    rxData.m_pvt.dop.tdop,
    rxData.m_pvt.dop.gdop
    );

  fprintf( fid, "%.5g,%.5g",
    rxData.m_pvt.pos_apvf,
    rxData.m_pvt.vel_apvf
    );

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

   fprintf( fid, "%.13g,%.14g,%.8g,%.7g,%.7g,%.7g\n", 
     rxData.m_pvt_fixed.latitudeDegs,
     rxData.m_pvt_fixed.longitudeDegs,
     rxData.m_pvt_fixed.height,
     northing,
     easting, 
     up
    );

  if( result == FALSE )
  {
    GNSS_ERROR_MSG( "GEODESY_ComputePositionDifference returned FALSE." );
    return false;
  }

  fflush( fid );

  return true;
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
  const unsigned nr_items = 37+6;
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
  data[i] = rxData.m_pvt_lsq.vel_apvf; i++;

  data[i] = rxData.m_pvt_fixed.latitudeDegs; i++;
  data[i] = rxData.m_pvt_fixed.longitudeDegs; i++;
  data[i] = rxData.m_pvt_fixed.height; i++;

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

  data[i] = northing; i++;
  data[i] = easting; i++;
  data[i] = up; i++;

  if( result == FALSE )
  {
    GNSS_ERROR_MSG( "GEODESY_ComputePositionDifference returned FALSE." );
    return false;
  }


  if( PVT.isEmpty() )
  {
    PVT = data;
  }
  else
  {
    if( !PVT.AddColumn( data, 0 ) )
    {
      GNSS_ERROR_MSG( "if( !PVT.AddColumn( data, 0 ) )" );
      return false;
    }
  }  
  return true;
}



bool OutputObservationData( GNSS_RxData* rxData, bool isRover )
{
  unsigned i = 0;
  char msg[256];
  char fname[24];
  FILE* fid = NULL;
  
  for( i = 0; i < rxData->m_nrValidObs; i++ )
  {
    fid = NULL;

    if( rxData->m_ObsArray[i].freqType == GNSS_GPSL1 )
    {
      if( isRover )
        sprintf( fname, "GPSL1_PRN%02d.csv", rxData->m_ObsArray[i].id );
      else
        sprintf( fname, "GPSL1_PRN%02d_BASE.csv", rxData->m_ObsArray[i].id );
      
      // check if file exists
      fid = fopen( fname, "r" );
      if( !fid )
      {
        fid = fopen( fname, "w" );
        if( !fid )
        {
          sprintf( msg, "Unable to open %s\n", fname );
          GNSS_ERROR_MSG( msg );
          return false;
        }
        fprintf( fid, "GPS time of week(s),GPS week, ID,Channel," );
        fprintf( fid, "System,Code Type,Frequency Type," );
        fprintf( fid, "Elevation (deg), Azimuth (deg)," );
        fprintf( fid, "PSR (m), ADR (cycles), PSR-ADR (m), Doppler (Hz),C/No (dB-Hz),Lock Time (s)," );
        fprintf( fid, "isActive,isCodeLocked,isPhaseLocked,isParityValid,isPsrValid,isAdrValid,isDopplerValid,isGrouped,isAutoAssigned,isCarrierSmoothed," );
        fprintf( fid, "isEphemerisValid,isAlmanacValid,isAboveElevationMask,isAboveCNoMask,isAboveLockTimeMask,isNotUserRejected,isNotPsrRejected,isNotAdrRejected,isNotDopplerRejected,isNoCycleSlipDetected," );
        fprintf( fid, "isPsrUsedInSolution,isDopplerUsedInSolution,isAdrUsedInSolution,isDifferentialPsrAvailable,isDifferentialDopplerAvailable,isDifferentialAdrAvailable,useTropoCorrection,useBroadcastIonoCorrection,isTimeDifferntialPsrAvailable,isTimeDifferntialDopplerAvailable," );
        fprintf( fid, "stdev PSR (m),stdev adr (cycles),stdev Doppler (Hz)," );
        fprintf( fid, "PSR residual (m), Doppler residual (m/s), ADR residual (m)," );
        fprintf( fid, "ambiguity\n" );
      }
      else
      {
        fclose(fid);
        fid = NULL;
        fid = fopen( fname, "a+" );
        if( !fid )
        {
          sprintf( msg, "Unable to open %s\n", fname );
          GNSS_ERROR_MSG( msg );
          return false;
        }
      }

      fprintf( fid, "%.3Lf,%d,%d,%d,",
        rxData->m_pvt.time.gps_tow,
        rxData->m_pvt.time.gps_week,
        rxData->m_ObsArray[i].id,
        rxData->m_ObsArray[i].channel );

      fprintf( fid, "%d,%d,%d,",
        rxData->m_ObsArray[i].system,   //!< The satellite system associated with this channel.
        rxData->m_ObsArray[i].codeType, //!< The code type for this channel.
        rxData->m_ObsArray[i].freqType ); //!< The frequency type for this channel.

      fprintf( fid, "%.2Lf,%.2Lf,", 
        rxData->m_ObsArray[i].satellite.elevation*RAD2DEG,
        rxData->m_ObsArray[i].satellite.azimuth*RAD2DEG );

      fprintf( fid, "%.4Lf,%.4Lf,%.4Lf,%.3Lf,%.1f,%.2f,",
        rxData->m_ObsArray[i].psr,
        rxData->m_ObsArray[i].adr,
        rxData->m_ObsArray[i].psr-(rxData->m_ObsArray[i].adr*GPS_WAVELENGTHL1),
        rxData->m_ObsArray[i].doppler,
        rxData->m_ObsArray[i].cno,
        rxData->m_ObsArray[i].locktime );

      fprintf( fid, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,",
        rxData->m_ObsArray[i].flags.isActive,                //!< This flag indicates that the channel is active for use. If this is not set, no other flags are valid for use.
        rxData->m_ObsArray[i].flags.isCodeLocked,            //!< Indicates if the code tracking is locked.
        rxData->m_ObsArray[i].flags.isPhaseLocked,           //!< Indicates if the phase tracking is locked.
        rxData->m_ObsArray[i].flags.isParityValid,           //!< Indicates if the phase parity if valid.      
        rxData->m_ObsArray[i].flags.isPsrValid,              //!< Indicates if the pseudorange valid for use.
        rxData->m_ObsArray[i].flags.isAdrValid,              //!< Indicates if the ADR is valid for use.
        rxData->m_ObsArray[i].flags.isDopplerValid,          //!< Indicates if the Doppler if valid for use.
        rxData->m_ObsArray[i].flags.isGrouped,               //!< Indicates if this channel has another associated channel. eg. L1 and L2 measurements.
        rxData->m_ObsArray[i].flags.isAutoAssigned,          //!< Indicates if the channel was receiver assigned (otherwise, the user forced this channel assignment).
        rxData->m_ObsArray[i].flags.isCarrierSmoothed );       //!< Indicates if the pseudorange has carrier smoothing enabled.
        
      fprintf( fid, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,",
        rxData->m_ObsArray[i].flags.isEphemerisValid,        //!< Indicates if this channel has valid associated ephemeris information. 
        rxData->m_ObsArray[i].flags.isAlmanacValid,          //!< Indicates if this channel has valid associated almanac information.
        rxData->m_ObsArray[i].flags.isAboveElevationMask,    //!< Indicates if the satellite tracked is above the elevation mask.    
        rxData->m_ObsArray[i].flags.isAboveCNoMask,          //!< Indciates if the channel's C/No is above a threshold value.
        rxData->m_ObsArray[i].flags.isAboveLockTimeMask,     //!< Indicates if the channel's locktime is above a treshold value.
        rxData->m_ObsArray[i].flags.isNotUserRejected,       //!< Indicates if the user has not forced the rejection of this channel or PRN.
        rxData->m_ObsArray[i].flags.isNotPsrRejected,        //!< Indicates if the pseudorange was not rejetced (ie Fault Detection and Exclusion).
        rxData->m_ObsArray[i].flags.isNotAdrRejected,        //!< Indicates if the ADR was not rejetced (ie Fault Detection and Exclusion).
        rxData->m_ObsArray[i].flags.isNotDopplerRejected,    //!< Indicates if the Doppler was not rejected (ie Fault Detection and Exclusion).
        rxData->m_ObsArray[i].flags.isNoCycleSlipDetected );   //!< Indicates that no cycle slip has occurred at this epoch.
        
      fprintf( fid, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,",
        rxData->m_ObsArray[i].flags.isPsrUsedInSolution,     //!< Indicates if some part (pseudorange) of this channel's measurement was used in the position solution.
        rxData->m_ObsArray[i].flags.isDopplerUsedInSolution, //!< Indicates if some part (Doppler) of this channel's measurement was used in the velocity solution.
        rxData->m_ObsArray[i].flags.isAdrUsedInSolution,     //!< Indicates if the the ADR is used in the solution.
        rxData->m_ObsArray[i].flags.isDifferentialPsrAvailable,     //!< Indicates if a matching pseudrange observation is available from another receiver.
        rxData->m_ObsArray[i].flags.isDifferentialDopplerAvailable, //!< Indicates if a matching Doppler observation is available from another receiver.
        rxData->m_ObsArray[i].flags.isDifferentialAdrAvailable,     //!< Indicates if a matching ADR observation is available from another receiver.
        rxData->m_ObsArray[i].flags.useTropoCorrection,         //!< Indicates that the tropospheric correction should be applied.
        rxData->m_ObsArray[i].flags.useBroadcastIonoCorrection, //!< Indicates that the broadcast ionospheric correction should be applied.
        rxData->m_ObsArray[i].flags.isTimeDifferentialPsrAvailable,
        rxData->m_ObsArray[i].flags.isTimeDifferentialDopplerAvailable );

      fprintf( fid, "%.3f,%.3f,%.3f,",
        rxData->m_ObsArray[i].stdev_psr,         //!< The estimated pseudorange measurement standard deviation [m].
        rxData->m_ObsArray[i].stdev_adr,         //!< The estimated accumulated Doppler range measurement standard deviation [cycles].
        rxData->m_ObsArray[i].stdev_doppler );     //!< The estimated Doppler measurement standard deviation [Hz].

      fprintf( fid, "%.3f,%.3f,%.3f,",
        rxData->m_ObsArray[i].psr_misclosure,     //!< The measured psr minus the computed psr estimate [m].
        rxData->m_ObsArray[i].doppler_misclosure, //!< The measured Doppler minus the computed Doppler estimate [m].
        rxData->m_ObsArray[i].adr_misclosure );   //!< The measured ADR minus the computed ADR estimate [m]. This is the between receiver differential adr misclosure.

      fprintf( fid, "%.3Lf\n",  
        rxData->m_ObsArray[i].ambiguity );         //!< The estimated float ambiguity [m].    

      fclose( fid );
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

    if( fabs(timeDiff) < 0.010 ) // must match to 10 ms
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

  return true;
}









