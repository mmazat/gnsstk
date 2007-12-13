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


/// \brief    Try to sychronize the base and rover measurement sources.
/// \return   true if successful, false if error.
bool GetNextSetOfSynchronousMeasurements( 
  GNSS_RxData &rxDataBase,  //!< The base station receiver data.
  bool &endOfStreamBase,    //!< A boolean that indicates if the end of the data has been reached for the base station.
  GNSS_RxData &rxDataRover, //!< The rover station receiver data.
  bool &endOfStreamRover,   //!< A boolean that indicates if the end of the data has been reached for the rover station.
  bool &isSynchronized      //!< A boolean to indicate if the base and rover are synchronized.
  );


int main( int argc, char* argv[] )
{
  GNSS_RxData rxDataBase;
  GNSS_RxData rxDataRover;
  GNSS_Estimator Estimator;
  bool isValidPath;
  bool result;
  bool endOfStreamBase = false;
  bool endOfStreamRover = false;
  bool isSynchronized = false;

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
#ifdef EXTRACTSVDATA
  FILE *svfid = NULL;
  unsigned short prn = 11;
#endif

  GNSS_OptionFile opt;

  std::string OptionFilePath;

  bool useLSQ = true;
  bool useEKF = false;
  bool useRTK = false;
  
  bool isAtFirstEpoch = true;

  GNSS_structPVT firstPVT;
  GNSS_structPVT secondPVT;
  bool firstPVT_isSet = false;

  try
  {
    printf( "\nUSAGE: EGNSS <option file path>\n" );
    if( argc != 2 )
    {      
      return 0;
    }
    OptionFilePath = argv[1];

    if( !opt.ReadAndInterpretOptions( OptionFilePath ) )
    {
      printf( "\nInvalid option file.\n" );
      return 1;
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
    }
    else if( opt.m_ProcessingMethod == "EKF" )
    {
      useLSQ = true; // least squares is used for the first epoch
      useEKF = true;
      useRTK = false;
    }
    else if( opt.m_ProcessingMethod == "RTK" )
    {
      useLSQ = true; // least squares is used for the first epoch
      useEKF = false;
      useRTK  = true;
    }


#ifndef _CRT_SECURE_NO_DEPRECATE    
    // Open the output file.
    if( fopen_s( &fid, opt.m_OutputFilePath.c_str(), "w" ) != 0 )
      return 1;
#else
    fid = fopen( opt.m_OutputFilePath.c_str(), "w" );
    if( fid == NULL )
      return 1;
#endif
    
//#define EXTRACTSVDATA      
#ifdef EXTRACTSVDATA
    //if( fopen_s( &svfid, "C:\\Zen\\Zenautics\\repos\\prj\\ZNav\\bin\\rover\\oem4wc\\ZNAV_SV.txt", "w" ) != 0 )
      //return 1;

#ifndef _CRT_SECURE_NO_DEPRECATE    
    if( fopen_s( &svfid, "prn.txt", "w" ) != 0 )
      return 1;
#else
    svfid = fopen( "theprn.txt", "w" );
    if( svfid == NULL )
      return 1;
#endif
    
#endif
    
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
        printf( "\nBad path to the reference station data.\n" );
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
        0.0, 0.0, 0.0,  // position is fixed
        0.0, 0.0, 0.0,
        100.0,
        10.0 ); // position is fixed
      if( !result )
        return 1;
    }

    if( !opt.m_Rover.isValid )
    {
      return 1;
    }

    if( !opt.m_Reference.isValid && opt.m_RINEXNavDataPath.length() != 0 )
    {
      // Stand-Alone mode
      result = rxDataRover.Initialize( opt.m_Rover.DataPath.c_str(), isValidPath, opt.m_Rover.DataType, opt.m_RINEXNavDataPath.c_str() );
    }
    else
    {
      result = rxDataRover.Initialize( opt.m_Rover.DataPath.c_str(), isValidPath, opt.m_Rover.DataType, NULL );
    }
    if( !result )
      return 1;
    if( opt.m_klobuchar.isValid )
    {
      rxDataRover.m_klobuchar = opt.m_klobuchar;
      if( !opt.m_Reference.useIono )
        rxDataRover.m_DisableIonoCorrection = true;
    }
    else
    {
      rxDataRover.m_DisableIonoCorrection = true;
    }
    rxDataRover.m_elevationMask = opt.m_elevationMask*DEG2RAD;
    rxDataRover.m_locktimeMask  = opt.m_locktimeMask;
    rxDataRover.m_cnoMask       = opt.m_cnoMask;

    if( !opt.m_Rover.useTropo )
    {
      rxDataRover.m_DisableTropoCorrection = true;
    }

    
    result = rxDataRover.SetInitialPVT(
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
      return 1; 

    if( !opt.m_Reference.isValid )
    {
      ////
      // This is processing of stand-alone data.

      while( !endOfStreamRover )
      {
        result = rxDataRover.LoadNext( endOfStreamRover );
        if( !result )
          return 1;
        if( rxDataRover.m_pvt.time.gps_week == 0 )
          continue;


        time = rxDataRover.m_pvt.time.gps_week*SECONDS_IN_WEEK + rxDataRover.m_pvt.time.gps_tow;

        if( time < start_time )
          continue;

        if( time > end_time )
          break;



        // exclude satellites as indicated in the option file
        for( j = 0; j < opt.m_Rover.nrSatsToExclude; j++ )
        {
          for( i = 0; i < rxDataRover.m_nrValidObs; i++ )
          {
            if( rxDataRover.m_ObsArray[i].id == opt.m_Rover.satsToExclude[j] )
            {
              rxDataRover.m_ObsArray[i].flags.isNotUserRejected = 0;
            }
          }
        } 

        if( useLSQ )
        {      
          result = Estimator.PerformLeastSquares_8StatePVT(
            &rxDataRover,
            NULL,
            wasPositionComputed,
            wasVelocityComputed );
          if( !result )
            return 1;

          if( !wasPositionComputed )
            continue;

          if( wasPositionComputed && wasVelocityComputed )
          {
            if( useEKF || useRTK )
            {
              useLSQ = false; // position/velocity is now seeded
              result = Estimator.InitializeStateVarianceCovariance_8StatePVGM(
                rxDataRover.m_pvt.std_lat,
                rxDataRover.m_pvt.std_lon,
                rxDataRover.m_pvt.std_hgt,
                rxDataRover.m_pvt.std_vn,
                rxDataRover.m_pvt.std_ve,
                rxDataRover.m_pvt.std_vup,
                rxDataRover.m_pvt.std_clk,
                rxDataRover.m_pvt.std_clkdrift,
                Estimator.m_P );
              if( !result )
                return 1;
            }
          }
        }
        else if( useEKF )
        {
          result = Estimator.PredictAhead_8StatePVGM(
            rxDataRover,
            dT,
            Estimator.m_T,
            Estimator.m_Q,
            Estimator.m_P );
          if( !result )
            return 1;

          result = Estimator.Kalman_Update_8StatePVGM(
            &rxDataRover,
            NULL,
            Estimator.m_P );
          if( !result )
            return 1;
        }
        else if( useRTK )
        {
          result = Estimator.PredictAhead_8StatePVGM_Float(
            rxDataRover,
            dT,
            Estimator.m_T,
            Estimator.m_Q,
            Estimator.m_P );
          if( !result )
            return 1;

          result = Estimator.Kalman_Update_8StatePVGM_SequentialMode_FloatSolution(
            &rxDataRover,
            NULL,
            Estimator.m_P );
          if( !result )
            return 1;
        }

        printf( "%12.3lf %5d %d %d %d \n", 
          rxDataRover.m_pvt.time.gps_tow, 
          rxDataRover.m_pvt.time.gps_week,         
          rxDataRover.m_pvt.nrPsrObsUsed,
          rxDataRover.m_pvt.nrDopplerObsUsed,
          rxDataRover.m_pvt.nrAdrObsUsed );    


        /*
        rxDataRover.Debug_WriteSuperMsg80CharsWide( 
          supermsg,
          8192,
          51.0916666667*DEG2RAD,
          -114.0000000000*DEG2RAD,
          1000.000,
          nrBytesInBuffer );

        printf( supermsg );
        */


        fprintf( fid, "%12.4lf %4d %20.10lf %20.10lf %15.3lf %20.10lf %4d %10.4lf %10.4lf %10.4lf %20.10lf %10.4lf %10.4lf %10.4lf %10.4lf %10.2f %10.2f %10.2f\n", 
          rxDataRover.m_pvt.time.gps_tow,
          rxDataRover.m_pvt.time.gps_week,
          rxDataRover.m_pvt.latitudeDegs,
          rxDataRover.m_pvt.longitudeDegs,
          rxDataRover.m_pvt.height,
          rxDataRover.m_pvt.clockOffset,
          rxDataRover.m_pvt.nrPsrObsUsed,
          rxDataRover.m_pvt.vn,
          rxDataRover.m_pvt.ve,
          rxDataRover.m_pvt.vup,
          rxDataRover.m_pvt.clockDrift,
          rxDataRover.m_pvt.std_lat,
          rxDataRover.m_pvt.std_lon,
          rxDataRover.m_pvt.std_hgt,
          rxDataRover.m_pvt.std_clk,
          rxDataRover.m_pvt.dop.hdop,
          rxDataRover.m_pvt.dop.vdop,
          rxDataRover.m_pvt.dop.tdop        
          );        
      }
    }
    else
    {
      while( !endOfStreamBase && !endOfStreamRover )
      {
        if( !isAtFirstEpoch ) 
          time_prev = rxDataRover.m_pvt.time.gps_week*SECONDS_IN_WEEK + rxDataRover.m_pvt.time.gps_tow;

        result = GetNextSetOfSynchronousMeasurements( 
          rxDataBase,
          endOfStreamBase,
          rxDataRover,
          endOfStreamRover,
          isSynchronized );
        if( !result )
          return 1;

        if( endOfStreamRover || endOfStreamBase )
          break;

        if( rxDataRover.m_nrValidObs == 0 )
          continue;

        // Check that the processing time is within the processing interval.
        time = rxDataRover.m_pvt.time.gps_week*SECONDS_IN_WEEK + rxDataRover.m_pvt.time.gps_tow;
        if( time < start_time )
          continue;
        if( time > end_time )
          break;

        if( rxDataRover.m_pvt.time.gps_tow > 242005 )
          int ggg = 99;

        if( rxDataRover.m_pvt.time.gps_tow > 353172 )
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
            printf( "\ndT is negative!\n");
            return 1; 
          }        
        }

        
        // exclude satellites as indicated in the option file
        for( j = 0; j < opt.m_Rover.nrSatsToExclude; j++ )
        {
          for( i = 0; i < rxDataRover.m_nrValidObs; i++ )
          {
            if( rxDataRover.m_ObsArray[i].id == opt.m_Rover.satsToExclude[j] )
            {
              rxDataRover.m_ObsArray[i].flags.isNotUserRejected = 0;
            }
          }
        } 

        if( useLSQ )
        {      
          result = Estimator.PerformLeastSquares_8StatePVT(
            &rxDataRover,
            &rxDataBase,
            wasPositionComputed,
            wasVelocityComputed );
          if( !result )
            return 1;

          if( !wasPositionComputed ) //KO Could change this to position only to seed filters
            continue;

          if( !wasVelocityComputed )
          {            
            // Compute two consective position fixes and perform the difference
            // to initial the velocity filter.            
            if( !firstPVT_isSet )
            {
              firstPVT = rxDataRover.m_pvt;
              firstPVT_isSet = true;
              continue;
            }
            else
            {
              secondPVT = rxDataRover.m_pvt;

              double N = 0;
              double M = 0;
              double deltaTime = (secondPVT.time.gps_week*SECONDS_IN_WEEK + secondPVT.time.gps_tow) -
                (firstPVT.time.gps_week*SECONDS_IN_WEEK + firstPVT.time.gps_tow);

              GEODESY_ComputeMeridianRadiusOfCurvature(
                GEODESY_REFERENCE_ELLIPSE_WGS84,
                rxDataRover.m_pvt.latitude,
                &M );

              GEODESY_ComputePrimeVerticalRadiusOfCurvature(
                GEODESY_REFERENCE_ELLIPSE_WGS84,
                rxDataRover.m_pvt.latitude,
                &N );

              result = rxDataRover.UpdateVelocityAndClockDrift(
                (secondPVT.latitude - firstPVT.latitude)*(M+secondPVT.height),
                (secondPVT.longitude - firstPVT.longitude)*((N+secondPVT.height)*cos(secondPVT.latitude)),
                secondPVT.height - firstPVT.height,
                secondPVT.clockOffset - firstPVT.clockOffset,
                5.0*deltaTime,
                5.0*deltaTime,
                5.0*deltaTime,
                5.0*deltaTime
                );     
              if( !result )
                return -1;
              wasVelocityComputed = true;
            }            
          }

          if( wasPositionComputed && wasVelocityComputed )
          {
            if( useEKF || useRTK )
            {            
              useLSQ = false; // position/velocity is now seeded
              result = Estimator.InitializeStateVarianceCovariance_8StatePVGM(
                rxDataRover.m_pvt.std_lat,
                rxDataRover.m_pvt.std_lon,
                rxDataRover.m_pvt.std_hgt,
                rxDataRover.m_pvt.std_vn,
                rxDataRover.m_pvt.std_ve,
                rxDataRover.m_pvt.std_vup,
                rxDataRover.m_pvt.std_clk,
                rxDataRover.m_pvt.std_clkdrift,
                Estimator.m_P );  //KO Could use LS m_P here to keep all information from LS step.
              
              if( !result )
                return 1;
            }
          }
        }
        else if( useEKF )
        {
          result = Estimator.PredictAhead_8StatePVGM(
            rxDataRover,
            dT,
            Estimator.m_T,
            Estimator.m_Q,
            Estimator.m_P );
          if( !result )
            return 1;

          result = Estimator.Kalman_Update_8StatePVGM(
            &rxDataRover,
            &rxDataBase,
            Estimator.m_P );
          if( !result )
            return 1;
        }
        else if( useRTK )
        {
          result = Estimator.PredictAhead_8StatePVGM_Float(
            rxDataRover,
            dT,
            Estimator.m_T,
            Estimator.m_Q,
            Estimator.m_P );
          if( !result )
            return 1;

          result = Estimator.Kalman_Update_8StatePVGM_SequentialMode_FloatSolution(
            &rxDataRover,
            &rxDataBase,
            Estimator.m_P );
          if( !result )
            return 1;
        }

        printf( "%12.3lf %5d %d %d %d \n", 
          rxDataRover.m_pvt.time.gps_tow, 
          rxDataRover.m_pvt.time.gps_week,         
          rxDataRover.m_pvt.nrPsrObsUsed,
          rxDataRover.m_pvt.nrDopplerObsUsed,
          rxDataRover.m_pvt.nrAdrObsUsed );    

        /*
        char supermsg[8192];
        unsigned nrBytesInBuffer;
        rxDataRover.Debug_WriteSuperMsg80CharsWide( 
          supermsg,
          8192,
          51.0916666667*DEG2RAD,
          -114.0000000000*DEG2RAD,
          1000.000,
          nrBytesInBuffer );

        printf( supermsg );
        */


        rxDataRover.m_prev_pvt = rxDataRover.m_pvt;
        rxDataBase.m_prev_pvt = rxDataBase.m_pvt;

        fprintf( fid, "%12.4lf %4d %20.10lf %20.10lf %15.3lf %20.10lf %4d %10.4lf %10.4lf %10.4lf %20.10lf %10.4lf %10.4lf %10.4lf %10.4lf %10.2f %10.2f %10.2f\n", 
          rxDataRover.m_pvt.time.gps_tow,
          rxDataRover.m_pvt.time.gps_week,
          rxDataRover.m_pvt.latitudeDegs,
          rxDataRover.m_pvt.longitudeDegs,
          rxDataRover.m_pvt.height,
          rxDataRover.m_pvt.clockOffset,
          rxDataRover.m_pvt.nrPsrObsUsed,
          rxDataRover.m_pvt.vn,
          rxDataRover.m_pvt.ve,
          rxDataRover.m_pvt.vup,
          rxDataRover.m_pvt.clockDrift,
          rxDataRover.m_pvt.std_lat,
          rxDataRover.m_pvt.std_lon,
          rxDataRover.m_pvt.std_hgt,
          rxDataRover.m_pvt.std_clk,
          rxDataRover.m_pvt.dop.hdop,
          rxDataRover.m_pvt.dop.vdop,
          rxDataRover.m_pvt.dop.tdop        
          );        

#ifdef EXTRACTSVDATA
        for( i = 0; i < rxDataRover.m_nrValidObs; i++ )
        {
          if( rxDataRover.m_ObsArray[i].id == prn &&           
            rxDataRover.m_ObsArray[i].system == GNSS_GPS &&
            rxDataRover.m_ObsArray[i].freqType == GNSS_GPSL1 
            ) 
          {
            fprintf( svfid, "%12.4lf %4d %10.1f %10.1f %10.1f %20.10lf %20.10lf %20.10lf %20.10lf\n", 
              rxDataRover.m_pvt.time.gps_tow,
              rxDataRover.m_pvt.time.gps_week,          
              rxDataRover.m_ObsArray[i].cno,
              rxDataRover.m_ObsArray[i].satellite.elevation*RAD2DEG,
              rxDataRover.m_ObsArray[i].satellite.azimuth*RAD2DEG,
              rxDataRover.m_ObsArray[i].psr_misclosure,
              rxDataRover.m_ObsArray[i].doppler_misclosure,
              rxDataRover.m_ObsArray[i].psr,
              rxDataRover.m_ObsArray[i].adr
              );
          }
        }        
#endif


      }
    }

    // close the output file
    fclose( fid );
#ifdef EXTRACTSVDATA
    fclose(svfid);
#endif
  }
  catch( MatrixException& matrixException )
  {
    printf( "%s", matrixException.GetExceptionMessage().c_str() );
  }






  return 0;
}



bool GetNextSetOfSynchronousMeasurements( 
  GNSS_RxData &rxDataBase,  //!< The base station receiver data.
  bool &endOfStreamBase,    //!< A boolean that indicates if the end of the data has been reached for the base station.
  GNSS_RxData &rxDataRover, //!< The rover station receiver data.
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
    return false;
  if( endOfStreamBase )
    return true;

  result = rxDataRover.LoadNext( endOfStreamRover );
  if( !result )
    return false;
  if( endOfStreamRover )
    return true;

  while( !endOfStreamBase && !endOfStreamRover )
  {
    timeBase  = rxDataBase.m_pvt.time.gps_week*SECONDS_IN_WEEK  + rxDataBase.m_pvt.time.gps_tow;
    timeRover = rxDataRover.m_pvt.time.gps_week*SECONDS_IN_WEEK + rxDataRover.m_pvt.time.gps_tow;
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
        return 1;
    }
    else
    {
      result = rxDataRover.LoadNext( endOfStreamRover );
      if( !result )
        return 1;
    }
  }

  return true;
}









