/**
\file    GNSS_OptionFile.cpp
\brief   The option file for EssentialGNSS.

\author  Glenn D. MacGougan (GDM)
\date    2007-11-28
\since   2006-12-30

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

#include <memory.h>
#include "constants.h"
#include "GNSS_OptionFile.h"
#include "rinex.h"
#include "geodesy.h"

namespace GNSS
{

  GNSS_OptionFile::~GNSS_OptionFile()
  {
  }


  GNSS_OptionFile::GNSS_OptionFile()
    : m_elevationMask(0.0),
    m_cnoMask(0.0),
    m_locktimeMask(0.0)
  {     
    memset( &m_klobuchar, 0, sizeof(m_klobuchar) );
  }



  bool GNSS_OptionFile::ReadAndInterpretOptions( const std::string OptionFilePath )
  {
    double d[4];
    unsigned n;
    bool result;
    bool useECEF = false;
    BOOL resultBOOL = FALSE;

    if( !ReadOptionFile( OptionFilePath ) )
      return false;

    m_OptionFilePath = OptionFilePath;

    if( !GetValue( "OutputFilePath", m_OutputFilePath ) )
      return false;

    if( !GetValue( "ProcessingMethod", m_ProcessingMethod ) )
      return false; 
    if( m_ProcessingMethod != "LSQ" && m_ProcessingMethod != "EKF" && m_ProcessingMethod != "RTK" )
      return false;


    if( !GetValue( "StartGPSWeek", m_StartTime.GPSWeek ) )
      return false;
    if( !GetValue( "StartGPSTimeOfWeek", m_StartTime.GPSTimeOfWeek ) )
      return false;

    if( !GetValue( "EndGPSWeek", m_EndTime.GPSWeek ) )
      return false;
    if( !GetValue( "EndGPSTimeOfWeek", m_EndTime.GPSTimeOfWeek ) )
      return false;

    if( !GetValue( "ElevationMask", m_elevationMask ) )
      return false;
    if( !GetValue( "CNoMask", m_cnoMask ) )
      return false;
    if( !GetValue( "LockTimeMask", m_locktimeMask ) )
      return false;

    if( !GetValue( "ProcessOnlyDGPS", m_processDGPSOnly ) )
      return false;

    GetValue( "RINEXNavigationDataPath", m_RINEXNavDataPath );      

    GetValue( "Reference_DataPath", m_Reference.DataPath );

    m_Reference.isValid = false;
    if( !m_Reference.DataPath.empty() )
    {
      if( DoesFileExist( m_Reference.DataPath ) )
      { 
        m_Reference.isValid = true;

        if( !GetValue( "Reference_DataType", m_Reference.DataTypeStr ) )
          return false;

        if( m_Reference.DataTypeStr.compare("NOVATELOEM4") == 0 )
          m_Reference.DataType = GNSS_RXDATA_NOVATELOEM4;
        else if( m_Reference.DataTypeStr.compare("RINEX2.1") == 0 )
          m_Reference.DataType = GNSS_RXDATA_RINEX21;
        else if( m_Reference.DataTypeStr.compare("RINEX2.11") == 0 )
          m_Reference.DataType = GNSS_RXDATA_RINEX211;
        else
          return false;


        if( !GetValue( "Reference_UseECEF", useECEF ) )
          return false;

        if( useECEF )
        {
          if( !GetValue( "Reference_ECEF_X", m_Reference.x ) )
            return false;
          if( !GetValue( "Reference_ECEF_Y", m_Reference.y ) )
            return false;
          if( !GetValue( "Reference_ECEF_Z", m_Reference.z ) )
            return false;

          resultBOOL = GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates(
            GEODESY_REFERENCE_ELLIPSE_WGS84,
            m_Reference.x,
            m_Reference.y,
            m_Reference.z,
            &(m_Reference.latitudeRads),
            &(m_Reference.longitudeRads),
            &(m_Reference.height)
            );
          if( resultBOOL == FALSE )
            return false;
          m_Reference.latitudeDegrees  = m_Reference.latitudeRads  * RAD2DEG;
          m_Reference.longitudeDegrees = m_Reference.longitudeRads * RAD2DEG;
        }
        else
        {
          GetValueArray( "Reference_Latitude", d, 4, n );
          if( n == 1 )
          {
            m_Reference.latitudeDegrees = d[0];
          }
          else if( n == 3 )
          {
            GetDMSValue( "Reference_Latitude", m_Reference.latitudeDegrees );
          }
          else
          {
            return false;
          }
          m_Reference.latitudeRads = m_Reference.latitudeDegrees*DEG2RAD;

          GetValueArray( "Reference_Longitude", d, 4, n );
          if( n == 1 )
          {
            m_Reference.longitudeDegrees = d[0];
          }
          else if( n == 3 )
          {
            GetDMSValue( "Reference_Longitude", m_Reference.longitudeDegrees );
          }
          else
          {
            return false;
          }
          m_Reference.longitudeRads = m_Reference.longitudeDegrees*DEG2RAD;

          if( !GetValue( "Reference_Height", m_Reference.height ) )
            return false;

          resultBOOL = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
            GEODESY_REFERENCE_ELLIPSE_WGS84,
            m_Reference.latitudeRads,
            m_Reference.longitudeRads,
            m_Reference.height,
            &m_Reference.x,
            &m_Reference.y,
            &m_Reference.z 
            );
          if( resultBOOL == FALSE )
            return false;
        }

        // The reference is known to sub mm.
        m_Reference.uncertaintyLatitudeOneSigma = 1.0e-04;
        m_Reference.uncertaintyLongitudeOneSigma = 1.0e-04;
        m_Reference.uncertaintyHeightOneSigma = 1.0e-04;

        if( !GetValue( "Reference_EnableTropoCorrection", m_Reference.useTropo ) )
          return false;
        if( !GetValue( "Reference_EnableIonoCorrection", m_Reference.useIono ) )
          return false;
        if( !GetValueArray( "Reference_ExcludeSatellites", m_Reference.satsToExclude, 64, n ) )
          return false;
        m_Reference.nrSatsToExclude = n;
      }
    }



    result = false;
    GetValue( "Iono_KlobucharIsValid", result );
    if( result )
    {
      m_klobuchar.isValid = 1;
      GetValue( "Iono_KlobucharReferenceWeek", m_klobuchar.week );
      GetValue( "Iono_KlobucharReferenceTime", m_klobuchar.tow );

      GetValueArray( "Iono_KlobucharAlphaParameters", d, 4, n );
      if( n == 4 )
      {
        m_klobuchar.alpha0 = d[0];
        m_klobuchar.alpha1 = d[1];
        m_klobuchar.alpha2 = d[2];
        m_klobuchar.alpha3 = d[3];
      }
      GetValueArray( "Iono_KlobucharBetaParameters", d, 4, n );
      if( n == 4 )
      {
        m_klobuchar.beta0 = d[0];
        m_klobuchar.beta1 = d[1];
        m_klobuchar.beta2 = d[2];
        m_klobuchar.beta3 = d[3];
      }
    }
    else
    {
      result = false;
      GetValue( "Iono_ObtainFromRINEXNavFile", result );
      if( result )
      {
        resultBOOL = RINEX_GetKlobucharIonoParametersFromNavFile(
          m_RINEXNavDataPath.c_str(),
          &m_klobuchar
          );
        if( resultBOOL == FALSE )
          return false;       
      }
      else
      {
        m_klobuchar.isValid = 0;
      }
    }

    m_Rover.isValid = false;
    if( !GetValue( "Rover_DataPath", m_Rover.DataPath ) )
      return false;

    if( m_Rover.DataPath.empty() )
      return false;
    if( !DoesFileExist( m_Rover.DataPath ) )
      return false;

    if( !GetValue( "Rover_DataType", m_Rover.DataTypeStr ) )
      return false;

    if( m_Rover.DataTypeStr.compare("NOVATELOEM4") == 0 )
      m_Rover.DataType = GNSS_RXDATA_NOVATELOEM4;
    else if( m_Rover.DataTypeStr.compare("RINEX2.1") == 0 )
      m_Rover.DataType = GNSS_RXDATA_RINEX21;
    else if( m_Rover.DataTypeStr.compare("RINEX2.11") == 0 )
      m_Rover.DataType = GNSS_RXDATA_RINEX211;
    else
      return false;

    if( !GetValue( "Rover_UseECEF", useECEF ) )
      return false;

    if( useECEF )
    {
      if( !GetValue( "Rover_ECEF_X", m_Rover.x ) )
        return false;
      if( !GetValue( "Rover_ECEF_Y", m_Rover.y ) )
        return false;
      if( !GetValue( "Rover_ECEF_Z", m_Rover.z ) )
        return false;

      resultBOOL = GEODESY_ConvertEarthFixedCartesianToGeodeticCurvilinearCoordinates(
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        m_Rover.x,
        m_Rover.y,
        m_Rover.z,
        &(m_Rover.latitudeRads),
        &(m_Rover.longitudeRads),
        &(m_Rover.height)
        );
      if( resultBOOL == FALSE )
        return false;
      m_Rover.latitudeDegrees  = m_Rover.latitudeRads  * RAD2DEG;
      m_Rover.longitudeDegrees = m_Rover.longitudeRads * RAD2DEG;
    }
    else
    {
      GetValueArray( "Rover_Latitude", d, 4, n );
      if( n == 1 )
      {
        m_Rover.latitudeDegrees = d[0];
      }
      else if( n == 3 )
      {
        GetDMSValue( "Rover_Latitude", m_Rover.latitudeDegrees );
      }
      else
      {
        return false;
      }
      m_Rover.latitudeRads = m_Rover.latitudeDegrees*DEG2RAD;

      GetValueArray( "Rover_Longitude", d, 4, n );
      if( n == 1 )
      {
        m_Rover.longitudeDegrees = d[0];
      }
      else if( n == 3 )
      {
        GetDMSValue( "Rover_Longitude", m_Rover.longitudeDegrees );
      }
      else
      {
        return false;
      }
      m_Rover.longitudeRads = m_Rover.longitudeDegrees*DEG2RAD;

      if( !GetValue( "Rover_Height", m_Rover.height ) )
        return false;

      resultBOOL = GEODESY_ConvertGeodeticCurvilinearToEarthFixedCartesianCoordinates(
        GEODESY_REFERENCE_ELLIPSE_WGS84,
        m_Rover.latitudeRads,
        m_Rover.longitudeRads,
        m_Rover.height,
        &m_Rover.x,
        &m_Rover.y,
        &m_Rover.z 
        );
      if( resultBOOL == FALSE )
        return false;
    }

    if( !GetValue( "Rover_UncertaintyLatitude", m_Rover.uncertaintyLatitudeOneSigma ) )
      return false;
    if( !GetValue( "Rover_UncertaintyLongitude", m_Rover.uncertaintyLongitudeOneSigma ) )
      return false;
    if( !GetValue( "Rover_UncertaintyHeight", m_Rover.uncertaintyHeightOneSigma ) )
      return false;

    if( !GetValue( "Rover_EnableTropoCorrection", m_Rover.useTropo ) )
      return false;
    if( !GetValue( "Rover_EnableIonoCorrection", m_Rover.useIono ) )
      return false;
    if( !GetValueArray( "Rover_ExcludeSatellites", m_Rover.satsToExclude, 64, n ) )
      return false;
    m_Rover.nrSatsToExclude = n;

    m_Rover.isValid = true;


    return true;
  }

} // end of namespace GNSS;


