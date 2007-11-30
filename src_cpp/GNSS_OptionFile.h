/**
\file    GNSS_OptionFile.h
\brief   The option file for EssentialGNSS.

\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
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

#ifndef _EGNSS_OPTIONFILE_H_
#define _EGNSS_OPTIONFILE_H_

#include <string>
#include "OptionFile.h"
#include "GNSS_Types.h"

namespace GNSS
{
  /// \brief   A derived option file class for the program.
  class GNSS_OptionFile : public OptionFile
  { 
  public: 

    /// \brief    The default constructor (no data allocated yet).
    GNSS_OptionFile();                                             

    /// \brief    The destructor.
    virtual ~GNSS_OptionFile();

  private:

    /// \brief   The copy constructor. Disabled!
    GNSS_OptionFile( const GNSS_OptionFile& rhs ) {};

    /// \brief   The assignment operator. Disabled!
    void operator=(const GNSS_OptionFile& rhs) {};

  public:

    /// \brief    Read and interpret the option file.
    /// \return   true if successful, false if error.
    bool ReadAndInterpretOptions( std::string OptionFilePath );

  public:

    struct stStationInformation
    {
      bool   isValid;          //!< Is this information valid.
      double latitudeRads;     //!< Station Latitude [rad].
      double latitudeDegrees;  //!< Station Latitude [deg].
      double longitudeRads;    //!< Station Longitude [rad].
      double longitudeDegrees; //!< Station Longitude [degrees].
      double height;           //!< Station Height [m]

      bool useTropo;           //!< A boolean to indicate if the tropospheric correction is enabled.
      bool useIono;            //!< A boolean to indicate if the broadcast ionospheric correction is enabled.

      double uncertaintyLatitudeOneSigma;  //!< Initial position uncertainty [m].
      double uncertaintyLongitudeOneSigma; //!< Initial position uncertainty [m].
      double uncertaintyHeightOneSigma;    //!< Initial position uncertainty [m].

      int satsToExclude[64];    //!< An array of satellite id (PRN for GPS) to exclude from processing.
      unsigned nrSatsToExclude; //!< The number of satellite id's specified in satsToExclude.

      std::string DataPath;     //!< The path to the datafile.

      // default constructor
      stStationInformation()
      : isValid(false),
        latitudeRads(0.0),
        latitudeDegrees(0.0),
        longitudeRads(0.0),
        longitudeDegrees(0.0),
        height(0.0),
        useTropo(true),
        useIono(true),
        uncertaintyLatitudeOneSigma(1.0),
        uncertaintyLongitudeOneSigma(1.0),
        uncertaintyHeightOneSigma(1.0),
        nrSatsToExclude(0)
      {
        unsigned i;
        for( i = 0; i < 64; i++ )
          satsToExclude[i] = 0;
      }
    };

    struct stGPSTime
    {
      int GPSWeek;
      double GPSTimeOfWeek;

      // default constructor
      stGPSTime() 
        : GPSWeek(-1), GPSTimeOfWeek(0.0)
      {}
    };

    /// The path to the option file.
    std::string m_OptionFilePath;

    /// The path to the output file.
    std::string m_OutputFilePath;

    /// A string is used to indicate the processing method.
    std::string m_ProcessingMethod;

    /// A boolean to indicate if only single difference measurements
    /// between the reference and rover station will be used.
    bool processDGPSOnly;

    /// The reference station information.
    stStationInformation m_Reference;

    /// The rover station information.
    stStationInformation m_Rover;

    /// The start time for processing.
    stGPSTime m_StartTime;

    /// The end time for processing.
    stGPSTime m_EndTime;

    /// The klobuchar ionospheric parameters.
    GNSS_structKlobuchar m_klobuchar;

    /// The elevation mask [degrees].
    double m_elevationMask;

    /// The C/N0 mask [dB-Hz].
    double m_cnoMask;

    /// The locktime mask [s].
    double m_locktimeMask;

  };

} // end namespace Zenautics

#endif // _EGNSS_OPTIONFILE_H_

