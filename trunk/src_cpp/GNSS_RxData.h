/**
\file    GNSS_RxData.h
\brief   The header file for the GNSS_RxData class.

\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
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

#ifndef _GNSS_RXDATA_H_
#define _GNSS_RXDATA_H_

#include <stdio.h>
#include <string>
#include "gnss_types.h"
#include "gps.h"
#include "rinex.h"


/// This is the fixed number of channels contained in the array 
/// for one GNSS_RxData object.
#define GNSS_RXDATA_NR_CHANNELS (48)
#define GNSS_RXDATA_MAX_OBS (GNSS_RXDATA_NR_CHANNELS*3) // psr, doppler, adr

/// This is the buffer length (in bytes) for a data message buffer used
/// by the receiver object in decoding data.
#define GNSS_RXDATA_MSG_LENGTH  (16384)

/// GDM - relates to a 'hack' to obtain an UWB range measurement from a 
/// comma delimited input file as an additional 'satellite measurement',
/// GNSS_structMeasurement, with the satellite position being that of the
/// reference station. This works well for the setup when both the 
/// reference and rover receivers have the same UWB mounts and antenna heights 
/// and similar mount alignments.
#define GDM_UWB_RANGE_HACK

#ifdef GDM_UWB_RANGE_HACK
#include "Matrix.h"
using namespace Zenautics; // for Matrix
#endif


namespace GNSS
{
  //============================================================================
  /// \class   GPS_BroadcastEphemerisAndAlmanacArray
  /// \brief   An array class for storing broadcast GPS ephemeris and almanac 
  ///          information.
  ///
  /// Each satellite is uniquely identified in GPS by it's PRN. Each PRN has
  /// an associated almanac structure, a most recent ephemeris structure,
  /// and a previous ephemeris structure (to faciliate ephemeris matching)
  /// 
  /// GPS         are  1-32    \n
  /// Pseudolites are  33-37   \n
  /// SBAS        are  120-138 \n
  /// WAAS, EGNOS, MSAS        \n
  ///
  /// WAAS: \n
  /// AOR-W       122 \n
  /// Anik        138 \n
  /// POR         134 \n
  /// PanAm       135 \n
  ///
  /// EGNOS: \n
  /// AOR-E       120 \n
  /// Artemis     124 \n
  /// IOR-W       126 \n
  /// IOR-E       131 \n
  ///
  /// MSAS: \n
  /// MTSAT-1     129 \n
  /// MTSAT-2     137 \n
  ///
  /// The index mapping is as follows:                      \n
  /// PRN 1-37    maps to indidex 0-36                      \n
  /// PRN 38-40   maps to indices 37-39 (reserved mappings) \n
  /// PRN 120-138 maps to indicex 40-58                     \n
  ///
  /// \author  Glenn D. MacGougan (GDM)
  /// \date    2007-10-29
  /// \since   2006-11-15
  ///
  class GPS_BroadcastEphemerisAndAlmanacArray
  {
  public:

    /// \brief    The default constructor (no data allocated yet).
    GPS_BroadcastEphemerisAndAlmanacArray();

    /// \brief    The destructor.
    virtual ~GPS_BroadcastEphemerisAndAlmanacArray();

    /// \brief    Add an ephemeris structure.
    /// \return   true if successful, false if error.
    bool AddEphemeris( const unsigned short prn, const GPS_structEphemeris &eph );

    /// \brief    Add an almanac structure.
    /// \return   true if successful, false if error.
    bool AddAlmanac( const unsigned short prn, const GPS_structAlmanac &alm );


    /// \brief    Check if ephemeris information is available for a PRN.
    /// \return   true if successful, false if error.    
    bool IsEphemerisAvailable( 
      const unsigned short prn, //!< The desired GPS PRN. (1-32 GPS, 120-138 SBAS).
      bool &isAvailable,        //!< This boolean indicates if ephemeris data is available or not.
      char iode = -1            //!< The issue of data for the ephemeris, -1 means get the most current.
      );

    /// \brief    Try to get the most current ephemeris or the ephemeris 
    ///           with the issue of data (ephemeris), iode, specified. 
    /// \remarks  (1) iode == -1, means retrieve the most current ephemeris. \n
    ///           (2) 
    /// \return   true if successful, false if error.
    bool GetEphemeris( 
      const unsigned short prn, //!< The desired GPS PRN. (1-32 GPS, 120-138 SBAS).
      GPS_structEphemeris &eph, //!< A reference to an ephemeris struct in which to store the data.
      bool &isAvailable,        //!< This boolean indicates if ephemeris data is available or not.
      char iode = -1            //!< The issue of data for the ephemeris, -1 means get the most current.
      );

    /**
    \brief    Try to get the week, and time of week of the most current ephemeris 
              for the prn specified if available.
    \author   Glenn D. MacGougan
    \date     2007-12-07
    \return   true if successful, false if error.
    */
    bool GetEphemerisTOW( 
      const unsigned short prn, //!< The desired GPS PRN. (1-32 GPS, 120-138 SBAS).
      bool &isAvailable,        //!< This boolean indicates if ephemeris data is available or not.
      unsigned short &week,     //!< The correct week corresponding to the time of week based on the Z-count in the Hand Over Word.
      unsigned &tow             //!< The time of week based on the Z-count in the Hand Over Word.
      );

  private:
    /// \brief   The copy constructor. Disabled!
    GPS_BroadcastEphemerisAndAlmanacArray( const GPS_BroadcastEphemerisAndAlmanacArray& rhs );

    /// \brief   The assignment operator. Disabled!
    void operator=(const GPS_BroadcastEphemerisAndAlmanacArray& rhs)
    {}

  protected:

    /// \brief   Allocate the array.
    /// \return  true if successful, false if error.
    bool AllocateArray();

    /// \brief   Get the index of the PRN in the array.
    /// \return  true if successful, false if error (e.g. unsupported PRN).
    bool GetIndexGivenPRN( const unsigned short prn, unsigned short &index );

  protected:

    struct GPS_structOrbitParameters
    {
      unsigned short prn;
      GPS_structEphemeris currentEph;
      GPS_structEphemeris previousEph;
      GPS_structAlmanac   almanac;
    };

    /// A pointer to the array of GPS satellite orbit information structs.
    GPS_structOrbitParameters* m_array; 

    /// The maximum number of elements in m_array.
    unsigned m_arrayLength;
  };


#ifdef ASDFASDF

  /**
  \class   GPS_AmbiguitiesArray
  \brief   An array class for storing GPS amibiguity information.
  Each satellite is uniquely identified in GPS by it's PRN. This
  PRN is used to map to an index internal to the GPS_AmbiguitiesArray.
  L1 and L2 are supported for now.

  GPS         are  1-32    \n
  Pseudolites are  33-37   \n
  SBAS        are  120-138 \n
  WAAS, EGNOS, MSAS        \n

  WAAS: \n
  AOR-W       122 \n
  Anik        138 \n
  POR         134 \n
  PanAm       135 \n

  EGNOS: \n
  AOR-E       120 \n
  Artemis     124 \n
  IOR-W       126 \n
  IOR-E       131 \n

  MSAS: \n
  MTSAT-1     129 \n
  MTSAT-2     137 \n

  The index mapping is as follows:                      \n
  PRN 1-37    maps to indidex 0-36                      \n
  PRN 38-40   maps to indices 37-39 (reserved mappings) \n
  PRN 120-138 maps to indicex 40-58                     \n

  \author  Glenn D. MacGougan (GDM)
  \date    2007-10-29
  \since   2006-11-15
  */
  class GPS_AmbiguitiesArray
  {
  public:

    /// \brief    The default constructor (no data allocated yet).
    GPS_AmbiguitiesArray();

    /// \brief    The destructor.
    virtual ~GPS_AmbiguitiesArray();

    /**
    \brief    Add a GPS ambiguity.
    \author   Glenn D. MacGougan
    \remarks  - There cannot be more than one channel associated with a PRN.
    \return   true if successful, false if error.
    */
    bool AddAmbiguity( 
      const unsigned short channel,       //!< The receiver channel. Useful for real receiver data. Not useful for RINEX data. 
      const unsigned short prn,           //!< The desired GPS PRN. (1-32 GPS, 120-138 SBAS).
      const GNSS_enumFrequency freqType,  //!< The frequency type for this channel.
      const double ambiguity,             //!< The float ambiguity value [m].
      const short state_index             //!< The associated index of the rows and columns of, P, the variance-covariance matrix of the estimated parameters. -1 means no estimated.
      );

    /// \brief    Check if this PRN/frequency's ambiguity is actively estimated. i.e. It has a valid state index.
    /// \return   true if successful, false if error.    
    bool IsAmbiguityActive( 
      const unsigned short prn, //!< The GPS PRN. (1-32 GPS, 120-138 SBAS).
      const GNSS_enumFrequency freqType,  //!< The frequency type for this channel.
      bool &isActive            //!< This boolean indicates if the ambiguity is being actively estimated.
      );

    /**
    \brief    Update the GPS ambiguity array for a single ambiguity.
    \author   Glenn D. MacGougan
    \remarks  - There cannot be more than one channel associated with a PRN.
    \return   true if successful, false if error.
    */
    bool UpdateAmbiguity( 
      const unsigned short channel,       //!< The receiver channel. Useful for real receiver data. Not useful for RINEX data. 
      const unsigned short prn,           //!< The desired GPS PRN. (1-32 GPS, 120-138 SBAS).
      const GNSS_enumFrequency freqType,  //!< The frequency type for this channel.
      const double ambiguity,             //!< The float ambiguity value [m].
      const unsigned short state_index    //!< The associated index of the rows and columns of, P, the variance-covariance matrix of the estimated parameters. -1 means no estimated.
      );

    /**
    \brief    Get the state index associated with this PRN/frequency.
    \author   Glenn D. MacGougan
    \return   true if successful, false if error.
    */   
    bool GetStateIndex(
      const unsigned short prn,           //!< The desired GPS PRN. (1-32 GPS, 120-138 SBAS).
      const GNSS_enumFrequency freqType,  //!< The frequency type for this channel.
      unsigned short &state_index         //!< The associated index of the rows and columns of, P, the variance-covariance matrix of the estimated parameters. -1 means no estimated.
      );

    /**
    \brief    Get the float ambiguity value associated with this PRN/frequency.
    \author   Glenn D. MacGougan
    \return   true if successful, false if error.
    */       
    bool GetFloatAmbiguityeValue(
      const unsigned short prn,           //!< The desired GPS PRN. (1-32 GPS, 120-138 SBAS).
      const GNSS_enumFrequency freqType,  //!< The frequency type for this channel.
      double &ambiguity                   //!< The float ambiguity value [m].
      );
      

  private:
    /// \brief   The copy constructor. Disabled!
    GPS_AmbiguitiesArray( const GPS_AmbiguitiesArray& rhs );

    /// \brief   The assignment operator. Disabled!
    void operator=(const GPS_AmbiguitiesArray& rhs)
    {}

  protected:

    /// \brief   Allocate the array.
    /// \return  true if successful, false if error.
    bool AllocateArray();

    /// \brief   Get the index of the PRN in the array.
    /// \return  true if successful, false if error (e.g. unsupported PRN).
    bool GetIndexGivenPRN( const unsigned short prn, unsigned short &index );

  protected:

    struct stAmbiguityInfo
    {
      unsigned short      channel;          //!< The receiver channel number associated with this measurement.
      unsigned short      prn;              //!< The GPS PRN. (1-32 GPS, 120-138 SBAS).
      short               state_index;      //!< The index of the corresponding row and column of the state variance-covariance matrix. -1 means no estimated.
      GNSS_enumFrequency  freqType;         //!< The frequency type for this channel.
      double              ambiguity;        //!< The float ambiguity value [m].
      int                 fixed_ambiguity;  //!< The fixed ambiguity value [cycles].
	  // GDM most recent ambiguity estimate
	  // GDM time of last above
	  // GDM flag indicating amb reset
	  // GDM quality indicator?
    };    

    struct stFullAmbiguityInfo
    {
      stAmbiguityInfo L1;
      stAmbiguityInfo L2;
      // add L5 when ready
    };

    /// A pointer to the array of GPS stFullAmbiguityInfo.
    stFullAmbiguityInfo* m_array; 

    /// The maximum number of elements in m_array.
    unsigned m_arrayLength;
  };


#endif


  //============================================================================
  /// \class   GNSS_RxData
  /// \brief   A class for handling GNSS information for ONE EPOCH for ONE
  ///          RECEIVER such as pseudorange, ADR, and Doppler measurements, 
  ///          user position, user velocity, and other associated information.
  ///          The previous epoch of data is also retained if available.
  /// 
  /// The 'measurement' data is public and can be accessed directly by the user! 
  /// This is struct style encapsulation and to allow easy access to the data but
  /// the user must be careful (especially with the observation array in to 
  /// avoiding out of bound array access).
  ///
  /// \author  Glenn D. MacGougan (GDM)
  /// \date    2007-10-29  
  /// \since   2006-11-13
  /// 
  class GNSS_RxData
  { 
  public: 

    /// \brief    The default constructor (no data allocated yet).
    GNSS_RxData();                                             

    /// \brief    The destructor.
    virtual ~GNSS_RxData();

  private:

    /// \brief   The copy constructor. Disabled!
    GNSS_RxData( const GNSS_RxData& rhs );

    /// \brief   The assignment operator. Disabled!
    GNSS_RxData& operator=(const GNSS_RxData& rhs)
    { return *this; }

  public: 

    /// \brief   Set all data in the measurement array to zero.
    /// \return  true if successful, false if error.    
    bool ZeroAllMeasurements();


    /// \brief   Set all PVT data to zero.
    /// \return  true if successful, false if error.        
    bool ZeroPVT();


    /// \brief   Set the initial receiver position, velocity, and time.
    /// \return  true if successful, false if error.        
    bool SetInitialPVT( 
      const double latitudeRads,   //!< The latitude [rad].
      const double longitudeRads,  //!< The longitude [rad].
      const double height,         //!< The orthometric height [m].
      const double vn,             //!< The northing velocity [m/s].
      const double ve,             //!< The easting velocity [m/s].
      const double vup,            //!< The up velocity [m/s].
      const double clk,            //!< The clock offset [m].
      const double clkdrift,       //!< The clock drift [m/s].
      const double std_lat,        //!< The standard deviation uncertainty in the latitude [m].
      const double std_lon,        //!< The standard deviation uncertainty in the longitude [m].
      const double std_hgt,        //!< The standard deviation uncertainty in the height [m].
      const double std_vn,         //!< The standard deviation uncertainty in the northing velocity [m/s].
      const double std_ve,         //!< The standard deviation uncertainty in the easting velocity [m/s].
      const double std_vup,        //!< The standard deviation uncertainty in the up velocity [m/s].
      const double std_clk,        //!< The standard deviation uncertainty in the clock offset [m].
      const double std_clkdrift,   //!< The standard deviation uncertainty in the clock drift [m/s].
      const double undulation = 0  //!< The undulation if known [m].
      );

    /// \brief   Update the receiver position and clock offset.
    /// \return  true if successful, false if error.            
    bool UpdatePositionAndRxClock( 
      const double latitudeRads,   //!< The latitude [rad].
      const double longitudeRads,  //!< The longitude [rad].
      const double height,         //!< The orthometric height [m].
      const double clk,            //!< The clock offset [m].
      const double std_lat,        //!< The standard deviation uncertainty in the latitude [m].
      const double std_lon,        //!< The standard deviation uncertainty in the longitude [m].
      const double std_hgt,        //!< The standard deviation uncertainty in the height [m].
      const double std_clk         //!< The standard deviation uncertainty in the clock offset [m].
      );

    /// \brief   Update the receiver velocity and clock drift.
    /// \return  true if successful, false if error.                
    bool UpdateVelocityAndClockDrift( 
      const double vn,             //!< The northing velocity [m/s].
      const double ve,             //!< The easting velocity [m/s].
      const double vup,            //!< The up velocity [m/s].
      const double clkdrift,       //!< The clock drift [m/s].
      const double std_vn,         //!< The standard deviation uncertainty in the northing velocity [m/s].
      const double std_ve,         //!< The standard deviation uncertainty in the easting velocity [m/s].
      const double std_vup,        //!< The standard deviation uncertainty in the up velocity [m/s].
      const double std_clkdrift    //!< The standard deviation uncertainty in the clock drift [m/s].
      );


  
    /// \brief  Get degrees, minutes, seconds from a decimal degrees angle.
    /// \return  true if successful, false if error.        
    static bool GetDMS( 
      const double angleDegs,  //!< The angle [degrees].
      short &degrees,          //!< The degrees part.
      short &minutes,          //!< The minutes part.
      float &seconds,          //!< The seconds part.
      char *dms_str,           //!< A DMS string e.g. dms_str = "-180'59'59.9999\""
      const unsigned maxLength_dms_str //!< The maximum length of the dms_str string.
      );
  



    /**
    \brief   Initialize the receiver data object with data path information.
    \author  Glenn D. MacGougan
    \date    2007-12-07
    \return  true if successful, false if error.    
    */
    bool Initialize( 
      const char* path,                  //!< The path to the Observation data file. 
      bool &isValidPath,                 //!< A boolean to indicate if the path is valid.
      const GNSS_enumRxDataType rxType,  //!< The receiver data type.
      const char* RINEX_ephemeris_path   //!< The path to a RINEX ephemeris file, NULL if not available.
      );


    /// \brief   Load the next epoch of data.
    /// \return  true if successful, false if error.
    /// \param   endOfStream - indicates if the end of the input source 
    ///          was reached and no further data is available.
    bool LoadNext( bool &endOfStream );


    /// \brief   Load the next epoch of data of GNSS_RXDATA_NOVATELOEM4 data.
    /// \return  true if successful, false if error.
    /// \param   endOfStream - indicates if the end of the input source 
    ///          was reached and no further data is available.    
    bool LoadNext_NOVATELOEM4( bool &endOfStream );


    /// \brief   Load the next epoch of data of GNSS_RXDATA_RINEX21 data.
    /// \return  true if successful, false if error.
    /// \param   endOfStream - indicates if the end of the input source 
    ///          was reached and no further data is available.    
    bool LoadNext_RINEX21( bool &endOfStream );


    /// \brief   Load the next epoch of data of GNSS_RXDATA_RINEX211 data.
    /// \return  true if successful, false if error.
    /// \param   endOfStream - indicates if the end of the input source 
    ///          was reached and no further data is available.    
    bool LoadNext_RINEX211( bool &endOfStream );


    /**
    \brief   Check the header on the RINEX Observation file to confirm it
             has the correct version and the file type is Observation.
    \author  Glenn D. MacGougan
    \date    2007-12-07
    \param   filepath - The C string path to the observation file.
    \param   isValid - A boolean indicating if the file is valid for use.
    \return  true if successful, false if error.
    */
    bool CheckRINEXObservationHeader( const char *filepath, bool &isValid );

    /**
    \brief   If a RINEX GPS Navigation file is associated with this receiver,
             load all the ephemeris records into the member array.
    \author  Glenn D. MacGougan
    \date    2007-12-07
    \return  true if successful, false if error.
    */
    bool LoadRINEXNavigationData(void);

    /**
    \brief   If a RINEX GPS Navigation file is associated with this receiver,
             Load m_EphAlmArray appropriately based on the current receiver time.
    \author  Glenn D. MacGougan
    \date    2007-12-07
    \return  true if successful, false if error.
    */
    bool UpdateTheEphemerisArrayWithUsingRINEX();

    /// \brief  Check for cycle slips using the phase rate prediction method.
    ///
    /// \post   m_ObsArray[i].flags.isNoCycleSlipDetected is set for each observation.
    ///
    /// \return true if successful, false if error.
    bool CheckForCycleSlips_UsingPhaseRatePrediction( 
      const double nrThresholdCycles //!< The maximum number of cycles to use as the threshold to detect a slip [cycles].
      );


    /// \brief  A debugging function for printing all of the observation array
    /// to a file.
    ///
    /// \return true if successful, false if error.
    bool DebugPrintObservationArray( const char *filepath );


    bool Debug_WriteSuperMsg80CharsWide(
      char* buffer,                    //!< A large character buffer (8KB min).
      const unsigned maxBufferLength,  //!< The maximum buffer length [bytes].
      const double referenceLatitude,  //!< Reference position latitude [rad].
      const double referenceLongitude, //!< Reference position longitude [rad].
      const double referenceHeight,    //!< Reference position height [m].
      unsigned& nrBytesInBuffer );      //!< The number of bytes set in the buffer.  
  
  public:

    /// The array of GNSS measurements.
    GNSS_structMeasurement m_ObsArray[GNSS_RXDATA_NR_CHANNELS];
    
    /// The number of usable items in m_ObsArray.
    unsigned char m_nrValidObs;


    /// The previous observation set.
    GNSS_structMeasurement m_prev_ObsArray[GNSS_RXDATA_NR_CHANNELS];

    /// The number of usable items in m_prevObsArray.
    unsigned char m_prev_nrValidObs;

    /// The number of GPS L1 observations in m_ObsArray.
    unsigned char m_nrGPSL1Obs;

    /// The receiver's position, velocity, and time information.
    GNSS_structPVT  m_pvt;

    /// The receiver's previous position, velocity, and time information.
    GNSS_structPVT  m_prev_pvt;

    /// The klobuchar ionospheric correction parameters for GPS.
    GNSS_structKlobuchar m_klobuchar;

    /// This is an object for storing and handling GPS ephemeris and almanac
    /// information for valid GPS system satellites.
    GPS_BroadcastEphemerisAndAlmanacArray  m_EphAlmArray;

    /// This is the elevation mask angle [rads]. 
    /// The default is 5 degrees.
    double m_elevationMask; 

    /// This is the carrier to noise density ratio mask value [dB-Hz].
    /// The default is 28.0 dB-Hz.
    double m_cnoMask;

    /// This is the minimum allowable locktime [s].
    /// The default is 0.0.
    double m_locktimeMask;

    /// The maximum usable age for an ephemeris [s].
    /// The default is 4 hours (3600*4).
    unsigned m_maxAgeEphemeris; 


    /// A boolean to indicate if the tropospheric correction is to be disabled for all satellites.
    bool m_DisableTropoCorrection;

    /// A boolean to indicate if the ionospheric correction is to be disabled for all satellites.
    bool m_DisableIonoCorrection;


    /// This boolean indicates that a positive millisecond jump (the psr increased by 1 ms * c) occurred at this epoch.
    bool m_msJumpDetected_Positive;

    /// This boolean indicates that a negative millisecond jump (the psr decreased by 1 ms * c) occurred at this epoch.
    bool m_msJumpDetected_Negative;


    /// This boolean indicates that a non modulo 1 millisecond jump occured at this epoch;
    bool m_clockJumpDetected;

    /// This is the value of the non modulo 1 millisecond jump [m].
    double m_clockJump;

    
    // GDM_HACK

    /// A datum point for which to compute the Northing, Easting, and Vertical corresponding to the receiver's position and velocity information.
    GNSS_structPVT  m_datum_pvt;    

    /// \brief  Set the datum point for which to compute the Northing, 
    ///         Easting, and Vertical corresponding to the receiver's 
    ///         position and velocity information.
    bool SetDatumPVT( 
      const double latitudeRads,
      const double longitudeRads,
      const double height 
      );

#ifdef GDM_UWB_RANGE_HACK

    /// \brief  The UWB range hack data.
    /// To use GDM_UWB_RANGE_HACK
    /// The user must set the hack on
    /// \code
    /// GNSS_RxData rx; 
    /// bool result = rx.EnableAndLoadUWBData( "uwb.csv", reference_x, reference_y, reference_z );
    /// \endcode
    /// The UWB range is loaded all at once into a Matrix
    /// 
    struct struct_UWB
    {
      bool isHackOn;       //!< A boolean to indicate if the UWB range hack is enabled for this receiver data.
      char  filepath[512]; //!< The path to the UWB range data.
      double x;  //!< The UWB 'satellite' position ECEF x (WGS84). The position of the reference station in the dual identical GPS-UWB mount case.
      double y;  //!< The UWB 'satellite' position ECEF y (WGS84). The position of the reference station in the dual identical GPS-UWB mount case.
      double z;  //!< The UWB 'satellite' position ECEF z (WGS84). The position of the reference station in the dual identical GPS-UWB mount case.

      /// The UWB range data is loaded into this matrix. After loading the matrix has
      /// column 0 = gps time of week (s), 
      /// column 1 = gps week (weeks), 
      /// column 2 = range (m) - converted from range in feet
      Matrix data;

      /// A boolean to indicate if there is a valid UWB range for the current processing epoch.
      bool isValidForThisEpoch;

      /// The index of the measurement in the obs array if available, -1 if not.
      int index_in_obs_array;
    
      /// A simple constructor.
      struct_UWB() : isHackOn(false), x(0.0), y(0.0), z(0.0), isValidForThisEpoch(false), index_in_obs_array(-1)
      {
        filepath[0] = '\0';
      };
    };

    /// \brief  The UWB range data. see struct_UWB for details.
    struct_UWB m_UWB;

    /// Enable the use of UWB range measurements. Load all the UWB data.
    bool EnableAndLoadUWBData( 
      const char* filepath,  //!< The path to the UWB range data.
      const double x, //!< The UWB 'satellite' position ECEF x (WGS84). The position of the reference station in the dual identical GPS-UWB mount case.
      const double y, //!< The UWB 'satellite' position ECEF y (WGS84). The position of the reference station in the dual identical GPS-UWB mount case.
      const double z, //!< The UWB 'satellite' position ECEF z (WGS84). The position of the reference station in the dual identical GPS-UWB mount case.
      const bool isStatic //!< If the data is static, measurement outliers based on a somewaht ad-hoc 2 sigma rejection are removed.
      );

    /// \brief  Load the UWB range for the current epoch as an additional measurement in m_ObsArray.
    bool LoadUWBRangeForThisEpoch();
#endif

  protected:

    /// A file pointer to the input.
    FILE* m_fid;

    /// A large message buffer.
    unsigned char m_message[GNSS_RXDATA_MSG_LENGTH];

    /// The length of the message in the message buffer.
    unsigned short m_messageLength;

    /// The receiver data type.
    GNSS_enumRxDataType m_rxDataType;


  protected: // RINEX related parameters
  
    /// The RINEX Observation Header must be checked once.
    bool m_CheckRinexObservationHeader;

    /// Decoded RINEX Observation Header information.
    RINEX_structDecodedHeader m_RINEX_obs_header;

    /// A boolean to indicate RINEX ephemeris information is used.
    bool m_RINEX_use_eph;

    struct 
    {
      /// The file path to the RINEX ephemeris data.
      std::string filepath;

      /// All of the RINEX ephemeris data is loaded into a single array at start up.
      GPS_structEphemeris *eph_array;

      /// The length of the m_RINEX_eph_array.
      unsigned max_array_length;

      /// The length of the m_RINEX_eph_array.
      unsigned array_length;

      /// An associated ionospheric model.
      GNSS_structKlobuchar iono_model;
    } m_RINEX_eph;

    /// An index into m_RINEX_eph.eph_array that is used to keep track of
    /// operations.
    unsigned m_RINEX_eph_index;

    
  };





} // end namespace GNSS

#endif // _GNSS_RXDATA_H_

