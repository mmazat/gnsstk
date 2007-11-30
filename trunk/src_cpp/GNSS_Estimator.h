/**
\file    GNSS_Estimator.h
\brief   The header file for the GNSS_Estimator class.

\author  Glenn D. MacGougan (GDM)
\date    2007-10-29
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

#ifndef _GNSS_ESTIMATOR_H_
#define _GNSS_ESTIMATOR_H_

#include <stdio.h>
#include <list>
#include "GNSS_Types.h"
#include "Matrix.h"

using namespace Zenautics; // for Matrix
using namespace std;

namespace GNSS
{

  // forwarding declaration
  class GNSS_RxData;


  //============================================================================
  /// \brief   A class for estimating navigation information provided GNSS
  ///          measurements.
  /// 
  /// \author  Glenn D. MacGougan (GDM)
  /// \date    2006-11-16
  /// 
  class GNSS_Estimator
  { 
  public: 

    /// \brief    The default constructor (no data allocated yet).
    GNSS_Estimator();                                             

    /// \brief    The destructor.
    virtual ~GNSS_Estimator();

  private:

    /// \brief   The copy constructor. Disabled!
    GNSS_Estimator( const GNSS_Estimator& rhs );

    /// \brief   The assignment operator. Disabled!
    void operator=(const GNSS_Estimator& rhs)
    {}

  public:

    bool Initialize(
      double latitudeRads,
      double longitudeRads,
      double height,
      double std_lat,
      double std_lon,
      double std_hgt 
      );

    bool InitializeDPGS(
      const double referenceLatitudeRads,
      const double referenceLongitudeRads,
      const double refernceHeight,
      double latitudeRads,
      double longitudeRads,
      double height,
      double std_lat,
      double std_lon,
      double std_hgt 
      );


    /// \brief    Determine the satellite clock corrections, positions, and 
    ///           velocities for the rover receiver and the reference
    ///           receiver if aviailable (!NULL). Also determine if the 
    ///           ephemeris is valid (not too old). If differential, 
    ///           rxBaseData != NULL, the reference station receiver 
    ///           ephemeris data is used.
    ///
    /// \pre      The following must be valid:        \n
    /// rxData.m_pvt must be a valid estimate.        \n
    /// rxData.m_maxAgeEphemeris must be set.         \n
    /// rxData.m_time must be a valid time.           \n           
    /// if differential                               \n
    ///   rxBaseData.m_pvt must be a valid estimate.  \n
    ///   rxBaseData.m_maxAgeEphemeris must be set.   \n
    ///   rxBaseData.m_time must be a valid time.     \n               
    ///
    /// \post     The following are set:                      \n
    /// rxData.m_ObsArray[i].flags.isEphemerisValid           \n
    /// rxData.m_ObsArray[i].satellite                        \n
    /// rxData.m_ObsArray[i].corrections.prcSatClk            \n
    /// rxData.m_ObsArray[i].corrections.rrcSatClkDrift       \n
    /// if differntial                                        \n
    ///   rxBaseData.m_ObsArray[i].flags.isEphemerisValid     \n
    ///   rxBaseData.m_ObsArray[i].satellite                  \n
    ///   rxBaseData.m_ObsArray[i].corrections.prcSatClk      \n
    ///   rxBaseData.m_ObsArray[i].corrections.rrcSatClkDrift \n    
    ///
    /// \return   true if successful, false if error.    
    bool DetermineSatellitePVT_GPSL1( 
      GNSS_RxData *rxData,      //!< The pointer to the receiver data.    
      GNSS_RxData *rxBaseData,  //!< The pointer to the reference receiver data. NULL if not available.
      unsigned &nrValidEph      //!< The number of GPS L1 channels with valid ephemeris for the rover.
      );

    /// \brief    Determine the tropospheric and ionospheric delay for each
    ///           GPS L1 channel in rxData. 
    ///
    /// \pre      The following values must be valid: \n
    /// rxData.m_time (all values) \n
    /// rxData.m_pvt must be a valid estimate \n
    /// rxData.m_ObsArray[i].satellite.elevation \n
    /// rxData.m_ObsArray[i].satellite.azimuth \n
    /// rxData.m_klobuchar \n
    ///
    /// \post     The following values are set: \n
    /// rxData.m_ObsArray[i].corrections.prcTropoDry \n
    /// rxData.m_ObsArray[i].corrections.prcTropoWet \n
    /// rxData.m_ObsArray[i].corrections.prcIono \n
    bool DetermineAtmosphericCorrections_GPSL1( 
      GNSS_RxData &rxData  //!< The receiver data.        
      );
  
  

    /// \brief    Determine the usable GPS L1 pseudroange measurements.
    ///
    /// \pre      The rxData.m_ObsArray[i].flags values must be set 
    ///           properly including: \n
    /// rxData.m_ObsArray[i].flags.isCodeLocked         \n 
    /// rxData.m_ObsArray[i].flags.isPsrValid           \n
    /// rxData.m_ObsArray[i].flags.isEphemerisValid     \n
    /// rxData.m_ObsArray[i].flags.isNotPsrRejected     \n
    /// rxData.m_ObsArray[i].flags.isNotUserRejected    \n
    ///
    /// \post     The following flags are set:
    /// rxData.m_ObsArray[i].flags.isAboveElevationMask \n
    /// rxData.m_ObsArray[i].flags.isAboveCNoMask       \n
    /// rxData.m_ObsArray[i].flags.isAboveLockTimeMask  \n
    /// rxData.m_ObsArray[i].flags.isUsedInPosSolution.    
    ///
    /// \return   true if successful, false if error.    
    bool DetermineUsablePseudorangeMeasurementsForThePositionSolution_GPSL1( 
      GNSS_RxData &rxData,             //!< The receiver data.
      unsigned &nrUsablePseudoranges   //!< the number of usable GPS L1 pseudorange measurements.
      );  


    /// \brief    Determine the usable GPS L1 ADR measurements.
    ///
    /// \pre      The rxData.m_ObsArray[i].flags values must be set 
    ///           properly including: \n
    /// rxData.m_ObsArray[i].flags.isCodeLocked         &
    /// rxData.m_ObsArray[i].flags.isPhaseLocked        &
    /// rxData.m_ObsArray[i].flags.isParityValid        & // if not, there is a half cycle amibiguity.
    /// rxData.m_ObsArray[i].flags.isAdrValid           &
    /// rxData.m_ObsArray[i].flags.isAboveElevationMask &
    /// rxData.m_ObsArray[i].flags.isAboveCNoMask       &
    /// rxData.m_ObsArray[i].flags.isAboveLockTimeMask  &
    /// rxData.m_ObsArray[i].flags.isNotUserRejected    &
    /// rxData.m_ObsArray[i].flags.isNotAdrRejected     &
    /// rxData.m_ObsArray[i].flags.isEphemerisValid
    ///
    /// \post     The following flags are set:
    /// rxData.m_ObsArray[i].flags.isAboveElevationMask \n
    /// rxData.m_ObsArray[i].flags.isAboveCNoMask       \n
    /// rxData.m_ObsArray[i].flags.isAboveLockTimeMask  \n
    /// rxData.m_ObsArray[i].flags.isAdrUsedSolution.    
    ///
    /// \return   true if successful, false if error.        
    bool DetermineUsableAdrMeasurements_GPSL1( 
      GNSS_RxData &rxData,   //!< The receiver data.
      unsigned &nrUsableAdr  //!< The number of usable GPS L1 adr measurements.
      );
  


    /// \brief    Determine the differential GPS L1 pseudorange measurements.
    ///
    /// \pre      The following must be valid:              \n
    /// rxData->m_ObsArray[i].flags.isUsedInPosSolution     \n
    /// rxBaseData->m_ObsArray[i].flags.isUsedInPosSolution \n
    ///
    /// \post     The following are set:                           \n
    /// rxData->m_ObsArray[i].flags.isDifferentialPsrAvailable     \n
    /// rxBaseData->m_ObsArray[i].flags.isDifferentialPsrAvailable \n
    /// rxData->m_ObsArray[i].index_differential                   \n
    /// rxBaseData->m_ObsArray[i].index_differential               \n
    ///
    /// \return   true if successful, false if error.        
    bool DetermineDifferentialPseudorangeMeasurementsForThePositionSolution_GPSL1( 
      GNSS_RxData *rxData,                  //!< The pointer to the receiver data.    
      GNSS_RxData *rxBaseData,              //!< The pointer to the reference receiver data. 
      const bool setToUseOnlyDifferntial,   //!< This indicates that only differential measurements should be used.
      unsigned &nrDifferentialPseudoranges  //!< The number of usable GPS L1 pseudorange measurements.    
      );


    /// \brief    Determine the differential GPS L1 ADR measurements.
    ///           Allow only differential ADR measurements.
    ///
    /// \pre      The following must be valid:              \n
    /// rxData->m_ObsArray[i].flags.isAdrUsedInSolution     \n
    /// rxBaseData->m_ObsArray[i].flags.isAdrUsedInSolution \n
    ///
    /// \post     The following are set:                           \n
    /// rxData->m_ObsArray[i].flags.isDifferentialAdrAvailable     \n
    /// rxBaseData->m_ObsArray[i].flags.isDifferentialAdrAvailable \n
    /// rxData->m_ObsArray[i].index_differential_adr               \n
    /// rxBaseData->m_ObsArray[i].index_differential_adr           \n
    ///
    /// \return   true if successful, false if error.            
    bool DetermineDifferentialAdrMeasurements_GPSL1( 
      GNSS_RxData *rxData,                  //!< The pointer to the receiver data.    
      GNSS_RxData *rxBaseData,              //!< The pointer to the reference receiver data. 
      const bool setToUseOnlyDifferential,  //!< This indicates that only differential measurements should be used.
      unsigned &nrDifferentialAdr           //!< The number of usable GPS L1 ADR measurements.    
      );
  
  

    /// \brief    Determine the design matrix for the GPS L1 
    ///           position solution based on pseudroange measurements.
    ///
    /// \pre      The following must be valid:
    /// rxData.m_pvt
    /// rxData.m_ObsArray[i].satellite
    /// rxData.m_ObsArray[i].flags.isUsedInPosSolution
    /// 
    /// \return   true if successful, false if error.        
    bool DetermineDesignMatrixForThePositionSolution_GPSL1( 
      GNSS_RxData &rxData,      //!< The receiver data.
      const unsigned nrRowsInH, //!< The number of valid rows in H.    
      Matrix &H                 //!< The position & rx clock design matrix [n x 4].
      );


    /// \brief    Determine the measurement weight matrix for the GPS L1 
    ///           position solution based on pseudroange measurement
    ///           standard deviation values specified in 
    ///           rxData.m_ObsArray[i].stdev_psr.
    ///
    /// \pre      The following must be valid:         \n
    /// rxData.m_ObsArray[i].flags.isUsedInPosSolution \n
    /// rxData.m_ObsArray[i].stdev_psr (not zero)      \n
    /// 
    /// \return   true if successful, false if error.            
    bool DetermineMeasurementWeightMatrixForThePositionSolution_GPSL1( 
      GNSS_RxData &rxData,                 //!< The receiver data.
      const unsigned nrUsablePseudoranges, //!< The number of usable GPS L1 pseudorange measurements.
      Matrix &W                            //!< The inverse of the pseudorange measurement variance-coraiance matrix [nP x nP].
      );

    /// \brief    Determine the measurement variance-coariance matrix for 
    ///           the GPS L1 position solution based on pseudroange measurement
    ///           standard deviation values specified in 
    ///           rxData.m_ObsArray[i].stdev_psr.
    ///
    /// \pre      The following must be valid:         \n
    /// rxData.m_ObsArray[i].flags.isUsedInPosSolution \n
    /// rxData.m_ObsArray[i].stdev_psr (not zero)      \n
    /// 
    /// \return   true if successful, false if error.                
    bool DetermineMeasurementVarianceCovarianceMatrixForThePositionSolution_GPSL1( 
      GNSS_RxData &rxData,  //!< The receiver data.
      const unsigned n,     //!< The number of measurements and pseudo-measurements (constraints).
      Matrix &R             //!< The pseudorange measurement variance-covariance matrix [n x n].
      );
  

    /// \brief    Compute the GPS L1 pseudroange misclosures.
    ///
    /// \pre      The following must be valid:           \n
    /// rxData->m_ObsArray[i].flags.isUsedInPosSolution  \n
    /// rxData->m_ObsArray[i].satellite.clkdrift         \n
    /// rxData->m_ObsArray[i].corrections.prcTropoDry    \n
    /// rxData->m_ObsArray[i].corrections.prcTropoWet    \n
    /// rxData->m_ObsArray[i].corrections.prcIono        \n
    /// rxData->m_ObsArray[i].range                      \n
    /// rxData->m_pvt.clockOffset                        \n
    /// if diffential, the same above for rxBaseData.
    /// 
    /// \post     The following is set:     \n
    /// rxData.m_ObsArray[i].psr_misclosure \n
    ///
    /// \return   true if successful, false if error.
    bool DeterminePseudorangeMisclosures_GPSL1( 
      GNSS_RxData *rxData,     //!< The pointer to the receiver data.    
      GNSS_RxData *rxBaseData, //!< The pointer to the reference receiver data. NULL if not available.    
      const unsigned nrPsr,    //!< The number of GPS L1 pseudorange measurements.
      Matrix &w                //!< The pseudorange misclosure vector [nP x 1].
      );
  

    /// \brief    Compute the differntial GPS L1 ADR misclosures.
    ///
    /// \pre      The following must be valid:           \n
    /// rxData->m_ObsArray[i].flags.isAdrUsedInSolution  \n
    /// rxData->m_ObsArray[i].satellite.clkdrift         \n
    /// rxData->m_ObsArray[i].corrections.prcTropoDry    \n
    /// rxData->m_ObsArray[i].corrections.prcTropoWet    \n
    /// rxData->m_ObsArray[i].corrections.prcIono        \n
    /// rxData->m_ObsArray[i].range                      \n
    /// rxData->m_pvt.clockOffset                        \n
    /// if diffential, the same above for rxBaseData.
    /// 
    /// \post     The following is set:     \n
    /// rxData.m_ObsArray[i].adr_misclosure \n
    ///
    /// \return   true if successful, false if error.    
    bool DetermineSingleDifferenceADR_Misclosures_GPSL1( 
      GNSS_RxData *rxData,     //!< The pointer to the receiver data.    
      GNSS_RxData *rxBaseData, //!< The pointer to the reference receiver data. NULL if not available.    
      const unsigned n,        //!< The number of misclosures.
      Matrix &w                //!< The adr misclosure vector [n x 1].
      );
  
  

    /// \brief    Determine the usable GPS L1 Doppler measurements.
    /// \pre      The rxData.m_ObsArray[i].flags values must be set 
    ///           properly including: \n
    /// rxData.m_ObsArray[i].flags.isCodeLocked         \n
    /// rxData.m_ObsArray[i].flags.isDopplerValid       \n
    /// rxData.m_ObsArray[i].flags.isAboveElevationMask \n
    /// rxData.m_ObsArray[i].flags.isAboveCNoMask       \n
    /// rxData.m_ObsArray[i].flags.isAboveLockTimeMask  \n
    /// rxData.m_ObsArray[i].flags.isNotUserRejected    \n
    /// rxData.m_ObsArray[i].flags.isNotDopplerRejected \n
    /// rxData.m_ObsArray[i].flags.isEphemerisValid     \n
    ///
    /// \post     The rxData.m_ObsArray[i].flags.isUsedInVelSolution
    ///           will be set to 1(used) or 0(not used).
    /// \return   true if successful, false if error.        
    bool DetermineUsableDopplerMeasurementsForTheVelocitySolution_GPSL1( 
      GNSS_RxData &rxData,       //!< The receiver data.
      unsigned &nrUsableDopplers //!< the number of usable GPS L1 Doppler measurements.
      );


    /// \brief    Determine the differential GPS L1 Doppler measurements.
    ///
    /// \pre      The following must be valid:              \n
    /// rxData->m_ObsArray[i].flags.isUsedInVelSolution     \n
    /// rxBaseData->m_ObsArray[i].flags.isUsedInVelSolution \n
    ///
    /// \post     The following are set:                               \n
    /// rxData->m_ObsArray[i].flags.isDifferentialDopplerAvailable     \n
    /// rxBaseData->m_ObsArray[i].flags.isDifferentialDopplerAvailable \n
    /// rxData->m_ObsArray[i].index_differential                       \n
    /// rxBaseData->m_ObsArray[i].index_differential                   \n
    ///
    /// \return   true if successful, false if error.            
    bool DetermineDifferentialDopplerMeasurementsForTheVelocitySolution_GPSL1( 
      GNSS_RxData *rxData,                  //!< The pointer to the receiver data.    
      GNSS_RxData *rxBaseData,              //!< The pointer to the reference receiver data. 
      const bool setToUseOnlyDifferential,  //!< This indicates that only differential measurements should be used.
      unsigned &nrDifferentialDoppler       //!< The number of usable differential GPS L1 Doppler measurements.    
      );
  

    /// \brief    Compute the GPS L1 Doppler misclosures.
    ///
    /// \pre      The folloing must be valid:           \n
    /// rxData.m_ObsArray[i].flags.isUsedInVelSolution  \n
    /// rxData.m_ObsArray[i].satellite.clkdrift         \n
    /// rxData.m_ObsArray[i].rangerate                  \n
    /// rxData.m_pvt.clockDrift                         \n
    ///
    /// \post     The following is set:     \n
    /// rxData.m_ObsArray[i].doppler_misclosure \n
    ///
    /// \return   true if successful, false if error.    
    bool DetermineDopplerMisclosures_GPSL1( 
      GNSS_RxData *rxData,     //!< The pointer to the receiver data.    
      GNSS_RxData *rxBaseData, //!< The pointer to the reference receiver data. NULL if not available.    
      const unsigned n,        //!< The number of misclosures.
      Matrix &w                //!< The pseudorange misclosure vector [n x 1].
      );
  
  
  
    /// \brief    Determine the design matrix for the GPS L1 
    ///           velocity solution based on Doppler measurements.
    ///
    /// \pre      The following must be valid:
    /// rxData.m_pvt
    /// rxData.m_ObsArray[i].satellite
    /// rxData.m_ObsArray[i].flags.isUsedInVelSolution
    /// 
    /// \return   true if successful, false if error.            
    bool DetermineDesignMatrixForTheVelocitySolution_GPSL1( 
      GNSS_RxData &rxData,             //!< The receiver data.
      const unsigned nrUsableDopplers, //!< The number of usable GPS L1 Doppler measurements.
      Matrix &H                        //!< The velocity & rx clock drift design matrix [nD x 4].
    );

    /// \brief    Determine the measurement weight matrix for the GPS L1 
    ///           velocity solution based on Doppler measurement
    ///           standard deviation values specified in 
    ///           rxData.m_ObsArray[i].stdev_doppler.
    ///
    /// \pre      The following must be valid:         \n
    /// rxData.m_ObsArray[i].flags.isUsedInVelSolution \n
    /// rxData.m_ObsArray[i].stdev_doppler (not zero)  \n
    /// 
    /// \return   true if successful, false if error.                
    bool DetermineMeasurementWeightMatrixForTheVelocitySolution_GPSL1( 
      GNSS_RxData &rxData,  //!< The receiver data.
      const unsigned n,     //!< The number of rows in W.
      Matrix &W             //!< The inverse of the Doppler measurement variance-coraiance matrix [n x n].
      );

    /// \brief    Determine the measurement variance-covariance matrix for 
    ///           the GPS L1 velocity solution based on Doppler measurement
    ///           standard deviation values specified in 
    ///           rxData.m_ObsArray[i].stdev_doppler.
    ///
    /// \pre      The following must be valid:         \n
    /// rxData.m_ObsArray[i].flags.isUsedInVelSolution \n
    /// rxData.m_ObsArray[i].stdev_doppler (not zero)  \n
    /// 
    /// \return   true if successful, false if error.                    
    bool DetermineMeasurementVarianceCovarianceMatrixForTheVelocitySolution_GPSL1( 
      GNSS_RxData &rxData,  //!< The receiver data.
      const unsigned n,     //!< The number of rows in R.
      Matrix &R             //!< The inverse of the Doppler measurement variance-coraiance matrix [n x n].
      );


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
    ///   (1) The math model incorrect, (not likely)
    ///   (2) The weight matrix/observation variance matrix is optimistic 
    ///      (observations are worse than what you said). \n
    ///   (3) Their is a blunder in the observations that is skewing the normality 
    ///       of the solution, and a local test for blunder my be needed. \n
    ///
    /// \return   true if successful, false if error.                    
    bool PerformGlobalTestAndTestForMeasurementFaults( 
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
      );
  




    bool ComputeTransitionMatrix_8StatePVGM(
      const double dT,            //!< The change in time since the last update [s].
      Matrix &T,                  //!< The transition matrix [8 x 8].
      const double betaVn,        //!< The Gauss Markov beta for northing velocity [1/s].
      const double betaVe,        //!< The Gauss Markov beta for easting velocity [1/s].
      const double betaVup,       //!< The Gauss Markov beta for up velocity [1/s].
      const double betaClkDrift   //!< The Gauss Markov beta for the clock bias [1/s].
      );

    bool ComputeTransitionMatrix_8StatePVGM_Float(
      const double dT,            //!< The change in time since the last update [s].
      Matrix &T,                  //!< The transition matrix [(8 + nrAmb) x (8 + nrAmb)]. 
      const double betaVn,        //!< The Gauss Markov beta for northing velocity [1/s].
      const double betaVe,        //!< The Gauss Markov beta for easting velocity [1/s].
      const double betaVup,       //!< The Gauss Markov beta for up velocity [1/s].
      const double betaClkDrift   //!< The Gauss Markov beta for the clock bias [1/s].
      );
  

    bool ComputeProcessNoiseMatrix_8StatePVGM(
      const double dT,           //!< The change in time since the last update [s].
      Matrix &Q,                 //!< The process noise matrix [8 x 8].
      const double betaVn,       //!< The Gauss Markov beta for northing velocity [1/s].
      const double betaVe,       //!< The Gauss Markov beta for easting velocity [1/s].
      const double betaVup,      //!< The Gauss Markov beta for up velocity [1/s].
      const double betaClkDrift, //!< The Gauss Markov beta for the clock bias [1/s].
      const double qVn,          //!< The process noise value for northing velocity.
      const double qVe,          //!< The process noise value for easting  velocity.
      const double qVup,         //!< The process noise value for up       velocity.
      const double qClkDrift     //!< The process noise value for clock drift.
      );

    bool ComputeProcessNoiseMatrix_8StatePVGM_Float(
      const double dT,           //!< The change in time since the last update [s].
      Matrix &Q,                 //!< The process noise matrix [(8 + nrAmb) x (8 + nrAmb)].
      const double betaVn,       //!< The Gauss Markov beta for northing velocity [1/s].
      const double betaVe,       //!< The Gauss Markov beta for easting velocity [1/s].
      const double betaVup,      //!< The Gauss Markov beta for up velocity [1/s].
      const double betaClkDrift, //!< The Gauss Markov beta for the clock bias [1/s].
      const double qVn,          //!< The process noise value for northing velocity.
      const double qVe,          //!< The process noise value for easting  velocity.
      const double qVup,         //!< The process noise value for up       velocity.
      const double qClkDrift     //!< The process noise value for clock drift.
      );
  


    /// \brief    Initialize the state variance-covariance matrix 
    ///           for the 8 state PV Gauss Markov model.
    ///
    /// \return   true if successful, false if error.
    bool InitializeStateVarianceCovariance_8StatePVGM(
      const double std_lat,        //!< The standard deviation uncertainty in the latitude [m].
      const double std_lon,        //!< The standard deviation uncertainty in the longitude [m]. 
      const double std_hgt,        //!< The standard deviation uncertainty in the height [m].
      const double std_vn,         //!< The standard deviation uncertainty in the northing velocity [m/s].
      const double std_ve,         //!< The standard deviation uncertainty in the easting velocity [m/s].
      const double std_vup,        //!< The standard deviation uncertainty in the up velocity [m/s].
      const double std_clk,        //!< The standard deviation uncertainty in the clock offset [m].
      const double std_clkdrift,   //!< The standard deviation uncertainty in the clock drift [m/s].    
      Matrix &P                    //!< The variance covariance of the states [8x8].
      );
  

    bool PredictAhead_8StatePVGM(
      GNSS_RxData &rxData, //!< The receiver data.
      const double dT,     //!< The change in time since the last update [s].
      Matrix &T,           //!< The transition matrix                                 [8 x 8] (output).
      Matrix &Q,           //!< The process noise matrix                              [8 x 8] (output).
      Matrix &P            //!< The state variance covariance matrix                  [8 x 8] (input/output).      
      );

    bool PredictAhead_8StatePVGM_Float(
      GNSS_RxData &rxData, //!< The receiver data.
      const double dT,     //!< The change in time since the last update [s].
      Matrix &T,           //!< The transition matrix                                 [(8 + nrAmb) x (8 + nrAmb)] (output).
      Matrix &Q,           //!< The process noise matrix                              [(8 + nrAmb) x (8 + nrAmb)] (output).
      Matrix &P            //!< The state variance covariance matrix                  [(8 + nrAmb) x (8 + nrAmb)] (input/output).      
      );


    
    bool Kalman_Update_8StatePVGM(
      GNSS_RxData *rxData,      //!< A pointer to the rover receiver data. This must be a valid pointer.
      GNSS_RxData *rxBaseData,  //!< A pointer to the reference receiver data if available. NULL if not available.
      Matrix &P                 //!< The variance-covariance of the states.
      );

    bool Kalman_Update_8StatePVGM_SequentialMode(
      GNSS_RxData *rxData,      //!< A pointer to the rover receiver data. This must be a valid pointer.
      GNSS_RxData *rxBaseData,  //!< A pointer to the reference receiver data if available. NULL if not available.
      Matrix &P                 //!< The variance-covariance of the states.
    );


    bool Kalman_Update_8StatePVGM_SequentialMode_FloatSolution(
      GNSS_RxData *rxData,      //!< A pointer to the rover receiver data. This must be a valid pointer.
      GNSS_RxData *rxBaseData,  //!< A pointer to the reference receiver data if available. NULL if not available.
      Matrix &P                 //!< The variance-covariance of the states.
    );
  
  
    


    /// \brief    Update the rest of m_pvt.time struct based on the 
    ///           gps week and time of week only.
    ///
    /// \return   true if successful, false if error.
    bool UpdateTime(
      GNSS_RxData &rxData  //!< The receiver data. The m_pvt.time struct is updated.    
      );

  
    /// \brief    Perform least squares using pseudoranges and Doppler 
    ///           measurments to determine the eight states:
    ///           (lat,lon,hgt,vn,ve,vup,clk,clkdrift).
    ///
    /// \return   true if successful, false if error.
    bool PerformLeastSquares_8StatePVT(
      GNSS_RxData *rxData,       //!< A pointer to the rover receiver data. This must be a valid pointer.
      GNSS_RxData *rxBaseData,   //!< A pointer to the reference receiver data if available. NULL if not available.
      bool &wasPositionComputed, //!< A boolean to indicate if a position solution was computed.    
      bool &wasVelocityComputed  //!< A boolean to indicate if a velocity solution was computed.    
      );
      

    /// \brief    A static function for outputting a matrix to the console.
    bool PrintMatToDebug( const char *name, Matrix& M );



    /// \brief    Deal with changes in ambiguities. Remove the 
    ///           rows and columns from P of the ambiguities that
    ///           are no longer active. Add rows and columns to P
    ///           for new ambiguities.
    ///
    /// \return   true if successful, false if error.    
    bool DetermineAmbiguitiesChanges( 
      GNSS_RxData *rxData,     //!< Pointer to the receiver data.
      GNSS_RxData *rxBaseData, //!< Pointer to the reference receiver data if any (NULL if not available).
      Matrix &P,               //!< The state variance-covariance matrix.
      bool& changeOccured 
      );

    // Publically accessable data
  public: 

    // number of unknown paramters: u 
    // number of observations:      n
        
    Matrix m_x;   //!< The states,                                                  [u x 1].
    Matrix m_dx;  //!< The iterative update to the states,                          [u x 1].
    Matrix m_P;   //!< The states variance-covariance matrix,                       [u x u].
    Matrix m_H;   //!< The design matrix,                                           [n x u].
    Matrix m_w;   //!< The observation misclosure vector,                           [n x 1].
    Matrix m_R;   //!< The variance covariance matrix of the observations,          [n x n].
    Matrix m_W;   //!< The inverse of m_R,                                          [n x n].
    Matrix m_r;   //!< The diagonal of the observations variance-covariance matrix, [n x 1].
    Matrix m_T;   //!< The transition matrix,                                       [u x u].
    Matrix m_Q;   //!< The process noise matrix,                                    [u x u].
    
    Matrix m_T_prev;   //!< The previous transition matrix,                         [u x u].
    Matrix m_Q_prev;   //!< The previous process noise matrix,                      [u x u].
    
    double m_betaVn;
    double m_betaVe;
    double m_betaVup;
    double m_betaClkDrift;

    double m_qVn;
    double m_qVe;
    double m_qVup;
    double m_qClkDrift;

    struct stAmbiguityInfo
    {
      unsigned short      channel;     //!< The channel number associated with this measurement.
      unsigned short      id;          //!< The unique id for this channel (eg PRN for GPS).    
      unsigned int        state_index; //!< The index of the corresponding row and column of the state variance-covariance matrix.
      GNSS_enumSystem     system;      //!< The satellite system associated with this channel.
      GNSS_enumFrequency  freqType;    //!< The frequency type for this channel.
	  // GDM most recent ambiguity estimate
	  // GDM time of last above
	  // GDM flag indicating amb reset
	  // GDM quality indicator?
    };    

    /// This list keeps track of which amibiguities are active. i.e. already included in the state vector
    /// and state variance-covariance matrix.
	/// GDM store in RxData?
    std::list<stAmbiguityInfo> m_ActiveAmbiguitiesList;

  protected:

    Matrix HtW;  //!< The design matrix, H, transposed times W                      u x n.
    Matrix Ninv; //!< The inverse of the normal matrix, N = (H^T*W*H)^-1,           u x u. 

    FILE* m_debug;
  };

} // end namespace GNSS

#endif // _GNSS_ESTIMATOR_H_

