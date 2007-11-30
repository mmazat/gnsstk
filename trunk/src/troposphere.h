/**
\file    troposphere.h
\brief   GNSS core 'c' function library: GPS tropospheric calculations
\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2005-08-22

\b REFERENCES \n
- RTCA, 2001. MINIMUM OPERATIONAL PERFORMANCE STANDARDS FOR GLOBAL POSITIONING SYSTEM/WIDE AREA 
  AUGMENTATION SYSTEM AIRBORNE EQUIPMENT. RTCA/DO-229C. Prepared by SC-159. November 28, 2001. 
  Supersedes DO-229B. Available at http://www.rtca.org/doclist.asp . pp. 338-340 of 586 in PDF. \n
- Guo, J. and R. B. Langely 2003. A New Tropospheric Propagation Delay Mapping Function for 
  Elevation Angles Down to 2 degrees. ION GPS 2003, 9-12 Sept. 2003, Portland OR. \n
- Parkinson, B. and J. J. Spilker (1996). Global Positioning System: Theory And
  Applications Volume 1. American Institute of Aeronautics and Astronautics, Inc.,
  Washinton D.C. \n

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

#ifndef _C_TROPOSPHERE_H
#define _C_TROPOSPHERE_H

#ifdef __cplusplus
extern "C" {
#endif


/// Computes the dry and wet mapping function values based on the UNBabc
/// model in the reference below.
/// 
/// \author   Glenn D. MacGougan (GDM)
/// \date     2005-08-22
/// \since    2005-08-22
/// 
/// \b REFERENCES \n
/// [1] Guo, J. and R. B. Langely 2003. A New Tropospheric Propagation Delay Mapping Function for 
///     Elevation Angles Down to 2 degrees. ION GPS 2003, 9-12 Sept. 2003, Portland OR
/// 
void TROPOSPHERE_GetDayAndWetMappingValues_UsingThe_UNBabc_Model( 
  const double elevation,  //!< satellite elevation angle                     [rad]
  const double latitude,   //!< user latitude                                 [rad]
  const double height,     //!< user height (orthometric, ie above sea level) [m]
  double* drymap,          //!< dry delay scale factor                        []
  double* wetmap           //!< wet delay scale factor                        []
  );


/// Computes the dry and wet zenith delays based on the WAAS model in the reference below.
/// 
/// \author   Glenn D. MacGougan (GDM)
/// \date     2005-08-22
/// \since    2005-08-22
/// 
/// \b REFERENCES \n
/// [1] RTCA, 2001. MINIMUM OPERATIONAL PERFORMANCE STANDARDS FOR GLOBAL POSITIONING SYSTEM/WIDE AREA 
///     AUGMENTATION SYSTEM AIRBORNE EQUIPMENT. RTCA/DO-229C. Prepared by SC-159. November 28, 2001. 
///     Supersedes DO-229B. Available at http://www.rtca.org/doclist.asp . pp. 338-340 of 586 in PDF
/// 
void TROPOSPHERE_DetermineZenithDelayValues_WAAS_Model(
  const double latitude,          //!< user latitude        [rad]
  const double height,            //!< user height          [m]
  const unsigned short dayofyear, //!< day of year (1-366)  [days]    
  double* zenith_dry_delay,       //!< dry zenith delay     [m]
  double* zenith_wet_delay        //!< wet zenith delay     [m]
  );


/// Computes the dry and wet delays given the delay zenith values using the UNBabc
/// Mapping function, described in [1], which requires elevation angle, latitude and height.
/// 
/// \author   Glenn D. MacGougan (GDM)
/// \date     2005-08-22
/// \since    2005-08-22
///
/// \b REFERENCES \n
/// [1] Guo, J. and R. B. Langely 2003. A New Tropospheric Propagation Delay Mapping Function for 
///     Elevation Angles Down to 2 degrees. ION GPS 2003, 9-12 Sept. 2003, Portland OR
/// 
void TROPOSPHERE_GetDryAndWetDelay_UsingThe_UNBabc_MappingFunction(  
  const double zenith_dry_delay,  //!< dry zenith delay                              [m]
  const double zenith_wet_delay,  //!< wet zenith delay                              [m]
  const double elevation,         //!< satellite elevation angle (must be >= 2 deg)  [rad]
  const double latitude,          //!< user latitude                                 [rad]
  const double height,            //!< user height (orthometric, ie above sea level) [m]  
  double*      drydelay,          //!< dry delay mapped to this elevation angle      [m]
  double*      wetdelay           //!< wet delay mapped to this elevation angle      [m]
  );

#ifdef __cplusplus
}
#endif

#endif // _C_TROPOSPHERE_H_
