/**
\file    ionosphere.h
\brief   GNSS core 'c' function library: GPS ionospheric calculations.
\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2005-08-14

\remarks
- Get better klobuchar (alpha and beta parameters from:
http://www.aiub.unibe.ch/ionosphere/ \n
ftp://ftp.unibe.ch/aiub/CODE/ \n
http://www.aiub.unibe.ch/download/CODE/ \n
e.g. Download CGIM3280.07N from ftp://ftp.unibe.ch/aiub/CODE/2007/ 
for 2007-11-29.

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

#ifndef _C_IONOSPHERE_H_
#define _C_IONOSPHERE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "basictypes.h"

/**
\brief Given the user geodetic position, GPS time of week, the azimuth and elevation to the satellite
in question, and the alpha and beta Klobuchar terms, compute the ionospheric delay for this
satellite.

\author   Glenn D. MacGougan (GDM)
\date     2007-11-29
\since    2005-08-14

\remarks
- The ionospheric correction is referred to the L1 frequency; if the user is operating on the
  L2 frequency, the correction term must be multiplied by \n
  (f_L1/f_L2)^2 = (1575.42/1227.6)^2 = (77/60)^2 = 1.64694444...

\b REFERENCES \n
- GPS ICD 200C, pp. 126-128
*/
BOOL IONOSPHERE_GetL1KlobucharCorrection(   
  const double  alpha0,     //!< coefficients of a cubic equation representing the amplitude of the vertical delay [s]
  const double  alpha1,     //!< coefficients of a cubic equation representing the amplitude of the vertical delay [s/semi-circle]
  const double  alpha2,     //!< coefficients of a cubic equation representing the amplitude of the vertical delay [s/semi-circle^2]
  const double  alpha3,     //!< coefficients of a cubic equation representing the amplitude of the vertical delay [s/semi-circle^3]  
  const double  beta0,      //!< coefficients of a cubic equation representing the period of the model [s]
  const double  beta1,      //!< coefficients of a cubic equation representing the period of the model [s/semi-circle]
  const double  beta2,      //!< coefficients of a cubic equation representing the period of the model [s/semi-circle^2]
  const double  beta3,      //!< coefficients of a cubic equation representing the period of the model [s/semi-circle^3]
  const double  latitude,   //!< user geodetic latitude  [rad]
  const double  longitude,  //!< user geodetic longitude [rad]
  const double  elevation,  //!< elevation angle between the user and the satellite [rad]
  const double  azimuth,    //!< azimuth angle between the user and the satellite, measured clockwise positive from the true North [rad]   
  const double  gpstow,     //!< receiver computed gps time of week [s]
  double* ionospheric_delay //!< computed ionospheric correction [m]
  );

#ifdef __cplusplus
}
#endif


#endif // _C_IONOSPHERE_H_
