/** 
\file    test_ionosphere.c
\brief   unit tests for ionosphere.c/.h
\author  Glenn D. MacGougan (GDM)
\date    2007-11-29
\since   2007-11-29

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
#include "Basic.h"     // CUnit/Basic.h
#include "ionosphere.h"
#include "constants.h"



/*
The following function was extracted and modified from: IonoModel.cpp of the 
The GPSTK Project, LGPL license, http://www.gpstk.org
for test purposes only.

Under the terms of the LGPL license, It is allowed to modify
and redistribut this code. However, the code for this function is not re-released 
with BSD license. It continues to be covered by th LGPL.

Modified code produced by Glenn D. MacGougan, 2007-11-29.
*/
static double test_IONOSPHERE_getCorrectionL1_gpstk_3rdParty(
  double gpstow,
  double alpha[4],
  double beta[4],
  const double  latitude,   //!< user geodetic latitude  [rad]
  const double  longitude,  //!< user geodetic longitude [rad]
  const double  svel,       //!< elevation angle between the user and the satellite [rad]
  const double  svaz        //!< azimuth angle between the user and the satellite, measured clockwise positive from the true North [rad]   
  );


int init_suite_IONOSPHERE(void)
{
  return 0;
}

int clean_suite_IONOSPHERE(void)
{
  return 0;
}


void test_IONOSPHERE_GetL1KlobucharCorrection(void)
{
  // We'll use CODE post-mission produced Klobuchar parameters:
  /*
     2              NAVIGATION DATA     GPS                 RINEX VERSION / TYPE
INXFIT V4.3         AIUB                29-NOV-07 13:58     PGM / RUN BY / DATE 
CODE'S GLOBAL IONOSPHERE MAPS FOR DAY 328, 2007             COMMENT             
Contact address: stefan.schaer or igsauto @aiub.unibe.ch    COMMENT             
Web site:        http://www.aiub.unibe.ch/ionosphere/       COMMENT             
Data archive:    ftp://ftp.unibe.ch/aiub/CODE/              COMMENT             
                 http://www.aiub.unibe.ch/download/CODE/    COMMENT             
    1.0836D-08 -6.6222D-09 -3.4546D-07 -5.5205D-07          ION ALPHA           
    1.0706D+05 -1.1893D+05 -9.7785D+05 -1.3581D+06          ION BETA            
                                                            END OF HEADER     
  */

  double alpha[4];
  double beta[4];

  double latitude;
  double longitude;
  double azimuth;
  double elevation;
  double gpstow;
  double az;
  double el;
  BOOL result;
  double ionospheric_delay;
  double ionospheric_delay_test;

  alpha[0] = 1.0836e-08;
  alpha[1] = -6.6222e-09;
  alpha[2] = -3.4546e-07;
  alpha[3] = -5.5205e-07;

  beta[0] = 1.0706e+05;
  beta[1] = -1.1893e+05;
  beta[2] = -9.7785e+05;
  beta[3] = -1.3581e+06;

  // for Calgary
  latitude = 51*DEG2RAD;
  longitude = -114*DEG2RAD;
  
  for( az = 0; az < 360; az+= 15 )
  {
    azimuth = az*DEG2RAD;
    for( el = 0; el <= 90; el += 15 )
    {
      elevation = el*DEG2RAD;
      for( gpstow = 345600; gpstow < 5*86400.0; gpstow += 3600.0 )
      {
        result = IONOSPHERE_GetL1KlobucharCorrection(   
          alpha[0],
          alpha[1],
          alpha[2],
          alpha[3],
          beta[0],
          beta[1],
          beta[2],
          beta[3],
          latitude,  
          longitude, 
          elevation, 
          azimuth,   
          gpstow,    
          &ionospheric_delay 
          );
        CU_ASSERT_FATAL( result );

        ionospheric_delay_test = test_IONOSPHERE_getCorrectionL1_gpstk_3rdParty(
          gpstow,
          alpha,
          beta,
          latitude,
          longitude,  
          elevation,
          azimuth        
          );
        CU_ASSERT_DOUBLE_EQUAL( ionospheric_delay, ionospheric_delay_test, 1e-04 );          
        //printf( "%4d %4d %6d %20.10g\n", (int)az, (int)el, (int)gpstow, ionospheric_delay-ionospheric_delay_test );
      }
    }
  }
}






/*
The following function was extracted and modified from: IonoModel.cpp of the 
The GPSTK Project, LGPL license, http://www.gpstk.org
for test purposes only.

Under the terms of the LGPL license, It is allowed to modify
and redistribut this code. However, the code for this function is not re-released 
with BSD license. It continues to be covered by th LGPL.

Modified code produced by Glenn D. MacGougan, 2007-11-29.
*/
double test_IONOSPHERE_getCorrectionL1_gpstk_3rdParty(
  double gpstow,
  double alpha[4],
  double beta[4],
  const double  latitude,   //!< user geodetic latitude  [rad]
  const double  longitude,  //!< user geodetic longitude [rad]
  const double  svel,       //!< elevation angle between the user and the satellite [rad]
  const double  svaz        //!< azimuth angle between the user and the satellite, measured clockwise positive from the true North [rad]   
  )
{
  // all angle units are in semi-circles (radians / TWO_PI)
  // Note: math functions (cos, sin, etc.) require arguments in
  // radians so all semi-circles must be multiplied by TWO_PI

  double azRad;
  double svE; 
  double phi_u;
  double lambda_u;
  double psi;
  double phi_i;
  double lambda_i;
  double phi_m;
  double iAMP;
  double iPER;
  double t;
  double x;
  double iF;
  double t_iono;
  double correction;

  azRad = svaz;
  svE = svel / PI; // semi-circles

  phi_u = latitude / PI; // semi-circles
  lambda_u = longitude / PI; // semi-circles

  psi = (0.0137 / (svE + 0.11)) - 0.022;

  phi_i = phi_u + psi * cos(azRad);
  if (phi_i > 0.416)
    phi_i = 0.416;
  if (phi_i < -0.416)
    phi_i = -0.416;

  lambda_i = lambda_u + psi * sin(azRad) / cos(phi_i*PI);

  phi_m = phi_i + 0.064 * cos((lambda_i - 1.617)*PI);

  iAMP = 0.0;
  iPER = 0.0;

  iPER = beta[0] + beta[1]*phi_m + beta[2]*phi_m*phi_m + beta[3]*phi_m*phi_m*phi_m;
  
  // amplitude of the vertical delay
  iAMP = alpha[0] + alpha[1]*phi_m + alpha[2]*phi_m*phi_m + alpha[3]*phi_m*phi_m*phi_m;
  
  iAMP = alpha[0]+phi_m*(alpha[1]+phi_m*(alpha[2]+phi_m*alpha[3]));
  iPER =  beta[0]+phi_m*( beta[1]+phi_m*( beta[2]+phi_m* beta[3]));

  if (iAMP < 0.0)
    iAMP = 0.0;
  if (iPER < 72000.0)
    iPER = 72000.0;

  t = 43200.0 * lambda_i + gpstow;  
  if (t >= 86400.0)
    t -= 86400.0;
  if (t < 0)
    t += 86400.0;

  x = TWOPI * (t - 50400.0) / iPER; // x is in radians

  iF = 1.0 + 16.0 * (0.53 - svE)*(0.53 - svE)*(0.53 - svE);

  t_iono = 0.0;
  if (fabs(x) < 1.57)
    t_iono = iF * (5.0e-9 + iAMP * (1 + x*x * (-0.5 + x*x/24.0)));
  else
    t_iono = iF * 5.0e-9;

  /*
  if( freq == L2 )
  {
    // see ICD-GPS-200 20.3.3.3.3.2
    t_iono *= GAMMA_GPS;  //  GAMMA_GPS = (fL1 / fL2)^2
  }
  */

  correction = t_iono * LIGHTSPEED;

  return correction;
}

