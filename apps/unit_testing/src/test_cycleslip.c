/** 
\file    test_cycleslip.h
\brief   unit tests for cycle_slip.c/.h
\author  Kyle O'Keefe (KO)
\date    2008-03-27
\since   2008-03-27

\b "LICENSE INFORMATION" \n
Copyright (c) 2008, refer to 'author' doxygen tags \n
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
#include "cycle_slip.h"
#include "constants.h"

int init_suite_CYCLESLIP(void)
{
  return 0;
}

int clean_suite_CYCLESLIP(void)
{
  return 0;
}

/* from file nw2_3240.07o: second and third epochs

     7    C1    D1    L1    L2    P2    S1    S2            # / TYPES OF OBSERV 
 07 11 20 18 49  3.0000000  0  8G04G08G09G11G15G17G26G28
  23232247.859 6      4451.949 6   -563066.531 6   -398459.53147  23232244.62147
        44.250          28.000  
  24671637.383 3     -3336.293 3    365976.457 3    263239.90245  24671635.27345
        37.000          18.250  
  21522996.977 6      3139.113 6   -403936.914 6   -284542.16848  21522992.52048
        47.250          34.750  
  23347466.266 6     -1867.191 6    222912.895 6    158091.50847  23347461.73447
        44.500          27.000  
  23884219.336 5     -2016.617 5    217326.695 5    157787.91446  23884216.50846
        39.750          24.500  
  20148143.688 7       405.156 7    -63101.598 7    -43712.34049  20148138.84449
        51.000          42.750  
  24260966.570 5     -2742.875 5    299220.691 5    216571.60246  24260964.03946
        39.500          21.250  
  21536533.852 7     -1263.844 7    148802.059 7    105388.02048  21536528.37948
        48.750          36.500  
 07 11 20 18 49  4.0000000  0  8G04G08G09G11G15G17G26G28
  23231400.445 6      4451.164 6   -567517.695 6   -401927.96947  23231397.28547
        43.750          26.750  
  24672272.383 3     -3336.891 3    369313.348 3    265840.05945  24672270.07445
        36.500          17.750  
  21522399.852 6      3137.965 6   -407074.879 6   -286987.33648  21522395.38348
        47.250          34.500  
  23347821.195 6     -1868.238 6    224781.133 6    159547.26647  23347816.96947
        44.000          26.500  
  23884603.867 5     -2017.457 5    219344.152 5    159359.94146  23884600.55146
        41.250          25.000  
  20148066.242 7       404.039 7    -63505.637 7    -44027.17649  20148061.64149
        51.000          42.500  
  24261488.750 5     -2743.551 5    301964.242 5    218709.43046  24261485.98446
        40.750          21.000  
  21536774.102 7     -1264.723 7    150066.781 7    106373.52048  21536768.68448
        48.250          36.000  
*/

void test_CYCLESLIP_CheckForCycleSlipUsingPhaseRatePrediction(void)
{
  unsigned short previous_week;
  double   previous_tow;  
  double   previous_Doppler;    
  double   previous_adr;
  unsigned short current_week;
  double   current_tow;
  double   current_Doppler;
  double   current_adr;
//  double   specifiedThresholdInCycles;
  BOOL    wasSlipDetected = FALSE;
  double  sizeOfSlipInCycles = 0;

  BOOL result;

  /* take PRN04 L1 for testing */
  previous_week    = 1454;
  previous_tow     = 240543;
  previous_Doppler = 4451.949;
  previous_adr     = -563066.531;
  current_week     = 1454;
  current_tow      = 240544;
  current_Doppler  = 4451.164;
  current_adr      = -567517.695;

  result = CYCLESLIP_CheckForCycleSlipUsingPhaseRatePrediction(
    previous_week, 
    previous_tow,
    previous_Doppler,    
    previous_adr,
    current_week,
    current_tow,
    current_Doppler,
    current_adr,
    1, // set specifiedThresholdInCycles to 1 cycle
    &wasSlipDetected,
    &sizeOfSlipInCycles
  );
 
  CU_ASSERT_FATAL( result );

}


void test_CYCLESLIP_CheckForCycleSlipUsingDualFrequencyPhase(void)
{
  unsigned short week; 
  double   tow;
  double   adr_Frequency1;
  double   adr_Frequency2;
  double   wavelength1;
  double   wavelength2;
  BOOL     wasSlipDetected = FALSE;
  double   sizeOfSlipInCycles = 0;

  BOOL result;

  /*take PRN04 L1&L2 for testing 
  7    C1    D1    L1    L2    P2    S1    S2            # / TYPES OF OBSERV 
 07 11 20 18 49  3.0000000  0  8G04G08G09G11G15G17G26G28
  23232247.859 6      4451.949 6   -563066.531 6   -398459.53147  23232244.62147
        44.250          28.000  
  */
  week           = 1454; 
  tow            = 240543;
  adr_Frequency1 = -563066.531;
  adr_Frequency2 = -398459.531;
  wavelength1    = 0.19029367279836488; // L1 wavelength
  wavelength2    = 0.24421021342456825; // L2 wavelength

  result = CYCLESLIP_CheckForCycleSlipUsingDualFrequencyPhase(
    adr_Frequency1,
    adr_Frequency2,
    wavelength1,
    wavelength2,
    1,
    &wasSlipDetected,
    &sizeOfSlipInCycles
    );

   CU_ASSERT_FATAL( result );

}

void test_CYCLESLIP_CheckForCycleSlipUsingTripleDifferencePhase(void)
{
  /* reference receiver data
  8    C1    D1    D2    L1    L2    P2    S1    S2      # / TYPES OF OBSERV
 07 11 20 18 50 30.0000000  0  8G04G08G09G11G15G17G26G28
  .
  .
  .

  24478043.883 5     -3740.438 5     -2914.64846    187937.197 5    122712.37646
  24478041.43846        39.600          21.700  
  21729246.461 6     -2275.540 6     -1773.14847    181031.021 6    121812.88247
  21729240.57447        45.800          33.100  
 07 11 20 18 50 31.0000000  0  8G04G08G09G11G15G17G26G28
  .
  .
  .

  24478756.125 4     -3740.687 4     -2914.82546    191677.884 4    125627.20146
  24478753.12546        38.400          21.800  
  21729679.164 6     -2275.961 6     -1773.47947    183306.982 6    123586.36047
  21729673.80547        45.500          33.700 
  */

  /* rover receiver data
  7    C1    D1    L1    L2    P2    S1    S2            # / TYPES OF OBSERV 

 07 11 20 18 50 30.0000000  0  9G04G08G09G11G12G15G17G26G28
  .
  .
  .

  24606655.953 6     -2798.246 6   2115829.996 6   1632111.21147  24606653.22347
        43.000          27.000  
  21857854.102 7     -1333.359 7   1837353.309 7   1421142.19948  21857848.94148
        48.250          36.500  
 07 11 20 18 50 31.0000000  0  9G04G08G09G11G12G15G17G26G28
  .
  .
  .

  24607188.367 5     -2798.664 5   2118628.660 5   1634291.98447  24607185.59847
        42.250          27.250  
  21858108.109 7     -1333.949 7   1838687.258 7   1422181.64148  21858102.52048
        48.500          36.250  
  */
  unsigned short current_week; 
  double   current_tow;
  unsigned short previous_week; 
  double   previous_tow;
  double   adr_reference_rx_base_sat;
  double   adr_reference_rx;
  double   adr_rover_rx_base_sat;
  double   adr_rover_rx;
  double   prev_adr_reference_rx_base_sat;
  double   prev_adr_reference_rx;
  double   prev_adr_rover_rx_base_sat;
  double   prev_adr_rover_rx;
  double   wavelength;
  BOOL     wasSlipDetected = FALSE;
  double   sizeOfSlipInCycles = 0;

  BOOL result;

  /*take PRN26&28 L1 for testing, PRN28 as base satellite */
  current_week    = 1454; 
  current_tow     = 240631;
  previous_week   = 1454; 
  previous_tow    = 240630;

  adr_reference_rx_base_sat = 183306.982;
  adr_reference_rx          = 191677.884;
  adr_rover_rx_base_sat     = 1838687.258;
  adr_rover_rx              = 2118628.660;

  prev_adr_reference_rx_base_sat = 181031.021;
  prev_adr_reference_rx          = 187937.197;
  prev_adr_rover_rx_base_sat     = 1837353.309;
  prev_adr_rover_rx              = 2115829.996;

  wavelength = 0.19029367279836488; // L1 wavelength

  result = CYCLESLIP_CheckForCycleSlipUsingTripleDifferencePhase(
    current_week, 
    current_tow,
    previous_week, 
    previous_tow,  
    adr_reference_rx_base_sat,
    adr_reference_rx,
    adr_rover_rx_base_sat,
    adr_rover_rx,
    prev_adr_reference_rx_base_sat,
    prev_adr_reference_rx,
    prev_adr_rover_rx_base_sat,
    prev_adr_rover_rx,    
    1,
    &wasSlipDetected,
    &sizeOfSlipInCycles
    );

  CU_ASSERT_FATAL( result );

}