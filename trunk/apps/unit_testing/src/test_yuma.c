/** 
\file    test_yuma.c
\brief   unit tests for yuma.c/.h
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
#include "yuma.h"


int init_suite_YUMA(void)
{
  return 0;
}

int clean_suite_YUMA(void)
{
  return 0;
}



/* yuma431.txt
******** Week 431 almanac for PRN-01 ********
ID:                         01
Health:                     000
Eccentricity:               0.7102489471E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9908943176
Rate of Right Ascen(r/s):  -0.7632479537E-008
SQRT(A)  (m 1/2):           5153.553711
Right Ascen at Week(rad):  -0.1041447520E+001
Argument of Perigee(rad):  -1.798838139
Mean Anom(rad):            -0.2883712769E+001
Af0(s):                     0.1697540283E-003
Af1(s/s):                   0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-02 ********
ID:                         02
Health:                     000
Eccentricity:               0.8688926697E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9451141357
Rate of Right Ascen(r/s):  -0.8130882634E-008
SQRT(A)  (m 1/2):           5153.601074
Right Ascen at Week(rad):   0.3086080551E+001
Argument of Perigee(rad):   2.422875524
Mean Anom(rad):            -0.2096464515E+001
Af0(s):                     0.1554489136E-003
Af1(s/s):                   0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-03 ********
ID:                         03
Health:                     000
Eccentricity:               0.1021337509E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9255142212
Rate of Right Ascen(r/s):  -0.8123606676E-008
SQRT(A)  (m 1/2):           5153.592773
Right Ascen at Week(rad):   0.1944178343E+001
Argument of Perigee(rad):   0.794445515
Mean Anom(rad):            -0.2681670308E+001
Af0(s):                     0.1525878906E-003
Af1(s/s):                   0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-04 ********
ID:                         04
Health:                     000
Eccentricity:               0.8028030396E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9444789886
Rate of Right Ascen(r/s):  -0.8141796570E-008
SQRT(A)  (m 1/2):           5153.604492
Right Ascen at Week(rad):   0.3105142951E+001
Argument of Perigee(rad):   0.320883989
Mean Anom(rad):             0.5347421169E+000
Af0(s):                    -0.3147125244E-004
Af1(s/s):                  -0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-05 ********
ID:                         05
Health:                     000
Eccentricity:               0.8495330811E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9410877228
Rate of Right Ascen(r/s):  -0.8141796570E-008
SQRT(A)  (m 1/2):           5153.628418
Right Ascen at Week(rad):   0.9098359346E+000
Argument of Perigee(rad):   1.218577743
Mean Anom(rad):            -0.2140271664E-001
Af0(s):                     0.5731582642E-003
Af1(s/s):                   0.2182787284E-010
week:                        431

******** Week 431 almanac for PRN-06 ********
ID:                         06
Health:                     000
Eccentricity:               0.5452632904E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9331188202
Rate of Right Ascen(r/s):  -0.8050847100E-008
SQRT(A)  (m 1/2):           5153.580078
Right Ascen at Week(rad):   0.2007190228E+001
Argument of Perigee(rad):  -1.667430043
Mean Anom(rad):             0.1462155223E+001
Af0(s):                     0.2021789551E-003
Af1(s/s):                  -0.1091393642E-010
week:                        431

******** Week 431 almanac for PRN-07 ********
ID:                         07
Health:                     000
Eccentricity:               0.1010322571E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9347114563
Rate of Right Ascen(r/s):  -0.8032657206E-008
SQRT(A)  (m 1/2):           5153.721191
Right Ascen at Week(rad):   0.1981814384E+001
Argument of Perigee(rad):  -1.631943583
Mean Anom(rad):             0.1292079210E+001
Af0(s):                    -0.2288818359E-004
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-08 ********
ID:                         08
Health:                     000
Eccentricity:               0.1020097733E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9812278748
Rate of Right Ascen(r/s):  -0.7828930393E-008
SQRT(A)  (m 1/2):           5153.615723
Right Ascen at Week(rad):   0.1546382904E-002
Argument of Perigee(rad):   2.837109804
Mean Anom(rad):             0.1522171497E+001
Af0(s):                    -0.1316070557E-003
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-09 ********
ID:                         09
Health:                     000
Eccentricity:               0.1938629150E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9690589905
Rate of Right Ascen(r/s):  -0.7978087524E-008
SQRT(A)  (m 1/2):           5153.638672
Right Ascen at Week(rad):  -0.8547973633E-001
Argument of Perigee(rad):   1.377475858
Mean Anom(rad):             0.1178699851E+001
Af0(s):                     0.9918212891E-004
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-10 ********
ID:                         10
Health:                     000
Eccentricity:               0.7751941681E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9663391113
Rate of Right Ascen(r/s):  -0.7938069757E-008
SQRT(A)  (m 1/2):           5153.684570
Right Ascen at Week(rad):  -0.2117694139E+001
Argument of Perigee(rad):   0.499766946
Mean Anom(rad):            -0.1432633519E+001
Af0(s):                    -0.1602172852E-003
Af1(s/s):                  -0.1091393642E-010
week:                        431

******** Week 431 almanac for PRN-11 ********
ID:                         11
Health:                     000
Eccentricity:               0.7613658905E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.8926105499
Rate of Right Ascen(r/s):  -0.8629285730E-008
SQRT(A)  (m 1/2):           5153.660645
Right Ascen at Week(rad):   0.2913496137E+001
Argument of Perigee(rad):   0.546045899
Mean Anom(rad):             0.1971806884E+001
Af0(s):                     0.2670288086E-004
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-12 ********
ID:                         12
Health:                     000
Eccentricity:               0.3116130829E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9625511169
Rate of Right Ascen(r/s):  -0.7905327948E-008
SQRT(A)  (m 1/2):           5153.675293
Right Ascen at Week(rad):   0.9990948439E+000
Argument of Perigee(rad):  -1.092309713
Mean Anom(rad):             0.2499961734E+001
Af0(s):                    -0.3499984741E-003
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-13 ********
ID:                         13
Health:                     000
Eccentricity:               0.3281116486E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9950408936
Rate of Right Ascen(r/s):  -0.7592461770E-008
SQRT(A)  (m 1/2):           5153.674805
Right Ascen at Week(rad):  -0.1053738832E+001
Argument of Perigee(rad):   1.408947825
Mean Anom(rad):            -0.1709633946E+001
Af0(s):                     0.2250671387E-003
Af1(s/s):                   0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-14 ********
ID:                         14
Health:                     000
Eccentricity:               0.3743171692E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9888858795
Rate of Right Ascen(r/s):  -0.7654307410E-008
SQRT(A)  (m 1/2):           5153.594238
Right Ascen at Week(rad):  -0.1070010424E+001
Argument of Perigee(rad):  -2.134685159
Mean Anom(rad):            -0.2045622945E+001
Af0(s):                    -0.3166198730E-003
Af1(s/s):                   0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-15 ********
ID:                         15
Health:                     000
Eccentricity:               0.1007080078E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9574222565
Rate of Right Ascen(r/s):  -0.7956259651E-008
SQRT(A)  (m 1/2):           5153.536621
Right Ascen at Week(rad):  -0.1105578065E+001
Argument of Perigee(rad):  -2.075240493
Mean Anom(rad):            -0.3428694010E+000
Af0(s):                    -0.7438659668E-004
Af1(s/s):                   0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-16 ********
ID:                         16
Health:                     000
Eccentricity:               0.4120349884E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9643554688
Rate of Right Ascen(r/s):  -0.7883500075E-008
SQRT(A)  (m 1/2):           5153.726074
Right Ascen at Week(rad):   0.1015131950E+001
Argument of Perigee(rad):  -0.562877297
Mean Anom(rad):            -0.1362055540E+000
Af0(s):                     0.1268386841E-003
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-17 ********
ID:                         17
Health:                     000
Eccentricity:               0.3050804138E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9593334198
Rate of Right Ascen(r/s):  -0.7788912626E-008
SQRT(A)  (m 1/2):           5153.644043
Right Ascen at Week(rad):   0.2055675030E+001
Argument of Perigee(rad):  -2.966418147
Mean Anom(rad):            -0.1160219073E+001
Af0(s):                     0.4768371582E-004
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-18 ********
ID:                         18
Health:                     000
Eccentricity:               0.8884906769E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9504356384
Rate of Right Ascen(r/s):  -0.8076312952E-008
SQRT(A)  (m 1/2):           5153.591797
Right Ascen at Week(rad):  -0.2104108930E+001
Argument of Perigee(rad):  -2.521630764
Mean Anom(rad):            -0.1232171059E-001
Af0(s):                    -0.2222061157E-003
Af1(s/s):                   0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-19 ********
ID:                         19
Health:                     000
Eccentricity:               0.4151821136E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9571952820
Rate of Right Ascen(r/s):  -0.7803464541E-008
SQRT(A)  (m 1/2):           5153.574219
Right Ascen at Week(rad):   0.2113353252E+001
Argument of Perigee(rad):  -0.678779721
Mean Anom(rad):            -0.1759031773E+001
Af0(s):                     0.9536743164E-005
Af1(s/s):                   0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-20 ********
ID:                         20
Health:                     000
Eccentricity:               0.3303527832E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9499492645
Rate of Right Ascen(r/s):  -0.8123606676E-008
SQRT(A)  (m 1/2):           5153.631836
Right Ascen at Week(rad):  -0.2157547712E+001
Argument of Perigee(rad):   1.297490835
Mean Anom(rad):            -0.1667904854E-001
Af0(s):                     0.1249313354E-003
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-21 ********
ID:                         21
Health:                     000
Eccentricity:               0.1308870316E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9380130768
Rate of Right Ascen(r/s):  -0.8218194125E-008
SQRT(A)  (m 1/2):           5153.701660
Right Ascen at Week(rad):   0.3121827245E+001
Argument of Perigee(rad):  -2.783395767
Mean Anom(rad):             0.1315448403E+001
Af0(s):                     0.7629394531E-004
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-22 ********
ID:                         22
Health:                     000
Eccentricity:               0.5015373230E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9480495453
Rate of Right Ascen(r/s):  -0.8094502846E-008
SQRT(A)  (m 1/2):           5153.622559
Right Ascen at Week(rad):  -0.2097227693E+001
Argument of Perigee(rad):  -1.735002279
Mean Anom(rad):            -0.1406764150E+001
Af0(s):                     0.2040863037E-003
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-23 ********
ID:                         23
Health:                     000
Eccentricity:               0.5223274231E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9710712433
Rate of Right Ascen(r/s):  -0.7821654435E-008
SQRT(A)  (m 1/2):           5153.543945
Right Ascen at Week(rad):  -0.1093096137E+001
Argument of Perigee(rad):   2.745398283
Mean Anom(rad):            -0.2477374077E+001
Af0(s):                     0.3147125244E-003
Af1(s/s):                   0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-24 ********
ID:                         24
Health:                     000
Eccentricity:               0.8298397064E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9546833038
Rate of Right Ascen(r/s):  -0.8029019227E-008
SQRT(A)  (m 1/2):           5153.602051
Right Ascen at Week(rad):  -0.3138392091E+001
Argument of Perigee(rad):  -0.751108646
Mean Anom(rad):            -0.1998051405E+000
Af0(s):                     0.5054473877E-004
Af1(s/s):                   0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-25 ********
ID:                         25
Health:                     000
Eccentricity:               0.1172208786E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9624862671
Rate of Right Ascen(r/s):  -0.8043571142E-008
SQRT(A)  (m 1/2):           5153.639160
Right Ascen at Week(rad):  -0.1430066824E+000
Argument of Perigee(rad):  -1.294391394
Mean Anom(rad):             0.1407431364E+000
Af0(s):                     0.4863739014E-003
Af1(s/s):                   0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-26 ********
ID:                         26
Health:                     000
Eccentricity:               0.1842498779E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9930324554
Rate of Right Ascen(r/s):  -0.7610651664E-008
SQRT(A)  (m 1/2):           5153.533203
Right Ascen at Week(rad):  -0.1054520845E+001
Argument of Perigee(rad):   0.893498898
Mean Anom(rad):             0.3093351603E+001
Af0(s):                     0.1125335693E-003
Af1(s/s):                   0.1091393642E-010
week:                        431

******** Week 431 almanac for PRN-27 ********
ID:                         27
Health:                     000
Eccentricity:               0.2089309692E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9670639038
Rate of Right Ascen(r/s):  -0.7970811566E-008
SQRT(A)  (m 1/2):           5153.587402
Right Ascen at Week(rad):  -0.1090148687E+000
Argument of Perigee(rad):  -1.767943978
Mean Anom(rad):             0.3054419756E+000
Af0(s):                     0.1516342163E-003
Af1(s/s):                   0.3637978807E-011
week:                        431

******** Week 431 almanac for PRN-28 ********
ID:                         28
Health:                     000
Eccentricity:               0.1308441162E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9624786377
Rate of Right Ascen(r/s):  -0.7868948160E-008
SQRT(A)  (m 1/2):           5153.655762
Right Ascen at Week(rad):   0.1024014950E+001
Argument of Perigee(rad):  -2.157932401
Mean Anom(rad):            -0.8401706219E+000
Af0(s):                    -0.1144409180E-004
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-30 ********
ID:                         30
Health:                     000
Eccentricity:               0.1036357880E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9472885132
Rate of Right Ascen(r/s):  -0.8061761037E-008
SQRT(A)  (m 1/2):           5153.641602
Right Ascen at Week(rad):   0.9601974487E+000
Argument of Perigee(rad):   1.369896889
Mean Anom(rad):            -0.4680734873E+000
Af0(s):                     0.5435943604E-004
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-31 ********
ID:                         31
Health:                     000
Eccentricity:               0.6537437439E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9655532837
Rate of Right Ascen(r/s):  -0.8010829333E-008
SQRT(A)  (m 1/2):           5153.565918
Right Ascen at Week(rad):  -0.5225157738E-001
Argument of Perigee(rad):  -1.318798423
Mean Anom(rad):             0.2062978387E+001
Af0(s):                     0.0000000000E+000
Af1(s/s):                   0.0000000000E+000
week:                        431

******** Week 431 almanac for PRN-32 ********
ID:                         32
Health:                     063
Eccentricity:               0.1481294632E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9716835022
Rate of Right Ascen(r/s):  -0.7843482308E-008
SQRT(A)  (m 1/2):           5153.470703
Right Ascen at Week(rad):  -0.2059595823E+001
Argument of Perigee(rad):  -1.354981422
Mean Anom(rad):            -0.2236462355E+001
Af0(s):                     0.1573562622E-003
Af1(s/s):                   0.1091393642E-010
week:                        431
*/


void test_YUMA_ReadAlmanacDataFromFile(void)
{
  YUMA_structAlmanac test;
  YUMA_structAlmanac alm[32];
  unsigned char number_read=0;
  BOOL result;
  unsigned i = 0;

  memset( alm, 0, sizeof(YUMA_structAlmanac)*32 );
  result = YUMA_ReadAlmanacDataFromFile( 
    "yuma431.txt",
    alm,
    32,
    &number_read );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_FATAL( number_read == 31 ); // PRN29 is not in the file

  /*
  ******** Week 431 almanac for PRN-01 ********
  ID:                         01
  Health:                     000
  Eccentricity:               0.7102489471E-002
  Time of Applicability(s):  589824.0000
  Orbital Inclination(rad):   0.9908943176
  Rate of Right Ascen(r/s):  -0.7632479537E-008
  SQRT(A)  (m 1/2):           5153.553711
  Right Ascen at Week(rad):  -0.1041447520E+001
  Argument of Perigee(rad):  -1.798838139
  Mean Anom(rad):            -0.2883712769E+001
  Af0(s):                     0.1697540283E-003
  Af1(s/s):                   0.3637978807E-011
  week:                        431
  */
  test.prn    = 1;
  test.health = 0;
  test.week   = 431;
  test.ecc      = 0.7102489471E-002;
  test.toa      = 589824.0000;
  test.i0       = 0.9908943176;
  test.omegadot = -0.7632479537E-008;
  test.sqrta    = 5153.553711;
  test.omega0   = -0.1041447520E+001;
  test.w        = -1.798838139;
  test.m0       = -0.2883712769E+001;
  test.af0      = 0.1697540283E-003;
  test.af1      = 0.3637978807E-011;
  test.is_af0_af1_high_precision = 0;

  i = 0;
  CU_ASSERT( alm[i].prn == test.prn );
  CU_ASSERT( alm[i].health == test.health );
  CU_ASSERT( alm[i].week == test.week );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].ecc, test.ecc, 1e-11 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].toa, test.toa, 1e-2 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].i0, test.i0, 1e-9 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].omegadot, test.omegadot, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].sqrta, test.sqrta, 1e-6 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].omega0, test.omega0, 1e-8 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].w, test.w, 1e-8 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].m0, test.m0, 1e-7 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].af0, test.af0, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].af1, test.af1, 1e-20 );
  CU_ASSERT( alm[0].is_af0_af1_high_precision == test.is_af0_af1_high_precision );

    /*
ID:                         02
Health:                     000
Eccentricity:               0.8688926697E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9451141357
Rate of Right Ascen(r/s):  -0.8130882634E-008
SQRT(A)  (m 1/2):           5153.601074
Right Ascen at Week(rad):   0.3086080551E+001
Argument of Perigee(rad):   2.422875524
Mean Anom(rad):            -0.2096464515E+001
Af0(s):                     0.1554489136E-003
Af1(s/s):                   0.3637978807E-011
week:                        431
*/
  test.prn    = 2;
  test.health = 0;
  test.week   = 431;
  test.ecc      = 0.8688926697E-002;
  test.toa      = 589824.0000;
  test.i0       = 0.9451141357;
  test.omegadot = -0.8130882634E-008;
  test.sqrta    = 5153.601074;
  test.omega0   = 0.3086080551E+001;
  test.w        = 2.422875524;
  test.m0       = -0.2096464515E+001;
  test.af0      = 0.1554489136E-003;
  test.af1      = 0.3637978807E-011;
  test.is_af0_af1_high_precision = 0;

  i = 1;
  CU_ASSERT( alm[i].prn == test.prn );
  CU_ASSERT( alm[i].health == test.health );
  CU_ASSERT( alm[i].week == test.week );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].ecc, test.ecc, 1e-11 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].toa, test.toa, 1e-2 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].i0, test.i0, 1e-9 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].omegadot, test.omegadot, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].sqrta, test.sqrta, 1e-6 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].omega0, test.omega0, 1e-8 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].w, test.w, 1e-8 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].m0, test.m0, 1e-7 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].af0, test.af0, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].af1, test.af1, 1e-20 );
  CU_ASSERT( alm[0].is_af0_af1_high_precision == test.is_af0_af1_high_precision );

/*
ID:                         03
Health:                     000
Eccentricity:               0.1021337509E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9255142212
Rate of Right Ascen(r/s):  -0.8123606676E-008
SQRT(A)  (m 1/2):           5153.592773
Right Ascen at Week(rad):   0.1944178343E+001
Argument of Perigee(rad):   0.794445515
Mean Anom(rad):            -0.2681670308E+001
Af0(s):                     0.1525878906E-003
Af1(s/s):                   0.3637978807E-011
week:                        431
*/
  test.prn    = 3;
  test.health = 0;
  test.week   = 431;
  test.ecc      = 0.1021337509E-001;
  test.toa      = 589824.0000;
  test.i0       = 0.9255142212;
  test.omegadot = -0.8123606676E-008;
  test.sqrta    = 5153.592773;
  test.omega0   = 0.1944178343E+001;
  test.w        = 0.794445515;
  test.m0       = -0.2681670308E+001;
  test.af0      = 0.1525878906E-003;
  test.af1      = 0.3637978807E-011;
  test.is_af0_af1_high_precision = 0;

  i = 2;
  CU_ASSERT( alm[i].prn == test.prn );
  CU_ASSERT( alm[i].health == test.health );
  CU_ASSERT( alm[i].week == test.week );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].ecc, test.ecc, 1e-11 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].toa, test.toa, 1e-2 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].i0, test.i0, 1e-9 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].omegadot, test.omegadot, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].sqrta, test.sqrta, 1e-6 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].omega0, test.omega0, 1e-8 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].w, test.w, 1e-8 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].m0, test.m0, 1e-7 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].af0, test.af0, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].af1, test.af1, 1e-20 );
  CU_ASSERT( alm[0].is_af0_af1_high_precision == test.is_af0_af1_high_precision );
/*
ID:                         04
Health:                     000
Eccentricity:               0.8028030396E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9444789886
Rate of Right Ascen(r/s):  -0.8141796570E-008
SQRT(A)  (m 1/2):           5153.604492
Right Ascen at Week(rad):   0.3105142951E+001
Argument of Perigee(rad):   0.320883989
Mean Anom(rad):             0.5347421169E+000
Af0(s):                    -0.3147125244E-004
Af1(s/s):                  -0.3637978807E-011
week:                        431
*/
/*
ID:                         05
Health:                     000
Eccentricity:               0.8495330811E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9410877228
Rate of Right Ascen(r/s):  -0.8141796570E-008
SQRT(A)  (m 1/2):           5153.628418
Right Ascen at Week(rad):   0.9098359346E+000
Argument of Perigee(rad):   1.218577743
Mean Anom(rad):            -0.2140271664E-001
Af0(s):                     0.5731582642E-003
Af1(s/s):                   0.2182787284E-010
week:                        431
*/
/*
ID:                         06
Health:                     000
Eccentricity:               0.5452632904E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9331188202
Rate of Right Ascen(r/s):  -0.8050847100E-008
SQRT(A)  (m 1/2):           5153.580078
Right Ascen at Week(rad):   0.2007190228E+001
Argument of Perigee(rad):  -1.667430043
Mean Anom(rad):             0.1462155223E+001
Af0(s):                     0.2021789551E-003
Af1(s/s):                  -0.1091393642E-010
week:                        431
*/
/*
ID:                         07
Health:                     000
Eccentricity:               0.1010322571E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9347114563
Rate of Right Ascen(r/s):  -0.8032657206E-008
SQRT(A)  (m 1/2):           5153.721191
Right Ascen at Week(rad):   0.1981814384E+001
Argument of Perigee(rad):  -1.631943583
Mean Anom(rad):             0.1292079210E+001
Af0(s):                    -0.2288818359E-004
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
/*
ID:                         08
Health:                     000
Eccentricity:               0.1020097733E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9812278748
Rate of Right Ascen(r/s):  -0.7828930393E-008
SQRT(A)  (m 1/2):           5153.615723
Right Ascen at Week(rad):   0.1546382904E-002
Argument of Perigee(rad):   2.837109804
Mean Anom(rad):             0.1522171497E+001
Af0(s):                    -0.1316070557E-003
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
/*
ID:                         09
Health:                     000
Eccentricity:               0.1938629150E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9690589905
Rate of Right Ascen(r/s):  -0.7978087524E-008
SQRT(A)  (m 1/2):           5153.638672
Right Ascen at Week(rad):  -0.8547973633E-001
Argument of Perigee(rad):   1.377475858
Mean Anom(rad):             0.1178699851E+001
Af0(s):                     0.9918212891E-004
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
/*
ID:                         10
Health:                     000
Eccentricity:               0.7751941681E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9663391113
Rate of Right Ascen(r/s):  -0.7938069757E-008
SQRT(A)  (m 1/2):           5153.684570
Right Ascen at Week(rad):  -0.2117694139E+001
Argument of Perigee(rad):   0.499766946
Mean Anom(rad):            -0.1432633519E+001
Af0(s):                    -0.1602172852E-003
Af1(s/s):                  -0.1091393642E-010
week:                        431
*/
/*
ID:                         11
Health:                     000
Eccentricity:               0.7613658905E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.8926105499
Rate of Right Ascen(r/s):  -0.8629285730E-008
SQRT(A)  (m 1/2):           5153.660645
Right Ascen at Week(rad):   0.2913496137E+001
Argument of Perigee(rad):   0.546045899
Mean Anom(rad):             0.1971806884E+001
Af0(s):                     0.2670288086E-004
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
/*
ID:                         12
Health:                     000
Eccentricity:               0.3116130829E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9625511169
Rate of Right Ascen(r/s):  -0.7905327948E-008
SQRT(A)  (m 1/2):           5153.675293
Right Ascen at Week(rad):   0.9990948439E+000
Argument of Perigee(rad):  -1.092309713
Mean Anom(rad):             0.2499961734E+001
Af0(s):                    -0.3499984741E-003
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
/*
ID:                         13
Health:                     000
Eccentricity:               0.3281116486E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9950408936
Rate of Right Ascen(r/s):  -0.7592461770E-008
SQRT(A)  (m 1/2):           5153.674805
Right Ascen at Week(rad):  -0.1053738832E+001
Argument of Perigee(rad):   1.408947825
Mean Anom(rad):            -0.1709633946E+001
Af0(s):                     0.2250671387E-003
Af1(s/s):                   0.3637978807E-011
week:                        431
*/
/*
ID:                         14
Health:                     000
Eccentricity:               0.3743171692E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9888858795
Rate of Right Ascen(r/s):  -0.7654307410E-008
SQRT(A)  (m 1/2):           5153.594238
Right Ascen at Week(rad):  -0.1070010424E+001
Argument of Perigee(rad):  -2.134685159
Mean Anom(rad):            -0.2045622945E+001
Af0(s):                    -0.3166198730E-003
Af1(s/s):                   0.3637978807E-011
week:                        431
*/
/*
ID:                         15
Health:                     000
Eccentricity:               0.1007080078E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9574222565
Rate of Right Ascen(r/s):  -0.7956259651E-008
SQRT(A)  (m 1/2):           5153.536621
Right Ascen at Week(rad):  -0.1105578065E+001
Argument of Perigee(rad):  -2.075240493
Mean Anom(rad):            -0.3428694010E+000
Af0(s):                    -0.7438659668E-004
Af1(s/s):                   0.3637978807E-011
week:                        431
*/
/*
ID:                         16
Health:                     000
Eccentricity:               0.4120349884E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9643554688
Rate of Right Ascen(r/s):  -0.7883500075E-008
SQRT(A)  (m 1/2):           5153.726074
Right Ascen at Week(rad):   0.1015131950E+001
Argument of Perigee(rad):  -0.562877297
Mean Anom(rad):            -0.1362055540E+000
Af0(s):                     0.1268386841E-003
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
/*
ID:                         17
Health:                     000
Eccentricity:               0.3050804138E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9593334198
Rate of Right Ascen(r/s):  -0.7788912626E-008
SQRT(A)  (m 1/2):           5153.644043
Right Ascen at Week(rad):   0.2055675030E+001
Argument of Perigee(rad):  -2.966418147
Mean Anom(rad):            -0.1160219073E+001
Af0(s):                     0.4768371582E-004
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
/*
ID:                         18
Health:                     000
Eccentricity:               0.8884906769E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9504356384
Rate of Right Ascen(r/s):  -0.8076312952E-008
SQRT(A)  (m 1/2):           5153.591797
Right Ascen at Week(rad):  -0.2104108930E+001
Argument of Perigee(rad):  -2.521630764
Mean Anom(rad):            -0.1232171059E-001
Af0(s):                    -0.2222061157E-003
Af1(s/s):                   0.3637978807E-011
week:                        431
*/
/*
ID:                         19
Health:                     000
Eccentricity:               0.4151821136E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9571952820
Rate of Right Ascen(r/s):  -0.7803464541E-008
SQRT(A)  (m 1/2):           5153.574219
Right Ascen at Week(rad):   0.2113353252E+001
Argument of Perigee(rad):  -0.678779721
Mean Anom(rad):            -0.1759031773E+001
Af0(s):                     0.9536743164E-005
Af1(s/s):                   0.3637978807E-011
week:                        431
*/
/*
ID:                         20
Health:                     000
Eccentricity:               0.3303527832E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9499492645
Rate of Right Ascen(r/s):  -0.8123606676E-008
SQRT(A)  (m 1/2):           5153.631836
Right Ascen at Week(rad):  -0.2157547712E+001
Argument of Perigee(rad):   1.297490835
Mean Anom(rad):            -0.1667904854E-001
Af0(s):                     0.1249313354E-003
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
/*
ID:                         21
Health:                     000
Eccentricity:               0.1308870316E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9380130768
Rate of Right Ascen(r/s):  -0.8218194125E-008
SQRT(A)  (m 1/2):           5153.701660
Right Ascen at Week(rad):   0.3121827245E+001
Argument of Perigee(rad):  -2.783395767
Mean Anom(rad):             0.1315448403E+001
Af0(s):                     0.7629394531E-004
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
/*
ID:                         22
Health:                     000
Eccentricity:               0.5015373230E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9480495453
Rate of Right Ascen(r/s):  -0.8094502846E-008
SQRT(A)  (m 1/2):           5153.622559
Right Ascen at Week(rad):  -0.2097227693E+001
Argument of Perigee(rad):  -1.735002279
Mean Anom(rad):            -0.1406764150E+001
Af0(s):                     0.2040863037E-003
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
/*
ID:                         23
Health:                     000
Eccentricity:               0.5223274231E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9710712433
Rate of Right Ascen(r/s):  -0.7821654435E-008
SQRT(A)  (m 1/2):           5153.543945
Right Ascen at Week(rad):  -0.1093096137E+001
Argument of Perigee(rad):   2.745398283
Mean Anom(rad):            -0.2477374077E+001
Af0(s):                     0.3147125244E-003
Af1(s/s):                   0.3637978807E-011
week:                        431
*/
/*
ID:                         24
Health:                     000
Eccentricity:               0.8298397064E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9546833038
Rate of Right Ascen(r/s):  -0.8029019227E-008
SQRT(A)  (m 1/2):           5153.602051
Right Ascen at Week(rad):  -0.3138392091E+001
Argument of Perigee(rad):  -0.751108646
Mean Anom(rad):            -0.1998051405E+000
Af0(s):                     0.5054473877E-004
Af1(s/s):                   0.3637978807E-011
week:                        431
*/
/*
ID:                         25
Health:                     000
Eccentricity:               0.1172208786E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9624862671
Rate of Right Ascen(r/s):  -0.8043571142E-008
SQRT(A)  (m 1/2):           5153.639160
Right Ascen at Week(rad):  -0.1430066824E+000
Argument of Perigee(rad):  -1.294391394
Mean Anom(rad):             0.1407431364E+000
Af0(s):                     0.4863739014E-003
Af1(s/s):                   0.3637978807E-011
week:                        431
*/
/*
ID:                         26
Health:                     000
Eccentricity:               0.1842498779E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9930324554
Rate of Right Ascen(r/s):  -0.7610651664E-008
SQRT(A)  (m 1/2):           5153.533203
Right Ascen at Week(rad):  -0.1054520845E+001
Argument of Perigee(rad):   0.893498898
Mean Anom(rad):             0.3093351603E+001
Af0(s):                     0.1125335693E-003
Af1(s/s):                   0.1091393642E-010
week:                        431
*/
/*
ID:                         27
Health:                     000
Eccentricity:               0.2089309692E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9670639038
Rate of Right Ascen(r/s):  -0.7970811566E-008
SQRT(A)  (m 1/2):           5153.587402
Right Ascen at Week(rad):  -0.1090148687E+000
Argument of Perigee(rad):  -1.767943978
Mean Anom(rad):             0.3054419756E+000
Af0(s):                     0.1516342163E-003
Af1(s/s):                   0.3637978807E-011
week:                        431
*/
/*
ID:                         28
Health:                     000
Eccentricity:               0.1308441162E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9624786377
Rate of Right Ascen(r/s):  -0.7868948160E-008
SQRT(A)  (m 1/2):           5153.655762
Right Ascen at Week(rad):   0.1024014950E+001
Argument of Perigee(rad):  -2.157932401
Mean Anom(rad):            -0.8401706219E+000
Af0(s):                    -0.1144409180E-004
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
/*
ID:                         30
Health:                     000
Eccentricity:               0.1036357880E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9472885132
Rate of Right Ascen(r/s):  -0.8061761037E-008
SQRT(A)  (m 1/2):           5153.641602
Right Ascen at Week(rad):   0.9601974487E+000
Argument of Perigee(rad):   1.369896889
Mean Anom(rad):            -0.4680734873E+000
Af0(s):                     0.5435943604E-004
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
/*
ID:                         31
Health:                     000
Eccentricity:               0.6537437439E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9655532837
Rate of Right Ascen(r/s):  -0.8010829333E-008
SQRT(A)  (m 1/2):           5153.565918
Right Ascen at Week(rad):  -0.5225157738E-001
Argument of Perigee(rad):  -1.318798423
Mean Anom(rad):             0.2062978387E+001
Af0(s):                     0.0000000000E+000
Af1(s/s):                   0.0000000000E+000
week:                        431
*/
  test.prn    = 31;
  test.health = 0;
  test.week   = 431;
  test.ecc      = 0.6537437439E-002;
  test.toa      = 589824.0000;
  test.i0       = 0.9655532837;
  test.omegadot = -0.8010829333E-008;
  test.sqrta    = 5153.565918;
  test.omega0   = -0.5225157738E-001;
  test.w        = -1.318798423;
  test.m0       = 0.2062978387E+001;
  test.af0      = 0.0000000000E+000;
  test.af1      = 0.0000000000E+000;
  test.is_af0_af1_high_precision = 0;

  i = 29;
  CU_ASSERT( alm[i].prn == test.prn );
  CU_ASSERT( alm[i].health == test.health );
  CU_ASSERT( alm[i].week == test.week );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].ecc, test.ecc, 1e-11 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].toa, test.toa, 1e-2 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].i0, test.i0, 1e-9 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].omegadot, test.omegadot, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].sqrta, test.sqrta, 1e-6 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].omega0, test.omega0, 1e-8 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].w, test.w, 1e-8 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].m0, test.m0, 1e-7 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].af0, test.af0, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].af1, test.af1, 1e-20 );
  CU_ASSERT( alm[0].is_af0_af1_high_precision == test.is_af0_af1_high_precision );
/*
ID:                         32
Health:                     063
Eccentricity:               0.1481294632E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9716835022
Rate of Right Ascen(r/s):  -0.7843482308E-008
SQRT(A)  (m 1/2):           5153.470703
Right Ascen at Week(rad):  -0.2059595823E+001
Argument of Perigee(rad):  -1.354981422
Mean Anom(rad):            -0.2236462355E+001
Af0(s):                     0.1573562622E-003
Af1(s/s):                   0.1091393642E-010
week:                        431
*/
  test.prn    = 32;
  test.health = 63;
  test.week   = 431;
  test.ecc      = 0.1481294632E-001;
  test.toa      = 589824.0000;
  test.i0       = 0.9716835022;
  test.omegadot = -0.7843482308E-008;
  test.sqrta    = 5153.470703;
  test.omega0   = -0.2059595823E+001;
  test.w        = -1.354981422;
  test.m0       = -0.2236462355E+001;
  test.af0      = 0.1573562622E-003;
  test.af1      = 0.1091393642E-010;
  test.is_af0_af1_high_precision = 0;

  i = 30;
  CU_ASSERT( alm[i].prn == test.prn );
  CU_ASSERT( alm[i].health == test.health );
  CU_ASSERT( alm[i].week == test.week );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].ecc, test.ecc, 1e-11 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].toa, test.toa, 1e-2 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].i0, test.i0, 1e-9 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].omegadot, test.omegadot, 1e-17 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].sqrta, test.sqrta, 1e-6 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].omega0, test.omega0, 1e-8 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].w, test.w, 1e-8 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].m0, test.m0, 1e-7 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].af0, test.af0, 1e-12 );
  CU_ASSERT_DOUBLE_EQUAL( alm[i].af1, test.af1, 1e-20 );
  CU_ASSERT( alm[0].is_af0_af1_high_precision == test.is_af0_af1_high_precision );


}

void test_YUMA_WriteAlmanacDataToFile(void)
{
  YUMA_structAlmanac alm[32];
  YUMA_structAlmanac alm_test[32];
  unsigned char number_read=0;
  BOOL result;
  unsigned i = 0;

  memset( alm, 0, sizeof(YUMA_structAlmanac)*32 );
  memset( alm_test, 0, sizeof(YUMA_structAlmanac)*32 );

  // read in the data file
  result = YUMA_ReadAlmanacDataFromFile( 
    "yuma431.txt",
    alm,
    32,
    &number_read );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_FATAL( number_read == 31 ); // PRN29 is not in the file

  // write a new data file
  result = YUMA_WriteAlmanacDataToFile(
    "yuma.txt",
    alm,
    number_read );
  CU_ASSERT_FATAL( result );

  // read in the new data file
  result = YUMA_ReadAlmanacDataFromFile( 
    "yuma.txt",
    alm_test,
    32,
    &number_read );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_FATAL( number_read == 31 ); // PRN29 is not in the file

  for( i = 0; i < number_read; i++ )
  {
    CU_ASSERT( alm[i].prn == alm_test[i].prn );    
    CU_ASSERT( alm[i].health == alm_test[i].health );
    CU_ASSERT( alm[i].week == alm_test[i].week );
    CU_ASSERT_DOUBLE_EQUAL( alm[i].ecc, alm_test[i].ecc, 1e-11 );
    CU_ASSERT_DOUBLE_EQUAL( alm[i].toa, alm_test[i].toa, 1e-2 );
    CU_ASSERT_DOUBLE_EQUAL( alm[i].i0, alm_test[i].i0, 1e-9 );
    CU_ASSERT_DOUBLE_EQUAL( alm[i].omegadot, alm_test[i].omegadot, 1e-17 );
    CU_ASSERT_DOUBLE_EQUAL( alm[i].sqrta, alm_test[i].sqrta, 1e-6 );
    CU_ASSERT_DOUBLE_EQUAL( alm[i].omega0, alm_test[i].omega0, 1e-8 );
    CU_ASSERT_DOUBLE_EQUAL( alm[i].w, alm_test[i].w, 1e-8 );
    CU_ASSERT_DOUBLE_EQUAL( alm[i].m0, alm_test[i].m0, 1e-7 );
    CU_ASSERT_DOUBLE_EQUAL( alm[i].af0, alm_test[i].af0, 1e-12 );
    CU_ASSERT_DOUBLE_EQUAL( alm[i].af1, alm_test[i].af1, 1e-20 );
    CU_ASSERT( alm[0].is_af0_af1_high_precision == alm_test[i].is_af0_af1_high_precision );
  }
  remove("yuma.txt");
}


void test_YUMA_WriteSingleAlmanacElementToBuffer(void)
{
  char buffer[1024];
  YUMA_structAlmanac alm[32];
  unsigned char number_read=0;
  BOOL result;  
  char test_buffer[] = "******** Week 431 almanac for PRN-01 ********\n\
ID:                         01\n\
Health:                     000\n\
Eccentricity:               0.007102489471\n\
Time of Applicability(s):   589824.0000\n\
Orbital Inclination(rad):   0.9908943176\n\
Rate of Right Ascen(r/s):   -7.632479537e-009\n\
SQRT(A)  (m 1/2):           5153.553711\n\
Right Ascen at Week(rad):   -1.04144752\n\
Argument of Perigee(rad):   -1.798838139\n\
Mean Anom(rad):             -2.883712769\n\
Af0(s):                     0.0001697540283\n\
Af1(s/s):                   3.637978807e-012\n\
week:                       431\n\n";

  memset( alm, 0, sizeof(YUMA_structAlmanac)*32 );

  // read in the data file
  result = YUMA_ReadAlmanacDataFromFile( 
    "yuma431.txt",
    alm,
    32,
    &number_read );
  CU_ASSERT_FATAL( result );
  CU_ASSERT_FATAL( number_read == 31 ); // PRN29 is not in the file

  result = YUMA_WriteSingleAlmanacElementToBuffer(
    alm[0],
    buffer,
    1024 
    );
  CU_ASSERT_FATAL( result );

  /*
  printf( buffer );
  printf( test_buffer );
  {
    int ii = strlen( buffer);
    int jj = strlen( test_buffer );
    int gg= 0;
  }
  */

  CU_ASSERT( strcmp( buffer, test_buffer ) == 0 );  
}

