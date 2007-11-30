/** 
\file    test_geodesy.h
\brief   unit tests for geodesy.c/.h
\author  Glenn D. MacGougan (GDM)
\date    2007-11-26
\since   2007-11-26

2007-11-26, GDM, Creation \n

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
#ifndef _C_TEST_GEODESY_H_
#define _C_TEST_GEODESY_H_

#ifdef __cplusplus
extern "C" {
#endif


/** 
\brief  The suite initialization function.
\return Returns zero on success, non-zero otherwise.
*/
int init_suite_GEODESY(void);

/** 
\brief  The suite cleanup function.
\return Returns zero on success, non-zero otherwise.
*/
int clean_suite_GEODESY(void);


/** \brief  Test GEODESY_GetReferenceEllipseParameters(). */
void test_GEODESY_GetReferenceEllipseParameters(void);

/** \brief  Test GEODESY coordinate conversions. */
void test_GEODESY_ConvertCoordinates(void);

/** \brief  Test GEODESY_ComputeNorthingEastingVertical(). */
void test_GEODESY_ComputeNorthingEastingVertical(void);

/** \brief  Test GEODESY_ComputePositionDifference(). */
void test_GEODESY_ComputePositionDifference(void);

/** \brief  Test GEODESY_ComputeMeridianRadiusOfCurvature(). */
void test_GEODESY_ComputeMeridianRadiusOfCurvature(void);

/** \brief  Test GEODESY_ComputePrimeVerticalRadiusOfCurvature(). */
void test_GEODESY_ComputePrimeVerticalRadiusOfCurvature(void);

/** \brief  Test GEODESY_ComputeMeridianArcBetweenTwoLatitudes(). */
void test_GEODESY_ComputeMeridianArcBetweenTwoLatitudes(void);

/** \brief  Test GEODESY_ComputeParallelArcBetweenTwoLongitudes(). */
void test_GEODESY_ComputeParallelArcBetweenTwoLongitudes(void);

/** \brief  Test GEODESY_RotateVectorFromLocalGeodeticFrameToEarthFixedFrame(). */
void test_GEODESY_RotateVectorFromLocalGeodeticFrameToEarthFixedFrame(void);

/** \brief  Test GEODESY_RotateVectorFromEarthFixedFrameToLocalGeodeticFrame(). */
void test_GEODESY_RotateVectorFromEarthFixedFrameToLocalGeodeticFrame(void);

/** \brief  Test GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame(). */
void test_GEODESY_ComputeAzimuthAndElevationAnglesBetweenToPointsInTheEarthFixedFrame(void);

#ifdef __cplusplus
}
#endif

#endif // _C_TEST_GEODESY_H_
