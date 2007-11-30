/** 
\file    test_time_conversion.h
\brief   unit tests for time_conversion.c/.h
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
#ifndef _C_TEST_TIMECONV_H_
#define _C_TEST_TIMECONV_H_

#ifdef __cplusplus
extern "C" {
#endif


/** 
\brief  The suite initialization function.
\return Returns zero on success, non-zero otherwise.
*/
int init_suite_TIMECONV(void);

/** 
\brief  The suite cleanup function.
\return Returns zero on success, non-zero otherwise.
*/
int clean_suite_TIMECONV(void);


/** \brief  Test TIMECONV_GetJulianDateFromGPSTime(). */
void test_TIMECONV_GetJulianDateFromGPSTime(void);

/** \brief  Test TIMECONV_GetJulianDateFromUTCTime(). */
void test_TIMECONV_GetJulianDateFromUTCTime(void);

/** \brief  Test TIMECONV_GetGPSTimeFromJulianDate(). */
void test_TIMECONV_GetGPSTimeFromJulianDate(void);

/** \brief  Test TIMECONV_GetUTCTimeFromJulianDate(). */
void test_TIMECONV_GetUTCTimeFromJulianDate(void);

/** \brief  Test TIMECONV_GetGPSTimeFromUTCTime(). */
void test_TIMECONV_GetGPSTimeFromUTCTime(void);

/** \brief  Test TIMECONV_GetUTCTimeFromGPSTime(). */
void test_TIMECONV_GetUTCTimeFromGPSTime(void);

/** \brief  Test TIMECONV_DetermineUTCOffset(). */
void test_TIMECONV_DetermineUTCOffset(void);

/** \brief  Test TIMECONV_GetNumberOfDaysInMonth(). */
void test_TIMECONV_GetNumberOfDaysInMonth(void);

/** \brief  Test TIMECONV_IsALeapYear(). */
void test_TIMECONV_IsALeapYear(void);

/** \brief  Test TIMECONV_GetDayOfYear(). */
void test_TIMECONV_GetDayOfYear(void);

#ifdef __cplusplus
}
#endif

#endif // _C_TEST_TIMECONV_H_
