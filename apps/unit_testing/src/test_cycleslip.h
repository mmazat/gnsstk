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

#ifndef _C_TEST_CYCLESLIP_H
#define _C_TEST_CYCLESLIP_H

#ifdef __cplusplus
extern "C" {
#endif

/** 
\brief  The suite initialization function.
\return Returns zero on success, non-zero otherwise.
*/
int init_suite_CYCLESLIP(void);

/** 
\brief  The suite cleanup function.
\return Returns zero on success, non-zero otherwise.
*/
int clean_suite_CYCLESLIP(void);

/** \brief Test CYCLESLIP_CheckForCycleSlipUsingPhaseRatePrediction(). */
void test_CYCLESLIP_CheckForCycleSlipUsingPhaseRatePrediction(void);

/** \brief Test CYCLESLIP_CheckForCycleSlipUsingDualFrequencyPhase(). */
void test_CYCLESLIP_CheckForCycleSlipUsingDualFrequencyPhase(void);

/** \brief test_CYCLESLIP_CheckForCycleSlipUsingTripleDifferencePhase(). */
void test_CYCLESLIP_CheckForCycleSlipUsingTripleDifferencePhase(void);

#ifdef __cplusplus
}
#endif

#endif //_C_TEST_CYCLESLIP_H
