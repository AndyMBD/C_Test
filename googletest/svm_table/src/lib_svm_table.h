/**
 * @file
 * @brief API for the svm table library
 * @internal
 *
 * @copyright (C) 2021 Melexis N.V.
 *
 * Melexis N.V. is supplying this code for use with Melexis N.V. processor based microcontrollers only.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 * INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.  MELEXIS N.V. SHALL NOT IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * @endinternal
 *
 * @ingroup libraries
 *
 * @brief Public API for the svm table library
 * @details
 * @{
 */
#ifndef SVM_TABLE_H
#define SVM_TABLE_H

#include <stdint.h>
#include "compiler_abstraction.h"

/** Number of steps for one electrical rotation : a complete sine */
#if defined(HAS_192PTS_SVM_M_TABLE) || defined(HAS_192PTS_SVM_LH_TABLE)
#define MOTOR_SVM_PERIOD (192u)
#define MOTOR_SVM_SIZE (320u)
#else
#define MOTOR_SVM_PERIOD (384u)
#define MOTOR_SVM_SIZE (640u)
#endif

#if defined(HAS_192PTS_SVM_LH_TABLE) || defined(HAS_384PTS_SVM_LH_TABLE)
typedef struct
{
    uint16_t a;
    uint16_t b;
    uint16_t c;
    uint16_t d;
} svm_vector_t;
#else
typedef struct
{
    int16_t a;
    int16_t b;
    int16_t c;
    int16_t d;
} svm_vector_t;
#endif

/** Getter for the Space Vector Modulation table
 *
 * @note No modulo-operation is applied on the given index. The given index needs to be
 * below #MOTOR_SVM_SIZE
 * @param index Index in the space vector modulation table
 * @return Value for space vector modulation at given index.
 */
#if defined(HAS_192PTS_SVM_LH_TABLE) || defined(HAS_384PTS_SVM_LH_TABLE)
uint16_t SVM_Table_getValue(uint16_t index);
#else
int16_t SVM_Table_getValue(uint16_t index);
#endif

/** @brief Getter for the Space Vector Modulation table
 * @note it is possible to use this function to get 1 to 4 samples from the table
 * Because this function is inlined no time/space is lost by not using all returned values
 *
 * Some usage examples:
 *
 *   - Get 4 samples:
 * @code
 *     svm_vector_t v = SVM_Table_getDirectValue(index, 0, 48, 96, 144);
 *
 *     test1 = v.a;
 *     test2 = v.b;
 *     test3 = v.c;
 *     test3 = v.d;
 * @endcode
 *
 *   - Get 3 samples:
 * @code
 *     svm_vector_t v = SVM_Table_getDirectValue(index, 0, 48, 96, 144);
 *
 *     test1 = v.a;
 *     test2 = v.b;
 *     test3 = v.c;
 * @endcode
 *
 *   - Get only 1 sample:
 * @code
 *     svm_vector_t v = SVM_Table_getDirectValue(index, 0, 0, 0, 0);
 *     test1 = v.a;
 * @endcode
 *
 * @note No modulo-operation is applied on the given index. The given index needs to be
 * below #MOTOR_SVM_SIZE
 *
 * @param[in] index Index in the space vector modulation table
 * @param[in] o1, o2, o3, o4 index offsets of the desired samples
 *
 * @return The space vector modulations samples at given index and offsets.
 */
STATIC INLINE svm_vector_t SVM_Table_getDirectValue(uint16_t index,
                                                    uint16_t o1,
                                                    uint16_t o2,
                                                    uint16_t o3,
                                                    uint16_t o4);

#ifndef UNITTEST
#include "lib_svm_table_inline_impl.h"
#endif /* UNITTEST */

#endif
/*/ @} */

