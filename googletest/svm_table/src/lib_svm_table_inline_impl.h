/**
 * @file
 * @brief SVM table support library functions
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
 * @brief
 * @details
 * @{
 */

#ifndef LIB_SVM_TABLE_INLINE_IMPL_H
#define LIB_SVM_TABLE_INLINE_IMPL_H

STATIC INLINE svm_vector_t SVM_Table_getDirectValue(uint16_t index,
                                                    uint16_t o1,
                                                    uint16_t o2,
                                                    uint16_t o3,
                                                    uint16_t o4)
{
    /* Declare SVM (Space Vector Modulation) table */
    #if defined(HAS_192PTS_SVM_LH_TABLE) || defined(HAS_384PTS_SVM_LH_TABLE)
    extern const uint16_t SpaceVectorModulation[];

    /* Set a pointer to the constant SVM table */
    uint16_t const* p = &SpaceVectorModulation[index];
    #else
    extern const int16_t SpaceVectorModulation[];

    /* Set a pointer to the constant SVM table */
    int16_t const* p = &SpaceVectorModulation[index];
    #endif

    /* Fill the vector with values from the SVM table */
    svm_vector_t vector = {
        .a = p[o1],
        .b = p[o2],
        .c = p[o3],
        .d = p[o4]
    };

    return vector;
}

#endif /* LIB_MOTOR_TABLE_INLINE_IMPL_H */
/*/ @} */
