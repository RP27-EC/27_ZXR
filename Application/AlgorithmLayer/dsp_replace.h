/**
  ******************************************************************************
  * @file    dsp_replace.h
  * @brief   用标准 C 库 float 数学替换 CMSIS-DSP 的 FastMath / QuaternionMath
  *
  * 目的：去掉 commontables.o 里约 580KB(下板 H723) / 约 899KB(上板 F407) 的
  *       FFT 旋转因子表 + sin 查表。这些表被 arm_sin_f32 / arm_cos_f32 引用，
  *       导致整个段被强制链接进 Flash，大幅拖慢 Keil 烧录。
  *
  * 做法：把工程实际用到的 4 个函数 + 四元数归一化重定义为标准库实现
  *       (sinf/cosf/sqrtf/atan2f，硬件 FPU 直算、零查表)。
  *       链接器会自动丢弃未被引用的 CMSIS 对象（含 commontables.o / fastmathfunctions.o）。
  *
  * 注意：
  *   - Matrix(arm_mat_*) 仍走 CMSIS，保留不动。
  *   - 下板须关闭 MicroLIB(Options->Target 取消 Use MicroLIB)，否则 sinf/cosf 链接不到。
  *   - 本头文件先 include "arm_math.h" 再重定义，因此放置顺序不影响正确性。
  ******************************************************************************
  */
#ifndef DSP_REPLACE_H
#define DSP_REPLACE_H

#include "arm_math.h"
#include <math.h>
#include <stdint.h>

/* 若 CMSIS 已用宏定义同名符号，先取消，避免重定义告警 */
#ifdef arm_sin_f32
#undef arm_sin_f32
#endif
#ifdef arm_cos_f32
#undef arm_cos_f32
#endif
#ifdef arm_sqrt_f32
#undef arm_sqrt_f32
#endif
#ifdef arm_atan2_f32
#undef arm_atan2_f32
#endif
#ifdef arm_quaternion_normalize_f32
#undef arm_quaternion_normalize_f32
#endif

/* FastMath：直接映射到标准库（无查表，不再依赖 commontables） */
#define arm_sin_f32(x)             sinf(x)
#define arm_cos_f32(x)             cosf(x)
#define arm_sqrt_f32(in, pOut)     (*(pOut) = sqrtf(in))
#define arm_atan2_f32(y, x, pOut)  (*(pOut) = atan2f((y), (x)))

/* 四元数归一化：用 sqrtf 实现，独立于 QuaternionMath 组件（支持原地归一化） */
#define arm_quaternion_normalize_f32(pIn, pOut, n)                                \
    do {                                                                           \
        const float *_q = (const float *)(pIn);                                     \
        float       *_o = (float *)(pOut);                                          \
        uint32_t     _n = (uint32_t)(n);                                            \
        for (uint32_t _i = 0; _i < _n; _i++) {                                      \
            const float *_a = _q + 4 * _i;                                          \
            float       *_b = _o + 4 * _i;                                          \
            float _s = _a[0] * _a[0] + _a[1] * _a[1] + _a[2] * _a[2] + _a[3] * _a[3];\
            float _r = (_s > 1e-8f) ? (1.0f / sqrtf(_s)) : 0.0f;                    \
            _b[0] = _a[0] * _r;                                                     \
            _b[1] = _a[1] * _r;                                                     \
            _b[2] = _a[2] * _r;                                                     \
            _b[3] = _a[3] * _r;                                                     \
        }                                                                            \
    } while (0)

#endif /* DSP_REPLACE_H */
