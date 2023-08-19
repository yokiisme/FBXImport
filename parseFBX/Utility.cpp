#include "Utility.h"

Matrix4x4f::Matrix4x4f(const Matrix3x3f& other)
{
    m_Data[0] = other.m_Data[0];
    m_Data[1] = other.m_Data[1];
    m_Data[2] = other.m_Data[2];
    m_Data[3] = 0.0F;

    m_Data[4] = other.m_Data[3];
    m_Data[5] = other.m_Data[4];
    m_Data[6] = other.m_Data[5];
    m_Data[7] = 0.0F;

    m_Data[8] = other.m_Data[6];
    m_Data[9] = other.m_Data[7];
    m_Data[10] = other.m_Data[8];
    m_Data[11] = 0.0F;

    m_Data[12] = 0.0F;
    m_Data[13] = 0.0F;
    m_Data[14] = 0.0F;
    m_Data[15] = 1.0F;
}

Matrix3x3f& Matrix3x3f::operator=(const Matrix4x4f& other)
{
    m_Data[0] = other.m_Data[0];
    m_Data[1] = other.m_Data[1];
    m_Data[2] = other.m_Data[2];

    m_Data[3] = other.m_Data[4];
    m_Data[4] = other.m_Data[5];
    m_Data[5] = other.m_Data[6];

    m_Data[6] = other.m_Data[8];
    m_Data[7] = other.m_Data[9];
    m_Data[8] = other.m_Data[10];
    return *this;
}

Matrix3x3f& Matrix3x3f::operator*=(const Matrix4x4f& inM)
{
    int i;
    for (i = 0; i < 3; i++)
    {
        float v[3] = { Get(i, 0), Get(i, 1), Get(i, 2) };
        Get(i, 0) = v[0] * inM.Get(0, 0) + v[1] * inM.Get(1, 0) + v[2] * inM.Get(2, 0);
        Get(i, 1) = v[0] * inM.Get(0, 1) + v[1] * inM.Get(1, 1) + v[2] * inM.Get(2, 1);
        Get(i, 2) = v[0] * inM.Get(0, 2) + v[1] * inM.Get(1, 2) + v[2] * inM.Get(2, 2);
    }
    return *this;
}

inline void PrefetchRead(const void* p)
{
    _mm_prefetch((const char*)p, _MM_HINT_T0);
}
inline void Prefetch(const void* p) { PrefetchRead(p); }

inline void MultiplyMatrices4x4NATIVE(const Simd128& m10, const Simd128& m11, const Simd128& m12, const Simd128& m13, const Simd128& m20, const Simd128& m21, const Simd128& m22, const Simd128& m23, Simd128& rm0, Simd128& rm1, Simd128& rm2, Simd128& rm3)
{
    const Simd128 m20_X = V4Splat(m20, 0);
    const Simd128 m21_X = V4Splat(m21, 0);
    const Simd128 m22_X = V4Splat(m22, 0);
    const Simd128 m23_X = V4Splat(m23, 0);
    const Simd128 rm0_0 = V4Mul(m20_X, m10);
    const Simd128 rm1_0 = V4Mul(m21_X, m10);
    const Simd128 rm2_0 = V4Mul(m22_X, m10);
    const Simd128 rm3_0 = V4Mul(m23_X, m10);
    const Simd128 m20_Y = V4Splat(m20, 1);
    const Simd128 m21_Y = V4Splat(m21, 1);
    const Simd128 m22_Y = V4Splat(m22, 1);
    const Simd128 m23_Y = V4Splat(m23, 1);
    const Simd128 rm0_1 = V4MulAdd(m20_Y, m11, rm0_0);
    const Simd128 rm1_1 = V4MulAdd(m21_Y, m11, rm1_0);
    const Simd128 rm2_1 = V4MulAdd(m22_Y, m11, rm2_0);
    const Simd128 rm3_1 = V4MulAdd(m23_Y, m11, rm3_0);
    const Simd128 m20_Z = V4Splat(m20, 2);
    const Simd128 m21_Z = V4Splat(m21, 2);
    const Simd128 m22_Z = V4Splat(m22, 2);
    const Simd128 m23_Z = V4Splat(m23, 2);
    const Simd128 rm0_2 = V4MulAdd(m20_Z, m12, rm0_1);
    const Simd128 rm1_2 = V4MulAdd(m21_Z, m12, rm1_1);
    const Simd128 rm2_2 = V4MulAdd(m22_Z, m12, rm2_1);
    const Simd128 rm3_2 = V4MulAdd(m23_Z, m12, rm3_1);
    const Simd128 m20_W = V4Splat(m20, 3);
    const Simd128 m21_W = V4Splat(m21, 3);
    const Simd128 m22_W = V4Splat(m22, 3);
    const Simd128 m23_W = V4Splat(m23, 3);
    rm0 = V4MulAdd(m20_W, m13, rm0_2);
    rm1 = V4MulAdd(m21_W, m13, rm1_2);
    rm2 = V4MulAdd(m22_W, m13, rm2_2);
    rm3 = V4MulAdd(m23_W, m13, rm3_2);
}


inline void MultiplyMatrices4x4(const Matrix4x4f* __restrict lhs, const Matrix4x4f* __restrict rhs, Matrix4x4f* __restrict res)
{
    float* m = res->m_Data;
    const float* m1 = lhs->m_Data;
    const float* m2 = rhs->m_Data;
    Simd128 rm0, rm1, rm2, rm3;

    Prefetch((const char*)m1);
    Prefetch((const char*)m2);

    const Simd128 m10 = V4LoadUnaligned(m1, 0x0);
    const Simd128 m11 = V4LoadUnaligned(m1, 0x4);
    const Simd128 m12 = V4LoadUnaligned(m1, 0x8);
    const Simd128 m13 = V4LoadUnaligned(m1, 0xC);

    const Simd128 m20 = V4LoadUnaligned(m2, 0x0);
    const Simd128 m21 = V4LoadUnaligned(m2, 0x4);
    const Simd128 m22 = V4LoadUnaligned(m2, 0x8);
    const Simd128 m23 = V4LoadUnaligned(m2, 0xC);

    MultiplyMatrices4x4NATIVE(m10, m11, m12, m13, m20, m21, m22, m23, rm0, rm1, rm2, rm3);

    V4StoreUnaligned(rm0, m, 0x0);
    V4StoreUnaligned(rm1, m, 0x4);
    V4StoreUnaligned(rm2, m, 0x8);
    V4StoreUnaligned(rm3, m, 0xC);
}

Matrix4x4f& Matrix4x4f::operator*=(const Matrix4x4f& inM1)
{
    Matrix4x4f tmp;
    MultiplyMatrices4x4(this, &inM1, &tmp);
    *this = tmp;
    return *this;
}


bool Matrix3x3f::Invert()
{
    ///@TODO make a fast but robust inverse matrix 3x3
    Matrix4x4f m = *this;
    bool success = InvertMatrix4x4_Full(m.GetPtr(), m.GetPtr());
    *this = m;
    return success;
}

Matrix4x4f& Matrix4x4f::operator=(const Matrix3x3f& other)
{
    m_Data[0] = other.m_Data[0];
    m_Data[1] = other.m_Data[1];
    m_Data[2] = other.m_Data[2];
    m_Data[3] = 0.0F;

    m_Data[4] = other.m_Data[3];
    m_Data[5] = other.m_Data[4];
    m_Data[6] = other.m_Data[5];
    m_Data[7] = 0.0F;

    m_Data[8] = other.m_Data[6];
    m_Data[9] = other.m_Data[7];
    m_Data[10] = other.m_Data[8];
    m_Data[11] = 0.0F;

    m_Data[12] = 0.0F;
    m_Data[13] = 0.0F;
    m_Data[14] = 0.0F;
    m_Data[15] = 1.0F;
    return *this;
}

inline bool InvertMatrix4x4_Full(const float* m, float* out)
{
	float wtmp[4][8];
	float m0, m1, m2, m3, s;
	float* r0, * r1, * r2, * r3;

	r0 = wtmp[0], r1 = wtmp[1], r2 = wtmp[2], r3 = wtmp[3];

	r0[0] = MAT(m, 0, 0); r0[1] = MAT(m, 0, 1);
	r0[2] = MAT(m, 0, 2); r0[3] = MAT(m, 0, 3);
	r0[4] = 1.0; r0[5] = r0[6] = r0[7] = 0.0;

	r1[0] = MAT(m, 1, 0); r1[1] = MAT(m, 1, 1);
	r1[2] = MAT(m, 1, 2); r1[3] = MAT(m, 1, 3);
	r1[5] = 1.0; r1[4] = r1[6] = r1[7] = 0.0;

	r2[0] = MAT(m, 2, 0); r2[1] = MAT(m, 2, 1);
	r2[2] = MAT(m, 2, 2); r2[3] = MAT(m, 2, 3);
	r2[6] = 1.0; r2[4] = r2[5] = r2[7] = 0.0;

	r3[0] = MAT(m, 3, 0); r3[1] = MAT(m, 3, 1);
	r3[2] = MAT(m, 3, 2); r3[3] = MAT(m, 3, 3);
	r3[7] = 1.0; r3[4] = r3[5] = r3[6] = 0.0;

	/* choose pivot - or die */
	if (Abs(r3[0]) > Abs(r2[0]))
		SWAP_ROWS(r3, r2);
	if (Abs(r2[0]) > Abs(r1[0]))
		SWAP_ROWS(r2, r1);
	if (Abs(r1[0]) > Abs(r0[0]))
		SWAP_ROWS(r1, r0);
	if (0.0F == r0[0])
		RETURN_ZERO;

	/* eliminate first variable     */
	m1 = r1[0] / r0[0]; m2 = r2[0] / r0[0]; m3 = r3[0] / r0[0];
	s = r0[1]; r1[1] -= m1 * s; r2[1] -= m2 * s; r3[1] -= m3 * s;
	s = r0[2]; r1[2] -= m1 * s; r2[2] -= m2 * s; r3[2] -= m3 * s;
	s = r0[3]; r1[3] -= m1 * s; r2[3] -= m2 * s; r3[3] -= m3 * s;
	s = r0[4];
	if (s != 0.0F)
	{
		r1[4] -= m1 * s; r2[4] -= m2 * s; r3[4] -= m3 * s;
	}
	s = r0[5];
	if (s != 0.0F)
	{
		r1[5] -= m1 * s; r2[5] -= m2 * s; r3[5] -= m3 * s;
	}
	s = r0[6];
	if (s != 0.0F)
	{
		r1[6] -= m1 * s; r2[6] -= m2 * s; r3[6] -= m3 * s;
	}
	s = r0[7];
	if (s != 0.0F)
	{
		r1[7] -= m1 * s; r2[7] -= m2 * s; r3[7] -= m3 * s;
	}

	/* choose pivot - or die */
	if (Abs(r3[1]) > Abs(r2[1]))
		SWAP_ROWS(r3, r2);
	if (Abs(r2[1]) > Abs(r1[1]))
		SWAP_ROWS(r2, r1);
	if (0.0F == r1[1])
		RETURN_ZERO;

	/* eliminate second variable */
	m2 = r2[1] / r1[1]; m3 = r3[1] / r1[1];
	r2[2] -= m2 * r1[2]; r3[2] -= m3 * r1[2];
	r2[3] -= m2 * r1[3]; r3[3] -= m3 * r1[3];
	s = r1[4]; if (0.0F != s)
	{
		r2[4] -= m2 * s; r3[4] -= m3 * s;
	}
	s = r1[5]; if (0.0F != s)
	{
		r2[5] -= m2 * s; r3[5] -= m3 * s;
	}
	s = r1[6]; if (0.0F != s)
	{
		r2[6] -= m2 * s; r3[6] -= m3 * s;
	}
	s = r1[7]; if (0.0F != s)
	{
		r2[7] -= m2 * s; r3[7] -= m3 * s;
	}

	/* choose pivot - or die */
	if (Abs(r3[2]) > Abs(r2[2]))
		SWAP_ROWS(r3, r2);
	if (0.0F == r2[2])
		RETURN_ZERO;

	/* eliminate third variable */
	m3 = r3[2] / r2[2];
	r3[3] -= m3 * r2[3]; r3[4] -= m3 * r2[4];
	r3[5] -= m3 * r2[5]; r3[6] -= m3 * r2[6];
	r3[7] -= m3 * r2[7];

	/* last check */
	if (0.0F == r3[3])
		RETURN_ZERO;

	s = 1.0F / r3[3];          /* now back substitute row 3 */
	r3[4] *= s; r3[5] *= s; r3[6] *= s; r3[7] *= s;

	m2 = r2[3];                /* now back substitute row 2 */
	s = 1.0F / r2[2];
	r2[4] = s * (r2[4] - r3[4] * m2), r2[5] = s * (r2[5] - r3[5] * m2),
		r2[6] = s * (r2[6] - r3[6] * m2), r2[7] = s * (r2[7] - r3[7] * m2);
	m1 = r1[3];
	r1[4] -= r3[4] * m1; r1[5] -= r3[5] * m1,
		r1[6] -= r3[6] * m1; r1[7] -= r3[7] * m1;
	m0 = r0[3];
	r0[4] -= r3[4] * m0; r0[5] -= r3[5] * m0,
		r0[6] -= r3[6] * m0; r0[7] -= r3[7] * m0;

	m1 = r1[2];                /* now back substitute row 1 */
	s = 1.0F / r1[1];
	r1[4] = s * (r1[4] - r2[4] * m1); r1[5] = s * (r1[5] - r2[5] * m1),
		r1[6] = s * (r1[6] - r2[6] * m1); r1[7] = s * (r1[7] - r2[7] * m1);
	m0 = r0[2];
	r0[4] -= r2[4] * m0; r0[5] -= r2[5] * m0,
		r0[6] -= r2[6] * m0; r0[7] -= r2[7] * m0;

	m0 = r0[1];                /* now back substitute row 0 */
	s = 1.0F / r0[0];
	r0[4] = s * (r0[4] - r1[4] * m0); r0[5] = s * (r0[5] - r1[5] * m0),
		r0[6] = s * (r0[6] - r1[6] * m0); r0[7] = s * (r0[7] - r1[7] * m0);

	MAT(out, 0, 0) = r0[4]; MAT(out, 0, 1) = r0[5], MAT(out, 0, 2) = r0[6]; MAT(out, 0, 3) = r0[7];
	MAT(out, 1, 0) = r1[4]; MAT(out, 1, 1) = r1[5], MAT(out, 1, 2) = r1[6]; MAT(out, 1, 3) = r1[7];
	MAT(out, 2, 0) = r2[4]; MAT(out, 2, 1) = r2[5], MAT(out, 2, 2) = r2[6]; MAT(out, 2, 3) = r2[7];
	MAT(out, 3, 0) = r3[4]; MAT(out, 3, 1) = r3[5], MAT(out, 3, 2) = r3[6]; MAT(out, 3, 3) = r3[7];

	return true;
}
