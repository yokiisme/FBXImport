#pragma once
#include "floatConversion.h"
#include "stringFormat.h"
struct Vector4f
{
public:
	float x, y, z, w;
	Vector4f() {}
	Vector4f(float inx, float iny, float inz, float inw) { x = inx; y = iny; z = inz; w = inw; }
	void Set(float inX, float inY, float inZ, float inW) { x = inX; y = inY; z = inZ; w = inW; }
};

struct Vector3f
{
public:
	float x, y, z;
	Vector3f() {}
	Vector3f(float u, float v, float w) { x = u; y = v; z = w; }
	float& operator[](int i) //应该有一个Assert
	{
		return (&x)[i];
	}
	const float& operator[](int i) const { return (&x)[i]; }
	bool operator==(const Vector3f& v) const { return x == v.x && y == v.y && z == v.z; }
	bool operator!=(const Vector3f& v) const { return x != v.x || y != v.y || z != v.z; }

	Vector3f& operator+=(const Vector3f& inV) { x += inV.x; y += inV.y; z += inV.z; return *this; }
	Vector3f& operator-=(const Vector3f& inV) { x -= inV.x; y -= inV.y; z -= inV.z; return *this; }
	Vector3f& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
	Vector3f& operator/=(float s);

	Vector3f operator-() const { return Vector3f(-x, -y, -z); }

	Vector3f& Scale(const Vector3f& inV) { x *= inV.x; y *= inV.y; z *= inV.z; return *this; }


	static const float    epsilon;
	static const float    infinity;
	static const Vector3f infinityVec;
	static const Vector3f zero;
	static const Vector3f one;
	static const Vector3f xAxis;
	static const Vector3f yAxis;
	static const Vector3f zAxis;

};

inline Vector3f Scale(const Vector3f& lhs, const Vector3f& rhs) { return Vector3f(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z); }

inline Vector3f operator+(const Vector3f& lhs, const Vector3f& rhs) { return Vector3f(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z); }
inline Vector3f operator-(const Vector3f& lhs, const Vector3f& rhs) { return Vector3f(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z); }
inline Vector3f Cross(const Vector3f& lhs, const Vector3f& rhs);
inline float Dot(const Vector3f& lhs, const Vector3f& rhs) { return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z; }
inline float Volume(const Vector3f& inV) { return inV.x * inV.y * inV.z; }

inline Vector3f operator*(const Vector3f& inV, const float s) { return Vector3f(inV.x * s, inV.y * s, inV.z * s); }
inline Vector3f operator*(const float s, const Vector3f& inV) { return Vector3f(inV.x * s, inV.y * s, inV.z * s); }
inline Vector3f operator*(const Vector3f& lhs, const Vector3f& rhs) { return Vector3f(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z); }
inline Vector3f operator/(const Vector3f& inV, const float s) { Vector3f temp(inV); temp /= s; return temp; }
inline Vector3f Inverse(const Vector3f& inVec) { return Vector3f(1.0F / inVec.x, 1.0F / inVec.y, 1.0F / inVec.z); }
inline float SqrMagnitude(const Vector3f& inV) { return Dot(inV, inV); }
inline float Magnitude(const Vector3f& inV) { return SqrtImpl(Dot(inV, inV)); }

inline static Vector3f NormalizeRobustInternal(const Vector3f& a, float& l, float& div, float eps)
{
	float a0, a1, a2, aa0, aa1, aa2;
	a0 = a.x;
	a1 = a.y;
	a2 = a.z;

	aa0 = Abs(a0);
	aa1 = Abs(a1);
	aa2 = Abs(a2);

	if (aa1 > aa0)
	{
		if (aa2 > aa1)
		{
			a0 /= aa2;
			a1 /= aa2;
			l = InvSqrt(a0 * a0 + a1 * a1 + 1.0F);
			div = aa2;
			return Vector3f(a0 * l, a1 * l, CopySignf(l, a2));
		}
		else
		{
			// aa1 is largest
			a0 /= aa1;
			a2 /= aa1;
			l = InvSqrt(a0 * a0 + a2 * a2 + 1.0F);
			div = aa1;
			return Vector3f(a0 * l, CopySignf(l, a1), a2 * l);
		}
	}
	else
	{
		if (aa2 > aa0)
		{
			// aa2 is largest
			a0 /= aa2;
			a1 /= aa2;
			l = InvSqrt(a0 * a0 + a1 * a1 + 1.0F);
			div = aa2;
			return Vector3f(a0 * l, a1 * l, CopySignf(l, a2));
		}
		else
		{
			// aa0 is largest
			if (aa0 <= 0)
			{
				l = 0;
				div = 1;
				return Vector3f(0.0F, 1.0F, 0.0F);
			}

			a1 /= aa0;
			a2 /= aa0;
			l = InvSqrt(a1 * a1 + a2 * a2 + 1.0F);
			div = aa0;
			return Vector3f(CopySignf(l, a0), a1 * l, a2 * l);
		}
	}
}

inline static Vector3f NormalizeRobust(const Vector3f& a)
{
	float l, div;
	return NormalizeRobustInternal(a, l, div, 0.00001f);
}

// Normalizes a vector, returns default vector if it can't be normalized
inline Vector3f NormalizeSafe(const Vector3f& inV, const Vector3f& defaultV = Vector3f::zero);

inline Vector3f NormalizeSafe(const Vector3f& inV, const Vector3f& defaultV)
{
	float mag = Magnitude(inV);
	if (mag > Vector3f::epsilon)
		return inV / mag;
	else
		return defaultV;
}


inline Vector3f Cross(const Vector3f& lhs, const Vector3f& rhs)
{
	return Vector3f(
		lhs.y * rhs.z - lhs.z * rhs.y,
		lhs.z * rhs.x - lhs.x * rhs.z,
		lhs.x * rhs.y - lhs.y * rhs.x);
}

inline Vector3f& Vector3f::operator/=(float s)
{
	if(CompareApproximately(s, 0.0F))
		ReportError("The factor is 0.0F!");
	x /= s;
	y /= s;
	z /= s;
	return *this;
}

struct Vector2f
{
public:
	float x, y;
	Vector2f() {}
	Vector2f(float u, float v) { x = u; y = v; }
	Vector2f& operator-=(const Vector2f& inV) { x -= inV.x; y -= inV.y; return *this; }
};


inline Vector2f operator-(const Vector2f& lhs, const Vector2f& rhs) { return Vector2f(lhs.x - rhs.x, lhs.y - rhs.y); }
inline float Dot(const Vector2f& lhs, const Vector2f& rhs) { return lhs.x * rhs.x + lhs.y * rhs.y; }



