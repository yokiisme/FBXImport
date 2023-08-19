#pragma once
#include "RotationOrder.h"
#include "Conversion.h"
#include "math.h"
#include "Utility.h"
#include "floatConversion.h"
struct Quaternionf
{
public:
	float x, y, z, w;
	friend float Magnitude(const Quaternionf& q);
	friend Quaternionf Normalize(const Quaternionf& q) { return Quaternionf(q.x / Magnitude(q), q.y / Magnitude(q), q.z / Magnitude(q), q.w / Magnitude(q)); }
	Quaternionf() {}
	Quaternionf(float inx, float iny, float inz, float inw) { x = inx; y = iny; z = inz; w = inw; }

	void Set(float inX, float inY, float inZ, float inW)
	{
		x = inX;
		y = inY;
		z = inZ;
		w = inW;
	}
	void Set(const Quaternionf& aQuat)
	{
		x = aQuat.x;
		y = aQuat.y;
		z = aQuat.z;
		w = aQuat.w;
	}
	void Set(const float* array) { x = array[0]; y = array[1]; z = array[2]; w = array[3]; }

	bool operator==(const Quaternionf& q) const { return x == q.x && y == q.y && z == q.z && w == q.w; }
	bool operator!=(const Quaternionf& q) const { return x != q.x || y != q.y || z != q.z || w != q.w; }

	friend Quaternionf operator+(const Quaternionf& lhs, const Quaternionf& rhs)
	{
		Quaternionf q(lhs);
		return q += rhs;
	}

	friend Quaternionf  operator-(const Quaternionf& lhs, const Quaternionf& rhs)
	{
		Quaternionf t(lhs);
		return t -= rhs;
	}

	Quaternionf operator-() const
	{
		return Quaternionf(-x, -y, -z, -w);
	}

	Quaternionf operator*(const float s) const
	{
		return Quaternionf(x * s, y * s, z * s, w * s);
	}

	friend Quaternionf  operator*(const float s, const Quaternionf& q)
	{
		Quaternionf t(q);
		return t *= s;
	}

	friend Quaternionf  operator/(const Quaternionf& q, const float s)
	{
		Quaternionf t(q);
		return t /= s;
	}

	inline friend Quaternionf operator*(const Quaternionf& lhs, const Quaternionf& rhs)
	{
		return Quaternionf(
			lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
			lhs.w * rhs.y + lhs.y * rhs.w + lhs.z * rhs.x - lhs.x * rhs.z,
			lhs.w * rhs.z + lhs.z * rhs.w + lhs.x * rhs.y - lhs.y * rhs.x,
			lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z);
	}

	static Quaternionf identity() { return Quaternionf(0.0F, 0.0F, 0.0F, 1.0F); }

	const float* GetPtr() const { return &x; }
	float* GetPtr() { return &x; }

	const float& operator[](int i) const { return GetPtr()[i]; }
	float& operator[](int i) { return GetPtr()[i]; }

	inline Quaternionf& operator+=(const Quaternionf& aQuat)
	{
		x += aQuat.x;
		y += aQuat.y;
		z += aQuat.z;
		w += aQuat.w;
		return *this;
	}
	inline Quaternionf& operator-=(const Quaternionf& aQuat)
	{
		x -= aQuat.x;
		y -= aQuat.y;
		z -= aQuat.z;
		w -= aQuat.w;
		return *this;
	}
	inline Quaternionf& operator*=(float aScalar)
	{
		x *= aScalar;
		y *= aScalar;
		z *= aScalar;
		w *= aScalar;
		return *this;
	}
	inline Quaternionf& operator/=(const float aScalar)
	{
		if (CompareApproximately(aScalar, 0.0F))
			ReportError("The Scaler is 0.0F!");
		x /= aScalar;
		y /= aScalar;
		z /= aScalar;
		w /= aScalar;
		return *this;
	}
	inline Quaternionf& operator*=(const Quaternionf& rhs)
	{
		float tempx = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
		float tempy = w * rhs.y + y * rhs.w + z * rhs.x - x * rhs.z;
		float tempz = w * rhs.z + z * rhs.w + x * rhs.y - y * rhs.x;
		float tempw = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
		x = tempx; y = tempy; z = tempz; w = tempw;
		return *this;
	}
};

inline float Dot(const Quaternionf& q1, const Quaternionf& q2)
{
	return (q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w);
}

inline float SqrMagnitude(const Quaternionf& q)
{
	return Dot(q, q);
}

inline float Magnitude(const Quaternionf& q)
{
	return SqrtImpl(SqrMagnitude(q));
}

namespace
{
	//Indexes for values used to calculate euler angles
	enum Indexes
	{
		X1,
		X2,
		Y1,
		Y2,
		Z1,
		Z2,
		singularity_test,
		IndexesCount
	};

	//indexes for pre-multiplied quaternion values
	enum QuatIndexes
	{
		xx,
		xy,
		xz,
		xw,
		yy,
		yz,
		yw,
		zz,
		zw,
		ww,
		QuatIndexesCount
	};

	float qAsin(float a, float b)
	{
		return a * asin(clamp(b, -1.0f, 1.0f));
	}

	float qAtan2(float a, float b)
	{
		return atan2(a, b);
	}

	float qNull(float a, float b)
	{
		return 0;
	}

	typedef float (*qFunc)(float, float);

	qFunc qFuncs[kRotationOrderCount][3] =
	{
		{&qAtan2, &qAsin, &qAtan2},     //OrderXYZ
		{&qAtan2, &qAtan2, &qAsin},     //OrderXZY
		{&qAtan2, &qAtan2, &qAsin},     //OrderYZX,
		{&qAsin, &qAtan2, &qAtan2},     //OrderYXZ,
		{&qAsin, &qAtan2, &qAtan2},     //OrderZXY,
		{&qAtan2, &qAsin, &qAtan2},     //OrderZYX,
	};
}



inline Vector3f QuaternionToEuler(const Quaternionf& q, RotationOrder order = kOrderUnityDefault);
inline Quaternionf CreateQuaternionFromAxisQuaternions(const Quaternionf& q1, const Quaternionf& q2, const Quaternionf& q3, Quaternionf& result);
inline Quaternionf EulerToQuaternion(const Vector3f& euler, RotationOrder order = kOrderUnityDefault);


inline Quaternionf EulerToQuaternion(const Vector3f& someEulerAngles, RotationOrder order /*= kOrderUnityDefault*/)
{
	float cX(cos(someEulerAngles.x / 2.0f));
	float sX(sin(someEulerAngles.x / 2.0f));

	float cY(cos(someEulerAngles.y / 2.0f));
	float sY(sin(someEulerAngles.y / 2.0f));

	float cZ(cos(someEulerAngles.z / 2.0f));
	float sZ(sin(someEulerAngles.z / 2.0f));

	Quaternionf qX(sX, 0.0F, 0.0F, cX);
	Quaternionf qY(0.0F, sY, 0.0F, cY);
	Quaternionf qZ(0.0F, 0.0F, sZ, cZ);

	Quaternionf ret;

	switch (order)
	{
	case kOrderZYX: CreateQuaternionFromAxisQuaternions(qX, qY, qZ, ret); break;
	case kOrderYZX: CreateQuaternionFromAxisQuaternions(qX, qZ, qY, ret); break;
	case kOrderXZY: CreateQuaternionFromAxisQuaternions(qY, qZ, qX, ret); break;
	case kOrderZXY: CreateQuaternionFromAxisQuaternions(qY, qX, qZ, ret); break;
	case kOrderYXZ: CreateQuaternionFromAxisQuaternions(qZ, qX, qY, ret); break;
	case kOrderXYZ: CreateQuaternionFromAxisQuaternions(qZ, qY, qX, ret); break;
	}

	return ret;
}

inline Quaternionf CreateQuaternionFromAxisQuaternions(const Quaternionf& q1, const Quaternionf& q2, const Quaternionf& q3, Quaternionf& result)
{
    result = (q1 * q2) * q3;
    CompareApproximately(SqrMagnitude(result), 1.0F);
    return result;
}

inline Vector3f QuaternionToEuler(const Quaternionf& q, RotationOrder order)
{
    //AssertMsg(fabsf(SqrMagnitude(q) - 1.0f) < Vector3f::epsilon, "QuaternionToEuler: Input quaternion was not normalized");
    if (fabsf(SqrMagnitude(q) - 1.0f) < Vector3f::epsilon)
    {
        std::cout << "QuaternionToEuler: Input quaternion was not normalized" << std::endl;
        return Vector3f::zero;
    }
    //setup all needed values
    float d[QuatIndexesCount] = { q.x * q.x, q.x * q.y, q.x * q.z, q.x * q.w, q.y * q.y, q.y * q.z, q.y * q.w, q.z * q.z, q.z * q.w, q.w * q.w };

    //Float array for values needed to calculate the angles
    float v[IndexesCount] = { 0.0f };
    qFunc f[3] = { qFuncs[order][0], qFuncs[order][1], qFuncs[order][2] }; //functions to be used to calculate angles

    const float SINGULARITY_CUTOFF = 0.499999f;
    Vector3f rot;
    switch (order)
    {
    case kOrderZYX:
        v[singularity_test] = d[xz] + d[yw];
        v[Z1] = 2.0f * (-d[xy] + d[zw]);
        v[Z2] = d[xx] - d[zz] - d[yy] + d[ww];
        v[Y1] = 1.0f;
        v[Y2] = 2.0f * v[singularity_test];
        if (Abs(v[singularity_test]) < SINGULARITY_CUTOFF)
        {
            v[X1] = 2.0f * (-d[yz] + d[xw]);
            v[X2] = d[zz] - d[yy] - d[xx] + d[ww];
        }
        else //x == xzx z == 0
        {
            float a, b, c, e;
            a = d[xz] + d[yw];
            b = -d[xy] + d[zw];
            c = d[xz] - d[yw];
            e = d[xy] + d[zw];

            v[X1] = a * e + b * c;
            v[X2] = b * e - a * c;
            f[2] = &qNull;
        }
        break;
    case kOrderXZY:
        v[singularity_test] = d[xy] + d[zw];
        v[X1] = 2.0f * (-d[yz] + d[xw]);
        v[X2] = d[yy] - d[zz] - d[xx] + d[ww];
        v[Z1] = 1.0f;
        v[Z2] = 2.0f * v[singularity_test];

        if (Abs(v[singularity_test]) < SINGULARITY_CUTOFF)
        {
            v[Y1] = 2.0f * (-d[xz] + d[yw]);
            v[Y2] = d[xx] - d[zz] - d[yy] + d[ww];
        }
        else //y == yxy x == 0
        {
            float a, b, c, e;
            a = d[xy] + d[zw];
            b = -d[yz] + d[xw];
            c = d[xy] - d[zw];
            e = d[yz] + d[xw];

            v[Y1] = a * e + b * c;
            v[Y2] = b * e - a * c;
            f[0] = &qNull;
        }
        break;

    case kOrderYZX:
        v[singularity_test] = d[xy] - d[zw];
        v[Y1] = 2.0f * (d[xz] + d[yw]);
        v[Y2] = d[xx] - d[zz] - d[yy] + d[ww];
        v[Z1] = -1.0f;
        v[Z2] = 2.0f * v[singularity_test];

        if (Abs(v[singularity_test]) < SINGULARITY_CUTOFF)
        {
            v[X1] = 2.0f * (d[yz] + d[xw]);
            v[X2] = d[yy] - d[xx] - d[zz] + d[ww];
        }
        else //x == xyx y == 0
        {
            float a, b, c, e;
            a = d[xy] - d[zw];
            b = d[xz] + d[yw];
            c = d[xy] + d[zw];
            e = -d[xz] + d[yw];

            v[X1] = a * e + b * c;
            v[X2] = b * e - a * c;
            f[1] = &qNull;
        }
        break;
    case kOrderZXY:
    {
        v[singularity_test] = d[yz] - d[xw];
        v[Z1] = 2.0f * (d[xy] + d[zw]);
        v[Z2] = d[yy] - d[zz] - d[xx] + d[ww];
        v[X1] = -1.0f;
        v[X2] = 2.0f * v[singularity_test];

        if (Abs(v[singularity_test]) < SINGULARITY_CUTOFF)
        {
            v[Y1] = 2.0f * (d[xz] + d[yw]);
            v[Y2] = d[zz] - d[xx] - d[yy] + d[ww];
        }
        else //x == yzy z == 0
        {
            float a, b, c, e;
            a = d[xy] + d[zw];
            b = -d[yz] + d[xw];
            c = d[xy] - d[zw];
            e = d[yz] + d[xw];

            v[Y1] = a * e + b * c;
            v[Y2] = b * e - a * c;
            f[2] = &qNull;
        }
    }
    break;
    case kOrderYXZ:
        v[singularity_test] = d[yz] + d[xw];
        v[Y1] = 2.0f * (-d[xz] + d[yw]);
        v[Y2] = d[zz] - d[yy] - d[xx] + d[ww];
        v[X1] = 1.0f;
        v[X2] = 2.0f * v[singularity_test];

        if (Abs(v[singularity_test]) < SINGULARITY_CUTOFF)
        {
            v[Z1] = 2.0f * (-d[xy] + d[zw]);
            v[Z2] = d[yy] - d[zz] - d[xx] + d[ww];
        }
        else //x == zyz y == 0
        {
            float a, b, c, e;
            a = d[yz] + d[xw];
            b = -d[xz] + d[yw];
            c = d[yz] - d[xw];
            e = d[xz] + d[yw];

            v[Z1] = a * e + b * c;
            v[Z2] = b * e - a * c;
            f[1] = &qNull;
        }
        break;
    case kOrderXYZ:
        v[singularity_test] = d[xz] - d[yw];
        v[X1] = 2.0f * (d[yz] + d[xw]);
        v[X2] = d[zz] - d[yy] - d[xx] + d[ww];
        v[Y1] = -1.0f;
        v[Y2] = 2.0f * v[singularity_test];

        if (Abs(v[singularity_test]) < SINGULARITY_CUTOFF)
        {
            v[Z1] = 2.0f * (d[xy] + d[zw]);
            v[Z2] = d[xx] - d[zz] - d[yy] + d[ww];
        }
        else //x == zxz x == 0
        {
            float a, b, c, e;
            a = d[xz] - d[yw];
            b = d[yz] + d[xw];
            c = d[xz] + d[yw];
            e = -d[yz] + d[xw];

            v[Z1] = a * e + b * c;
            v[Z2] = b * e - a * c;
            f[0] = &qNull;
        }
        break;
    }

    rot = Vector3f(f[0](v[X1], v[X2]),
        f[1](v[Y1], v[Y2]),
        f[2](v[Z1], v[Z2]));

    //Assert(IsFinite(rot));

    return rot;
}

inline Quaternionf ExtractQuaternionFromFBXEuler(FbxVector4 euler, EFbxRotationOrder rotationOrder)
{
	euler[1] = -euler[1];
	euler[2] = -euler[2];

	RotationOrder newOrder = FbxOrderToUnityOrder(rotationOrder);

	Vector3f eulerV = Vector3f((float)euler[0], (float)euler[1], (float)euler[2]);
	eulerV *= (float)FBXSDK_DEG_TO_RAD;
	Quaternionf quat = EulerToQuaternion(eulerV, newOrder);

	// FBX SDK (i.e. FbxAMatrix::GetQ) doesn't guarante to return normalized quaternion - so we do that ourselves
	quat = Normalize(quat);

	return quat;
}

inline Quaternionf ExtractQuaternionFromFBXEulerOld(FbxVector4 euler)
{
	euler[1] = -euler[1];
	euler[2] = -euler[2];
	FbxAMatrix rotationMatrix;
	rotationMatrix.SetR(euler);

	FbxQuaternion rotationQuat = rotationMatrix.GetQ();
	Quaternionf q(
		static_cast<float>(rotationQuat[0]),
		static_cast<float>(rotationQuat[1]),
		static_cast<float>(rotationQuat[2]),
		static_cast<float>(rotationQuat[3])
	);

	q = Normalize(q);
	return q;
}

