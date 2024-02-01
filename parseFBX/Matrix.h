#pragma once

class Matrix3x3f;
class Matrix4x4f;

struct Matrix4x4f
{
	Matrix4x4f() {}
	float m_Data[16];
	enum InitIdentity { kIdentity };
	Matrix4x4f(InitIdentity) { SetIdentity(); }
	float* GetPtr() { return m_Data; }
	Matrix4x4f(const Matrix3x3f& other);
	float operator[](int index) const { return m_Data[index]; }
	float& operator[](int index) { return m_Data[index]; }
	Matrix4x4f& operator=(const Matrix3x3f& m);
	Matrix4x4f& operator*=(const Matrix4x4f& inM);
	float& Get(int row, int column) { return m_Data[row + (column * 4)]; }
	const float& Get(int row, int column) const { return m_Data[row + (column * 4)]; }
	Matrix4x4f& SetIdentity() {
		{
			Get(0, 0) = 1.0;   Get(0, 1) = 0.0;   Get(0, 2) = 0.0;   Get(0, 3) = 0.0;
			Get(1, 0) = 0.0;   Get(1, 1) = 1.0;   Get(1, 2) = 0.0;   Get(1, 3) = 0.0;
			Get(2, 0) = 0.0;   Get(2, 1) = 0.0;   Get(2, 2) = 1.0;   Get(2, 3) = 0.0;
			Get(3, 0) = 0.0;   Get(3, 1) = 0.0;   Get(3, 2) = 0.0;   Get(3, 3) = 1.0;
			return *this;
		}
	}
	Matrix4x4f& SetScale(const Vector3f& inScale)
	{
		Get(0, 0) = inScale.x;    Get(0, 1) = 0.0;           Get(0, 2) = 0.0;           Get(0, 3) = 0.0;
		Get(1, 0) = 0.0;           Get(1, 1) = inScale.y;    Get(1, 2) = 0.0;           Get(1, 3) = 0.0;
		Get(2, 0) = 0.0;           Get(2, 1) = 0.0;           Get(2, 2) = inScale.z;    Get(2, 3) = 0.0;
		Get(3, 0) = 0.0;           Get(3, 1) = 0.0;           Get(3, 2) = 0.0;           Get(3, 3) = 1.0;
		return *this;
	}

	Vector3f MultiplyPoint3(const Vector3f& v) const
	{
		Vector3f res;
		res.x = m_Data[0] * v.x + m_Data[4] * v.y + m_Data[8] * v.z + m_Data[12];
		res.y = m_Data[1] * v.x + m_Data[5] * v.y + m_Data[9] * v.z + m_Data[13];
		res.z = m_Data[2] * v.x + m_Data[6] * v.y + m_Data[10] * v.z + m_Data[14];
		return res;
	}

	Vector3f MultiplyVector3(const Vector3f& v) const
	{
		Vector3f res;
		res.x = m_Data[0] * v.x + m_Data[4] * v.y + m_Data[8] * v.z;
		res.y = m_Data[1] * v.x + m_Data[5] * v.y + m_Data[9] * v.z;
		res.z = m_Data[2] * v.x + m_Data[6] * v.y + m_Data[10] * v.z;
		return res;
	}

};


inline bool IsFinite(const Matrix4x4f& f)
{
	return
		IsFinite(f.m_Data[0]) & IsFinite(f.m_Data[1]) & IsFinite(f.m_Data[2]) &
		IsFinite(f.m_Data[4]) & IsFinite(f.m_Data[5]) & IsFinite(f.m_Data[6]) &
		IsFinite(f.m_Data[8]) & IsFinite(f.m_Data[9]) & IsFinite(f.m_Data[10]) &
		IsFinite(f.m_Data[12]) & IsFinite(f.m_Data[13]) & IsFinite(f.m_Data[14]) & IsFinite(f.m_Data[15]);
}

struct Matrix3x3f
{
	Matrix3x3f() {}
	float m_Data[9];
	enum InitIdentity { kIdentity };
	Matrix3x3f(InitIdentity) { SetIdentity(); }

	Matrix3x3f& operator*=(const class Matrix4x4f& inM);
	explicit Matrix3x3f(const class Matrix4x4f& other)
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
	}

	Matrix3x3f& SetIdentity() {
		{
			Get(0, 0) = 1.0F;  Get(0, 1) = 0.0F;  Get(0, 2) = 0.0F;
			Get(1, 0) = 0.0F;  Get(1, 1) = 1.0F;  Get(1, 2) = 0.0F;
			Get(2, 0) = 0.0F;  Get(2, 1) = 0.0F;  Get(2, 2) = 1.0F;
			return *this;
		}
	}
	Matrix3x3f& SetScale(const Vector3f& inScale)
	{
		Get(0, 0) = inScale.x;    Get(0, 1) = 0.0F;          Get(0, 2) = 0.0F;
		Get(1, 0) = 0.0F;          Get(1, 1) = inScale.y;    Get(1, 2) = 0.0F;
		Get(2, 0) = 0.0F;          Get(2, 1) = 0.0F;          Get(2, 2) = inScale.z;
		return *this;
	}
	float operator[](int index) const { return m_Data[index]; }
	float& operator[](int index) { return m_Data[index]; }
	float& Get(int row, int column) { return m_Data[row + (column * 3)]; }
	const float& Get(int row, int column) const { return m_Data[row + (column * 3)]; }
	Matrix3x3f& operator=(const class Matrix4x4f& m);

	Vector3f MultiplyPoint3(const Vector3f& v) const
	{
		Vector3f res;
		res.x = m_Data[0] * v.x + m_Data[3] * v.y + m_Data[6] * v.z;
		res.y = m_Data[1] * v.x + m_Data[4] * v.y + m_Data[7] * v.z;
		res.z = m_Data[2] * v.x + m_Data[5] * v.y + m_Data[8] * v.z;
		return res;
	}

	inline Vector3f MultiplyVector3(const Vector3f& v) const
	{
		Vector3f res;
		res.x = m_Data[0] * v.x + m_Data[3] * v.y + m_Data[6] * v.z;
		res.y = m_Data[1] * v.x + m_Data[4] * v.y + m_Data[7] * v.z;
		res.z = m_Data[2] * v.x + m_Data[5] * v.y + m_Data[8] * v.z;
		return res;
	}
	bool Invert();

	Matrix3x3f& Transpose()
	{
		std::swap(Get(0, 1), Get(1, 0));
		std::swap(Get(0, 2), Get(2, 0));
		std::swap(Get(2, 1), Get(1, 2));
		return *this;
	}

	void InvertTranspose()
	{
		Invert();
		Transpose();
	}

};
