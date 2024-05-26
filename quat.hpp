#pragma once

#include <cmath>

template< typename T >
struct matrix_t
{
	T data[16];
};

template< typename T >
struct vector3_t
{
	T x, y, z;
};

template< typename T >
class Quat
{
  public:
	Quat() : m_value{ 0, 0, 0, 0 } {}
	Quat(T a, T b, T c, T d) : m_value{ b, c, d, a } {}
	Quat(T angle, bool radians, const vector3_t< T > &axis)
	{
		T norm = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
		if (!norm)
		{
			Quat();
			return;
		}
		if (!radians)
			angle = angle * T(M_PI) / 180;
		T half_angle = angle / 2;
		T s = std::sin(half_angle);
		m_value[3] = std::cos(half_angle);
		m_value[0] = axis.x * s / norm;
		m_value[1] = axis.y * s / norm;
		m_value[2] = axis.z * s / norm;
	}

	Quat operator+(const Quat &other) const
	{
		Quat tmp = *this;
		tmp += other;
		return tmp;
	}

	Quat &operator+=(const Quat &other)
	{
		m_value[0] += other.m_value[0];
		m_value[1] += other.m_value[1];
		m_value[2] += other.m_value[2];
		m_value[3] += other.m_value[3];
		return *this;
	}

	Quat operator-(const Quat &other) const
	{
		Quat tmp = *this;
		tmp -= other;
		return tmp;
	}

	Quat &operator-=(const Quat &other)
	{
		m_value[0] -= other.m_value[0];
		m_value[1] -= other.m_value[1];
		m_value[2] -= other.m_value[2];
		m_value[3] -= other.m_value[3];
		return *this;
	}

	Quat operator*(const Quat &other) const
	{
		T a1 = m_value[3];
		T b1 = m_value[0];
		T c1 = m_value[1];
		T d1 = m_value[2];

		T a2 = other.m_value[3];
		T b2 = other.m_value[0];
		T c2 = other.m_value[1];
		T d2 = other.m_value[2];

		return { a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2,
				 a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2,
				 a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2,
				 a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2 };
	}

	Quat operator*(const vector3_t< T > &vec) const
	{
		Quat tmp;
		tmp.m_value[0] = m_value[3] * vec.x + m_value[1] * vec.z - m_value[2] * vec.y;
		tmp.m_value[1] = m_value[3] * vec.y - m_value[0] * vec.z + m_value[2] * vec.x;
		tmp.m_value[2] = m_value[3] * vec.z + m_value[0] * vec.y - m_value[1] * vec.x;
		tmp.m_value[3] = -m_value[0] * vec.x - m_value[1] * vec.y - m_value[2] * vec.z;
		return tmp;
	}

	Quat operator*(const T scalar) const
	{
		Quat tmp;
		tmp.m_value[0] = m_value[0] * scalar;
		tmp.m_value[1] = m_value[1] * scalar;
		tmp.m_value[2] = m_value[2] * scalar;
		tmp.m_value[3] = m_value[3] * scalar;
		return tmp;
	}

	Quat operator~() const { return Quat(m_value[3], -m_value[0], -m_value[1], -m_value[2]); }

	bool operator==(const Quat &other) const
	{
		return m_value[0] == other.m_value[0] && m_value[1] == other.m_value[1] && m_value[2] == other.m_value[2] &&
			   m_value[3] == other.m_value[3];
	}

	bool operator!=(const Quat &other) const { return !(*this == other); }

	explicit operator T() const
	{
		return std::sqrt(m_value[0] * m_value[0] + m_value[1] * m_value[1] + m_value[2] * m_value[2] + m_value[3] * m_value[3]);
	}

	const T *data() const { return m_value; }

	matrix_t< T > rotation_matrix() const
	{
		matrix_t< T > mat;
		T norm = T(*this);
		if (!norm)
		{
			for (int i = 0; i < 16; ++i)
			{
				mat.data[i] = 0;
			}
			return mat;
		}
		T x = m_value[0] / norm;
		T y = m_value[1] / norm;
		T z = m_value[2] / norm;
		T w = m_value[3] / norm;

		mat.data[0] = 1 - 2 * y * y - 2 * z * z;
		mat.data[1] = 2 * x * y + 2 * z * w;
		mat.data[2] = 2 * x * z - 2 * y * w;
		mat.data[3] = 0;

		mat.data[4] = 2 * x * y - 2 * z * w;
		mat.data[5] = 1 - 2 * x * x - 2 * z * z;
		mat.data[6] = 2 * z * y + 2 * x * w;
		mat.data[7] = 0;

		mat.data[8] = 2 * x * z + 2 * y * w;
		mat.data[9] = 2 * y * z - 2 * x * w;
		mat.data[10] = 1 - 2 * x * x - 2 * y * y;
		mat.data[11] = 0;

		mat.data[12] = 0;
		mat.data[13] = 0;
		mat.data[14] = 0;
		mat.data[15] = 1;

		return mat;
	}

	matrix_t< T > matrix() const
	{
		matrix_t< T > mat;
		mat.data[0] = m_value[3];
		mat.data[1] = -m_value[0];
		mat.data[2] = -m_value[1];
		mat.data[3] = -m_value[2];

		mat.data[4] = m_value[0];
		mat.data[5] = m_value[3];
		mat.data[6] = -m_value[2];
		mat.data[7] = m_value[1];

		mat.data[8] = m_value[1];
		mat.data[9] = m_value[2];
		mat.data[10] = m_value[3];
		mat.data[11] = -m_value[0];

		mat.data[12] = m_value[2];
		mat.data[13] = -m_value[1];
		mat.data[14] = m_value[0];
		mat.data[15] = m_value[3];
		return mat;
	}

	T angle(bool radians = true) const
	{
		T norm = (T(*this));
		if (!norm)
		{
			return 0;
		}
		T angle = 2 * std::acos(m_value[3] / norm);
		if (!radians)
			angle = angle * 180 / T(M_PI);
		return angle;
	}

	vector3_t< T > apply(const vector3_t< T > &vec) const
	{
		Quat tmp;
		T norm = T(*this);
		if (!norm)
		{
			return { 0, 0, 0 };
		}
		tmp.m_value[0] = m_value[0] / norm;
		tmp.m_value[1] = m_value[1] / norm;
		tmp.m_value[2] = m_value[2] / norm;
		tmp.m_value[3] = m_value[3] / norm;

		tmp = tmp * vec * ~tmp;

		return { tmp.m_value[0], tmp.m_value[1], tmp.m_value[2] };
	}

  private:
	T m_value[4];
};
