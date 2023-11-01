#ifndef DKVR_DRIVER_DATATYPE
#define DKVR_DRIVER_DATATYPE

#include <math.h>
#include <stdint.h>

namespace DKVR
{
    namespace DataType
    {
		
        struct Vector3f {
			float x, y, z;

			Vector3f() : x(0.0f), y(0.0f), z(0.0f) { };
			Vector3f(float x, float y, float z) : x(x), y(y), z(z) { };

			float Dot(const Vector3f &obj) const
			{
				return x * obj.x + y * obj.y + z * obj.z;
			}
		};

		struct Vector3s
		{
			int16_t x, y, z;

			Vector3s() : x(0), y(0), z(0){};
			Vector3s(int16_t x, int16_t y, int16_t z) : x(x), y(y), z(z){};

			int16_t operator[](size_t index) const
			{
				if (index == 0)
					return x;
				else if (index == 1)
					return y;
				else
					return z;
			}

			void operator-=(const Vector3s &obj)
			{
				x -= obj.x;
				y -= obj.y;
				z -= obj.z;
			}

			Vector3f ToVector3f(float divisor) const
			{
				return Vector3f(x / divisor, y / divisor, z / divisor);
			}
		};

		struct Vector4f {
			float x, y, z, w;

			Vector4f() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) { }
			Vector4f(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) { }
			Vector4f(Vector3f vector, float scalar) : x(vector.x), y(vector.y), z(vector.z), w(scalar) { }

			float Dot(const Vector3f &obj) const {
				return x * obj.x + y * obj.y + z * obj.z + w;
			}
		};

		struct Quaternion {
			float x, y, z, w;

			Quaternion() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) { }
			Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) { }
			Quaternion(Vector3f vector, float scalar) : x(vector.x), y(vector.y), z(vector.z), w(scalar) { }
		};

	} // namespace datatype
    
} // namespace dkvr


#endif // DKVR_DRIVER_DATATYPE
