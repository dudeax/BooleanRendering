#ifndef VECTOR
#define VECTOR

#include <math.h>
//
class Vector {
    public:
    __host__ __device__ Vector() {}
    __host__ __device__ Vector(double value) {x = value; y = value; z = value;}
    __host__ __device__ Vector(double xIn, double yIn, double zIn) {x = xIn; y = yIn; z = zIn;}
    //__host__ __device__ inline Vector operator+(const Vector& other);
    //__host__ __device__ inline Vector operator-(const Vector& other);
    //__host__ __device__ inline Vector operator*(const Vector& other);
    __host__ __device__ inline Vector operator*(const double& value) {return Vector(x * value, y * value, z * value);}
    __host__ __device__ inline Vector operator/(const double& value) {return Vector(x / value, y / value, z / value);}
    __host__ __device__ inline Vector cross(const Vector& other);
    __host__ __device__ inline double dot(const Vector& other);
    __host__ __device__ inline double magnitude() const {return std::sqrt(x * x + y * y + z * z);}
    __host__ __device__ inline Vector unit();
    double x, y, z;
};

__host__ __device__ inline Vector operator+(const Vector& self, const Vector& other) {
    return Vector(self.x + other.x, self.y + other.y, self.z + other.z);
}

__host__ __device__ inline Vector operator-(const Vector& self, const Vector& other) {
    return Vector(self.x - other.x, self.y - other.y, self.z - other.z);
}

__host__ __device__ inline Vector operator*(const Vector& self, const Vector& other) {
    return Vector(self.x * other.x, self.y * other.y, self.z * other.z);
}

__host__ __device__ inline Vector Vector::cross(const Vector& other) {
    return Vector((y * other.z) - (other.y * z), (z * other.x) - (other.z * x), (x * other.y) - (other.x * y));
}

__host__ __device__ inline double Vector::dot(const Vector& other) {
    return x * other.x + y * other.y + z * other.z;
}

__host__ __device__ inline Vector Vector::unit() {
    float amount = 1.0 / std::sqrt(x * x + y * y + z * z);
    return Vector(x * amount, y * amount, z * amount);
}

#endif
