#ifndef MATH_UTILS_H
#define MATH_UTILS_H

typedef struct {
    double x;
    double y;
    double z;
} Vec3;

typedef struct {
    Vec3 col1;
    Vec3 col2;
    Vec3 col3;
} Mat3;

double dot(Vec3 a, Vec3 b);
Vec3 vec3_add(Vec3 a, Vec3 b);
Vec3 vec3_scale(Vec3 a, double s);
double vec3_mag(Vec3 a);
Vec3 vec3_norm(Vec3 a);
Vec3 vec3_proj(Vec3 a, Vec3 b);
// b must be a unit vector
Vec3 vec3_proj_unit(Vec3 a, Vec3 b);
Vec3 vec3_neg(Vec3 a);
Vec3 vec3_sub(Vec3 a, Vec3 b);
Vec3 vec3_cross(Vec3 a, Vec3 b);

Mat3 mat3_mul(Mat3 a, Mat3 b);
Mat3 mat3_add(Mat3 a, Mat3 b);
Mat3 mat3_scale(Mat3 a, double s);

#endif // MATH_UTILS_H