#include "math_utils.h"
#include <math.h>

double dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 vec3_add(Vec3 a, Vec3 b) {
    return (Vec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 vec3_scale(Vec3 a, double s) {
    return (Vec3){a.x * s, a.y * s, a.z * s};
}

double vec3_mag(Vec3 a) {
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

Vec3 vec3_norm(Vec3 a) {
    return vec3_scale(a, 1.0 / vec3_mag(a));
}

Vec3 vec3_proj(Vec3 a, Vec3 b) {
    return vec3_scale(b, dot(a, b) / dot(b, b));
}

// b must be a unit vector
Vec3 vec3_proj_unit(Vec3 a, Vec3 b) {
    return vec3_scale(b, dot(a, b));
}

Vec3 vec3_neg(Vec3 a) {
    return (Vec3){-a.x, -a.y, -a.z};
}

Vec3 vec3_sub(Vec3 a, Vec3 b) {
    return (Vec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 vec3_cross(Vec3 a, Vec3 b) {
    return (Vec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Mat3 mat3_mul(Mat3 a, Mat3 b) {
    Vec3 col1 = {dot((Vec3){a.col1.x, a.col2.x, a.col3.x}, b.col1),
                 dot((Vec3){a.col1.y, a.col2.y, a.col3.y}, b.col1),
                 dot((Vec3){a.col1.z, a.col2.z, a.col3.z}, b.col1)};
    Vec3 col2 = {dot((Vec3){a.col1.x, a.col2.x, a.col3.x}, b.col2),
                 dot((Vec3){a.col1.y, a.col2.y, a.col3.y}, b.col2),
                 dot((Vec3){a.col1.z, a.col2.z, a.col3.z}, b.col2)};
    Vec3 col3 = {dot((Vec3){a.col1.x, a.col2.x, a.col3.x}, b.col3),
                 dot((Vec3){a.col1.y, a.col2.y, a.col3.y}, b.col3),
                 dot((Vec3){a.col1.z, a.col2.z, a.col3.z}, b.col3)};
    return (Mat3){col1, col2, col3};
}

Vec3 mat3_mul_vec3(Mat3 a, Vec3 b) {
    return (Vec3){dot((Vec3) {a.col1.x, a.col2.x, a.col3.x}, b), 
                  dot((Vec3) {a.col1.y, a.col2.y, a.col3.y}, b), 
                  dot((Vec3) {a.col1.z, a.col2.z, a.col3.z}, b)};
}

Mat3 mat3_add(Mat3 a, Mat3 b) {
    Vec3 col1 = {a.col1.x + b.col1.x, a.col1.y + b.col1.y, a.col1.z + b.col1.z};
    Vec3 col2 = {a.col2.x + b.col2.x, a.col2.y + b.col2.y, a.col2.z + b.col2.z};
    Vec3 col3 = {a.col3.x + b.col3.x, a.col3.y + b.col3.y, a.col3.z + b.col3.z};
    return (Mat3){col1, col2, col3};
}

Mat3 mat3_scale(Mat3 a, double s) {
    Vec3 col1 = {a.col1.x * s, a.col1.y * s, a.col1.z * s};
    Vec3 col2 = {a.col2.x * s, a.col2.y * s, a.col2.z * s};
    Vec3 col3 = {a.col3.x * s, a.col3.y * s, a.col3.z * s};
    return (Mat3){col1, col2, col3};
}

Mat3 mat3_transpose(Mat3 a) {
    Vec3 col1 = {a.col1.x, a.col2.x, a.col3.x};
    Vec3 col2 = {a.col1.y, a.col2.y, a.col3.y};
    Vec3 col3 = {a.col1.z, a.col2.z, a.col3.z};
    return (Mat3){col1, col2, col3};
}

QuatF quatf_scale(QuatF q, float a) {
    return (QuatF){q.x * a, q.y * a, q.z * a, q.w * a};
}

QuatF quatf_from_mat3(Mat3 a) {
    // https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
    // they tell us that their formula uses transposed matrix. I'm too lazy to do it manually for now...
    a = mat3_transpose(a);

    float t;
    QuatF q;

    if (a.col2.y < 0) {
        if (a.col1.x > a.col2.y) {
            t = 1 + a.col1.x - a.col2.y - a.col3.z;
            q = (QuatF) { t, a.col2.x+a.col1.y, a.col1.z+a.col3.x, a.col3.y-a.col2.z };
        } else {
            t = 1 - a.col1.x + a.col2.y - a.col3.z;
            q = (QuatF) { a.col2.x+a.col1.y, t, a.col3.y+a.col2.z, a.col1.z-a.col3.x };
        }
    } else {
        if (a.col1.x < -a.col2.y) {
            t = 1 - a.col1.x - a.col2.y + a.col3.z;
            q = (QuatF) { a.col1.z+a.col3.x, a.col3.y+a.col2.z, t, a.col2.x-a.col1.y };
        }
        else {
            t = 1 + a.col1.x + a.col2.y + a.col3.z;
            q = (QuatF) { a.col3.y-a.col2.z, a.col1.z-a.col3.x, a.col2.x-a.col1.y, t };
        }
    }
    q = quatf_scale(q, 0.5 / sqrt(t));

    return q;
}