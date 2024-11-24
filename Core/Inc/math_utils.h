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