#ifndef _H_MATH_VECTORS_
#define _H_MATH_VECTORS_

#define RAD2DEG(x) ((x)*57.295754f)
#define DEG2RAD(x) ((x)*0.0174533f)

struct vec2 {
  union {
    struct {
      float x;
      float y;
    };
    float asArray[2];
  };

  float &operator[](int i) { return asArray[i]; }
};

struct vec3 {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float asArray[3];
  };

  float &operator[](int i) { return asArray[i]; }
};
vec2 operator+(const vec2 &lhs, const vec2 &rhs);
vec3 operator+(const vec3 &lhs, const vec3 &rhs);
vec2 operator-(const vec2 &lhs, const vec2 &rhs);
vec3 operator-(const vec3 &lhs, const vec3 &rhs);
vec2 operator*(const vec2 &lhs, const vec2 &rhs);
vec3 operator*(const vec3 &lhs, const vec3 &rhs);
vec2 operator*(const vec2 &lhs, float rhs);
vec3 operator*(const vec3 &lhs, float rhs);
bool operator==(const vec2 &lhs, const vec2 &rhs);
bool operator==(const vec3 &lhs, const vec3 &rhs);
bool operator!=(const vec2 &lhs, const vec2 &rhs);
bool operator!=(const vec3 &lhs, const vec3 &rhs);

float dot(const vec2 &lhs, const vec2 &rhs);
float dot(const vec3 &lhs, const vec3 &rhs);

float magnitude(const vec2 &vec);
float magnitude(const vec3 &vec);
float magnitude_sq(const vec2 &vec);
float magnitude_sq(const vec3 &vec);

void normalize(vec2 &vec);
void normalize(vec3 &vec);
vec2 normalized(const vec2 &vec);
vec3 normalized(const vec3 &vec);

vec3 cross(const vec3 &lhs, const vec3 &rhs);

float angle(const vec2 &lhs, const vec2 &rhs);
float angle(const vec3 &lhs, const vec3 &rhs);

vec2 project(const vec2 &length, const vec2 &direction);
vec3 project(const vec3 &length, const vec3 &direction);

vec2 perpendicular(const vec2 &len, const vec2 &dir);
vec3 perpendicular(const vec3 &len, const vec3 &dir);

vec2 reflection(const vec2 &vec, const vec2 &normal);
vec3 reflection(const vec3 &vec, const vec3 &normal);

#endif // _H_MATH_VECTORS_