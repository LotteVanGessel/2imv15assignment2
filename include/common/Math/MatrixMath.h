#ifndef MATRIXMATH_H
#define MATRIXMATH_H

struct Data{
    float* data;
    int n;
    float& operator[](int i);
    float& operator[](int i) const;
    Data& operator+=(const Data &rhs);
    Data& operator-=(const Data &rhs);
    Data& operator*=(const float rhs);
};

// access by [index] or (row_index, col_index)
struct Mat2 : Data{
    Mat2();
    Mat2(float* x);
    static inline int ind(int r, int c);
    float& operator()(int r, int c);
    float& operator()(int r, int c) const;
    Mat2 get_inverse();
    float det();
    void print();
};

struct RotMat2 : Mat2{
    float c, s;
    void update(float omega);
};

struct Vec2 : Data{
    Vec2();
    Vec2(float* x);
    Vec2(float x, float y);
    void print();
};

inline void swap(Data &x, int i, int j);

Mat2 matmult(const Mat2 &A, const Mat2 &B, Mat2 &R);

void transpose_in_place(Mat2 &mat);

Vec2 operator*(const Mat2 &mat, const Vec2 &v);

void matmult(const Mat2 &mat, const Vec2 &v, Vec2 &result);

void transmatmult(Mat2 &mat, const Vec2 &v, Vec2 &result);

Vec2 operator+(const Vec2 &u, const Vec2 &v);

Vec2 operator-(const Vec2 &u, const Vec2 &v);


void vecadd(const float* u, const float* v, float* result, int n);

void vecadd(const Data &u, const Data &v, Data &result);

void scalarmult(const float scalar, const Data &x, Data &result);

void rotate90(const Vec2 &v, Vec2 &result);

void scalarmult(const float scalar, const float* x, float* r, int n);

void scalarmult(const float scalar, float* x, int n);
#endif