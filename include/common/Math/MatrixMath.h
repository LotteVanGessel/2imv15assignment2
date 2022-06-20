#include <cstdlib>

struct Data{
    float* data;
    int n;
          float& operator[](int i)       { return data[i]; }
    const float& operator[](int i) const { return data[i]; }
    Data& operator+=(const Data &rhs){ int i; for(i = 0; i < n; ++i) data[i]+=rhs[i]; return *this; }
    Data& operator-=(const Data &rhs){ int i; for(i = 0; i < n; ++i) data[i]-=rhs[i]; return *this; }
    Data& operator*=(const float rhs){ int i; for(i = 0; i < n; ++i) data[i]*=rhs   ; return *this;}
};

// access by [index] or (row_index, col_index)
struct Mat2 : Data{
    Mat2(){
        data = (float*) malloc(4*sizeof(float));
        n = 4;
    }
    Mat2(float* data){
        data = data;
        n = 4;
    }
    static inline int ind(int r, int c) { return 2*r + c; }
          float& operator()(int r, int c)       { return data[Mat2::ind(r, c)]; }
    const float& operator()(int r, int c) const { return data[Mat2::ind(r, c)]; }
    Mat2 get_inverse(){
        Mat2 inv;
        inv[0] = data[3];
        inv[1] = -data[1];
        inv[2] = -data[2];
        inv[3] = data[0];
        inv *= (1/det()); //in place multiplication
    }
    float det(){
        return data[0]*data[3]-data[1]*data[2];
    }
};

struct Vec2 : Data{
    Vec2(){
        data = (float*) malloc(2*sizeof(float));
        data[0] = 0;
        data[1] = 0;
        n = 2;
    }
    Vec2(float* data){
      data = data;
      n = 2;  
    } 
    Vec2(float x, float y){
        data = (float*) malloc(2*sizeof(float));
        data[0] = x;
        data[1] = y;
        n = 2;
    }
};

float temp;
inline void swap(Data &x, int i, int j){
    temp = x[i]; x[i] = x[j]; x[j] = temp;
}

void transpose_in_place(Mat2 &mat){
    swap(mat, 1, 2);
}

Vec2 operator*(const Mat2 &mat, const Vec2 &v){
    return Vec2(mat[0]*v[0] + mat[1]*v[1], mat[2]*v[0] + mat[3]*v[1]);
}

void matmult(const Mat2 &mat, const Vec2 &v, Vec2 &result){
    result[0] = mat[0]*v[0] + mat[1]*v[1];
    result[1] = mat[2]*v[0] + mat[3]*v[1];
}

void transmatmult(Mat2 &mat, const Vec2 &v, Vec2 &result){
    transpose_in_place(mat);
    matmult(mat, v, result);
    transpose_in_place(mat);
}

Vec2 operator+(const Vec2 &u, const Vec2 &v){
    return Vec2(u[0]+v[0], u[1]+v[1]);
}

Vec2 operator-(const Vec2 &u, const Vec2 &v){
    return Vec2(u[0]-v[0], u[1]-v[1]);
}


void vecadd(const float* u, const float* v, float* result, int n){
    int i; for(i = 0; i < n; ++i) result[i] = u[i] + v[i];
}

void vecadd(const Data &u, const Data &v, Data &result){
    vecadd(u.data, v.data, result.data, u.n);
}

void scalarmult(const float scalar, const Data &x, Data &result){
    int i;
    for (i = 0; i < x.n; i++){
        result[i] = scalar * x[i];
    }
}

void rotate90(const Vec2 &v, Vec2 &result){
    result[0] = -v[1]; result[1] = v[0];
}

void scalarmult(const float scalar, const float* x, float* r, int n){
    int i; for (i = 0; i < n; i++) r[i] = x[i]*scalar;
}

void scalarmult(const float scalar, float* x, int n){
    int i; for (i = 0; i < n; i++) x[i]*=scalar;
}
