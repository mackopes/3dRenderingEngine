#pragma once
#include <cassert>

template <int n> struct vec {
  double data[n] = {0}; // If too few elements are used for initialisation, the init array is padded with 0s
  double operator[](const int i) { assert(i >= 0 && i < n); return data[i]; }
};

template<> struct vec<2> {
  double x{}, y{};
  vec(double x, double y): x(x), y(y) {}
  double operator[](const int i) { assert(i >= 0 && i < 2); return i ? y : x; }
};

template<> struct vec<3> {
  double x{}, y{}, z{};
  vec(double x, double y, double z): x(x), y(y), z(z) {}
  double operator[](const int i) { assert(i >= 0 && i < 3); switch(i) {case 0: return x; case 1: return y; case 2: return z;}}
};

template <int n> double operator*(const vec<n> &lhs, const vec<n> &rhs) {
  double ret = 0;
  for (int i = 0; i < n; ++i) {
    ret += lhs[i] * rhs[i];
  }
  return ret;
}

template<int n> vec<n> operator+(const vec<n> &lhs, const vec<n> &rhs) {
  vec<n> ret;
  for(int i = 0; i < n; ++i) {
    ret[i] = lhs[i] + rhs[i];
  }
  return ret;

  // can be rewritten as for(int i = n; i--; rhs[i] = lhs[i] + rhs[i]); but why would you do that?
}
