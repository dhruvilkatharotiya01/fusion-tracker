#pragma once
// Small fixed-size matrices. The tracker only needs sizes up to 4x4, so a short
// header keeps the project dependency-free and every operation easy to follow.
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace fusion {

template <std::size_t R, std::size_t C>
struct Mat {
    std::array<double, R * C> a{};

    double& operator()(std::size_t r, std::size_t c) { return a[r * C + c]; }
    double operator()(std::size_t r, std::size_t c) const { return a[r * C + c]; }

    static Mat identity() {
        static_assert(R == C, "identity needs a square matrix");
        Mat m;
        for (std::size_t i = 0; i < R; ++i) m(i, i) = 1.0;
        return m;
    }

    Mat<C, R> t() const {
        Mat<C, R> out;
        for (std::size_t r = 0; r < R; ++r)
            for (std::size_t c = 0; c < C; ++c) out(c, r) = (*this)(r, c);
        return out;
    }
};

template <std::size_t N>
using Vec = Mat<N, 1>;

template <std::size_t N>
Vec<N> vec(const std::array<double, N>& values) {
    Vec<N> out;
    out.a = values;
    return out;
}

template <std::size_t R, std::size_t C>
Mat<R, C> operator+(Mat<R, C> x, const Mat<R, C>& y) {
    for (std::size_t i = 0; i < R * C; ++i) x.a[i] += y.a[i];
    return x;
}

template <std::size_t R, std::size_t C>
Mat<R, C> operator-(Mat<R, C> x, const Mat<R, C>& y) {
    for (std::size_t i = 0; i < R * C; ++i) x.a[i] -= y.a[i];
    return x;
}

template <std::size_t R, std::size_t C>
Mat<R, C> operator*(double s, Mat<R, C> x) {
    for (auto& v : x.a) v *= s;
    return x;
}

template <std::size_t R, std::size_t K, std::size_t C>
Mat<R, C> operator*(const Mat<R, K>& x, const Mat<K, C>& y) {
    Mat<R, C> out;
    for (std::size_t r = 0; r < R; ++r)
        for (std::size_t k = 0; k < K; ++k) {
            const double v = x(r, k);
            for (std::size_t c = 0; c < C; ++c) out(r, c) += v * y(k, c);
        }
    return out;
}

// Gauss-Jordan elimination with partial pivoting.
template <std::size_t N>
Mat<N, N> inverse(Mat<N, N> m) {
    Mat<N, N> inv = Mat<N, N>::identity();
    for (std::size_t col = 0; col < N; ++col) {
        std::size_t pivot = col;
        for (std::size_t r = col + 1; r < N; ++r)
            if (std::abs(m(r, col)) > std::abs(m(pivot, col))) pivot = r;
        if (std::abs(m(pivot, col)) < 1e-300) throw std::runtime_error("singular matrix");
        if (pivot != col)
            for (std::size_t c = 0; c < N; ++c) {
                std::swap(m(col, c), m(pivot, c));
                std::swap(inv(col, c), inv(pivot, c));
            }
        const double p = m(col, col);
        for (std::size_t c = 0; c < N; ++c) {
            m(col, c) /= p;
            inv(col, c) /= p;
        }
        for (std::size_t r = 0; r < N; ++r) {
            if (r == col) continue;
            const double f = m(r, col);
            if (f == 0.0) continue;
            for (std::size_t c = 0; c < N; ++c) {
                m(r, c) -= f * m(col, c);
                inv(r, c) -= f * inv(col, c);
            }
        }
    }
    return inv;
}

}  // namespace fusion
