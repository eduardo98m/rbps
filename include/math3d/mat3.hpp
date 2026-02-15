#pragma once
#include "vec3.hpp"
#include "quat.hpp"
#include <iomanip>

namespace m3d
{

    struct mat3
    {
        vec3 cols[3]; // Column-major

        mat3()
        {
            cols[0] = {1, 0, 0};
            cols[1] = {0, 1, 0};
            cols[2] = {0, 0, 1};
        }

        mat3(scalar m00, scalar m01, scalar m02,
             scalar m10, scalar m11, scalar m12,
             scalar m20, scalar m21, scalar m22)
        {
            cols[0] = {m00, m10, m20};
            cols[1] = {m01, m11, m21};
            cols[2] = {m02, m12, m22};
        }

        vec3 &operator[](int i) { return cols[i]; }
        const vec3 &operator[](int i) const { return cols[i]; }

        vec3 operator*(const vec3 &v) const
        {
            return {
                cols[0].x * v.x + cols[1].x * v.y + cols[2].x * v.z,
                cols[0].y * v.x + cols[1].y * v.y + cols[2].y * v.z,
                cols[0].z * v.x + cols[1].z * v.y + cols[2].z * v.z};
        }

        mat3 operator*(const mat3 &rhs) const
        {
            mat3 res;
            for (int c = 0; c < 3; c++)
            {
                res[c] = (*this) * rhs[c];
            }
            return res;
        }

        bool operator==(const mat3 &other) const
        {
            return cols[0] == other[0] && cols[1] == other[1] && cols[2] == other[2];
        }

        bool is_approx(const mat3 &other, scalar precision = EPSILON) const
        {
            return cols[0].is_approx(other[0], precision) &&
                   cols[1].is_approx(other[1], precision) &&
                   cols[2].is_approx(other[2], precision);
        }

        mat3 transpose() const
        {
            return mat3(
                cols[0].x, cols[0].y, cols[0].z,
                cols[1].x, cols[1].y, cols[1].z,
                cols[2].x, cols[2].y, cols[2].z);
        }

        scalar determinant() const
        {
            return cols[0].x * (cols[1].y * cols[2].z - cols[1].z * cols[2].y) -
                   cols[1].x * (cols[0].y * cols[2].z - cols[0].z * cols[2].y) +
                   cols[2].x * (cols[0].y * cols[1].z - cols[0].z * cols[1].y);
        }

        mat3 inverse() const
        {
            scalar det = this->determinant();
            if (m3d::abs(det) < EPSILON)
                return mat3(); // Return identity on failure

            scalar invDet = 1.0 / det;
            mat3 res;
            res[0].x = (cols[1].y * cols[2].z - cols[1].z * cols[2].y) * invDet;
            res[0].y = (cols[0].z * cols[2].y - cols[0].y * cols[2].z) * invDet;
            res[0].z = (cols[0].y * cols[1].z - cols[0].z * cols[1].y) * invDet;
            res[1].x = (cols[1].z * cols[2].x - cols[1].x * cols[2].z) * invDet;
            res[1].y = (cols[0].x * cols[2].z - cols[0].z * cols[2].x) * invDet;
            res[1].z = (cols[1].x * cols[0].z - cols[0].x * cols[1].z) * invDet;
            res[2].x = (cols[1].x * cols[2].y - cols[1].y * cols[2].x) * invDet;
            res[2].y = (cols[0].y * cols[2].x - cols[0].x * cols[2].y) * invDet;
            res[2].z = (cols[0].x * cols[1].y - cols[1].x * cols[0].y) * invDet;
            return res;
        }

        friend std::ostream &operator<<(std::ostream &os, const mat3 &m)
        {
            os << "\n";
            for (int r = 0; r < 3; r++)
            {
                os << "| " << std::setw(8) << m[0][r] << " "
                   << std::setw(8) << m[1][r] << " "
                   << std::setw(8) << m[2][r] << " |\n";
            }
            return os;
        }
    };

    inline mat3 mat3_cast(const quat &q)
    {
        mat3 m;
        scalar x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
        scalar xx = q.x * x2, xy = q.x * y2, xz = q.x * z2;
        scalar yy = q.y * y2, yz = q.y * z2, zz = q.z * z2;
        scalar wx = q.w * x2, wy = q.w * y2, wz = q.w * z2;

        m[0] = {1 - (yy + zz), xy + wz, xz - wy};
        m[1] = {xy - wz, 1 - (xx + zz), yz + wx};
        m[2] = {xz + wy, yz - wx, 1 - (xx + yy)};
        return m;
    }

    struct smat3
    {
        scalar xx, yy, zz;
        scalar xy, xz, yz;

        smat3() : xx(0), yy(0), zz(0), xy(0), xz(0), yz(0) {}
        smat3(scalar xx, scalar yy, scalar zz, scalar xy, scalar xz, scalar yz)
            : xx(xx), yy(yy), zz(zz), xy(xy), xz(xz), yz(yz) {}

        explicit smat3(const mat3 &m)
            : xx(m[0].x), yy(m[1].y), zz(m[2].z), xy(0.5 * (m[1].x + m[0].y)) // Average off-diagonal elements
              ,
              xz(0.5 * (m[2].x + m[0].z)), yz(0.5 * (m[2].y + m[1].z))
        {
        }

        vec3 operator*(const vec3 &v) const
        {
            return {
                xx * v.x + xy * v.y + xz * v.z,
                xy * v.x + yy * v.y + yz * v.z,
                xz * v.x + yz * v.y + zz * v.z};
        }

        smat3 operator*(scalar s) const
        {
            return smat3(xx * s, yy * s, zz * s, xy * s, xz * s, yz * s);
        }

        bool is_approx(const smat3 &o, scalar p = EPSILON) const
        {
            return m3d::abs(xx - o.xx) < p && m3d::abs(yy - o.yy) < p && m3d::abs(zz - o.zz) < p &&
                   m3d::abs(xy - o.xy) < p && m3d::abs(xz - o.xz) < p && m3d::abs(yz - o.yz) < p;
        }

        friend std::ostream &operator<<(std::ostream &os, const smat3 &m)
        {
            return os << "smat3(diag: " << m.xx << "," << m.yy << "," << m.zz
                      << " off: " << m.xy << "," << m.xz << "," << m.yz << ")";
        }
    };

    // Free function for scalar * smat3 (commutative)
    inline smat3 operator*(scalar s, const smat3 &m)
    {
        return m * s;
    }

    // mat3 * smat3 multiplication
    inline mat3 operator*(const mat3 &lhs, const smat3 &rhs)
    {
        // Expand smat3 to full matrix and multiply
        // rhs as matrix is:
        // | xx  xy  xz |
        // | xy  yy  yz |
        // | xz  yz  zz |

        mat3 result;
        // Each column of result = lhs * column of rhs
        result[0] = lhs * vec3{rhs.xx, rhs.xy, rhs.xz};
        result[1] = lhs * vec3{rhs.xy, rhs.yy, rhs.yz};
        result[2] = lhs * vec3{rhs.xz, rhs.yz, rhs.zz};
        return result;
    }
}