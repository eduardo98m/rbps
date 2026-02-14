#pragma once
#include "vec3.hpp"
#include "quat.hpp"
#include <iomanip>

namespace m3d
{

    struct SymMat3; // Forward declaration

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

        static mat3 transpose(const mat3 &m)
        {
            return mat3(
                m[0].x, m[0].y, m[0].z,
                m[1].x, m[1].y, m[1].z,
                m[2].x, m[2].y, m[2].z);
        }

        scalar determinant() const
        {
            return cols[0].x * (cols[1].y * cols[2].z - cols[1].z * cols[2].y) -
                   cols[1].x * (cols[0].y * cols[2].z - cols[0].z * cols[2].y) +
                   cols[2].x * (cols[0].y * cols[1].z - cols[0].z * cols[1].y);
        }

        static mat3 inverse(const mat3 &m)
        {
            scalar det = m.determinant();
            if (std::abs(det) < EPSILON)
                return mat3(); // Return identity on failure

            scalar invDet = 1.0 / det;
            mat3 res;
            res[0].x = (m[1].y * m[2].z - m[1].z * m[2].y) * invDet;
            res[0].y = (m[0].z * m[2].y - m[0].y * m[2].z) * invDet;
            res[0].z = (m[0].y * m[1].z - m[0].z * m[1].y) * invDet;
            res[1].x = (m[1].z * m[2].x - m[1].x * m[2].z) * invDet;
            res[1].y = (m[0].x * m[2].z - m[0].z * m[2].x) * invDet;
            res[1].z = (m[1].x * m[0].z - m[0].x * m[1].z) * invDet;
            res[2].x = (m[1].x * m[2].y - m[1].y * m[2].x) * invDet;
            res[2].y = (m[0].y * m[2].x - m[0].x * m[2].y) * invDet;
            res[2].z = (m[0].x * m[1].y - m[1].x * m[0].y) * invDet;
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

    struct SymMat3
    {
        scalar xx, yy, zz;
        scalar xy, xz, yz;

        SymMat3() : xx(0), yy(0), zz(0), xy(0), xz(0), yz(0) {}
        SymMat3(scalar xx, scalar yy, scalar zz, scalar xy, scalar xz, scalar yz)
            : xx(xx), yy(yy), zz(zz), xy(xy), xz(xz), yz(yz) {}

        vec3 operator*(const vec3 &v) const
        {
            return {
                xx * v.x + xy * v.y + xz * v.z,
                xy * v.x + yy * v.y + yz * v.z,
                xz * v.x + yz * v.y + zz * v.z};
        }

        bool is_approx(const SymMat3 &o, scalar p = EPSILON) const
        {
            return std::abs(xx - o.xx) < p && std::abs(yy - o.yy) < p && std::abs(zz - o.zz) < p &&
                   std::abs(xy - o.xy) < p && std::abs(xz - o.xz) < p && std::abs(yz - o.yz) < p;
        }

        friend std::ostream &operator<<(std::ostream &os, const SymMat3 &m)
        {
            return os << "SymMat3(diag: " << m.xx << "," << m.yy << "," << m.zz
                      << " off: " << m.xy << "," << m.xz << "," << m.yz << ")";
        }
    };
}