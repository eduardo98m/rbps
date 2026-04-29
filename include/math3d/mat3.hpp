#pragma once
#include "vec3.hpp"
#include "quat.hpp"
#include <iomanip>

/**
 * @file mat3.hpp
 * @brief 3×3 matrices: dense `mat3` and symmetric `smat3`.
 * @ingroup math3d
 */

namespace m3d
{

    /**
     * @brief Dense 3×3 matrix in column-major layout.
     *
     * Used for general linear transforms: rotation matrices, basis changes,
     * and as the dense form of the inertia tensor when needed by code that
     * doesn't exploit symmetry.
     *
     * The element-wise constructor takes arguments in **row-major reading
     * order** (`m00, m01, m02, m10, ...`) but stores them column-major; this
     * matches how matrices are typically written down on paper.
     *
     * @code
     * mat3 R = mat3_cast(quat::from_axis_angle({0,1,0}, PI/2));
     * vec3 r = R * vec3{1, 0, 0};   // rotated point
     * @endcode
     *
     * @ingroup math3d
     */
    struct mat3
    {
        vec3 cols[3]; ///< Column-major storage; `cols[i]` is the i-th column.

        /** @brief Identity matrix. */
        mat3()
        {
            cols[0] = {1, 0, 0};
            cols[1] = {0, 1, 0};
            cols[2] = {0, 0, 1};
        }

        /**
         * @brief Element-wise constructor in row-major reading order.
         *
         * Parameters are named `mRC` where R is the row and C is the column.
         */
        mat3(scalar m00, scalar m01, scalar m02,
             scalar m10, scalar m11, scalar m12,
             scalar m20, scalar m21, scalar m22)
        {
            cols[0] = {m00, m10, m20};
            cols[1] = {m01, m11, m21};
            cols[2] = {m02, m12, m22};
        }

        /** @brief Indexed mutable column access. */
        vec3 &operator[](int i) { return cols[i]; }
        /** @brief Indexed const column access. */
        const vec3 &operator[](int i) const { return cols[i]; }

        /** @brief Matrix–vector product (`M * v`). */
        vec3 operator*(const vec3 &v) const
        {
            return {
                cols[0].x * v.x + cols[1].x * v.y + cols[2].x * v.z,
                cols[0].y * v.x + cols[1].y * v.y + cols[2].y * v.z,
                cols[0].z * v.x + cols[1].z * v.y + cols[2].z * v.z};
        }

        /** @brief Matrix–matrix product (`(*this) * rhs`). */
        mat3 operator*(const mat3 &rhs) const
        {
            mat3 res;
            for (int c = 0; c < 3; c++)
            {
                res[c] = (*this) * rhs[c];
            }
            return res;
        }

        /** @brief Bitwise-exact equality. Use `is_approx` for tolerance. */
        bool operator==(const mat3 &other) const
        {
            return cols[0] == other[0] && cols[1] == other[1] && cols[2] == other[2];
        }

        /**
         * @brief Tolerance-based equality (per-element).
         * @param other     Matrix to compare against.
         * @param precision Absolute per-element tolerance (default `EPSILON`).
         */
        bool is_approx(const mat3 &other, scalar precision = EPSILON) const
        {
            return cols[0].is_approx(other[0], precision) &&
                   cols[1].is_approx(other[1], precision) &&
                   cols[2].is_approx(other[2], precision);
        }

        /** @brief Transpose. */
        mat3 transpose() const
        {
            return mat3(
                cols[0].x, cols[0].y, cols[0].z,
                cols[1].x, cols[1].y, cols[1].z,
                cols[2].x, cols[2].y, cols[2].z);
        }

        /** @brief 3×3 determinant. */
        scalar determinant() const
        {
            return cols[0].x * (cols[1].y * cols[2].z - cols[1].z * cols[2].y) -
                   cols[1].x * (cols[0].y * cols[2].z - cols[0].z * cols[2].y) +
                   cols[2].x * (cols[0].y * cols[1].z - cols[0].z * cols[1].y);
        }

        /**
         * @brief Inverse via classical adjugate.
         *
         * Returns the identity matrix when the determinant is below `EPSILON`,
         * to keep callers from having to special-case singular inputs.
         */
        mat3 inverse() const
        {
            scalar det = this->determinant();
            if (m3d::abs(det) < EPSILON)
                return mat3();

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

        /** @brief Pretty-printed stream insertion (3 rows in `| a b c |` form). */
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

    /**
     * @brief Convert a unit quaternion to its equivalent rotation `mat3`.
     *
     * @param q  Unit quaternion. The result is undefined if `q` is far from unit length.
     * @return The 3×3 rotation matrix that performs the same rotation as `q`.
     * @ingroup math3d
     */
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

    /**
     * @brief Symmetric 3×3 matrix stored as 6 unique scalars.
     *
     * Used for the inertia tensor (which is always symmetric). Storing only
     * the upper triangle saves memory and lets the operators exploit symmetry
     * for slightly cheaper math.
     *
     * Layout when expanded to a dense matrix:
     * ```
     * | xx  xy  xz |
     * | xy  yy  yz |
     * | xz  yz  zz |
     * ```
     *
     * @ingroup math3d
     */
    struct smat3
    {
        scalar xx, yy, zz; ///< Diagonal entries.
        scalar xy, xz, yz; ///< Upper-triangle off-diagonal entries.

        /** @brief Zero matrix. */
        smat3() : xx(0), yy(0), zz(0), xy(0), xz(0), yz(0) {}
        /** @brief Component-wise constructor (diagonal first, then upper-triangle). */
        smat3(scalar xx, scalar yy, scalar zz, scalar xy, scalar xz, scalar yz)
            : xx(xx), yy(yy), zz(zz), xy(xy), xz(xz), yz(yz) {}

        /**
         * @brief Construct from a dense `mat3`, projecting it onto the symmetric subspace.
         *
         * Off-diagonal entries are averaged (`(m_ij + m_ji) / 2`) so a slightly
         * non-symmetric input is silently cleaned up rather than preserving
         * one side and discarding the other.
         */
        explicit smat3(const mat3 &m)
            : xx(m[0].x), yy(m[1].y), zz(m[2].z), xy(0.5 * (m[1].x + m[0].y)),
              xz(0.5 * (m[2].x + m[0].z)), yz(0.5 * (m[2].y + m[1].z))
        {
        }

        /** @brief Symmetric matrix–vector product. */
        vec3 operator*(const vec3 &v) const
        {
            return {
                xx * v.x + xy * v.y + xz * v.z,
                xy * v.x + yy * v.y + yz * v.z,
                xz * v.x + yz * v.y + zz * v.z};
        }

        /** @brief Component-wise scaling. */
        smat3 operator*(scalar s) const
        {
            return smat3(xx * s, yy * s, zz * s, xy * s, xz * s, yz * s);
        }

        /** @brief 3×3 determinant via the symmetric formula. */
        scalar determinant() const
        {
            return xx * yy * zz + 2 * xy * xz * yz - xx * yz * yz - yy * xz * xz - zz * xy * xy;
        }

        /**
         * @brief Inverse, preserved as a symmetric matrix.
         *
         * Returns the zero matrix when the determinant is below `EPSILON`.
         */
        smat3 inverse() const
        {
            scalar det = determinant();

            if (m3d::abs(det) < EPSILON)
                return smat3();

            scalar invDet = 1.0 / det;

            smat3 result;

            result.xx = (yy * zz - yz * yz) * invDet;
            result.yy = (xx * zz - xz * xz) * invDet;
            result.zz = (xx * yy - xy * xy) * invDet;

            result.xy = (xz * yz - xy * zz) * invDet;
            result.xz = (xy * yz - xz * yy) * invDet;
            result.yz = (xy * xz - xx * yz) * invDet;

            return result;
        }

        /**
         * @brief Tolerance-based equality (per-element).
         * @param o Matrix to compare against.
         * @param p Absolute per-element tolerance (default `EPSILON`).
         */
        bool is_approx(const smat3 &o, scalar p = EPSILON) const
        {
            return m3d::abs(xx - o.xx) < p && m3d::abs(yy - o.yy) < p && m3d::abs(zz - o.zz) < p &&
                   m3d::abs(xy - o.xy) < p && m3d::abs(xz - o.xz) < p && m3d::abs(yz - o.yz) < p;
        }

        /** @brief Stream insertion summarising the diagonal and off-diagonal entries. */
        friend std::ostream &operator<<(std::ostream &os, const smat3 &m)
        {
            return os << "smat3(diag: " << m.xx << "," << m.yy << "," << m.zz
                      << " off: " << m.xy << "," << m.xz << "," << m.yz << ")";
        }
    };

    /** @brief Left-side scalar multiplication for `smat3`. @ingroup math3d */
    inline smat3 operator*(scalar s, const smat3 &m)
    {
        return m * s;
    }

    /**
     * @brief Mixed `mat3 * smat3` product, returning a dense `mat3`.
     *
     * Used when transforming the (symmetric) inertia tensor into world space:
     * `I_world = R * I_local * R^T`. The intermediate result is generally not
     * symmetric, so a dense matrix is returned.
     *
     * @ingroup math3d
     */
    inline mat3 operator*(const mat3 &lhs, const smat3 &rhs)
    {
        // Expand smat3 to its dense form on the fly:
        // | xx  xy  xz |
        // | xy  yy  yz |
        // | xz  yz  zz |
        mat3 result;
        result[0] = lhs * vec3{rhs.xx, rhs.xy, rhs.xz};
        result[1] = lhs * vec3{rhs.xy, rhs.yy, rhs.yz};
        result[2] = lhs * vec3{rhs.xz, rhs.yz, rhs.zz};
        return result;
    }

} // namespace m3d
