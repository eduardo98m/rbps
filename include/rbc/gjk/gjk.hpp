#pragma once

#include "math3d/math3d.hpp"
#include <vector>
#include <limits>
#include <stdexcept>

namespace rbc
{
    // Forward declaration for your collision shape base class
    struct CollisionGeometry;

    /// @brief Initial guess to use for the GJK algorithm
    enum GJKInitialGuess
    {
        DefaultGuess,
        CachedGuess,
        BoundingVolumeGuess
    };

    /// @brief Variant to use for the GJK algorithm
    enum GJKVariant
    {
        DefaultGJK,
        NesterovAcceleration
    };

    /// @brief Which convergence criterion is used to stop the algorithm
    enum GJKConvergenceCriterion
    {
        VDB,
        DualityGap,
        Hybrid
    };

    /// @brief Whether the convergence criterion is scaled on the norm of the solution or not
    enum GJKConvergenceCriterionType
    {
        Relative,
        Absolute
    };

    /// @brief Flag declaration for specifying required params in CollisionResult
    enum CollisionRequestFlag
    {
        CONTACT = 0x00001,
        DISTANCE_LOWER_BOUND = 0x00002,
        NO_REQUEST = 0x01000
    };

    // --- Query Result Base ---

    /// @brief Base class for all query results
    struct QueryResult
    {
        /// @brief stores the last GJK ray when relevant.
        m3d::vec3 cached_gjk_guess;

        /// @brief stores the last support function vertex index, when relevant.
        m3d::vec2i cached_support_func_guess;

        QueryResult()
            : cached_gjk_guess(0, 0, 0), // Assuming m3d::vec3(x,y,z) exists
              cached_support_func_guess(-1)
        {
        } // Using your m3d::vec2i(int) constructor
    };

    // --- Query Request Base ---

    /// @brief Base class for all query requests
    struct QueryRequest
    {
        GJKInitialGuess gjk_initial_guess;
        GJKVariant gjk_variant;
        GJKConvergenceCriterion gjk_convergence_criterion;
        GJKConvergenceCriterionType gjk_convergence_criterion_type;

        m3d::scalar gjk_tolerance;
        size_t gjk_max_iterations;

        m3d::vec3 cached_gjk_guess;
        m3d::vec2i cached_support_func_guess;
        m3d::scalar collision_distance_threshold;

        QueryRequest()
            : gjk_initial_guess(GJKInitialGuess::DefaultGuess),
              gjk_variant(GJKVariant::DefaultGJK),
              gjk_convergence_criterion(GJKConvergenceCriterion::VDB),
              gjk_convergence_criterion_type(GJKConvergenceCriterionType::Relative),
              gjk_tolerance(1e-6),
              gjk_max_iterations(128),
              cached_gjk_guess(1, 0, 0),
              cached_support_func_guess(0), // Uses m3d::vec2i(0)
              collision_distance_threshold(1e-6)
        {
        } // Replaced Eigen dummy_precision

        QueryRequest(const QueryRequest &other) = default;
        QueryRequest &operator=(const QueryRequest &other) = default;

        inline void updateGuess(const QueryResult &result)
        {
            if (gjk_initial_guess == GJKInitialGuess::CachedGuess)
            {
                cached_gjk_guess = result.cached_gjk_guess;
                cached_support_func_guess = result.cached_support_func_guess;
            }
        }

        inline bool operator==(const QueryRequest &other) const
        {
            return gjk_initial_guess == other.gjk_initial_guess &&
                   cached_gjk_guess == other.cached_gjk_guess && // Ensure m3d::vec3 has operator==
                   cached_support_func_guess == other.cached_support_func_guess;
        }
    };

    // --- Contact ---

    /// @brief Contact information returned by collision
    struct Contact
    {
        const CollisionGeometry *o1;
        const CollisionGeometry *o2;
        int b1;
        int b2;
        m3d::vec3 normal;
        m3d::vec3 pos;
        m3d::scalar penetration_depth;

        static const int NONE = -1;

        Contact() : o1(nullptr), o2(nullptr), b1(NONE), b2(NONE) {}

        Contact(const CollisionGeometry *o1_, const CollisionGeometry *o2_, int b1_, int b2_)
            : o1(o1_), o2(o2_), b1(b1_), b2(b2_) {}

        Contact(const CollisionGeometry *o1_, const CollisionGeometry *o2_, int b1_,
                int b2_, const m3d::vec3 &pos_, const m3d::vec3 &normal_, m3d::scalar depth_)
            : o1(o1_), o2(o2_), b1(b1_), b2(b2_), normal(normal_), pos(pos_), penetration_depth(depth_) {}
    };

    // --- Collision Request ---

    /// @brief Request to the collision algorithm
    struct CollisionRequest : QueryRequest
    {
        size_t num_max_contacts;
        bool enable_contact;
        bool enable_distance_lower_bound;
        m3d::scalar security_margin;
        m3d::scalar break_distance;
        m3d::scalar distance_upper_bound;

        CollisionRequest(const CollisionRequestFlag flag, size_t num_max_contacts_)
            : num_max_contacts(num_max_contacts_),
              enable_contact(flag & CONTACT),
              enable_distance_lower_bound(flag & DISTANCE_LOWER_BOUND),
              security_margin(0),
              break_distance(1e-3),
              distance_upper_bound(std::numeric_limits<m3d::scalar>::max()) {}

        CollisionRequest()
            : num_max_contacts(1),
              enable_contact(false),
              enable_distance_lower_bound(false),
              security_margin(0),
              break_distance(1e-3),
              distance_upper_bound(std::numeric_limits<m3d::scalar>::max()) {}
    };

    // --- Collision Result ---

    /// @brief Collision result
    struct CollisionResult : QueryResult
    {
    private:
        std::vector<Contact> contacts;

    public:
        m3d::scalar distance_lower_bound;
        m3d::vec3 nearest_points[2];

        CollisionResult()
            : distance_lower_bound(std::numeric_limits<m3d::scalar>::max()) {}

        inline void updateDistanceLowerBound(const m3d::scalar &distance_lower_bound_)
        {
            if (distance_lower_bound_ < distance_lower_bound)
                distance_lower_bound = distance_lower_bound_;
        }

        inline void addContact(const Contact &c) { contacts.push_back(c); }

        bool isCollision() const { return !contacts.empty(); }
        size_t numContacts() const { return contacts.size(); }

        const Contact &getContact(size_t i) const
        {
            if (contacts.empty())
            {
                throw std::invalid_argument("The number of contacts is zero.");
            }
            return (i < contacts.size()) ? contacts[i] : contacts.back();
        }

        const std::vector<Contact> &getContacts() const { return contacts; }

        void clear()
        {
            contacts.clear();
            distance_lower_bound = std::numeric_limits<m3d::scalar>::max();
            // Reset nearest points if necessary, depending on your vec3 defaults
        }
    };

    // Ok

    struct CollisionShape
    {
        virtual ~CollisionShape() = default;
        // The core of GJK/EPA: furthest point in a direction (local space)
        virtual m3d::vec3 getSupport(const m3d::vec3 &direction) const = 0;
    };

    struct tf
    {
        m3d::quat rot;
        m3d::vec3 pos;

        // Helper to transform a point from local to world space
        m3d::vec3 transform(const m3d::vec3 &p) const
        {
            return pos + (m3d::rotate(rot, p));
        }

        // Helper to transform a direction (rot only)
        m3d::vec3 rotate(const m3d::vec3 &d) const
        {
            return m3d::rotate(rot, d);
        }

        // FCL uses inverseTimes frequently ( tf1.inv() * tf2 )
        // This represents tf2 expressed in the local frame of tf1
        tf inverseTimes(const tf &other) const
        {
            m3d::quat invRot = m3d::conjugate(rot);
            return {
                invRot * other.rot,
                m3d::rotate(invRot, other.pos - pos)};
        }
    };

    struct MinkowskiDiff
    {
        const CollisionShape *s1, *s2;
        tf tf1, tf2;

        // The core of GJK: The Support Function of the Difference
        m3d::vec3 support(const m3d::vec3 &dir) const
        {
            // furthest point of A in dir - furthest point of B in -dir
            // Note: we must transform direction into local space, get support,
            // then transform back to world space.
            return tf1.transform(s1->getSupport(tf1.rotateInverse(dir))) -
                   tf2.transform(s2->getSupport(tf2.rotateInverse(-dir)));
        }
    };

    bool shapeIntersect(const CollisionShape &s1, const tf &tf1,
                        const CollisionShape &s2, const tf &tf2,
                        CollisionResult &result)
    {

        // 1. Setup the Minkowski Difference
        // This is a "virtual" shape where every point is P = support(A, d) - support(B, -d)
        MinkowskiDiff shape(&s1, &s2, tf1, tf2);

        // 2. Run GJK
        GJK gjk;
        GJK::Status status = gjk.evaluate(shape, m3d::vec3(1, 0, 0));

        if (status == GJK::Inside)
        {
            // 3. Collision detected! Use EPA to get penetration info
            EPA epa;
            EPA::Status epa_status = epa.evaluate(gjk, shape);

            if (epa_status == EPA::Valid)
            {
                Contact contact;
                contact.normal = epa.normal; // The push-out direction
                contact.depth = epa.depth;   // How far they overlap

                // Calculate contact point (halfway in the overlap)
                contact.pos = epa.contact_point;

                result.addContact(contact);
                return true;
            }
        }

        return false; // No collision
    }

} // namespace rbc