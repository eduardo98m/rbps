#pragma once
#include <cstdint>
#include "rbc/shapes/Sphere.hpp"
#include "rbc/shapes/Box.hpp"

namespace rbc
{
#define RBC_SHAPE_LIST(X, ...)       \
    X(Sphere, sphere, ##__VA_ARGS__) \
    X(Box, box, ##__VA_ARGS__)

#define RBC_SHAPE_LIST_INNER(X, ...) \
    X(Sphere, sphere, ##__VA_ARGS__) \
    X(Box, box, ##__VA_ARGS__)

    // Automatically generate the enum, struct, union, and support switch based on the master list
    enum struct ShapeType : uint8_t
    {
#define GENERATE_ENUM(Type, name) Type,
        RBC_SHAPE_LIST(GENERATE_ENUM)
#undef GENERATE_ENUM
            Count // Utility value to know how many shapes we have (Useful for generating the function matrix)
    };

    // Automatically generate the union of shape data
    struct Shape
    {
        ShapeType type;

        union
        {
#define GENERATE_UNION(Type, name) Type name;
            RBC_SHAPE_LIST(GENERATE_UNION)
#undef GENERATE_UNION
        };

        Shape() {} // Defualt constructor (For list initialization)

// Generate constructors for each shape type for easy initialization
#define GENERATE_CONSTRUCTOR(Type, name) \
    Shape(const Type &s) : type(ShapeType::Type), name(s) {}
        RBC_SHAPE_LIST(GENERATE_CONSTRUCTOR)
#undef GENERATE_CONSTRUCTOR

        // Utility function so the matrix extracts the correct shape
        template <typename T>
        const T &get() const;
    };

// Getter specializations for each shape type (Generated automatically)
#define GENERATE_GETTER(Type, name) \
    template <>                     \
    inline const Type &Shape::get<Type>() const { return name; }
    RBC_SHAPE_LIST(GENERATE_GETTER)
#undef GENERATE_GETTER

    // Automatically generate the support switch
    inline m3d::vec3 shape_support(const Shape &shape, const m3d::vec3 &dir)
    {
        switch (shape.type)
        {
#define GENERATE_SUPPORT(Type, name) \
    case ShapeType::Type:            \
        return support(shape.name, dir);
            RBC_SHAPE_LIST(GENERATE_SUPPORT)
#undef GENERATE_SUPPORT
        default:
            return m3d::vec3();
        }
    }

    // -------------------------------------------------------------------------
    //  Generic AABB dispatch — lives here because this is the first place where
    //  ALL shape types and their compute_aabb() overloads are visible.
    //  Sphere.hpp and Box.hpp define their own overloads; we just dispatch here.
    // -------------------------------------------------------------------------
    inline AABB compute_aabb(const Shape &shape, const m3d::tf &tf)
    {
        switch (shape.type)
        {
#define GEN_AABB_CASE(Type, name) \
    case ShapeType::Type:         \
        return compute_aabb(shape.name, tf);
            RBC_SHAPE_LIST(GEN_AABB_CASE)
#undef GEN_AABB_CASE
        default:
            return AABB{};
        }
    }

    // Check that both lists have the same number of entries (Compile-time assertion)
    namespace test_shape_list
    {
#define COUNT_ELEMENTS(Type, Name, ...) +1

        // We calculate the length of both lists separately.
        constexpr size_t TotalShapesOuter = 0 RBC_SHAPE_LIST(COUNT_ELEMENTS);
        constexpr size_t TotalShapesInner = 0 RBC_SHAPE_LIST_INNER(COUNT_ELEMENTS);

        static_assert(TotalShapesOuter == TotalShapesInner,
                      "CRITICAL DESYNC: RBC_SHAPE_LIST and RBC_SHAPE_LIST_INNER must have the same entries!");

        // Also ensure the Enum matches the list (sanity check)
        static_assert(TotalShapesOuter == static_cast<size_t>(ShapeType::Count),
                      "CRITICAL: ShapeType::Count does not match RBC_SHAPE_LIST length.");
    } // End of shape list sanity checks
} // namespace rbc