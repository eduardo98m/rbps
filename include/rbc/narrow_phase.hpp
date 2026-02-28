#pragma once

#include "rbc/shapes/ShapeTypes.hpp"
#include "rbc/gjk/MinkowskiDiff.hpp"
#include "math3d/tf.hpp"
#include "rbc/Contact.hpp" 

namespace rbc {

    // Performs narrow-phase collision detection between two shapes.
    // Returns true if they are colliding, and populates 'out_contact'.
    bool test_narrow_phase(const Shape& shape_a, const m3d::tf& tf_a,
                           const Shape& shape_b, const m3d::tf& tf_b,
                           rbc::Contact& out_contact);

} // namespace rbc