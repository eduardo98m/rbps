#include "rbc/narrow_phase.hpp"
#include "rbc/gjk/GJK.hpp"
#include "rbc/gjk/EPA.hpp"
#include "rbc/Dispatcher.hpp"

namespace rbc
{

    bool test_narrow_phase(const Shape &shape_a, const m3d::tf &tf_a,
                           const Shape &shape_b, const m3d::tf &tf_b,
                           rbc::Contact &out_contact)
    {
        return dispatch(shape_a, tf_a, shape_b, tf_b, out_contact);
    }

} // namespace rbc