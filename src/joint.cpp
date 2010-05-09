#include "sim/joint.hpp"

namespace sim {

Joint::Joint(Body *oA, Body *oB)
   : _o_a(oA), _o_b(oB)
{
}

Joint::~Joint()
{
    if (_o_a)
        delete _o_a;
    if (_o_b)
        delete _o_b;
}

}
