#include "sim/body.hpp"

namespace sim {

Body::Body()
    : _vis(0), _pos(0., 0., 0.), _rot(0., 0., 0., 1.)
{
}

Body::~Body()
{
    if (_vis)
        delete _vis;
}

}
