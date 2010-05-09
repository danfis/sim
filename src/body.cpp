#include "sim/body.hpp"

namespace sim {

Body::Body()
    : _vis(0)
{
}

Body::~Body()
{
    if (_vis)
        delete _vis;
}

}
