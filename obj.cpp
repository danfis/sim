#include "obj.hpp"

namespace sim {

Obj::Obj()
    : _vis(0), _phys(0)
{
}

Obj::~Obj()
{
    if (_vis)
        delete _vis;
    if (_phys)
        delete _phys;
}

}
