#ifndef _SIM_OBJ_HPP_
#define _SIM_OBJ_HPP_

#include "obj_base.hpp"

namespace sim {

class Obj {
    VisObj *_vis;
    PhysObj *_phys;

  public:
    Obj();
    virtual ~Obj();

    VisObj *visObj() { return _vis; }
    PhysObj *physObj() { return _phys; }

    virtual void build() = 0;

  protected:
    void setVisObj(VisObj *o) { _vis = o; }
    void setPhysObj(PhysObj *o) { _phys = o; }
};

}

#endif /* _SIM_OBJ_HPP_ */
