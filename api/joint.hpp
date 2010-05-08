#ifndef _SIM_JOINT_HPP_
#define _SIM_JOINT_HPP_

#include "body.hpp"

namespace sim {

/**
 * Class representing joint (ODE-like).
 */
class Joint {
  protected:
    Body *_o_a, *_o_b; //< First and second body connect by joint

  public:
    Joint(Body *oA, Body *oB) : _o_a(oA), _o_b(oB) {}
    ~Joint()
        { if (_o_a) delete _o_a;
          if (_o_b) delete _o_b; }

    /* \{ */
    Body *objA() { return _o_a; }
    const Body *objA() const { return _o_a; }
    Body *objB() { return _o_b; }
    const Body *objB() const { return _o_b; }
    /* \} */

    /* \{ */
    /**
     * Activates joint in world.
     */
    virtual void activate();

    /**
     * Deactivates (removes) joint in world.
     */
    void deactivate();
    /* \} */
};

/**
 * ODE's hinge2 joint.
 * For more info see:
 *  http://opende.sourceforge.net/wiki/index.php/Manual_(All)#Hinge-2
 */
class JointHinge2 : public Joint {
  public:
    JointHinge2(Body *oA, Body *oB) : Joint(oA, oB) {}

    virtual void setLimitLinAxis1(Scalar from, Scalar to) = 0;
    virtual void setLimitLinAxis2(Scalar from, Scalar to) = 0;
    virtual void setLimitAngAxis1(Scalar from, Scalar to) = 0;
    virtual void setLimitAngAxis2(Scalar from, Scalar to) = 0;
};

} /* namespace sim */

#endif /* _SIM_JOINT_HPP_ */
