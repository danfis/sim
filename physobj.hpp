#ifndef _SIM_PHYS_OBJ_HPP_
#define _SIM_PHYS_OBJ_HPP_

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>

namespace sim {

/**
 * Physical representation of object.
 */
class PhysObj {
    btRigidBody *_body;
    btCollisionShape *_shape;

  public:
    PhysObj();
    virtual ~PhysObj();

    btRigidBody *body() { return _body; }
    const btRigidBody *body() const { return _body; }
    btCollisionShape *shape() { return _shape; }
    const btCollisionShape *shape() const { return _shape; }

  protected:
    void _setBody(btRigidBody *body) { _body = body; }
    void _setShape(btCollisionShape *shape) { _shape = shape; }
};


}

#endif /* _SIM_PHYS_OBJ_HPP_ */
