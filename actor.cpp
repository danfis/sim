#include "actor.hpp"
#include "msg.hpp"

namespace sim {

Actor::~Actor()
{
    std::list<Body *>::iterator bit, bit_end;
    std::list<Joint *>::iterator jit, jit_end;

    bit = _bodies.begin();
    bit_end = _bodies.end();
    for (; bit != bit_end; ++bit){
        delete *bit;
    }
    _bodies.clear();

    jit = _joints.begin();
    jit_end = _joints.end();
    for (; jit != jit_end; ++jit){
        delete *jit;
    }
    _joints.clear();
}

void Actor::setPos(const Vec3 &v)
{
    std::list<Body *>::iterator bit, bit_end;

    bit = _bodies.begin();
    bit_end = _bodies.end();
    for (; bit != bit_end; ++bit){
        (*bit)->setPos(v);
    }
}


void Actor::_addBody(Body *b)
{
    _bodies.push_back(b);
}

void Actor::_addJoint(Joint *b)
{
    _joints.push_back(b);
}


Robot1::Robot1()
    : Actor()
{
    sim::BodyBox *chasis;
    sim::BodyCylinderX *w[4];
    sim::JointHinge2 *j[4];
    size_t i;

    chasis = new sim::BodyBox(sim::Vec3(.6, 1., 0.4), 1.);
    for (i = 0; i < 4; i++){
        w[i] = new sim::BodyCylinderX(0.2, 0.2, 1.);
    }

    w[0]->setOffsetPos(0.405, 0.4, -0.2);
    w[1]->setOffsetPos(-0.405, 0.4, -0.2);
    w[2]->setOffsetPos(0.405, -0.4, -0.2);
    w[3]->setOffsetPos(-0.405, -0.4, -0.2);

    for (i = 0; i < 4; i++){
        j[i] = new sim::JointHinge2(chasis, w[i], w[i]->pos(),
                                    sim::Vec3(0., 0., 1.), sim::Vec3(1., 0., 0.));
    }

    _addBody(chasis);
    for (i = 0; i < 4; i++){
        _addBody(w[i]);
        _addJoint(j[i]);
    }
}

}
