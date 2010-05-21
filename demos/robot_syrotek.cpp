#include "robot_syrotek.hpp"
#include <sim/msg.hpp>
using sim::Vec3;
using sim::ode::BodyCompound;
using sim::Scalar;

#define SIZE(a) (sizeof(a) / sizeof(*a))

static Vec3 verts[] = { Vec3( 0.087,  0.045, 0.),
                        Vec3( 0.087, -0.045, 0.),
                        Vec3( 0.050, -0.081, 0.),
                        Vec3(-0.050, -0.081, 0.),
                        Vec3(-0.087, -0.045, 0.),
                        Vec3(-0.087,  0.045, 0.),
                        Vec3(-0.050,  0.081, 0.),
                        Vec3( 0.050,  0.081, 0.),
                        Vec3( 0.087,  0.045, 0.06),
                        Vec3( 0.087, -0.045, 0.06),
                        Vec3( 0.050, -0.081, 0.06),
                        Vec3(-0.050, -0.081, 0.06),
                        Vec3(-0.087, -0.045, 0.06),
                        Vec3(-0.087,  0.045, 0.06),
                        Vec3(-0.050,  0.081, 0.06),
                        Vec3( 0.050,  0.081, 0.06),

                        Vec3(0.075,  0.030, 0.06),
                        Vec3(0.075, -0.030, 0.06),
                        Vec3(0.024, -0.059, 0.06),
                        Vec3(-0.062, -0.059, 0.06),
                        Vec3(-0.074, -0.050, 0.06),
                        Vec3(-0.074,  0.050, 0.06),
                        Vec3(-0.062,  0.059, 0.06),
                        Vec3(0.024,  0.059, 0.06),
                        Vec3(0.075,  0.030, 0.18),
                        Vec3(0.075, -0.030, 0.18),
                        Vec3(0.024, -0.059, 0.18),
                        Vec3(-0.062, -0.059, 0.18),
                        Vec3(-0.074, -0.050, 0.18),
                        Vec3(-0.074,  0.050, 0.18),
                        Vec3(-0.062,  0.059, 0.18),
                        Vec3(0.024,  0.059, 0.18),
};
static unsigned int ids[] = { 0, 1, 2,
                              0, 2, 3,
                              0, 3, 4,
                              0, 4, 5,
                              0, 5, 6,
                              0, 6, 7,
                              8, 9, 10,
                              8, 10, 11,
                              8, 11, 12,
                              8, 12, 13,
                              8, 13, 14,
                              8, 14, 15,
                              0, 8, 9,
                              0, 9, 1,
                              1, 9, 10,
                              1, 10, 2,
                              2, 10, 11,
                              2, 11, 3,
                              3, 11, 12,
                              3, 12, 4,
                              4, 12, 13,
                              4, 13, 5,
                              5, 13, 14,
                              5, 14, 6,
                              6, 14, 15,
                              6, 15, 7,
                              7, 15, 8,
                              7, 8, 0,

                              /*
                                 16, 17, 18,
                                 16, 18, 19,
                                 16, 19, 20,
                                 16, 20, 21,
                                 16, 21, 22,
                                 16, 22, 23,
                               */
                              24, 25, 26,
                              24, 26, 27,
                              24, 27, 28,
                              24, 28, 29,
                              24, 29, 30,
                              24, 30, 31,
                              16, 24, 25,
                              16, 25, 17,
                              1 + 16, 9 + 16, 10 + 16,
                              1 + 16, 10 + 16, 2 + 16,
                              2 + 16, 10 + 16, 11 + 16,
                              2 + 16, 11 + 16, 3 + 16,
                              3 + 16, 11 + 16, 12 + 16,
                              3 + 16, 12 + 16, 4 + 16,
                              4 + 16, 12 + 16, 13 + 16,
                              4 + 16, 13 + 16, 5 + 16,
                              5 + 16, 13 + 16, 14 + 16,
                              5 + 16, 14 + 16, 6 + 16,
                              6 + 16, 14 + 16, 15 + 16,
                              6 + 16, 15 + 16, 7 + 16,
                              7 + 16, 15 + 16, 8 + 16,
                              7 + 16, 8 + 16, 0 + 16
};

RobotSyrotek::RobotSyrotek(sim::World *w, const Vec3 &pos)
    : _world(w), _pos(pos)
{
        int id;
        unsigned long robot_coll_id = (unsigned long)this;

        {
            size_t i;
            for (i = 0; i < SIZE(verts); i++){
                verts[i] += Vec3(0.03, 0., 0.);
            }
        }

        // create chasis
        _chasis = (BodyCompound *)_world->createBodyCompound();
        id = _chasis->addTriMesh(verts, SIZE(verts), ids, SIZE(ids));
        _chasis->visBody(id)->setColor(0., 0.1, 0.7, .6);
        _chasis->setPos(_pos);
        _chasis->setMassBox(Vec3(0.15, 0.15, 0.18), 1.);
        _chasis->collSetDontCollideId(robot_coll_id);
        //DBG("chasis: " << _chasis);

        _wheel[0] = _world->createBodyCylinderY(0.04, 0.01, 0.1);
        _wheel[0]->visBody()->setColor(1., 1., 1., 1.);
        _wheel[0]->setPos(_pos + Vec3(0.03, 0.14 / 2., 0.04 - 0.007));
        _wheel[0]->collSetDontCollideId(robot_coll_id);
        //DBG("_wheel[0]: " << _wheel[0]);

        _wheel[1] = _world->createBodyCylinderY(0.04, 0.01, 0.1);
        _wheel[1]->visBody()->setColor(1., 1., 1., 1.);
        _wheel[1]->setPos(_pos + Vec3(0.03, -0.14 / 2., 0.04 - 0.007));
        _wheel[1]->collSetDontCollideId(robot_coll_id);
        //DBG("_wheel[1]: " << _wheel[1]);

        _ball[0] = _world->createBodySphere(0.01, 0.1);
        _ball[0]->visBody()->setColor(0., 0., 0., 1.);
        _ball[0]->setPos(_pos + Vec3(0.143 / 2. + 0.03, 0., 0. + 0.01 - 0.0065));
        _ball[0]->collSetDontCollideId(robot_coll_id);
        //DBG("_ball[0]: " << _ball[0]);

        _ball[1] = _world->createBodySphere(0.01, 0.1);
        _ball[1]->visBody()->setColor(0., 0., 0., 1.);
        _ball[1]->setPos(_pos + Vec3(-0.143 / 2. + 0.03, 0., 0. + 0.01 - 0.0065));
        _ball[1]->collSetDontCollideId(robot_coll_id);
        //DBG("_ball[1]: " << _ball[1]);

        _jball[0] = (sim::ode::JointFixed *)_world->createJointFixed(_chasis, _ball[0]);
        _jball[1] = (sim::ode::JointFixed *)_world->createJointFixed(_chasis, _ball[1]);

        _jwheel[0] = (sim::ode::JointHinge2 *)
                        _world->createJointHinge2(_chasis, _wheel[0], _wheel[0]->pos(),
                                                  Vec3(0., 0., 1.), Vec3(0., -1., 0.));
        _jwheel[1] = (sim::ode::JointHinge2 *)
                        _world->createJointHinge2(_chasis, _wheel[1], _wheel[1]->pos(),
                                                  Vec3(0., 0., 1.), Vec3(0., -1., 0.));

        _jwheel[0]->setParamLimitLoHi(-0.0001, 0.0001);
        _jwheel[1]->setParamLimitLoHi(-0.0001, 0.0001);
        _jwheel[0]->setParamFMax2(100.);
        _jwheel[1]->setParamFMax2(100.);
}

void RobotSyrotek::activate()
{
    _chasis->activate();
    _wheel[0]->activate();
    _wheel[1]->activate();
    _ball[0]->activate();
    _ball[1]->activate();
    _jball[0]->activate();
    _jball[1]->activate();
    _jwheel[0]->activate();
    _jwheel[1]->activate();
}

void RobotSyrotek::addVelLeft(Scalar d)
{
    _vel[0] += d;
    _jwheel[0]->setParamVel2(_vel[0]);
}

void RobotSyrotek::addVelRight(Scalar d)
{
    _vel[1] += d;
    _jwheel[1]->setParamVel2(_vel[1]);
}



RobotSyrotekComp::RobotSyrotekComp()
    : sim::Component(), _sim(0), _robot(0)
{
}

RobotSyrotekComp::~RobotSyrotekComp()
{
    if (_robot)
        delete _robot;
}

void RobotSyrotekComp::init(sim::Sim *sim)
{
    Vec3 pos(0., 0., 0.08);

    _sim = sim;
    _robot = new RobotSyrotek(sim->world(), pos);
    _robot->activate();

    sim->regMessage(this, sim::MessageKeyPressed::Type);
}

void RobotSyrotekComp::finish()
{
}

void RobotSyrotekComp::processMessage(const sim::Message &_msg)
{
    if (_msg.type() == sim::MessageKeyPressed::Type){
        const sim::MessageKeyPressed &msg = (const sim::MessageKeyPressed &)_msg;
        int key = msg.key();

        //DBG("Component: " << this << " - key pressed: " << msg.key());

        if (key == 'a'){
            _robot->addVelLeft(0.1);
        }else if (key == 'z'){
            _robot->addVelLeft(-0.1);
        }else if (key == 's'){
            _robot->addVelRight(0.1);
        }else if (key == 'x'){
            _robot->addVelRight(-0.1);
        }

        DBG("Velocity: " << _robot->velLeft() << " " << _robot->velRight());
    }
}
