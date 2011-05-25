#include "arena.hpp"

#define ICE_LEN 6.
#define ICE_ANGLE (M_PI / 720)

PowerSource::PowerSource(sim::Sim *sim, const Vec3 &pos, const Quat &rot)
    : sim::Component()
{
    osg::Vec4 color(0.7, 0.7, 0.1, 1.);
    int id;

    _body = sim->world()->createBodyCompound();
    id = _body->addBox(Vec3(.5, .5, .5));
    _body->visBody(id)->setColor(color);

    _body->setPos(pos);
    _body->setRot(rot);

    _body->collSetDontCollideId(100);

    _body->activate();
}

void PowerSource::init(sim::Sim *sim)
{
}


Arena::Arena(sim::Sim *sim)
    : sim::Component(), _frame(0)
{
    osg::Vec4 color(0., 0.7, 0.1, 1.);
    osg::Vec4 color2(0., 0.1, 0.7, 1.);
    int id;

    _frame = sim->world()->createBodyCompound();
    id = _frame->addBox(Vec3(30 - ICE_LEN, 30, 0.1), SIM_BODY_DEFAULT_VIS,
                        Vec3(ICE_LEN / 2., 0, 0));
    _frame->visBody(id)->setColor(color);
    _frame->visBody(id)->setTexture("wood.ppm");

    id = _frame->addBox(Vec3(0.1, 30., 2.), SIM_BODY_DEFAULT_VIS, Vec3(-15, 0., .75));
    _frame->visBody(id)->setColor(color);
    _frame->visBody(id)->setTexture("wood.ppm");

    id = _frame->addBox(Vec3(0.1, 30., 2.), SIM_BODY_DEFAULT_VIS, Vec3(15, 0., .75));
    _frame->visBody(id)->setColor(color);
    _frame->visBody(id)->setTexture("wood.ppm");

    id = _frame->addBox(Vec3(30., .1, 2.), SIM_BODY_DEFAULT_VIS, Vec3(0., 15, .75));
    _frame->visBody(id)->setColor(color);
    _frame->visBody(id)->setTexture("wood.ppm");

    id = _frame->addBox(Vec3(30., .1, 2.), SIM_BODY_DEFAULT_VIS, Vec3(0., -15, .75));
    _frame->visBody(id)->setColor(color);
    _frame->visBody(id)->setTexture("wood.ppm");

    _frame->collSetDontCollideId(100);
    _frame->activate();


    _ice = sim->world()->createBodyCompound();
    id = _ice->addBox(Vec3(ICE_LEN, 30, 0.1), SIM_BODY_DEFAULT_VIS,
                      Vec3(-15 + ICE_LEN / 2., 0, (ICE_LEN / 2.) * sin(ICE_ANGLE)),
                      Quat(Vec3(0, 1, 0), ICE_ANGLE));
    _ice->visBody(id)->setColor(color2);
    _ice->visBody(id)->setTexture("wood.ppm");

    _ice->collSetFriction(0.0001);
    _ice->collSetDontCollideId(100);
    _ice->activate();
}

void Arena::init(sim::Sim *sim)
{
    PowerSource *ps;

    ps = new PowerSource(sim, Vec3(0, -14.7, .7));
    _pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(0, 14.7, .7));
    _pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(14.7, 0, .7));
    _pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(-14.7, 0, .7));
    _pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(10, -14.7, 1.4));
    _pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(10, 14.7, 1.4));
    _pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(14.7, 10, 1.4));
    _pw_sources.push_back(ps);

    ps = new PowerSource(sim, Vec3(-14.7, 10, 1.4));
    _pw_sources.push_back(ps);
}
