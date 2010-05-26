#include <iostream>
#include <sim/sim.hpp>
#include <sim/ode/world.hpp>
#include <sim/msg.hpp>
#include <sim/sensor/camera.hpp>

#include "robot_syrotek.hpp"

using namespace sim::ode;
using sim::Scalar;
using sim::Vec3;
using sim::Quat;
using sim::Time;
using namespace std;

class FollowComp : public sim::Component {
    sim::Sim *_sim;
    sim::robot::Syrotek *_robot;
    sim::sensor::Camera *_cam;
    osg::Vec4 _ref_color;

  public:
    FollowComp()
        : sim::Component(),
          _sim(0), _robot(0), _cam(0),
          _ref_color(0.7, 0.1, 0., 0.6)
    {
    }

    ~FollowComp()
    {
        if (_robot)
            delete _robot;
    }

    void init(sim::Sim *sim)
    {
        DBG("");
        Vec3 pos(-0.2, 0., 0.08);

        _sim = sim;
        _robot = new sim::robot::Syrotek(_sim->world(), pos);

        _cam = new sim::sensor::Camera;
        //cam->attachToBody(_robot->chasis(), Vec3(0.13, 0., 0.18), Quat(Vec3(0., 0., 1.), M_PI / 2.));
        _cam->attachToBody(_robot->chasis(), Vec3(0.13, 0., 0.15), Quat(Vec3(0., 1., 0.), M_PI / 4.));
        _cam->visBodyEnable();
        _cam->setWidthHeight(300, 300);
        _cam->setBgColor(0., 0., 0., 1.);
        //cam->enableDump("cam/");
        _cam->enableView();
        sim->addComponent(_cam);

        _robot->activate();

        _sim->regPreStep(this);
    }

    void finish()
    {
    }

    Scalar _evalPixel(const osg::Vec4 &color)
    {
        Scalar e = 0.;

        for (size_t i = 0; i < 3; i++){
            e += (color[i] - _ref_color[i]) * (color[i] - _ref_color[i]);
        }
        e = sqrt(e);
        if (e < 0.0000001)
            e = 0.0000001;
        e = 1. / e;
        return e;
    }

    osg::Vec2 _findMean(osg::Image *image)
    {
        size_t width = image->s();
        size_t height = image->t();
        size_t w, h;
        osg::Vec2 mean(0., 0.);
        osg::Vec4 color;
        osg::Vec2 sum(0., 0.);
        Scalar eval, sum_eval;

        //DBG(image->s() << " " << image->t() << " " << image->r());
        for (w = 0; w < width; w++){
            for (h = 0; h < height; h++){
                color = image->getColor(w, h);
                eval = _evalPixel(color);
                //DBG(color.x() << " " << color.y() << " " << color.z());

                sum += osg::Vec2(w, h) * eval;
                sum_eval += eval;
            }
        }

        if (sum_eval > 0.){
            mean = sum / sum_eval;
        }else{
            mean = osg::Vec2(width, height);
        }

        mean -= osg::Vec2(width, height) / 2.;

        return mean;
    }

    void cbPreStep()
    {
        osg::Image *image = _cam->image();

        if (!image->valid())
            return;

        osg::Vec2 mean = _findMean(image);
        Scalar kf = 1., ko = 1.;
        Scalar vleft, vright;

        //DBG(mean.x() << " " << mean.y());
        vleft = mean.y();
        vright = mean.y();

        vright = vleft = mean.y() * kf;
        vleft += (mean.x() / 2.) * ko;
        vright += (-mean.x() / 2.) * ko;
        //DBG("v: " << vleft << " " << vright);

        _robot->setVelLeft(vleft);
        _robot->setVelRight(vright);
    }
};

class S : public sim::Sim {
  public:
    S()
        : Sim()
    {
        World *w = new World();

        setTimeStep(Time::fromMs(20));
        setTimeSubSteps(2);

        setWorld(w);

        w->setCFM(0.0001);
        w->setERP(0.8);
        //w->setStepType(World::STEP_TYPE_QUICK);
        w->setAutoDisable(0.01, 0.01, 5, 0.);

        w->setContactApprox1(true);
        w->setContactApprox2(true);
        w->setContactBounce(0.1, 0.1);

        createArena();
        createRobot();
        createFollower();
    }

    void createArena()
    {
        osg::Vec4 color(0., 0.7, 0.1, 1.);
        BodyCompound *c;
        int id;
        sim::ode::World *w = (sim::ode::World *)world();

        c = (BodyCompound *)w->createBodyCompound();
        id = c->addBox(Vec3(2., 2., 0.1));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, 2., 0.5), SIM_BODY_DEFAULT_VIS, Vec3(-1., 0., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, 2., 0.5), SIM_BODY_DEFAULT_VIS, Vec3(1., 0., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(2., .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., 1., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(2., .1, 0.3), SIM_BODY_DEFAULT_VIS, Vec3(0., -1., .25));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");

        id = c->addCube(.2, SIM_BODY_DEFAULT_VIS, Vec3(0.5, 0.5, .1));
        c->visBody(id)->setColor(0., .1, .8, 1.);
        c->visBody(id)->setTexture("wood.ppm");
        c->activate();

        {
            sim::Body *b;

            b = w->createBodyCube(0.1, 0.1);
            b->setPos(0., 0., 1.);
            b->activate();

            b = w->createBodySphere(0.1, 0.1);
            b->setPos(0.35, 0.4, .5);
            b->activate();
        }
    }

    void createRobot()
    {
        RobotSyrotekComp *comp;

        comp = new RobotSyrotekComp();

        addComponent(comp);
    }

    void createFollower()
    {
        addComponent(new FollowComp());
    }

};

int main(int argc, char *argv[])
{
    S s;

    s.run();

    return 0;
}
