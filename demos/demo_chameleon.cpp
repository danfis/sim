#include <iostream>
#include <sim/sim.hpp>
#include <sim/ode/world.hpp>
#include <sim/msg.hpp>
#include <sim/comp/syrotek.hpp>
#include "sim/comp/povray.hpp"

using namespace sim::ode;
using sim::Scalar;
using sim::Vec3;
using sim::Quat;
using sim::Time;
using namespace std;

class Syrotek : public sim::comp::Syrotek {
  public:
    Syrotek(const sim::Vec3 &pos)
        : sim::comp::Syrotek(pos)
    {
    }

    void init(sim::Sim *s)
    {
        sim::comp::Syrotek::init(s);
        _useKeyboard();

        _useCamera(100, 100);
        cam()->enableView();
        cam()->attachToBody(robot()->chasis(),
                            Vec3(0., 0., 0.),
                            Quat(Vec3(0., 1., 0.), M_PI / 2.));

        sim()->regPreStep(this);
    }

    void cbPreStep()
    {
        osg::Image *image = cam()->image();
        if (!image->valid())
            return;

        size_t width = image->s();
        size_t height = image->t();
        size_t w, h;
        osg::Vec4 color;
        Scalar r, g, b;

        r = g = b = 0.;
        for (w = 0; w < width; w++){
            for (h = 0; h < height; h++){
                color = image->getColor(w, h);
                r += color.r();
                g += color.g();
                b += color.b();
            }
        }

        r /= (Scalar)(width * height);
        g /= (Scalar)(width * height);
        b /= (Scalar)(width * height);

        robot()->setColor(r, g, b, 1.);
    }
};

static void randColor(osg::Vec4 *color)
{
    Scalar r, g, b;
    r = (Scalar)rand() / (Scalar)RAND_MAX;
    g = (Scalar)rand() / (Scalar)RAND_MAX;
    b = (Scalar)rand() / (Scalar)RAND_MAX;
    color->set(r, g, b, 1.);
}

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
    }

    void createArena()
    {
        Scalar size = 0.5;
        size_t width = 10, height = 10;
        Vec3 pos;
        osg::Vec4 color(0., 0.7, 0.1, 1.);
        BodyCompound *c;
        int id;
        sim::ode::World *w = (sim::ode::World *)world();

        srand(3456789);
        c = (BodyCompound *)w->createBodyCompound();
        for (size_t w = 0; w < width; w++){
            for (size_t h = 0; h < height; h++){
                randColor(&color);
                pos.set(w * size, h * size, 0.);
                id = c->addBox(Vec3(size, size, 0.01), SIM_BODY_DEFAULT_VIS, pos);
                c->visBody(id)->setColor(color);
            }
        }
        c->activate();
    }

    void createRobot()
    {
        Syrotek *comp = new Syrotek(Vec3(0., 0., 0.08));
        addComponent(comp);
    }

};

int main(int argc, char *argv[])
{
    S s;
    s.run();

    return 0;
}