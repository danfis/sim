#include <unistd.h>

#include "world.hpp"
#include "msg.hpp"

int main(int argc, char *argv[])
{
    sim::World *world = new sim::World();
    sim::ObjCube *cube = new sim::ObjCube(1., 1.);
    sim::ObjBox *c2 = new sim::ObjBox(10., 1., 10., 0.);
    sim::ObjSphere *s = new sim::ObjSphere(1.3, 2.);
    sim::ObjCube *c = new sim::ObjCube(1., 1.);
    sim::ObjCylinder *cyl = new sim::ObjCylinder(1., 2., 3.);
    sim::ObjCylinderX *cylx = new sim::ObjCylinderX(.5, 1., 3.);
    sim::ObjCylinderY *cyly = new sim::ObjCylinderY(.5, 1., 3.);

    c2->setPos(0., 0., -10.);
    s->setPos(-3., 0., 0.);
    c->setPos(-3., 0.2, 3.);
    cyl->setPos(3., 0., 0.);
    cylx->setPos(0., 3., 0.);
    cyly->setPos(0., -3., 0.);

    world->addObj(cube);
    world->addObj(c2);
    world->addObj(s);
    world->addObj(c);
    world->addObj(cyl);
    world->addObj(cylx);
    world->addObj(cyly);

    world->init();
    while (!world->done()){
        world->step();

        usleep(10000);
    }
    world->destroy();

    delete c2;
    delete cube;
    delete world;

    return 0;
}

