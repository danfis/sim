#include <unistd.h>

#include "world.hpp"
#include "joint.hpp"
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
    sim::ObjCube *r = new sim::ObjCube(1., 1.);
    sim::ObjCylinderX *rw1 = new sim::ObjCylinderX(0.3, 0.2, 1.);
    sim::ObjCylinderX *rw2 = new sim::ObjCylinderX(0.3, 0.2, 1.);
    sim::ObjCylinderX *rw3 = new sim::ObjCylinderX(0.3, 0.2, 1.);
    sim::ObjCylinderX *rw4 = new sim::ObjCylinderX(0.3, 0.2, 1.);
    sim::JointHinge2 *j1, *j2, *j3, *j4;
   
    r->setPos(0., 0., -5.);
    rw1->setPos(0.65, 0.5, -5.5);
    rw2->setPos(-0.65, 0.5, -5.5);
    rw3->setPos(0.65, -0.5, -5.5);
    rw4->setPos(-0.65, -0.5, -5.5);
    j1 = new sim::JointHinge2(r, rw1, rw1->pos(),
                              sim::Vec3(0., 0., 1.), sim::Vec3(1., 0., 0.));
    j2 = new sim::JointHinge2(r, rw2, rw2->pos(),
                              sim::Vec3(0., 0., 1.), sim::Vec3(1., 0., 0.));
    j3 = new sim::JointHinge2(r, rw3, rw3->pos(),
                              sim::Vec3(0., 0., 1.), sim::Vec3(1., 0., 0.));
    j4 = new sim::JointHinge2(r, rw4, rw4->pos(),
                              sim::Vec3(0., 0., 1.), sim::Vec3(1., 0., 0.));

    world->addObj(r);
    world->addObj(rw1);
    world->addObj(rw2);
    world->addObj(rw3);
    world->addObj(rw4);
    world->addJoint(j1);
    world->addJoint(j2);
    world->addJoint(j3);
    world->addJoint(j4);

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

