#include <unistd.h>
#include <iostream>

#include "world.hpp"
#include "joint.hpp"
#include "msg.hpp"

void createRobot(sim::World *world, const sim::Vec3 &pos)
{
    sim::ObjBox *chasis;
    sim::ObjCylinderX *w[4];
    sim::JointHinge2 *j[4];
    size_t i;

    chasis = new sim::ObjBox(sim::Vec3(.6, 1., 0.4), 1.);
    for (i = 0; i < 4; i++){
        w[i] = new sim::ObjCylinderX(0.2, 0.2, 1.);
    }

    chasis->setPos(pos);
    w[0]->setPos(pos.x() + 0.405, pos.y() + 0.4, pos.z() - 0.2);
    w[1]->setPos(pos.x() - 0.405, pos.y() + 0.4, pos.z() - 0.2);
    w[2]->setPos(pos.x() + 0.405, pos.y() - 0.4, pos.z() - 0.2);
    w[3]->setPos(pos.x() - 0.405, pos.y() - 0.4, pos.z() - 0.2);

    for (i = 0; i < 4; i++){
        j[i] = new sim::JointHinge2(chasis, w[i], w[i]->pos(),
                                    sim::Vec3(0., 0., 1.), sim::Vec3(1., 0., 0.));
    }

    world->addObj(chasis);
    for (i = 0; i < 4; i++){
        world->addObj(w[i]);
        world->addJoint(j[i]);
    }
}


void createP2P(sim::World *world, const sim::Vec3 &pos)
{
    sim::ObjCube *c[2];
    sim::JointPoint2Point *j;

    c[0] = new sim::ObjCube(1., 1.);
    c[1] = new sim::ObjCube(1., 1.);

    c[0]->setPos(pos);
    c[1]->setPos(pos + sim::Vec3(2., 2., 2.));

    j = new sim::JointPoint2Point(c[0], c[1], c[0]->pos(), c[0]->pos());

    world->addObj(c[0]);
    world->addObj(c[1]);
    world->addJoint(j);
}

void createHinge(sim::World *world, const sim::Vec3 &pos)
{
    sim::ObjCylinder *cy = new sim::ObjCylinder(0.5, 0.5, 1.);
    sim::ObjCube *cu = new sim::ObjCube(1., 1.);
    sim::JointHinge *j;

    cu->setPos(pos);
    cy->setPos(pos + sim::Vec3(0., 0., 2.));

    j = new sim::JointHinge(cu, cy, pos + sim::Vec3(0., 0., 1.), sim::Vec3(0., 1., 0.));

    world->addObj(cu);
    world->addObj(cy);
    world->addJoint(j);
}

void createFixed(sim::World *world, const sim::Vec3 &pos)
{
    sim::ObjCube *cu = new sim::ObjCube(1., 1.);
    sim::ObjSphere *s = new sim::ObjSphere(0.5, 3.);
    sim::JointFixed *j;

    cu->setPos(pos);
    s->setPos(pos + sim::Vec3(0.9, 0., 1.0));
    j = new sim::JointFixed(cu, s);

    world->addObj(cu);
    world->addObj(s);
    world->addJoint(j);
}


int main(int argc, char *argv[])
{
    size_t i;
    sim::World *world = new sim::World();
    sim::ObjCube *cube = new sim::ObjCube(1., 1.);
    sim::ObjBox *c2 = new sim::ObjBox(sim::Vec3(20., 20., 1.), 0.);
    sim::ObjSphere *s = new sim::ObjSphere(1.3, 2.);
    sim::ObjCube *c = new sim::ObjCube(1., 1.);
    sim::ObjCylinder *cyl = new sim::ObjCylinder(1., 2., 3.);
    sim::ObjCylinderX *cylx = new sim::ObjCylinderX(.5, 1., 3.);
    sim::ObjCylinderY *cyly = new sim::ObjCylinderY(.5, 1., 3.);

    createRobot(world, sim::Vec3(3., 3., -9.));
    createP2P(world, sim::Vec3(6., 6., -9.));
    createHinge(world, sim::Vec3(-6., 6., -9.));
    createFixed(world, sim::Vec3(-6., -6., -9.));

    c->visObj()->setColor(osg::Vec4(0., 0., 0., 1.));
    s->visObj()->setColor(osg::Vec4(0.6, 0.3, 0., 0.));

    cube->setPos(-6., 6., -5.);
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
    for (i = 0; i < 300; i++){
        world->step(false, true);
        usleep(10000);
        std::cerr << i << "\r";
    }
    std::cerr << std::endl;

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

