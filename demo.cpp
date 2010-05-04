#include <unistd.h>
#include <iostream>

#include "world.hpp"
#include "joint.hpp"
#include "actuator.hpp"
#include "msg.hpp"

void createRobot(sim::World *world, const sim::Vec3 &pos)
{
    sim::BodyBox *chasis;
    sim::ActuatorWheelCylinderX *w[4];
    size_t i;

    chasis = new sim::BodyBox(world, sim::Vec3(.6, 1., 0.4), 1.);
    for (i = 0; i < 4; i++){
        w[i] = new sim::ActuatorWheelCylinderX(world, 0.2, 0.2, 1.);
    }

    chasis->setPos(pos);
    w[0]->setPos(pos.x() + 0.405, pos.y() + 0.4, pos.z() - 0.2);
    w[1]->setPos(pos.x() - 0.405, pos.y() + 0.4, pos.z() - 0.2);
    w[2]->setPos(pos.x() + 0.405, pos.y() - 0.4, pos.z() - 0.2);
    w[3]->setPos(pos.x() - 0.405, pos.y() - 0.4, pos.z() - 0.2);

    for (i = 0; i < 4; i++){
        w[i]->connectToChasis(chasis);
    }

    chasis->activate();
    for (i = 0; i < 4; i++){
        w[i]->activate();
    }
}


void createP2P(sim::World *world, const sim::Vec3 &pos)
{
    sim::BodyCube *c[2];
    sim::JointPoint2Point *j;

    c[0] = new sim::BodyCube(world, 1., 1.);
    c[1] = new sim::BodyCube(world, 1., 1.);

    c[0]->setPos(pos);
    c[1]->setPos(pos + sim::Vec3(2., 2., 2.));

    j = new sim::JointPoint2Point(world, c[0], c[1], c[0]->pos(), c[0]->pos());

    c[0]->activate();
    c[1]->activate();
    j->activate();
}

void createHinge(sim::World *world, const sim::Vec3 &pos)
{
    sim::BodyCylinder *cy = new sim::BodyCylinder(world, 0.5, 0.5, 1.);
    sim::BodyCube *cu = new sim::BodyCube(world, 1., 1.);
    sim::JointHinge *j;

    cu->setPos(pos);
    cy->setPos(pos + sim::Vec3(0., 0., 2.));

    j = new sim::JointHinge(world, cu, cy, pos + sim::Vec3(0., 0., 1.), sim::Vec3(0., 1., 0.));

    cu->activate();
    cy->activate();
    j->activate();
}

void createFixed(sim::World *world, const sim::Vec3 &pos)
{
    sim::BodyCube *cu = new sim::BodyCube(world, 1., 1.);
    sim::BodySphere *s = new sim::BodySphere(world, 0.5, 3.);
    sim::JointFixed *j;

    cu->setPos(pos);
    s->setPos(pos + sim::Vec3(0.9, 0., 1.0));
    j = new sim::JointFixed(world, cu, s);

    cu->activate();
    s->activate();
    j->activate();
}


int main(int argc, char *argv[])
{
    size_t i;
    sim::World *world = new sim::World();
    sim::BodyCube *cube = new sim::BodyCube(world, 1., 1.);
    sim::BodyBox *c2 = new sim::BodyBox(world, sim::Vec3(20., 20., 1.), 0.);
    sim::BodySphere *s = new sim::BodySphere(world, 1.3, 2.);
    sim::BodyCube *c = new sim::BodyCube(world, 1., 1.);
    sim::BodyCylinder *cyl = new sim::BodyCylinder(world, 1., 2., 3.);
    sim::BodyCylinderX *cylx = new sim::BodyCylinderX(world, .5, 1., 3.);
    sim::BodyCylinderY *cyly = new sim::BodyCylinderY(world, .5, 1., 3.);
    sim::Robot4Wheels *r1 = new sim::Robot4Wheels(world);

    r1->setPos(3., 5., -7.);
    r1->activate();

    createRobot(world, sim::Vec3(3., 3., -9.));
    createP2P(world, sim::Vec3(6., 6., -9.));
    createHinge(world, sim::Vec3(-6., 6., -9.));
    createFixed(world, sim::Vec3(-6., -6., -9.));

    c->visBody()->setColor(osg::Vec4(0., 0., 0., 1.));
    s->visBody()->setColor(osg::Vec4(0.6, 0.3, 0., 0.));

    cube->setPos(-6., 6., -5.);
    c2->setPos(0., 0., -10.);
    s->setPos(-3., 0., 0.);
    c->setPos(-3., 0.2, 3.);
    cyl->setPos(3., 0., 0.);
    cylx->setPos(0., 3., 0.);
    cyly->setPos(0., -3., 0.);

    cube->activate();
    c2->activate();
    s->activate();
    c->activate();
    cyl->activate();
    cylx->activate();
    cyly->activate();
    //world->addActor(r1);

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

