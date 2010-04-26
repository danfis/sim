#include <unistd.h>

#include "world.hpp"
#include "msg.hpp"

int main(int argc, char *argv[])
{
    sim::World *world = new sim::World();
    sim::ObjCube *cube = new sim::ObjCube(1., 1.);
    sim::ObjBox *c2 = new sim::ObjBox(10., 1., 10., 0.);

    c2->setPos(0., -10., 5.2);

    world->addObj(cube);
    world->addObj(c2);

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

