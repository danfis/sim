#include <unistd.h>

#include "visworld.hpp"
#include "physworld.hpp"
#include "msg.hpp"

int main(int argc, char *argv[])
{
    float vsp_pos[3];
    float pos[3], rot[4];
    sim::VisWorld *vw = new sim::VisWorld();
    sim::VisObjCube *vbox = new sim::VisObjCube(1.);
    sim::VisObjCube *vbox2 = new sim::VisObjCube(10.);
    sim::VisObjSphere *vsp = new sim::VisObjSphere(0.5);
    sim::PhysWorld *pw = new sim::PhysWorld();
    sim::PhysObjCube *pbox = new sim::PhysObjCube(1., 1.);
    sim::PhysObjBox *pbox2 = new sim::PhysObjBox(10., 10., 10., 0.);

    vw->addObj(vbox);
    vw->addObj(vsp);
    vw->addObj(vbox2);


    pbox2->setPosition(0., -10., 0.);

    pw->addObj(pbox);
    pw->addObj(pbox2);

    vw->init();
    pw->init();
    while (!vw->done() && !pw->done()){
        //DBG("Step");

        pw->step();

        pbox->getPosition(pos);
        pbox->getRotation(rot);
        vbox->setPosition(pos);
        vbox->setRotation(rot);

        pbox2->getPosition(pos);
        pbox2->getRotation(rot);
        vbox2->setPosition(pos);
        vbox2->setRotation(rot);

        vw->step();

        usleep(100000);

        //vsp->getPosition(vsp_pos);
        //vsp_pos[0] += 0.01;
        //vsp->setPosition(vsp_pos);
    }
    pw->destroy();
    vw->destroy();

    delete vsp;
    delete vbox;
    delete vw;

    return 0;
}
