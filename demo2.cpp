#include <unistd.h>

#include "visworld.hpp"
#include "msg.hpp"

int main(int argc, char *argv[])
{
    float vsp_pos[3];
    sim::VisWorld *vw = new sim::VisWorld();
    sim::VisObjBox *vbox = new sim::VisObjBox(1.);
    sim::VisObjSphere *vsp = new sim::VisObjSphere(0.5);

    vw->addObj(vbox);
    vw->addObj(vsp);

    vw->init();
    while (!vw->done()){
        DBG("Step");

        vw->step();

        usleep(100);

        vsp->getPosition(vsp_pos);
        vsp_pos[0] += 0.01;
        vsp->setPosition(vsp_pos);
    }
    vw->destroy();

    delete vsp;
    delete vbox;
    delete vw;

    return 0;
}
