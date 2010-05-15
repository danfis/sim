#include <unistd.h>
#include <iostream>

#include "sim/bullet/world.hpp"
#include "sim/sim.hpp"
#include "msg.hpp"

using namespace sim::bullet;
using sim::Vec3;

class S : public sim::Sim {
  public:
    S()
        : Sim()
    {
        World *w = new World();
        sim::Body *b;

        setWorld(w);
    
        {
            Vec3 verts[] = { Vec3( 0.087,  0.045, 0.),
                             Vec3( 0.087, -0.045, 0.),
                             Vec3( 0.050, -0.081, 0.),
                             Vec3(-0.050, -0.081, 0.),
                             Vec3(-0.087, -0.045, 0.),
                             Vec3(-0.087,  0.045, 0.),
                             Vec3(-0.050,  0.081, 0.),
                             Vec3( 0.050,  0.081, 0.),
                             Vec3( 0.087,  0.045, 0.06),
                             Vec3( 0.087, -0.045, 0.06),
                             Vec3( 0.050, -0.081, 0.06),
                             Vec3(-0.050, -0.081, 0.06),
                             Vec3(-0.087, -0.045, 0.06),
                             Vec3(-0.087,  0.045, 0.06),
                             Vec3(-0.050,  0.081, 0.06),
                             Vec3( 0.050,  0.081, 0.06) };
            unsigned int inds[] = { 0, 1, 2,
                                    0, 2, 3,
                                    0, 3, 4,
                                    0, 4, 5,
                                    0, 5, 6,
                                    0, 6, 7,
                                    8, 9, 10,
                                    8, 10, 11,
                                    8, 11, 12,
                                    8, 12, 13,
                                    8, 13, 14,
                                    8, 14, 15,
                                    0, 8, 9,
                                    0, 9, 1,
                                    1, 9, 10,
                                    1, 10, 2,
                                    2, 10, 11,
                                    2, 11, 3,
                                    3, 11, 12,
                                    3, 12, 4,
                                    4, 12, 13,
                                    4, 13, 5,
                                    5, 13, 14,
                                    5, 14, 6,
                                    6, 14, 15,
                                    6, 15, 7,
                                    7, 15, 8,
                                    7, 8, 0
                                  };

            sim::VisBody *vis = new sim::VisBodyTriMesh(verts, sizeof(verts) / sizeof(Vec3),
                                                        inds, sizeof(inds) / sizeof(unsigned int));
            vis->setOffset(Vec3(0., 0., -0.04));
            b = w->createBodyBox(Vec3(2 * 0.087, 2 * 0.081, 0.06), 2., vis);
            b = new BodyConvexHull(w, verts, sizeof(verts) / sizeof(Vec3), 1., vis);
            b->visBody()->setColor(osg::Vec4(0., 0., 0.7, 1.));
            b->setPos(Vec3(0.1, 0., -9.));
            b->activate();
        }

        {
            Vec3 verts[] = { Vec3(-2., 2., 0.),
                             Vec3(0., 2., 0.5),
                             Vec3(2., 2., 0.),
                             Vec3(-2., 0., 0.),
                             Vec3(0., 0., 0.5),
                             Vec3(2., 0., 0.),
                             Vec3(-2., -2., 0.),
                             Vec3(0., -2., 0.5),
                             Vec3(2., -2., 0.) };
            unsigned int ind[] = { 0, 1, 4,
                                   1, 2, 5,
                                   0, 4, 3,
                                   1, 5, 4,
                                   3, 4, 7,
                                   4, 5, 8,
                                   3, 7, 6,
                                   4, 8, 7 };
            b = w->createBodyTriMesh(verts, 9, ind, 24);
            b->setPos(0., 0., -10.);
            b->visBody()->setColor(0.9, 0.6, 0.3, 1.);
            b->activate();
        }
    }

    void init()
    {
        sim::Sim::init();
        for (size_t i = 0; i < 300; i++){
            visWorld()->step();
            std::cerr << i << "\r";
            usleep(10000);
        }
        std::cerr << std::endl;

        timeRealRestart();
    }
};


int main(int argc, char *argv[])
{
    S s;

    s.run();

    return 0;
}

