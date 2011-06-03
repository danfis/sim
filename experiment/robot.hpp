#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <sim/sim.hpp>
#include <sim/world.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/comp/sssa.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/msg.hpp>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>


#include "finder.hpp"

class Sim;

#define K1 1.E-4, 0.,    0.,    1.E-4, 0.,    0.,     0., \
           0.,    0.E-5, 0.,    0.,    0.E-5, 0.,     0., \
           0.,    0.,    0.,    0.,    0.,    0.,     0., \
           0.,    0.,    0.,    0.001, 0.,    0.,     0., \
           0.,    0.,    0.,    0.,    0.001, 0.,     0., \
           0.,    0.,    0.,    0.,    0.,    0.,     0., \
           0.,    0.,    0.001, 0.,    0.,    0.,     -40.

#define K2 -0.01, 0.,    0.,    0.,    0.,    0.,     0., \
           0.,    0.0,  0.,    0.,    0.,    0.,     0., \
           0.,    0.,    0.,    0.,    0.,    0.,     0., \
           0.,    0.,    0.,    0.001, 0.,    0.,     0., \
           0.,    0.,    0.,    0.,    0.0,  0.,     0., \
           0.,    0.,    0.,    0.,    0.,    0.,     0., \
           0.,    0.,    0.,    0.,    0.,    0.,     1.

#define K3 -0.1,  0.,    0.,    0.,    0.,    0.,     0., \
           0.,    -0.1,  0.,    0.,    0.,    0.,     0., \
           0.,    0.,    -0.1,  0.,    0.,    0.,     0., \
           0.,    0.,    0.,    -0.1,  0.,    0.,     0., \
           0.,    0.,    0.,    0.,    -0.1,  0.,     0., \
           0.,    0.,    0.,    0.,    0.,    -0.1,   0., \
           0.,    0.,    0.,    0.,    0.,    0.,     -0.1

#define K4 K3
#define K5 K3
#define K6 K3

#define A 20.0,  0.,    0.,    0.,    0.,    0.,     0., \
          0.,    0.0,  0.,    0.,    0.,    0.,     0., \
          0.,    0.,    1.0,    0.,    0.,    0.,     0., \
          0.,    0.,    0.,    20.0,  0.,    0.,     0., \
          0.,    0.,    0.,    0.,    0.0,  0.,     0., \
          0.,    0.,    0.,    0.,    0.,    0.,     0., \
          0.,    0.,    0.,    0.,    0.,    0.,     0.

#define FPS 10

#define WAIT_FOR_ROBOT_TRESHOLD 30000

#define WIDTH 320
#define HEIGHT 240

#define VEL_OFFSET 3

using sim::Vec3;
using sim::Quat;

void setMatrix(gsl_matrix *m, ...);

class Robot : public sim::comp::SSSA {
    Sim *_sim;
    sim::sensor::Camera *_cam;

    blobf::finder_t *_finder, *_finder2;

    int _counter;
    int _wait_for_robot;
    sim::Vec3 _last_pos;
    gsl_vector *_s, *_h, *_a;
    gsl_matrix *_K1, *_K2, *_K3, *_K4, *_K5, *_K6;
    gsl_matrix *_A;

  public:
    Robot(const Vec3 &pos = Vec3(0, 0, 0),
          const Quat &rot = Quat(0, 0, 0, 1),
          bool use_cam = false);
    virtual ~Robot();

    void init(sim::Sim *sim);

    void processMessage(const sim::Message &msg);

    void cbPreStep();

    const gsl_vector *hormone() const { return _h; }
    int waitForRobot() const { return _wait_for_robot; }

  protected:
    void _keyPressedMsg(const sim::MessageKeyPressed &msg);

    /**
     * Fills _s with input data.
     */
    void _gatherInput();

    void _updateHormone();
    void _updateActions();
};

#endif /* ROBOT_HPP */
