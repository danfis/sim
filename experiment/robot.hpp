#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <sim/sim.hpp>
#include <sim/world.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/comp/sssa.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/msg.hpp>

#include <fermat/vec3.h>
#include <fermat/mat3.h>

#define K1 0.1, 0,   0, \
           0,   0.1, 0, \
           0,   0,   0.1

#define K2 0.1, 0,   0, \
           0,   0.1, 0, \
           0,   0,   0.1

#define K3 0.1, 0,   0, \
           0,   0.1, 0, \
           0,   0,   0.1
#define K4 K3
#define K5 K3
#define K6 K3

#define A 0.1, 0,   0, \
          0,   0.1, 0, \
          0,   0,   0.1

#define INIT_S 0, 0, 0
#define INIT_H 0, 0, 0

#define FPS 10

#define WIDTH 320
#define HEIGHT 240

using sim::Vec3;
using sim::Quat;

class Robot : public sim::comp::SSSA {
    sim::sensor::Camera *_cam;

    int _counter;
    fer_vec3_t _s, _h, _a;
    fer_mat3_t _K1, _K2, _K3, _K4, _K5, _K6;
    fer_mat3_t _A;

  public:
    Robot(const Vec3 &pos = Vec3(0, 0, 0),
          const Quat &rot = Quat(0, 0, 0, 1),
          bool use_cam = false);

    void init(sim::Sim *sim);

    void processMessage(const sim::Message &msg);

    void cbPreStep();

  protected:
    void _keyPressedMsg(const sim::MessageKeyPressed &msg);

    /**
     * Fills _s with input data.
     */
    void _gatherInput();
};

#endif /* ROBOT_HPP */
