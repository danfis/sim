#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <sim/sim.hpp>
#include <sim/world.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/comp/sssa.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/msg.hpp>

using sim::Vec3;
using sim::Quat;

class Robot : public sim::comp::SSSA {
    sim::sensor::Camera *_cam;

  public:
    Robot(const Vec3 &pos = Vec3(0, 0, 0),
          const Quat &rot = Quat(0, 0, 0, 1),
          bool use_cam = false);
    void init(sim::Sim *sim);
    void processMessage(const sim::Message &msg);
    void _keyPressedMsg(const sim::MessageKeyPressed &msg);
};

#endif /* ROBOT_HPP */
