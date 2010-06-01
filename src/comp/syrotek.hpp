#ifndef _SIM_COMP_SYROTEK_HPP_
#define _SIM_COMP_SYROTEK_HPP_

#include <sim/sim.hpp>
#include <sim/component.hpp>
#include <sim/robot/syrotek.hpp>
#include <sim/comp/joystick.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/sensor/rangefinder.hpp>


namespace sim {

namespace comp {

/**
 * Controller for robot::Syrotek.
 *
 * TODO
 */
class Syrotek : public sim::Component {
  protected:
    sim::Sim *_sim;
    sim::Vec3 _pos;
    osg::Vec4 _color;

    sim::robot::Syrotek *_robot;
    sim::sensor::Camera *_cam;
    sim::sensor::RangeFinder *_range_finder;
    sim::comp::Joystick *_joystick;

  public:
    Syrotek(const sim::Vec3 &pos,
            const osg::Vec4 &color = osg::Vec4(0.7, 0.1, 0., 0.6));
    ~Syrotek();

    const sim::Sim *sim() const { return _sim; }
    sim::Sim *sim() { return _sim; }
    const sim::robot::Syrotek *robot() const { return _robot; }
    sim::robot::Syrotek *robot() { return _robot; }
    const sim::sensor::Camera *cam() const { return _cam; }
    sim::sensor::Camera *cam() { return _cam; }
    const sim::sensor::RangeFinder *rangeFinder() const { return _range_finder; }
    sim::sensor::RangeFinder *rangeFinder() { return _range_finder; }
    const sim::comp::Joystick *joystick() const { return _joystick; }
    sim::comp::Joystick *joystick() { return _joystick; }
    const osg::Vec4 &color() const { return _color; }

    void init(sim::Sim *sim);
    void finish();

    void processMessage(const sim::Message &msg);

  protected:
    void _keyPressedMsg(const sim::MessageKeyPressed &msg);
    void _joystickMsg(const sim::comp::JoystickMessage &msg);

    void _useCamera(int width, int height);
    void _useRangeFinder();
    void _useJoystick();
    void _useKeyboard();
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_SYROTEK_HPP_ */
