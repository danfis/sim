#ifndef _SIM_ROBOT_SYROTEK_HPP_
#define _SIM_ROBOT_SYROTEK_HPP_

#include <sim/world.hpp>
#include <sim/body.hpp>
#include <sim/joint.hpp>
#include <sim/ode/joint.hpp>

namespace sim {

namespace robot {

class SSSA {
  protected:
    sim::World *_world;
    sim::Vec3 _pos;
    osg::Vec4 _color;
    Scalar _arm_offset;

    sim::Body *_chasis;
    sim::Body *_arm;
    sim::ode::JointHinge *_arm_joint; //!< joint between chasis and arm

    sim::Body *_wheel_left[6];
    sim::ode::JointHinge *_wheel_left_joint[6];
    sim::Body *_wheel_right[6];
    sim::ode::JointHinge *_wheel_right_joint[6];

    sim::robot::SSSA *_ball_conn; //!< Robot connected to arm's ball
    sim::ode::JointHinge *_ball_joint; //!< Joint used for connection via arm

    sim::robot::SSSA *_sock_conn[3]; //!< Robots connected to sockets

    Scalar _vel_left, _vel_right, _vel_arm;
    bool _arm_fixed;

  public:
    SSSA(sim::World *w, const sim::Vec3 &pos = sim::Vec3(0., 0., 0.08),
         const osg::Vec4 &chasis_color = osg::Vec4(0., 0.1, 0.7, 0.6));

    const sim::Body *chassis() const { return _chasis; }
    sim::Body *chassis() { return _chasis; }
    const sim::Body *arm() const { return _arm; }
    sim::Body *arm() { return _arm; }
    const sim::Joint *ballJoint() const { return _ball_joint; }
    sim::Joint *ballJoint() { return _ball_joint; }

    size_t numSockets() const { return 3; }

    Scalar armOffset() const { return _arm_offset; }

    /**
     * Sets arm offset (in radians). This method has effect only before
     * activation.
     */
    void setArmOffset(Scalar offset)
        { _arm_offset = offset; }

    /**
     * Returns angle of arm is rotated about - range is -pi/2..pi/2.
     */
    Scalar armAngle() const;

    /**
     * Returns true if arm is fixed in armAngle() position.
     */
    bool armFixed() const { return _arm_fixed; }

    /**
     * Fix arm in current position.
     */
    void fixArm();
    void unfixArm();

    /**
     * Try to reach specified angle on arm.
     * If arm is already in specified angle - arm is fixed.
     * If not velocity is set to move arm properly.
     * True is returned if arm was fixed, false otherwise.
     */
    bool reachArmAngle(Scalar angle);

    /**
     * Sets arm's velocity as desired value.
     */
    void setVelArm(sim::Scalar vel);
    void addVelArm(sim::Scalar d);
    sim::Scalar velArm() const { return _vel_arm; }

    void setVelLeft(sim::Scalar vel);
    void addVelLeft(sim::Scalar d);
    sim::Scalar velLeft() const { return _vel_left; }

    void setVelRight(sim::Scalar vel);
    void addVelRight(sim::Scalar d);
    sim::Scalar velRight() const { return _vel_right; }

    /**
     * Returns position, direction and up vector of specified socket.
     * Direction and up vector is always unit-length.
     */
    void socketSetup(size_t idx, Vec3 *pos, Vec3 *dir, Vec3 *up) const;

    /**
     * Ball is socket on the end of arm.
     * This function does same thing as socketPosDir() but on arm's socket.
     */
    void ballSetup(Vec3 *pos, Vec3 *dir, Vec3 *up) const;

    /**
     * Returns number of socket of other robot this robot can connect to
     * (using ball on arm). If it is not possible to connect returns -1.
     */
    int canConnectTo(const sim::robot::SSSA &robot) const;

    /**
     * Connects arm's ball to robot's socket if possible.
     */
    bool connectTo(sim::robot::SSSA &robot);

    /**
     * Returns true if given robot is conected to this robot's arm.
     */
    bool isConnected(const sim::robot::SSSA *robot) const;

    /**
     * Returns true if idx's socket is connected to robot's arm.
     */
    bool isSocketConnectedTo(size_t idx, const sim::robot::SSSA *robot) const;
    bool isAnySocketConnectedTo(const sim::robot::SSSA *robot) const;

    bool isSocketConnected(size_t idx) const { return _sock_conn[idx] != 0; }

    const sim::robot::SSSA *connectedRobot() const { return _ball_conn; }
    sim::robot::SSSA *connectedRobot() { return _ball_conn; }
    const sim::robot::SSSA *connectedRobotSocket(size_t idx) const
        { return _sock_conn[idx]; }
    sim::robot::SSSA *connectedRobotSocket(size_t idx)
        { return _sock_conn[idx]; }

    void activate();

  protected:
    void _createChasis(const osg::Vec4 &color);
    void _createArm(const osg::Vec4 &color);
    void _createArmJoint();
    void _createWheels();
};

} /* namespace robot */

} /* namespace sim */

#endif /* _SIM_ROBOT_SYROTEK_HPP_ */
