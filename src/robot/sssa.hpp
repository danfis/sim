#ifndef _SIM_ROBOT_SYROTEK_HPP_
#define _SIM_ROBOT_SYROTEK_HPP_

#include <sim/world.hpp>
#include <sim/body.hpp>
#include <sim/joint.hpp>

namespace sim {

namespace robot {

class SSSA {
  protected:
    sim::World *_world;
    sim::Vec3 _pos;

    sim::Body *_chasis;
    sim::Body *_arm;
    sim::Joint *_arm_joint; //!< joint between chasis and arm

    sim::Body *_wheel_left[4];
    sim::Joint *_wheel_left_joint[4];
    sim::Body *_wheel_right[4];
    sim::Joint *_wheel_right_joint[4];

    sim::robot::SSSA *_ball_conn; //!< Robot connected to arm's ball
    sim::Joint *_ball_joint; //!< Joint used for connection via arm

    sim::robot::SSSA *_sock_conn[3]; //!< Robots connected to sockets

    // noy NULL if the robotis iss connected to the referenced robot
    sim::robot::SSSA *_socket1_connection;
    sim::robot::SSSA *_socket2_connection;
    sim::robot::SSSA *_socket3_connection;

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


    /**
     * Returns position and direction of specified socket via pos and dir
     * arguments. Direction vector is always unit-length.
     */
    void socketPosDir(size_t idx, Vec3 *pos, Vec3 *dir) const;

    /**
     * Ball is socket on the end of arm.
     * This function does same thing as socketPosDir() but on arm's socket.
     */
    void ballPosDir(Vec3 *pos, Vec3 *dir) const;

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

    /** true if given robot is cconected to this robot */
    bool isConnectedTo(sim::robot::SSSA *robot) const;

    /** true if given socket is connected to a robot */
    bool isConnected(const int sockedIdx) const;

    /** returns connected robot on socket 'i' */
    sim::robot::SSSA *connectedRobot(const int idx) const; 

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
