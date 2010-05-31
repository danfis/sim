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

    // joint between two robots
    sim::Joint *_linkJoint;

    sim::Vec3 _socket1;
    sim::Vec3 _socket2;
    sim::Vec3 _socket3;
    sim::Vec3 _armEnd;

    // noy NULL if the robotis iss connected to the referenced robot
    sim::robot::SSSA *_socket1_connection;
    sim::robot::SSSA *_socket2_connection;
    sim::robot::SSSA *_socket3_connection;

  public:
    SSSA(sim::World *w, const sim::Vec3 &pos = sim::Vec3(0., 0., 0.08),
            const osg::Vec4 &chasis_color = osg::Vec4(0., 0.1, 0.7, 0.6));

    sim::Body *chassis() const { return _chasis; }
    sim::Body *arm() const { return _arm; }
    sim::Joint *armJoint() const { return _arm_joint; } 
    sim::Joint *linkJoint() const { return _linkJoint; }

    sim::Vec3 socketPosition(const int idx);
    sim::Vec3 armPosition() const;

    /** returns number of maximum 'passive' connections */
    int maxConnections() const { return 3; }

    /** returns true if current robot can be connected to 'robot' */
    bool canConnect(sim::robot::SSSA *robot) const;

    /** connects two robots */
    int connect(sim::robot::SSSA *robot);

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
};

} /* namespace robot */

} /* namespace sim */

#endif /* _SIM_ROBOT_SYROTEK_HPP_ */
