#include "sssa.hpp"
#include <sim/msg.hpp>
#include <sim/ode/body.hpp>

#include "meshes/sssa_body.hpp"
#include "meshes/sssa_arm.hpp"
#include "meshes/sssa_active_wheel.hpp"
#include "meshes/sssa_passive_wheel.hpp"
#include "meshes/sssa_passive_wheel90.hpp"


using sim::Vec3;

namespace sim {

namespace robot {

SSSA::SSSA(sim::World *w, const Vec3 &pos,
                 const osg::Vec4 &color)
    : _world(w), _pos(pos)
{
        int id;
        unsigned long robot_coll_id = (unsigned long)this;

        // no other robot is not connected
        _linkJoint = NULL; 
        _socket1_connection = NULL;
        _socket2_connection = NULL;
        _socket3_connection = NULL;

        // position of secket to which other robots can be connected
        _socket1 = sim::Vec3(0,0.525,0);
        _socket2 = sim::Vec3(0,-0.525,0);
        _socket3 = sim::Vec3(0,0,0.525);
        _armEnd = sim::Vec3(0,0,0.720);

        const double posx = pos[0];
        const double posy = pos[1];
        const double posz = pos[2];
        const sim::Quat rotation(sim::Vec3(0,1,0),M_PI/2);
        //const sim::Quat rotation(sim::Vec3(0,1,0),0);

        _createChasis(color);
        _createArm(osg::Vec4(0.3, 0.2, 0.7, 1.));
        _createArmJoint();
        
        // loading wheels
        if (0) {
            // active wheel 1
            sim::Body *wheel = _world->createBodyTriMesh(sssa_active_wheel_verts,sssa_active_wheel_verts_len,sssa_active_wheel_ids,sssa_active_wheel_ids_len,0.05);
	    	wheel->setPos(posx+0.395,posy+0.351,posz+0.24);
            wheel->visBody()->setColor(0, 0, 1, 1.);
            wheel->activate();
		    wheel->collSetDontCollideId(robot_coll_id);

            sim::Joint *j1 = _world->createJointHinge(_chasis, wheel, sim::Vec3(posx+0.395, posy+0.351, posz+0.24), sim::Vec3(0., 1., 0.));
            j1->activate();



            // active wheel 2
            wheel = _world->createBodyTriMesh(sssa_active_wheel_verts,sssa_active_wheel_verts_len,sssa_active_wheel_ids,sssa_active_wheel_ids_len,0.05);
	    	wheel->setPos(posx+0.395,posy-0.351,posz+0.24);
            wheel->visBody()->setColor(0, 0, 1, 1.);
            wheel->activate();
		    wheel->collSetDontCollideId(robot_coll_id);
            sim::Joint *j2 = _world->createJointHinge(_chasis, wheel, sim::Vec3(posx+0.395, posy-0.351, posz+0.24), sim::Vec3(0., 1., 0.));
            j2->activate();



            // passive wheels on the left side
            sim::Vec3 passiveWheelsPositions[] = {
                sim::Vec3(-0.424,-0.458,0.269),
                sim::Vec3(-0.419,-0.458,-0.160),
                sim::Vec3(-0.159,-0.462,-0.425),
                sim::Vec3( 0.159,-0.462,-0.425),
                sim::Vec3( 0.419,-0.458,-0.160)
            };

            for(int i=0;i<5;i++) {
                wheel = _world->createBodyTriMesh(sssa_passive_wheel_verts,sssa_passive_wheel_verts_len,sssa_passive_wheel_ids,sssa_passive_wheel_ids_len,0.05);
	        	wheel->setPos(posx+passiveWheelsPositions[i][0],posy+passiveWheelsPositions[i][1],posz+passiveWheelsPositions[i][2]);
                wheel->visBody()->setColor(0, 1, 0, 1.);
                wheel->activate();
		        wheel->collSetDontCollideId(robot_coll_id);


                // create hinges for wheels
                sim::Joint *j = _world->createJointHinge(_chasis, wheel, 
                    sim::Vec3(posx+passiveWheelsPositions[i][0], posy+passiveWheelsPositions[i][1], posz+passiveWheelsPositions[i][2]), sim::Vec3(0., 1., 0.));
                j->activate();
            }


            // passiwe wheel on the right side
            sim::Vec3 passiveWheelsPositions90[] = {
                sim::Vec3(-0.424,0.514,0.269),
                sim::Vec3(-0.419,0.458,-0.160),
                sim::Vec3(-0.159,0.458,-0.425),
                sim::Vec3( 0.159,0.458,-0.425),
                sim::Vec3( 0.419,0.458,-0.160),
            };

            for(int i=0;i<5;i++) {
                wheel = _world->createBodyTriMesh(sssa_passive_wheel90_verts,sssa_passive_wheel90_verts_len,sssa_passive_wheel90_ids,sssa_passive_wheel90_ids_len,0.05);
	        	wheel->setPos(posx+passiveWheelsPositions90[i][0],posy+passiveWheelsPositions90[i][1],posz+passiveWheelsPositions90[i][2]);
                wheel->visBody()->setColor(0, 1, 0, 1.);
                wheel->activate();
		        wheel->collSetDontCollideId(robot_coll_id);
                
                sim::Joint *j = _world->createJointHinge(_chasis, wheel, 
                        sim::Vec3(posx+passiveWheelsPositions90[i][0], posy+passiveWheelsPositions90[i][1], posz+passiveWheelsPositions90[i][2]), sim::Vec3(0., 1., 0.));
                j->activate();
            }
        } // if make wheel
}

void SSSA::_createChasis(const osg::Vec4 &color)
{
    sim::ode::BodyCompound *b;
    int id;

    b = (sim::ode::BodyCompound *)_world->createBodyCompound();
    id = b->addTriMesh(sssa_body_verts, sssa_body_verts_len,
                       sssa_body_ids, sssa_body_ids_len);
    b->visBody(id)->setColor(color);

    // TODO: find out bounding box of trimesh
    b->setMassBox(Vec3(1., 1., 1.), 1.);

    b->setPos(_pos);
    b->collSetDontCollideId((unsigned long)this);

    _chasis = b;
}

void SSSA::_createArm(const osg::Vec4 &color)
{
    sim::ode::BodyCompound *b;
    int id;
    // TODO: find out correct offset
    Vec3 offset(0., 0., 0.45);

    b = (sim::ode::BodyCompound *)_world->createBodyCompound();
    id = b->addTriMesh(sssa_arm_verts, sssa_arm_verts_len,
                       sssa_arm_ids, sssa_arm_ids_len,
                       SIM_BODY_DEFAULT_VIS,
                       offset);
    b->visBody(id)->setColor(color);

    // TODO: find out bounding box of end of arm
    b->setMassBox(Vec3(0.5, 0.5, 0.1), 1.);

    b->setPos(_pos - offset);

    b->collSetDontCollideId((unsigned long)this);

    _arm = b;
}

void SSSA::_createArmJoint()
{
    _arm_joint = _world->createJointHinge(_chasis, _arm,
                                          _chasis->pos(),
                                          Vec3(0., 1., 0.));
    _arm_joint->setParamBounce(0.01);
    _arm_joint->setParamLimitLoHi(-M_PI / 2., M_PI / 2.);
}



void SSSA::activate()
{
    _chasis->activate();
    _arm->activate();
    _arm_joint->activate();
}

/** return true if two robots can be connected.
  * they can be conected if:
  *  - current robot is not connected to other robot (_linkJoint == NULL) AND
  *  - other robot is proper oriented and close enough 
  */
bool SSSA::canConnect(sim::robot::SSSA *robot) const {

    for(int i=0;i<3;i++) {
        sim::Vec3 otherSocket = robot->socketPosition(0);
    }
    return false;
}

    /** connects two robots */
int SSSA::connect(sim::robot::SSSA *robot) {
    return 0;
}

sim::Vec3 SSSA::socketPosition(const int idx) {
    sim::Vec3 result;
    if (idx == 0) {
        result = _chasis->rot()*_socket1;
    } else if (idx == 1) {
        result = _chasis->rot()*_socket2;
    } else {
        result = _chasis->rot()*_socket3;
    }

    result += _chasis->pos();
    return result;
}

sim::Vec3 SSSA::armPosition() const {
    sim::Vec3 result = _chasis->rot()*(_arm->rot()*_armEnd);
    result += _chasis->pos();
    return result;
}



bool SSSA::isConnectedTo(sim::robot::SSSA *robot) const {
    if (robot && (_socket1_connection == robot || _socket2_connection == robot || _socket3_connection == robot)) {
        return true;
    }
    return false;
}

bool SSSA::isConnected(const int idx) const {
    if (idx == 0 && _socket1_connection) {
        return true;
    }
    if (idx == 1 && _socket2_connection) {
        return true;
    }

    if (idx == 2 && _socket3_connection) {
        return true;
    }
    return false;
}

sim::robot::SSSA *SSSA::connectedRobot(const int idx) const {
    if (idx == 0) {
        return _socket1_connection;
    } else if (idx == 1) {
        return _socket2_connection;
    } else if (idx == 2) {
        return _socket3_connection;
    }
    return NULL;
}




}

}
