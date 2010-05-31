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
    : _world(w), _pos(pos),
      _chasis(0), _arm(0), _arm_joint(0),
      _ball_conn(0), _ball_joint(0)
{
    for (size_t i = 0; i < 3; i++){
        _sock_conn[i] = 0;
    }

    for (size_t i = 0; i < 4; i++){
        _wheel_right[i] = _wheel_left[i] = 0;
        _wheel_right_joint[i] = _wheel_left_joint[i] = 0;
    }

    _createChasis(color);
    _createArm(osg::Vec4(0.3, 0.2, 0.7, 1.));
    _createArmJoint();
    _createWheels();
}


void SSSA::socketPosDir(size_t idx, Vec3 *pos, Vec3 *dir) const
{
    if (idx >= 3)
        return;

    const Scalar off = 0.525;

    Vec3 offset; // offset from 0., 0., 0. position

    if (idx == 0){
        offset = Vec3(0., off, 0.);
    }else if (idx == 1){
        offset = Vec3(0., 0., off);
    }else if (idx == 2){
        offset = Vec3(0., -off, 0.);
    }

    // get direction vector
    *dir = _chasis->rot() * offset;

    // position is center of chasis plus *dir vector
    *pos = _chasis->pos() + *dir;

    // scale *dir vector to have unit length
    dir->normalize();
}

void SSSA::ballPosDir(Vec3 *pos, Vec3 *dir) const
{
    //const Scalar off = 0.720;
    const Vec3 offset(0., 0., -0.2); // offset from center of arm

    *dir = _arm->rot() * offset;
    *pos = _arm->pos() + *dir;
    dir->normalize();
}

int SSSA::canConnectTo(const sim::robot::SSSA &robot) const
{
    // maximal distance between ball and socket
    const Scalar max_dist = 0.5;
    // maximal angle (about x, y and z axis) ball and socket can differ
    const Scalar max_angle = 0.1;
    Vec3 ball_pos, ball_dir;
    Vec3 pos, dir;
    Scalar dist; // distance between ball and socket
    Scalar rot_diff;

    // obtain position and direction of this robot's ball
    ballPosDir(&ball_pos, &ball_dir);

    for (size_t i = 0; i < 3; i++){
        // check if socket isn't already connected
        if (robot.isSocketConnected(i))
            continue;

        // obtain pos and dir of robot's socket
        robot.socketPosDir(i, &pos, &dir);

        // compute distance between ball and socket
        dist = (ball_pos - pos).length();
        if (dist > max_dist)
            continue;


        // compute rotational difference as rotation from ball's direction
        // vector to socket direction vector - this is reason why is
        // socket's dir reversed
        dir = -dir;
        rot_diff = (dir - ball_dir).length();
        if (rot_diff > max_angle)
            break;
        // the code above really isn't computation of angle but dir and
        // ball_dir is _always_ unit vectors so this should compute
        // distance from one end to other end of vector
        // For me it looks sufficient solution.

        // if code gets here ball can connect to currently tested socket
        return i;
    }

    return -1;
}

bool SSSA::connectTo(sim::robot::SSSA &robot)
{
    int socket_num;

    socket_num = canConnectTo(robot);
    if (socket_num < 0)
        return false;

    // TODO

    return false;
}

bool SSSA::isConnected(const sim::robot::SSSA *robot) const
{
    return _ball_conn == robot;
}

bool SSSA::isSocketConnectedTo(size_t idx, const sim::robot::SSSA *robot) const
{
    return _sock_conn[idx] == robot;
}

bool SSSA::isAnySocketConnectedTo(const sim::robot::SSSA *robot) const
{
    for (size_t i = 0; i < 3; i++){
        if (_sock_conn[i] == robot)
            return true;
    }
    return false;
}


void SSSA::activate()
{
    _chasis->activate();
    _arm->activate();
    _arm_joint->activate();

#if 0
    for (size_t i = 0; i < 4; i++){
        _wheel_left[i]->activate();
        _wheel_left_joint[i]->activate();
        _wheel_right[i]->activate();
        _wheel_right_joint[i]->activate();
    }
#endif
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


void SSSA::_createWheels()
{
    // TODO

    // loading wheels
#if 0
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
#endif
}


}

}
