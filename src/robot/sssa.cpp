/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *                   Vojta Vonasek <vonasek@labe.felk.cvut.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "sssa.hpp"
#include <sim/msg.hpp>
#include <sim/common.hpp>

#include "meshes/sssa_body.hpp"
#include "meshes/sssa_arm.hpp"
#include "meshes/sssa_active_wheel.hpp"
#include "meshes/sssa_passive_wheel.hpp"
#include "meshes/sssa_passive_wheel90.hpp"


using sim::Vec3;

namespace sim {

namespace robot {

SSSA::SSSA(sim::World *w, const Vec3 &pos,
           const Quat &rot, const osg::Vec4 &color)
    : _world(w), _pos(pos), _rot(rot), _color(color),
      _chasis(0),
      _ball_conn(0), _ball_joint(0)
{
    for (size_t i = 0; i < 3; i++){
        _sock_conn[i] = 0;
    }
}

SSSA::~SSSA()
{
}

osg::Vec4 SSSA::chasisColor() const
{
    std::list<const VisBody *> bodies;
    chasis()->visBodyAll(&bodies);

    if (bodies.size() > 0){
        return bodies.front()->color();
    }
    return osg::Vec4(0., 0., 0., 1.);
}

void SSSA::setChasisColor(const osg::Vec4 &color)
{
    std::list<VisBody *> bodies;
    chasis()->visBodyAll(&bodies);

    for_each(std::list<VisBody *>::iterator, bodies){
        (*it)->setColor(color);
    }
}

Scalar SSSA::armAngle() const
{
    return _arm.joint->angle();
}

void SSSA::fixArm()
{
    Scalar angle = armAngle();

    _arm.joint->setParamLimitLoHi(angle, angle);
    _arm.fixed = true;

    setVelArm(0.);
}
void SSSA::unfixArm()
{
    _arm.joint->setParamLimitLoHi(-M_PI / 2., M_PI / 2.);
    _arm.fixed = false;
}

bool SSSA::reachArmAngle(Scalar angle)
{
    const Scalar eps = 0.01;
    const Scalar gain = 0.5;
    Scalar error;

    if (fabs(armAngle() - angle) < eps){
        fixArm();
        return true;
    }else{
        error = armAngle() - angle;
        setVelArm(-error * gain);
        return false;
    }
}

void SSSA::setVelArm(sim::Scalar vel)
{
    _arm.vel = vel;
    _arm.joint->setParamVel(vel);

    if (!isZero(vel)){
        unfixArm();
    }else if (!_arm.fixed){
        fixArm();
    }
}

void SSSA::addVelArm(sim::Scalar d)
{
    setVelArm(_arm.vel + d);
}


void SSSA::setVelLeft(sim::Scalar v)
{
    _wleft.vel = v;
    for (size_t i = 0; i < 6; i++)
        _wleft.joint[i]->setParamVel(_wleft.vel);
}

void SSSA::addVelLeft(sim::Scalar d)
{
    setVelLeft(_wleft.vel + d);
}

void SSSA::setVelRight(sim::Scalar v)
{
    _wright.vel = v;
    for (size_t i = 0; i < 6; i++)
        _wright.joint[i]->setParamVel(_wright.vel);
}

void SSSA::addVelRight(sim::Scalar d)
{
    setVelRight(_wright.vel + d);
}


void SSSA::socketSetup(size_t idx, Vec3 *pos, Vec3 *dir, Vec3 *up) const
{
    if (idx >= 3)
        return;

    const Scalar off = 0.525;

    Vec3 offset; // offset from 0., 0., 0. position
    Vec3 uporig;

    if (idx == 0){
        offset = Vec3(0., off, 0.);
        uporig = Vec3(1., 0., 0.);
    }else if (idx == 1){
        offset = Vec3(off, 0., 0.);
        uporig = Vec3(0., 0., -1.);
    }else if (idx == 2){
        offset = Vec3(0., -off, 0.);
        uporig = Vec3(1., 0., 0.);
    }

    // get direction vector
    *dir = _chasis->rot() * offset;

    // get up vector
    *up = _chasis->rot() * uporig;

    // position is center of chasis plus *dir vector
    *pos = _chasis->pos() + *dir;

    // scale *dir vector to have unit length
    dir->normalize();
    up->normalize();
}

void SSSA::ballSetup(Vec3 *pos, Vec3 *dir, Vec3 *up) const
{
    //const Scalar off = 0.720;
    const Vec3 offset(-0.07, 0., 0.); // offset from center of arm

    *dir = _arm.body->rot() * offset;
    *up = _arm.body->rot() * Vec3(0., 0., 1.);
    *pos = _arm.body->pos() + *dir;
    dir->normalize();
    up->normalize();
}

int SSSA::canConnectTo(const sim::robot::SSSA &robot) const
{
    // maximal distance between ball and socket
    const Scalar max_dist = 0.01;
    // maximal angle (about x, y and z axis) ball and socket can differ
    const Scalar max_angle = 0.1;
    Vec3 ball_pos, ball_dir, ball_up;
    Vec3 pos, dir, up;
    Scalar dist; // distance between ball and socket
    Scalar rot_diff;

    // obtain position and direction of this robot's ball
    ballSetup(&ball_pos, &ball_dir, &ball_up);
    DBG("ball setup: " << DBGV(ball_pos) << ", " << DBGV(ball_dir));

    for (size_t i = 0; i < 3; i++){
        // check if socket isn't already connected
        if (robot.isSocketConnected(i))
            continue;

        // obtain pos and dir of robot's socket
        robot.socketSetup(i, &pos, &dir, &up);
        DBG("sock " << i << ": " << DBGV(pos) << ", " << DBGV(dir));

        // compute distance between ball and socket
        dist = (ball_pos - pos).length();
        DBG("  dist: " << dist);
        if (dist > max_dist)
            continue;


        // compute rotational difference as rotation from ball's direction
        // vector to socket direction vector - this is reason why is
        // socket's dir reversed
        dir = -dir;
        rot_diff = (dir - ball_dir).length();
        DBG("  rot_diff: " << rot_diff);
        if (rot_diff > max_angle)
            continue;
        // the code above really isn't computation of angle but dir and
        // ball_dir is _always_ unit vectors so this should compute
        // distance from one end to other end of vector
        // For me it looks sufficient solution.

        // TODO
        //rot_diff = (up - ball_up).length();
        //if (rot_diff > max_angle)
        //    continue;

        // if code gets here ball can connect to currently tested socket
        return i;
    }

    return -1;
}

bool SSSA::connectTo(sim::robot::SSSA &robot)
{
    int socket_num;
    Vec3 pos, dir, up;

    socket_num = canConnectTo(robot);
    if (socket_num < 0)
        return false;

    ballSetup(&pos, &dir, &up);
    _ball_conn = &robot;
    _ball_joint = _world->createJointFixed(_arm.body, robot.chasis());
    robot._sock_conn[socket_num] = this;

    _ball_joint->activate();

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
    _createChasis(_color);
    _createArm(osg::Vec4(0.3, 0.2, 0.7, 1.));
    _createWheels();

    _chasis->activate();
    _arm.body->activate();
    _arm.joint->activate();

    for (size_t i = 0; i < 6; i++){
        _wleft.body[i]->activate();
        _wleft.joint[i]->activate();
        _wright.body[i]->activate();
        _wright.joint[i]->activate();
    }

    fixArm();
}


void SSSA::_createChasis(const osg::Vec4 &color)
{
    sim::Body *b;
    int id;

    b = _world->createBodyCompound();
    id = b->addTriMesh(sssa_body_verts, sssa_body_verts_len,
                       sssa_body_ids, sssa_body_ids_len);
    b->visBody(id)->setColor(color);

    const osg::BoundingSphere &bound = b->visBody(id)->node()->getBound();
    // TODO: find mass of chasis
    b->setMassCube(bound.radius() * 2., 1.);

    b->setPos(_pos);
    b->setRot(_rot);
    b->collSetDontCollideId((unsigned long)this);

    _chasis = b;
}

void SSSA::_createArm(const osg::Vec4 &color)
{
    sim::Body *b;
    Vec3 axis = _rot * Vec3(0., 1., 0.);
    int id;
    // TODO: find out correct offset
    Vec3 offset(0.65, 0., 0.);

    b = _world->createBodyCompound();
    id = b->addTriMesh(sssa_arm_verts, sssa_arm_verts_len,
                       sssa_arm_ids, sssa_arm_ids_len,
                       SIM_BODY_DEFAULT_VIS,
                       offset);
    b->visBody(id)->setColor(color);

    // TODO: find out bounding box of end of arm
    b->setMassBox(Vec3(0.14, 0.5, 0.5), 1.);

    b->setPos(_pos - (_rot * offset));
    b->setRot(_rot );

    b->collSetDontCollideId((unsigned long)this);

    _arm.body = b;

    _arm.joint = _world->createJointHinge(_chasis, _arm.body,
                                          _chasis->pos(), axis);

    _arm.joint->setParamBounce(0.01);
    _arm.joint->setParamLimitLoHi(-M_PI / 2., M_PI / 2.);
    //_arm.joint->setParamFMax(50);
    _arm.joint->setParamFMax(200);
    _arm.joint->setParamVel(_arm.vel);
}

void SSSA::_createArmJoint()
{
}


void SSSA::_createWheels()
{
    sim::Joint *j;

    const sim::Vec3 wheel_pos[] = {
        sim::Vec3( 0.250, -0.458, -0.419),
        sim::Vec3( 0.250, -0.458,  0.424),
        sim::Vec3(-0.160, -0.458,  0.419),
        sim::Vec3(-0.425, -0.458,  0.159),
        sim::Vec3(-0.425, -0.458, -0.159),
        sim::Vec3(-0.160, -0.458, -0.419),

        sim::Vec3( 0.250, 0.458, -0.419),
        sim::Vec3( 0.250, 0.458,  0.424),
        sim::Vec3(-0.160, 0.458,  0.419),
        sim::Vec3(-0.425, 0.458,  0.159),
        sim::Vec3(-0.425, 0.458, -0.159),
        sim::Vec3(-0.160, 0.458, -0.419)
    };

    for (size_t i = 0; i < 6; i++){
        // create wheels
        _wleft.body[i] = _world->createBodyCylinderY(0.11, 0.08, 0.1);
        _wright.body[i] = _world->createBodyCylinderY(0.11, 0.08, 0.1);

        // set up position
        _wleft.body[i]->setPos(_pos + (_rot * wheel_pos[i]));
        _wleft.body[i]->setRot(_wleft.body[i]->rot() * _rot);
        _wright.body[i]->setPos(_pos + (_rot * wheel_pos[6 + i]));
        _wright.body[i]->setRot(_wright.body[i]->rot() * _rot);

        // create joints
        j = _world->createJointHinge(_chasis, _wleft.body[i],
                                     _wleft.body[i]->pos(),
                                     _rot * Vec3(0., 1., 0.));
        _wleft.joint[i] = j;
        j = _world->createJointHinge(_chasis, _wright.body[i],
                                     _wright.body[i]->pos(),
                                     _rot * Vec3(0., 1., 0.));
        _wright.joint[i] = j;


        _wleft.body[i]->collSetDontCollideId((unsigned long)this);
        _wright.body[i]->collSetDontCollideId((unsigned long)this);

        _wleft.joint[i]->setParamFMax(100);
        _wleft.joint[i]->setParamVel(0.);
        _wright.joint[i]->setParamFMax(100);
        _wright.joint[i]->setParamVel(0.);
    }
}


}

}
