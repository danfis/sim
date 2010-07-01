/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "syrotek.hpp"
#include <sim/msg.hpp>

using sim::Vec3;

#define SIZE(a) (sizeof(a) / sizeof(*a))

static Vec3 verts[] = { Vec3( 0.117,  0.045, 0.),
                        Vec3( 0.117, -0.045, 0.),
                        Vec3( 0.080, -0.081, 0.),
                        Vec3(-0.020, -0.081, 0.),
                        Vec3(-0.057, -0.045, 0.),
                        Vec3(-0.057,  0.045, 0.),
                        Vec3(-0.020,  0.081, 0.),
                        Vec3( 0.080,  0.081, 0.),
                        Vec3( 0.117,  0.045, 0.06),
                        Vec3( 0.117, -0.045, 0.06),
                        Vec3( 0.080, -0.081, 0.06),
                        Vec3(-0.020, -0.081, 0.06),
                        Vec3(-0.057, -0.045, 0.06),
                        Vec3(-0.057,  0.045, 0.06),
                        Vec3(-0.020,  0.081, 0.06),
                        Vec3( 0.080,  0.081, 0.06),

                        Vec3(0.105,  0.030, 0.06),
                        Vec3(0.105, -0.030, 0.06),
                        Vec3(0.054, -0.059, 0.06),
                        Vec3(-0.032, -0.059, 0.06),
                        Vec3(-0.044, -0.050, 0.06),
                        Vec3(-0.044,  0.050, 0.06),
                        Vec3(-0.032,  0.059, 0.06),
                        Vec3(0.054,  0.059, 0.06),
                        Vec3(0.105,  0.030, 0.18),
                        Vec3(0.105, -0.030, 0.18),
                        Vec3(0.054, -0.059, 0.18),
                        Vec3(-0.032, -0.059, 0.18),
                        Vec3(-0.044, -0.050, 0.18),
                        Vec3(-0.044,  0.050, 0.18),
                        Vec3(-0.032,  0.059, 0.18),
                        Vec3(0.054,  0.059, 0.18),
};
static unsigned int ids[] = { 0, 1, 2,
                              0, 2, 3,
                              0, 3, 4,
                              0, 4, 5,
                              0, 5, 6,
                              0, 6, 7,
                              8, 9, 10,
                              8, 10, 11,
                              8, 11, 12,
                              8, 12, 13,
                              8, 13, 14,
                              8, 14, 15,
                              0, 8, 9,
                              0, 9, 1,
                              1, 9, 10,
                              1, 10, 2,
                              2, 10, 11,
                              2, 11, 3,
                              3, 11, 12,
                              3, 12, 4,
                              4, 12, 13,
                              4, 13, 5,
                              5, 13, 14,
                              5, 14, 6,
                              6, 14, 15,
                              6, 15, 7,
                              7, 15, 8,
                              7, 8, 0,

                              /*
                                 16, 17, 18,
                                 16, 18, 19,
                                 16, 19, 20,
                                 16, 20, 21,
                                 16, 21, 22,
                                 16, 22, 23,
                               */
                              24, 25, 26,
                              24, 26, 27,
                              24, 27, 28,
                              24, 28, 29,
                              24, 29, 30,
                              24, 30, 31,
                              16, 24, 25,
                              16, 25, 17,
                              1 + 16, 9 + 16, 10 + 16,
                              1 + 16, 10 + 16, 2 + 16,
                              2 + 16, 10 + 16, 11 + 16,
                              2 + 16, 11 + 16, 3 + 16,
                              3 + 16, 11 + 16, 12 + 16,
                              3 + 16, 12 + 16, 4 + 16,
                              4 + 16, 12 + 16, 13 + 16,
                              4 + 16, 13 + 16, 5 + 16,
                              5 + 16, 13 + 16, 14 + 16,
                              5 + 16, 14 + 16, 6 + 16,
                              6 + 16, 14 + 16, 15 + 16,
                              6 + 16, 15 + 16, 7 + 16,
                              7 + 16, 15 + 16, 8 + 16,
                              7 + 16, 8 + 16, 0 + 16
};


namespace sim {

namespace robot {

Syrotek::Syrotek(sim::World *w, const Vec3 &pos,
                 const osg::Vec4 &chasis_color)
    : _world(w), _pos(pos)
{
        int id;
        unsigned long robot_coll_id = (unsigned long)this;

        // create chasis
        _chasis = _world->createBodyCompound();
        id = _chasis->addTriMesh(verts, SIZE(verts), ids, SIZE(ids));
        _chasis->visBody(id)->setColor(chasis_color);
        _chasis->setPos(_pos);
        _chasis->setMassBox(Vec3(0.15, 0.15, 0.18), 2.);
        _chasis->collSetDontCollideId(robot_coll_id);
        //DBG("chasis: " << _chasis);

        _wheel[0] = _world->createBodyCylinderY(0.04, 0.01, 0.1);
        _wheel[0]->visBody()->setColor(1., 1., 1., 1.);
        _wheel[0]->setPos(_pos + Vec3(0.03, 0.14 / 2., 0.04 - 0.007));
        _wheel[0]->collSetDontCollideId(robot_coll_id);
        //DBG("_wheel[0]: " << _wheel[0]);

        _wheel[1] = _world->createBodyCylinderY(0.04, 0.01, 0.1);
        _wheel[1]->visBody()->setColor(1., 1., 1., 1.);
        _wheel[1]->setPos(_pos + Vec3(0.03, -0.14 / 2., 0.04 - 0.007));
        _wheel[1]->collSetDontCollideId(robot_coll_id);
        //DBG("_wheel[1]: " << _wheel[1]);

        _ball[0] = _world->createBodySphere(0.01, 0.1);
        _ball[0]->visBody()->setColor(0., 0., 0., 1.);
        _ball[0]->setPos(_pos + Vec3(0.143 / 2. + 0.03, 0., 0. + 0.01 - 0.0065));
        _ball[0]->collSetDontCollideId(robot_coll_id);
        //DBG("_ball[0]: " << _ball[0]);

        _ball[1] = _world->createBodySphere(0.01, 0.1);
        _ball[1]->visBody()->setColor(0., 0., 0., 1.);
        _ball[1]->setPos(_pos + Vec3(-0.143 / 2. + 0.03, 0., 0. + 0.01 - 0.0065));
        _ball[1]->collSetDontCollideId(robot_coll_id);
        //DBG("_ball[1]: " << _ball[1]);

        _jball[0] = _world->createJointFixed(_chasis, _ball[0]);
        _jball[1] = _world->createJointFixed(_chasis, _ball[1]);

        _jwheel[0] = _world->createJointHinge(_chasis, _wheel[0], _wheel[0]->pos(),
                                              Vec3(0., -1., 0.));
        _jwheel[1] = _world->createJointHinge(_chasis, _wheel[1], _wheel[1]->pos(),
                                              Vec3(0., -1., 0.));

        _jwheel[0]->setParamVel(0.);
        _jwheel[0]->setParamFMax(100.);
        _jwheel[1]->setParamVel(0.);
        _jwheel[1]->setParamFMax(100.);
}

Vec3 Syrotek::odometry()
{
    Scalar x, y, phi;
    const Vec3 &p = pos();

    x = p.x() - _odo_pos.x();
    y = p.y() - _odo_pos.y();
    Quat().slerp(phi, _odo_rot, rot());

    return Vec3(x, y, phi);
}

void Syrotek::resetOdometry()
{
    _odo_pos = pos();
    _odo_rot = rot();
}

void Syrotek::activate()
{
    _chasis->activate();
    _wheel[0]->activate();
    _wheel[1]->activate();
    _ball[0]->activate();
    _ball[1]->activate();
    _jball[0]->activate();
    _jball[1]->activate();
    _jwheel[0]->activate();
    _jwheel[1]->activate();

    resetOdometry();
}

void Syrotek::setColor(const osg::Vec4 &color)
{
    std::list<VisBody *> list;
    _chasis->visBodyAll(&list);
    if (list.size() > 0)
        list.front()->setColor(color);
}


void Syrotek::setVelLeft(Scalar v)
{
    _vel[0] = v;
    _jwheel[0]->setParamVel(v);
}

void Syrotek::addVelLeft(Scalar d)
{
    _vel[0] += d;
    _jwheel[0]->setParamVel(_vel[0]);
}


void Syrotek::setVelRight(Scalar v)
{
    _vel[1] = v;
    _jwheel[1]->setParamVel(v);
}

void Syrotek::addVelRight(Scalar d)
{
    _vel[1] += d;
    _jwheel[1]->setParamVel(_vel[1]);
}


}

}
