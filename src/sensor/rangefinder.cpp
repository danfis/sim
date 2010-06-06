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

#include <osg/ShapeDrawable>

#include "rangefinder.hpp"
#include "sim/msg.hpp"

namespace sim {
namespace sensor {


RangeFinder::RangeFinder(Scalar max_range, size_t num_beams, Scalar angle_range)
    : sim::Component(),
      _max_range(max_range), _num_beams(num_beams), _angle_range(angle_range),
      _offset_pos(0., 0., 0.), _offset_rot(0., 0., 0., 1.),
      _vis_enabled(false)
{
    _intersectors = new osgUtil::IntersectorGroup;
    _visitor = new osgUtil::IntersectionVisitor(_intersectors);

    _data.detected = new bool[_num_beams];
    _data.dist = new Scalar[_num_beams];
    _data.point = new Vec3[_num_beams];
    _data.local = new Vec3[_num_beams];

    for (size_t i = 0; i < _num_beams; i++){
        _data.detected[i] = false;
    }
}

RangeFinder::~RangeFinder()
{
    delete [] _data.detected;
    delete [] _data.dist;
    delete [] _data.point;
    delete [] _data.local;
}

void RangeFinder::init(sim::Sim *sim)
{
    _sim = sim;

    _sim->regPreStep(this);

    _createIntersectors();

    if (_vis_enabled)
        _createVis();

    if (_vis.valid())
        _sim->visWorld()->addOffScene(_vis.get());

    _updatePosition();
}

void RangeFinder::finish()
{
    if (_vis.valid())
        _sim->visWorld()->rmOffScene(_vis.get());
}

void RangeFinder::cbPreStep()
{
    osg::Node *root;

    if (_body)
        _updatePosition();

    // get root of scene
    root = _sim->visWorld()->sceneRoot();

    // traverse scene
    root->accept(*_visitor.get());

    // gather measured data
    osgUtil::LineSegmentIntersector *is;
    osgUtil::LineSegmentIntersector::Intersection inter;
    osgUtil::IntersectorGroup::Intersectors &its = _intersectors->getIntersectors();

    for (size_t i = 0; i < _num_beams; i++){
        is = (osgUtil::LineSegmentIntersector *)its[i].get();

        if (is->containsIntersections()){
            inter = is->getFirstIntersection();

            _data.detected[i] = true;
            _data.point[i] = inter.getWorldIntersectPoint();
            _data.local[i] = inter.getLocalIntersectPoint();
            _data.dist[i] = _data.local[i].length();

            //std::cerr << _data.detected[i] << ":" << _data.dist[i] << " ";
        }else{
            _data.detected[i] = false;
            _data.dist[i] = _max_range;
            _data.point[i] = is->getEnd();
            _data.local[i].set(0., 0., 0.); // TODO: This must be computed
        }

        // clear list of intersections
        is->getIntersections().clear();
    }

    //std::cerr << std::endl;

    if (_vis.valid())
        _updateVis();
}


void RangeFinder::_createIntersectors()
{
    osgUtil::LineSegmentIntersector *beam;

    for (size_t i = 0; i < _num_beams; i++){
        beam = new osgUtil::LineSegmentIntersector(Vec3(0., 0., 0.), Vec3(0., 0., 0.));
        _intersectors->addIntersector(beam);
    }
}

void RangeFinder::_createVis()
{
    Scalar size = 0.01;
    osg::Box *box;
    osg::ShapeDrawable *draw;
    osg::Geode *geode;
    osg::PositionAttitudeTransform *tr;

    _vis = new osg::Group;

    for (size_t i = 0; i < _num_beams; i++){
        box = new osg::Box(Vec3(0., 0., 0.), size);

        draw = new osg::ShapeDrawable(box);
        draw->setColor(osg::Vec4(1., 0., 0., 1.));

        geode = new osg::Geode;
        geode->addDrawable(draw);

        tr = new osg::PositionAttitudeTransform;
        tr->setAttitude(Quat(0., 0., 0, 1.));
        tr->setPosition(Vec3(0., 0., 0.));

        tr->addChild(geode);

        _vis->addChild(tr);
    }

    box = new osg::Box(Vec3(0., 0., 0.), size);

    draw = new osg::ShapeDrawable(box);
    draw->setColor(osg::Vec4(0., 0., 0., 1.));

    geode = new osg::Geode;
    geode->addDrawable(draw);

    tr = new osg::PositionAttitudeTransform;
    tr->setAttitude(Quat(0., 0., 0, 1.));
    tr->setPosition(Vec3(0., 0., 0.));

    tr->addChild(geode);

    _vis->addChild(tr);
}


void RangeFinder::_updatePosition()
{
    Vec3 pos;
    Quat rot;

    if (_body){
        rot = _offset_rot * _body->rot();
        pos = (rot * _offset_pos) + _body->pos();
    }else{
        pos = _offset_pos;
        rot = _offset_rot;
    }

    Scalar angle, angle_step;
    Vec3 from(0., 0., 0.), to, to_dir(_max_range, 0., 0.);

    angle = -_angle_range / 2.;
    angle_step = _angle_range / (Scalar)(_num_beams - 1);

    osgUtil::LineSegmentIntersector *is;
    osgUtil::IntersectorGroup::Intersectors &its = _intersectors->getIntersectors();
    for (size_t i = 0; i < _num_beams; i++){
        is = (osgUtil::LineSegmentIntersector *)its[i].get();

        from = Vec3(0., 0., 0.);
        to = Quat(Vec3(0., 0., 1.), angle) * to_dir;

        from += pos;
        to += pos;
        to = rot * to;

        is->setStart(from);
        is->setEnd(to);

        angle += angle_step;
    }
}

void RangeFinder::_updateVis()
{
    osg::PositionAttitudeTransform *tr;
    osg::ShapeDrawable *draw;

    for (size_t i = 0; i < _num_beams; i++){
        tr = (osg::PositionAttitudeTransform *)_vis->getChild(i);
        tr->setPosition(_data.point[i]);

        draw = (osg::ShapeDrawable *)((osg::Geode *)tr->getChild(0))->getDrawable(0);
        if (_data.detected[i]){
            draw->setColor(osg::Vec4(1., 0., 0., 1.));
        }else{
            draw->setColor(osg::Vec4(1., 0., 1., 1.));
        }
    }


    osgUtil::LineSegmentIntersector *is;
    osgUtil::IntersectorGroup::Intersectors &its = _intersectors->getIntersectors();
    is = (osgUtil::LineSegmentIntersector *)its[0].get();
    tr = (osg::PositionAttitudeTransform *)_vis->getChild(_num_beams);
    tr->setPosition(is->getStart());
}

}
}
