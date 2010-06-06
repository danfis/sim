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

#include <stdio.h>
#include <osgDB/WriteFile>
#include <osg/ShapeDrawable>

#include "camera.hpp"
#include <sim/msg.hpp>


namespace sim {
namespace sensor {

Camera::Camera()
    : sim::Component(),
      _cam(0), _image(0), _vis(0), _vis_enabled(false),
      _eye(0., 0., 0.), _at(0., 0., 0.), _zaxis(0., 0., 1.),
      _bgcolor(0.1, 0.1, 0.3, 1.),
      _width(100), _height(100),
      _body(0), _body_offset_pos(0., 0., 0.), _body_offset_rot(0., 0., 0., 1.),
      _dump_prefix(0)

{
    // create visual representation
    _vis = new VisBodyCone(0.005, 0.03);
    _vis->setColor(1., 0., 0., 1.);
}

Camera::~Camera()
{
}

void Camera::init(sim::Sim *sim)
{
    _sim = sim;


    if (_dump_prefix)
        _sim->regPreStep(this);

    _sim->regPostStep(this);

    _createCamera();
    _sim->visWorld()->addCam(_cam);

    if (_view_enabled){
        _createView();
        _sim->visWorld()->addView(_view);
    }

    if (_vis_enabled && _vis)
        _sim->visWorld()->addBody(_vis);

    _updatePosition();
}

void Camera::finish()
{
    _sim->visWorld()->rmCam(_cam);

    if (_view.valid())
        _sim->visWorld()->rmView(_view);

    if (_vis_enabled && _vis)
        _sim->visWorld()->rmBody(_vis);
}

void Camera::cbPreStep()
{
    static size_t counter = 0;
    static char fn[100];

    if (_dump_prefix && _image->valid()){
        sprintf(fn, "%s%06d.png", _dump_prefix, counter);
        osgDB::writeImageFile(*_image, fn);
        counter++;
    }
}

void Camera::cbPostStep()
{
    _updatePosition();
}

void Camera::attachToBody(const sim::Body *b, const Vec3 &pos, const Quat &rot)
{
    _body = b;
    _body_offset_pos = pos;
    _body_offset_rot = rot;
}

void Camera::_createCamera()
{
    Scalar aspect = (Scalar)_width / (Scalar)_height;

    // delete camera if already created
    if (_cam)
        _cam->unref();

    // set up camera
    _cam = new osg::Camera;
    _cam->setClearColor(_bgcolor);
    _cam->setProjectionMatrixAsPerspective(30, aspect, 0.01, 10.);
    _cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    _cam->setViewport(0, 0, _width, _height);
    _cam->setRenderOrder(osg::Camera::PRE_RENDER);

    // attach image to camera - each frame will be stored in _image
    _image = new osg::Image;
    //_image->allocateImage(width, height, 1, GL_RGBA, GL_FLOAT);
    _cam->attach(osg::Camera::COLOR_BUFFER, _image);

}

void Camera::_createView()
{
    Scalar aspect = (Scalar)_width / (Scalar)_height;

    if (_view.valid())
        _view->unref();

    // set up graphics context
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = _width;
    traits->height = _height;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;

    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

    // set up camera
    osg::Camera *cam = new osg::Camera;
    cam->setClearColor(_bgcolor);
    cam->setProjectionMatrixAsPerspective(30, aspect, 0.01, 10.);
    cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    cam->setViewport(0, 0, _width, _height);
    cam->setGraphicsContext(gc.get());

    // create view
    _view = new osgViewer::View;

    // assign camera
    _view->setCamera(cam);
}

void Camera::_updatePosition()
{
    if (_body){
        const Vec3 &pos = _body->pos();
        const Quat &rot = _body->rot();

        // Initial view matrix of camera is: eye is (0., 0., 0.),
        // looking at point (1., 0., 0.) and zaxis is (0., 0., 1.)
        // When applying body's transformation, eye position must moved to
        // pos from origin and rotated position offset must be added.
        // At point is computed this way: first direction vector must be
        // computed (= _body_offset_rot * Vec3(1., 0., 0.)), on this
        // direction vector must be applied rotation of body and finaly at
        // point is eye moved by direction vector.
        // Z axis is simply rotated zaxis by offset first then by body's
        // rotation.
        _eye = pos + (rot * _body_offset_pos);
        _at = rot * (_body_offset_rot * Vec3(1., 0., 0.)) + _eye;
        _zaxis = rot * (_body_offset_rot * Vec3(0., 0., 1.));
    }

    // transform camera
    _cam->setViewMatrixAsLookAt(_eye, _at, _zaxis);

    // transform camera in view
    if (_view.valid() && _view->getCamera()){
        _view->getCamera()->setViewMatrixAsLookAt(_eye, _at, _zaxis);
    }

    // transform VisBody
    if (_vis_enabled && _vis){
        // direction of camera
        Vec3 dir = _at - _eye;

        // rotate VisBody from z axis to direction
        Quat rot;
        rot.makeRotate(Vec3(0., 0., 1.), -dir);

        // set position and rotation
        _vis->setPos(_eye - (dir * 0.03));
        _vis->setRot(rot);
    }
}


}
}
