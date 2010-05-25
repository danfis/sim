#include <stdio.h>
#include <osgDB/WriteFile>
#include <osg/ShapeDrawable>

#include "camera.hpp"
#include <sim/msg.hpp>


namespace sim {
namespace sensor {

Camera::Camera()
    : sim::Component(),
      _eye(-0.8, 0., 0.1), _at(1., 0., 0.1), _zaxis(0., 0., 1.),
      _body(0)

{
}

Camera::~Camera()
{
}

void Camera::init(sim::Sim *sim)
{
    _sim = sim;

    _createCamera();

    _sim->visWorld()->addCam(_cam);

    _sim->regPreStep(this);
    _sim->regPostStep(this);
}

void Camera::finish()
{
    _sim->visWorld()->rmCam(_cam);
}

void Camera::cbPreStep()
{
    static size_t counter = 0;
    static char fn[100];

    if (_image->valid()){
        DBG(counter);
        sprintf(fn, "%06d.png", counter);
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
    _body_offset_pos = Vec3(0., 0., 0.2);
    _body_offset_rot = rot;
    _body_offset_rot = Quat(Vec3(0., 0., 1.), M_PI / 2.);
}

void Camera::_createCamera()
{
    osg::Vec4 bgcolor(0.1, 0.1, 0.3, 1.);
    int x, y, width, height;

    x = 0;
    y = 0;
    width = 512;
    height = 512;

    // set up camera
    if (_cam)
        _cam->unref();

    _cam = new osg::Camera;
    _cam->setClearColor(bgcolor);
    //_cam->setProjectionMatrixAsFrustum(-10, 10, -10, 10, -30, 30);
    _cam->setProjectionMatrixAsPerspective(30, (double)width / (double)height, 0.01, 10.);
    _cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    _cam->setViewport(x, y, width, height);
    _cam->setViewMatrixAsLookAt(_eye, _at, _zaxis);
    _cam->setRenderOrder(osg::Camera::PRE_RENDER);

    /*
    //_cam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER);
    DBG("Render Target:");
    DBG("Frame buffer object: " << osg::Camera::FRAME_BUFFER_OBJECT);
    DBG("Pixel buffer rtt: " << osg::Camera::PIXEL_BUFFER_RTT);
    DBG("Pixel buffer: " << osg::Camera::PIXEL_BUFFER);
    DBG("Frame buffer: " << osg::Camera::FRAME_BUFFER);
    DBG("Sep window: " << osg::Camera::SEPERATE_WINDOW);
    DBG("cur: " << _cam->getRenderTargetImplementation());
    */


    // attach image to camera - each frame will be stored in _image
    _image = new osg::Image;
    //_image->allocateImage(width, height, 1, GL_RGBA, GL_FLOAT);
    _cam->attach(osg::Camera::COLOR_BUFFER, _image);

    // create visual representation
    {
        // direction of camera
        Vec3 dir = _at - _eye;

        // rotate VisBody from z axis to direction
        Quat rot;
        rot.makeRotate(Vec3(0., 0., 1.), -dir);

        _vis = new VisBodyCone(0.005, 0.03);
        // place cone behind the camera!
        _vis->setPos(_eye - (dir * 0.03));
        _vis->setRot(rot);
        _vis->setColor(1., 0., 0., 1.);

        _sim->visWorld()->addBody(_vis);
    }
}

void Camera::_updatePosition()
{
    static size_t counter = 0;

    if (_body){
        const Vec3 &pos = _body->pos();
        const Quat &rot = _body->rot();

        // position of camera is pos, rot is rotation of camera from x-axis
        // direction
        _eye = pos + _body_offset_pos;
        _at = _eye + ((rot * _body_offset_rot) * Vec3(1., 0., 0.));
        _zaxis = rot * Vec3(0., 0., 1.);

        /*
        osg::Matrixd matrix;
        matrix.makeIdentity();
        matrix.makeTranslate(pos + Vec3(0., 0., 10.));
        matrix.makeRotate(rot);
        _cam->setViewMatrix(matrix);
        //_cam->setViewMatrixAsLookAt(Vec3(0., 0., 10), pos, _zaxis);
        {
            osg::Vec3f eye, at, zaxis;
            _cam->getViewMatrix().getLookAt(eye, at, zaxis);
            DBG(counter << ": " << _image->valid() << " " << DBGV(pos));
            DBG(DBGV(eye) << " " << DBGV(at) << " " << DBGV(zaxis));
        }
        */

        _cam->setViewMatrixAsLookAt(_eye, _at, _zaxis);

        {
            // direction of camera
            Vec3 dir = _at - _eye;

            // rotate VisBody from z axis to direction
            Quat rot;
            rot.makeRotate(Vec3(0., 0., 1.), -dir);
            _vis->setPos(_eye - (dir * 0.03));
            _vis->setRot(rot);
            _vis->setColor(1., 0., 0., 1.);
        }
    }

    counter++;
}


}
}
