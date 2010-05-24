#include <stdio.h>
#include <osgDB/WriteFile>

#include "camera.hpp"
#include <sim/msg.hpp>


namespace sim {
namespace sensor {

Camera::Camera()
    : sim::Component(),
      _eye(-1., 1., 10.), _at(0., 0., 0.), _zaxis(0., 0., 1.),
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
}

void Camera::finish()
{
    _sim->visWorld()->rmCam(_cam);
}

void Camera::cbPreStep()
{
    static size_t counter = 0;
    static char fn[100];

    if (_body){
        const Vec3 &pos = _body->pos();
        const Quat &rot = _body->rot();
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
    }

    if (_image->valid()){
        DBG(counter);
        sprintf(fn, "%06d.png", counter);
        osgDB::writeImageFile(*_image, fn);
        counter++;
    }
}

void Camera::attachToBody(const sim::Body *b)
{
    _body = b;
}

void Camera::_createCamera()
{
    osg::Vec4 bgcolor(0.1, 0.1, 0.3, 1.);
    int x, y, width, height;

    x = 0;
    y = 0;
    width = 1024;
    height = 512;

    // set up camera
    if (_cam)
        _cam->unref();

    _cam = new osg::Camera;
    _cam->setClearColor(bgcolor);
    //_cam->setProjectionMatrixAsFrustum(-10, 10, -10, 10, -30, 30);
    _cam->setProjectionMatrixAsPerspective(10, 2., 1, 30);
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
}

void Camera::_updatePosition()
{
}


}
}
