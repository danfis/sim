// copied official version of osgGA::TrackballManipulator v2.9.4 but RIGHT
// mouse button is disabled

#include "visworldmanip.hpp"
#include <osg/Quat>
#include <osg/Notify>
#include <osg/BoundsChecking>

using namespace osg;
using namespace osgGA;

namespace sim {

VisWorldManip::VisWorldManip()
{
    _modelScale = 0.01f;
    _minimumZoomScale = 0.05f;
    _allowThrow = true;
    _thrown = false;

    _distance = 1.0f;
    _trackballSize = 0.8f;
    _zoomDelta = 0.1f;
}


VisWorldManip::~VisWorldManip()
{
}


void VisWorldManip::setNode(osg::Node* node)
{
    _node = node;
    if (_node.get())
    {
        const osg::BoundingSphere& boundingSphere=_node->getBound();
        _modelScale = boundingSphere._radius;
    }
    if (getAutoComputeHomePosition()) computeHomePosition();
}


const osg::Node* VisWorldManip::getNode() const
{
    return _node.get();
}


osg::Node* VisWorldManip::getNode()
{
    return _node.get();
}


void VisWorldManip::home(double /*currentTime*/)
{
    if (getAutoComputeHomePosition()) computeHomePosition();
    computePosition(_homeEye, _homeCenter, _homeUp);
    _thrown = false;
}

void VisWorldManip::home(const GUIEventAdapter& ea ,GUIActionAdapter& us)
{
    home(ea.getTime());
    us.requestRedraw();
    us.requestContinuousUpdate(false);
}


void VisWorldManip::init(const GUIEventAdapter& ,GUIActionAdapter& )
{
    flushMouseEventStack();
}


void VisWorldManip::getUsage(osg::ApplicationUsage& usage) const
{
    usage.addKeyboardMouseBinding("Trackball: Space","Reset the viewing position to home");
    usage.addKeyboardMouseBinding("Trackball: +","When in stereo, increase the fusion distance");
    usage.addKeyboardMouseBinding("Trackball: -","When in stereo, reduce the fusion distance");
}

bool VisWorldManip::handle(const GUIEventAdapter& ea,GUIActionAdapter& us)
{
    switch(ea.getEventType())
    {
        case(GUIEventAdapter::FRAME):
            if (_thrown && _allowThrow)
            {
                if (calcMovement()) us.requestRedraw();
            }
            return false;
        default:
            break;
    }

    if (ea.getHandled()) return false;

    if (ea.getModKeyMask() != 0)
        return false;

    switch(ea.getEventType())
    {
        case(GUIEventAdapter::PUSH):
        {
            flushMouseEventStack();
            addMouseEvent(ea);
            if (calcMovement()) us.requestRedraw();
            us.requestContinuousUpdate(false);
            _thrown = false;
            return true;
        }

        case(GUIEventAdapter::RELEASE):
        {
            if (ea.getButtonMask()==0)
            {
            
                double timeSinceLastRecordEvent = _ga_t0.valid() ? (ea.getTime() - _ga_t0->getTime()) : DBL_MAX;
                if (timeSinceLastRecordEvent>0.02) flushMouseEventStack();

                if (isMouseMoving())
                {
                    if (calcMovement())
                    {
                        us.requestRedraw();
                        us.requestContinuousUpdate(true);
                        _thrown = _allowThrow;
                    }
                }
                else
                {
                    flushMouseEventStack();
                    addMouseEvent(ea);
                    if (calcMovement()) us.requestRedraw();
                    us.requestContinuousUpdate(false);
                    _thrown = false;
                }

            }
            else
            {
                flushMouseEventStack();
                addMouseEvent(ea);
                if (calcMovement()) us.requestRedraw();
                us.requestContinuousUpdate(false);
                _thrown = false;
            }
            return true;
        }

        case(GUIEventAdapter::DRAG):
        case(GUIEventAdapter::SCROLL):
        {
            addMouseEvent(ea);
            if (calcMovement()) us.requestRedraw();
            us.requestContinuousUpdate(false);
            _thrown = false;
            return true;
        }

        case(GUIEventAdapter::MOVE):
        {
            return false;
        }

        case(GUIEventAdapter::KEYDOWN):
            if (ea.getKey()== GUIEventAdapter::KEY_Space)
            {
                flushMouseEventStack();
                _thrown = false;
                home(ea,us);
                return true;
            }
            return false;
        case(GUIEventAdapter::FRAME):
            if (_thrown)
            {
                if (calcMovement()) us.requestRedraw();
            }
            return false;
        default:
            return false;
    }
}


bool VisWorldManip::isMouseMoving()
{
    if (_ga_t0.get()==NULL || _ga_t1.get()==NULL) return false;

    static const float velocity = 0.1f;

    float dx = _ga_t0->getXnormalized()-_ga_t1->getXnormalized();
    float dy = _ga_t0->getYnormalized()-_ga_t1->getYnormalized();
    float len = sqrtf(dx*dx+dy*dy);
    float dt = _ga_t0->getTime()-_ga_t1->getTime();

    return (len>dt*velocity);
}


void VisWorldManip::flushMouseEventStack()
{
    _ga_t1 = NULL;
    _ga_t0 = NULL;
}


void VisWorldManip::addMouseEvent(const GUIEventAdapter& ea)
{
    _ga_t1 = _ga_t0;
    _ga_t0 = &ea;
}

void VisWorldManip::setByMatrix(const osg::Matrixd& matrix)
{
    _center = osg::Vec3(0.0f,0.0f,-_distance)*matrix;
    _rotation = matrix.getRotate();
}

osg::Matrixd VisWorldManip::getMatrix() const
{
    return osg::Matrixd::translate(0.0,0.0,_distance)*osg::Matrixd::rotate(_rotation)*osg::Matrixd::translate(_center);
}

osg::Matrixd VisWorldManip::getInverseMatrix() const
{
    return osg::Matrixd::translate(-_center)*osg::Matrixd::rotate(_rotation.inverse())*osg::Matrixd::translate(0.0,0.0,-_distance);
}

void VisWorldManip::computePosition(const osg::Vec3& eye,const osg::Vec3& center,const osg::Vec3& up)
{

    osg::Vec3 lv(center-eye);

    osg::Vec3 f(lv);
    f.normalize();
    osg::Vec3 s(f^up);
    s.normalize();
    osg::Vec3 u(s^f);
    u.normalize();
    
    osg::Matrix rotation_matrix(s[0],     u[0],     -f[0],     0.0f,
                                s[1],     u[1],     -f[1],     0.0f,
                                s[2],     u[2],     -f[2],     0.0f,
                                0.0f,     0.0f,     0.0f,      1.0f);
                   
    _center = center;
    _distance = lv.length();
    _rotation = rotation_matrix.getRotate().inverse();
}


bool VisWorldManip::calcMovement()
{
    // mouse scroll is only a single event
    if (_ga_t0.get()==NULL) return false;

    float dx=0.0f;
    float dy=0.0f;
    unsigned int buttonMask=osgGA::GUIEventAdapter::NONE;

    if (_ga_t0->getEventType()==GUIEventAdapter::SCROLL)
    {
        dy = _ga_t0->getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_UP ? _zoomDelta : -_zoomDelta;
        buttonMask=GUIEventAdapter::SCROLL;
    } 
    else 
    {

        if (_ga_t1.get()==NULL) return false;
        dx = _ga_t0->getXnormalized()-_ga_t1->getXnormalized();
        dy = _ga_t0->getYnormalized()-_ga_t1->getYnormalized();
        float distance = sqrtf(dx*dx + dy*dy);
        
        // return if movement is too fast, indicating an error in event values or change in screen.
        if (distance>0.5)
        {
            return false;
        }
        
        // return if there is no movement.
        if (distance==0.0f)
        {
            return false;
        }

        buttonMask = _ga_t1->getButtonMask();
    }
    
    if (buttonMask==GUIEventAdapter::LEFT_MOUSE_BUTTON)
    {

        // rotate camera.

        osg::Vec3 axis;
        float angle;

        float px0 = _ga_t0->getXnormalized();
        float py0 = _ga_t0->getYnormalized();
        
        float px1 = _ga_t1->getXnormalized();
        float py1 = _ga_t1->getYnormalized();
        

        trackball(axis,angle,px1,py1,px0,py0);

        osg::Quat new_rotate;
        new_rotate.makeRotate(angle,axis);
        
        _rotation = _rotation*new_rotate;

        return true;

    }
    else if (buttonMask==GUIEventAdapter::MIDDLE_MOUSE_BUTTON ||
        buttonMask==(GUIEventAdapter::LEFT_MOUSE_BUTTON|GUIEventAdapter::RIGHT_MOUSE_BUTTON))
    {

        // pan model.

        float scale = -0.3f*_distance;

        osg::Matrix rotation_matrix;
        rotation_matrix.makeRotate(_rotation);

        osg::Vec3 dv(dx*scale,dy*scale,0.0f);

        _center += dv*rotation_matrix;
        
        return true;

    }
    else if ((buttonMask==GUIEventAdapter::SCROLL))
    {

        // zoom model.

        float fd = _distance;
        float scale = 1.0f+dy;
        if (fd*scale>_modelScale*_minimumZoomScale)
        {

            _distance *= scale;

        }
        else
        {

            // notify(DEBUG_INFO) << "Pushing forward"<<std::endl;
            // push the camera forward.
            float scale = -fd;

            osg::Matrix rotation_matrix(_rotation);

            osg::Vec3 dv = (osg::Vec3(0.0f,0.0f,-1.0f)*rotation_matrix)*(dy*scale);

            _center += dv;

        }

        return true;

    }

    return false;
}


/*
 * This size should really be based on the distance from the center of
 * rotation to the point on the object underneath the mouse.  That
 * point would then track the mouse as closely as possible.  This is a
 * simple example, though, so that is left as an Exercise for the
 * Programmer.
 */
void VisWorldManip::setTrackballSize(float size)
{
    _trackballSize = size;
     osg::clampBetweenRange(_trackballSize,0.1f,1.0f,"VisWorldManip::setTrackballSize(float)");
}

/*
 * Ok, simulate a track-ball.  Project the points onto the virtual
 * trackball, then figure out the axis of rotation, which is the cross
 * product of P1 P2 and O P1 (O is the center of the ball, 0,0,0)
 * Note:  This is a deformed trackball-- is a trackball in the center,
 * but is deformed into a hyperbolic sheet of rotation away from the
 * center.  This particular function was chosen after trying out
 * several variations.
 *
 * It is assumed that the arguments to this routine are in the range
 * (-1.0 ... 1.0)
 */
void VisWorldManip::trackball(osg::Vec3& axis,float& angle, float p1x, float p1y, float p2x, float p2y)
{
    /*
     * First, figure out z-coordinates for projection of P1 and P2 to
     * deformed sphere
     */

    osg::Matrix rotation_matrix(_rotation);


    osg::Vec3 uv = osg::Vec3(0.0f,1.0f,0.0f)*rotation_matrix;
    osg::Vec3 sv = osg::Vec3(1.0f,0.0f,0.0f)*rotation_matrix;
    osg::Vec3 lv = osg::Vec3(0.0f,0.0f,-1.0f)*rotation_matrix;

    osg::Vec3 p1 = sv * p1x + uv * p1y - lv * tb_project_to_sphere(_trackballSize, p1x, p1y);
    osg::Vec3 p2 = sv * p2x + uv * p2y - lv * tb_project_to_sphere(_trackballSize, p2x, p2y);

    /*
     *  Now, we want the cross product of P1 and P2
     */

// Robert,
//
// This was the quick 'n' dirty  fix to get the trackball doing the right 
// thing after fixing the Quat rotations to be right-handed.  You may want
// to do something more elegant.
//   axis = p1^p2;
axis = p2^p1;
    axis.normalize();

    /*
     *  Figure out how much to rotate around that axis.
     */
    float t = (p2 - p1).length() / (2.0 * _trackballSize);

    /*
     * Avoid problems with out-of-control values...
     */
    if (t > 1.0) t = 1.0;
    if (t < -1.0) t = -1.0;
    angle = inRadians(asin(t));

}


/*
 * Project an x,y pair onto a sphere of radius r OR a hyperbolic sheet
 * if we are away from the center of the sphere.
 */
float VisWorldManip::tb_project_to_sphere(float r, float x, float y)
{
    float d, t, z;

    d = sqrt(x*x + y*y);
                                 /* Inside sphere */
    if (d < r * 0.70710678118654752440)
    {
        z = sqrt(r*r - d*d);
    }                            /* On hyperbola */
    else
    {
        t = r / 1.41421356237309504880;
        z = t*t / d;
    }
    return z;
}

}