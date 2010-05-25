#include <osg/PolygonMode>
#include <osgDB/WriteFile>
#include <osgViewer/PixelBufferX11>
#include <osgViewer/Renderer>
#include <string.h>
#include <stdio.h>
#include <osg/AlphaFunc>

#include "visworld.hpp"
#include "visworldmanip.hpp"
#include "common.hpp"
#include "msg.hpp"

namespace sim {

//VisBody *a1,*a2,*a3;

VisWorld::VisWorld()
    : _window(true)
{
    _viewer = new osgViewer::CompositeViewer();
    _view_main = new osgViewer::View;

    _state_set = new osg::StateSet;

    _root = new osg::Group();

    _cams = new osg::Group();
    _root->addChild(_cams);

    _root_vis = new osg::Group();
    _root_vis->setStateSet(_state_set);
    _root->addChild(_root_vis);

    _bodies = new osg::Group();
    _root_vis->addChild(_bodies);

    _lights = new osg::Group();
    _root_vis->addChild(_lights);

    _setUpStateSet();
    _setUpLights();

	_axisCreated = false;
}

VisWorld::~VisWorld()
{
}

void VisWorld::addBody(VisBody *obj)
{
    osg::Node *n = obj->rootNode();
    if (!n || _bodies->containsNode(n))
        return;

    _bodies->addChild(n);
}


void VisWorld::rmBody(VisBody *obj)
{
    osg::Node *n = obj->rootNode();
    if (!n || !_bodies->containsNode(n))
        return;
    _bodies->removeChild(n);
}

void VisWorld::addCam(osg::Camera *cam)
{
    DBG("_cams: " << _cams);
    if (!_cams->containsNode(cam)){
        _cams->addChild(cam);
        cam->addChild(_root_vis);
    }
}

void VisWorld::rmCam(osg::Camera *cam)
{
    DBG("_cams: " << _cams);
    if (_cams->containsNode(cam)){
        _cams->removeChild(cam);
        cam->removeChild(_root_vis);
    }
}

void VisWorld::addView(osgViewer::View *view)
{
    view->setSceneData(_root_vis);
    _viewer->addView(view);
}

void VisWorld::rmView(osgViewer::View *view)
{
    _viewer->removeView(view);
}

void VisWorld::init()
{
    _view_main->setSceneData(_root);

    if (_window){
        if (!_view_main->getCameraManipulator()){
            //_viewer->setCameraManipulator(new osgGA::TrackballManipulator());
            _view_main->setCameraManipulator(new VisWorldManip());

            {
                osg::Vec3d eye, at, up;
                _view_main->getCameraManipulator()->getHomePosition(eye, at, up);
                DBG(DBGV(eye));
                DBG(DBGV(at));
                DBG(DBGV(up));
            }
        }

        _view_main->setUpViewInWindow(0, 0, 1200, 800);
    }

    _viewer->addView(_view_main);
    _viewer->realize();
}

void VisWorld::finish()
{
    _viewer->removeView(_view_main);
}

void VisWorld::step()
{
    if (_window){
        _viewer->frame(0.);
    }
}

bool VisWorld::done()
{
    return _viewer->done();
}


void VisWorld::_setUpLights()
{
    osg::ref_ptr<osg::Light> light;
    osg::ref_ptr<osg::LightSource> light_source;

    light_source = new osg::LightSource;

    light = new osg::Light(0);
    light->setPosition(osg::Vec4(0.0f, 0.0f, 10.0f, 1.0f));
    light->setDirection(osg::Vec3(0., 0., -1.));
    //light->setAmbient(osg::Vec4(0.6f, 0.6f, 0.6f, 1.0f));
    //light->setDiffuse(osg::Vec4(0.6f, 0.6f, 0.6f, 1.0f));
    light->setAmbient(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    light->setDiffuse(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    light->setSpotCutoff(180.);
    //light->setSpotExponent(10.0);

    light_source->setLight(light);
    light_source->setLocalStateSetModes(osg::StateAttribute::ON);
    light_source->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

    _lights->addChild(light_source);

    /*
    light_source = new osg::LightSource;

    light = new osg::Light(1);
    light->setPosition(osg::Vec4(-10.0f, -10.0f, 10.0f, 1.0f));
    light->setDirection(osg::Vec3(1., 1., -1.));
    light->setAmbient(osg::Vec4(1.0f, 1.0f, 0.7f, 1.0f));
    light->setDiffuse(osg::Vec4(1.0f, 1.0f, 0.7f, 1.0f));
    light->setSpotCutoff(100.);
    light->setSpotExponent(10.0);

    light_source->setLight(light);
    light_source->setLocalStateSetModes(osg::StateAttribute::ON);
    light_source->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

    _lights->addChild(light_source);
    */
}

void VisWorld::_setUpStateSet()
{
    _state_set->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    _state_set->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    _state_set->setMode(GL_BLEND, osg::StateAttribute::ON);
    _state_set->setMode(GL_LIGHT0, osg::StateAttribute::ON);
    _state_set->setMode(GL_LIGHT1, osg::StateAttribute::OFF);
    //_state_set->setMode(GL_DITHER, osg::StateAttribute::ON);
    //_state_set->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    osg::ref_ptr<osg::PolygonMode> polymode = new osg::PolygonMode();
    //polymode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
    //polymode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::POINT);
    polymode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
    _state_set->setAttributeAndModes(polymode, osg::StateAttribute::ON);
}

void VisWorld::toggleWireframe()
{
    osg::PolygonMode *pm = (osg::PolygonMode *)_state_set->getAttribute(osg::StateAttribute::POLYGONMODE);
    if (pm->getMode(osg::PolygonMode::FRONT_AND_BACK) == osg::PolygonMode::LINE){
        pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
    }else{
        pm->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
    }
}


void VisWorld::toggleAlpha()
{
	if (_state_set->getMode(GL_BLEND) == osg::StateAttribute::ON) {
		_state_set->setMode(GL_BLEND,osg::StateAttribute::OFF);
	} else {
		_state_set->setMode(GL_BLEND,osg::StateAttribute::ON);
	}
}

void VisWorld::toggleAxis() {

	
	if (!_axisCreated) {
		const double axisHeight = 10;
		const double axisRadius = 0.1;

		// axis z
		VisBody *b = new VisBodyCylinder(axisRadius,axisHeight);
		b->setColor(osg::Vec4(0,0,1,1));
		b->setPos(0,0,axisHeight/2);		
		this->addBody(b);
		_axisZ = b->node();

		// axis x
		b = new VisBodyCylinder(axisRadius,axisHeight);
		b->setColor(osg::Vec4(1,0,0,1));
		b->setPos(axisHeight/2,0,0);
		osg::Quat q(M_PI/2,osg::Vec3(0,1,0));
		b->setRot(q);
		this->addBody(b);
		_axisX = b->node();

		// axis y
		b = new VisBodyCylinder(axisRadius,axisHeight);
		b->setColor(osg::Vec4(0,1,0,1));
		b->setPos(0,axisHeight/2,0);
		b->setRot(osg::Quat(M_PI/2,osg::Vec3(1,0,0)));
		this->addBody(b);
		_axisY = b->node();

		_axisCreated = true;
	} else {
		
		if (_axisX->getNodeMask()) {
			_axisX->setNodeMask(0 );	
			_axisY->setNodeMask(0);	
			_axisZ->setNodeMask(0);
		} else {
			_axisX->setNodeMask(0xffffffff);	
			_axisY->setNodeMask(0xffffffff);	
			_axisZ->setNodeMask(0xffffffff);
		}


	}


}



}
