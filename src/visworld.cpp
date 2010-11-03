/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
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

#include <osg/PolygonMode>
#include <osgDB/WriteFile>
#include <osgViewer/PixelBufferX11>
#include <osgViewer/Renderer>
#include <osg/ShapeDrawable>
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

    _g_bodies = new osg::Group();
    _root_vis->addChild(_g_bodies);

    _lights = new osg::Group();
    _root_vis->addChild(_lights);

    _off_scene = new osg::Group();
    _root->addChild(_off_scene);

    _setUpStateSet();
    _setUpLights();
    _createCoordFrame();

    // set up viewer to run in single thread
    _viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
}

VisWorld::~VisWorld()
{
}

void VisWorld::addBody(VisBody *obj)
{
    osg::Node *n = obj->rootNode();
    if (!n || _g_bodies->containsNode(n))
        return;

    _g_bodies->addChild(n);
    _bodies.push_back(obj);
}


void VisWorld::rmBody(VisBody *obj)
{
    osg::Node *n = obj->rootNode();
    if (!n || !_g_bodies->containsNode(n))
        return;

    _g_bodies->removeChild(n);
    _bodies.remove(obj);
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

void VisWorld::addOffScene(osg::Node *n)
{
    _off_scene->addChild(n);
}

void VisWorld::rmOffScene(osg::Node *n)
{
    _off_scene->removeChild(n);
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
        //_view_main->setUpViewInWindow(0, 0, 120, 80);
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

void VisWorld::toggleAxis()
{
    if (_coord_frame->getValue(0)){
        _coord_frame->setAllChildrenOff();
    }else{
        _coord_frame->setAllChildrenOn();
    }
}


void VisWorld::_createCoordFrame()
{
    Scalar height = 10;
    Scalar radius = height / 50.;
    osg::ref_ptr<osg::Cylinder> cyl;
    osg::ref_ptr<osg::ShapeDrawable> draw;
    osg::ref_ptr<osg::Geode> geode;

    _coord_frame = new osg::Switch();

    geode = new osg::Geode();
    _coord_frame->addChild(geode);

    cyl = new osg::Cylinder(Vec3(0., 0., 0.), radius, height);
    cyl->setRotation(Quat(Vec3(0., 1., 0.), M_PI / 2.));
    cyl->setCenter(Vec3(1., 0., 0.) * (height / 2.));
    draw = new osg::ShapeDrawable(cyl);
    draw->setColor(osg::Vec4(1., 0., 0., 1.));
    geode->addDrawable(draw);

    cyl = new osg::Cylinder(Vec3(0., 0., 0.), radius, height);
    cyl->setRotation(Quat(Vec3(1., 0., 0.), M_PI / 2.));
    cyl->setCenter(Vec3(0., 1., 0.) * (height / 2.));
    draw = new osg::ShapeDrawable(cyl);
    draw->setColor(osg::Vec4(0., 1., 0., 1.));
    geode->addDrawable(draw);

    cyl = new osg::Cylinder(Vec3(0., 0., 0.), radius, height);
    cyl->setRotation(Quat(Vec3(0., 0., 1.), M_PI / 2.));
    cyl->setCenter(Vec3(0., 0., 1.) * (height / 2.));
    draw = new osg::ShapeDrawable(cyl);
    draw->setColor(osg::Vec4(0., 0., 1., 1.));
    geode->addDrawable(draw);

    _coord_frame->setAllChildrenOff();

    _root->addChild(_coord_frame);
}

}
