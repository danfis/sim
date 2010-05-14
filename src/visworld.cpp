#include <osgGA/TrackballManipulator>
#include <osg/AlphaFunc>
#include <osg/LightModel>

#include "visworld.hpp"
#include "msg.hpp"

namespace sim {

VisWorld::VisWorld()
    : _window(true)
{
    _viewer = new osgViewer::Viewer();

    _state_set = new osg::StateSet;

    _root = new osg::Group();
    _root->setStateSet(_state_set);

    _lights = new osg::Group();
    _root->addChild(_lights);
}

VisWorld::~VisWorld()
{
}

void VisWorld::addBody(VisBody *obj)
{
    osg::Node *n;
    n = obj->rootNode();
    if (n){
        _root->addChild(n);
    }
}


void VisWorld::init()
{
    _setUpStateSet();
    _setUpLights();

    _viewer->setSceneData(_root);

    if (_window){
        if (!_viewer->getCameraManipulator()){
            _viewer->setCameraManipulator(new osgGA::TrackballManipulator());
        }

        _viewer->setUpViewInWindow(0, 0, 1200, 800);
        _viewer->realize();
    }
}

void VisWorld::finish()
{
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
}

}
