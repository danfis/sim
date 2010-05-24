#include <osg/PolygonMode>
#include <osgDB/WriteFile>
#include <string.h>
#include <stdio.h>

#include "visworld.hpp"
#include "visworldmanip.hpp"
#include "msg.hpp"

namespace sim {

class CamCallback : public osg::Camera::DrawCallback {
    VisWorld *_vw;

  public:
    CamCallback(VisWorld *a) : _vw(a)
    {
    }

void operator()(const osg::Camera &cam) const
{
    static size_t counter = 0;
    static char fn[100];
    DBG(counter);
    DBG(_vw);
    DBG(_vw->_image);

    sprintf(fn, "%06d.png", counter);
    osgDB::writeImageFile(*_vw->_image, fn);
    counter++;
}
};

VisWorld::VisWorld()
    : _window(true)
{
    _viewer = new osgViewer::Viewer();

    _state_set = new osg::StateSet;

    _root = new osg::Group();
    _root->setStateSet(_state_set);

    _lights = new osg::Group();
    _root->addChild(_lights);

    _cam = new osg::Camera();
    _image = new osg::Image();

    _cam->addChild(_root);

    _cam->setClearColor(osg::Vec4(0.1f, 0.1f, 0.3f, 1.0f));
    //_cam->setProjectionMatrixAsFrustum(-10, 10, -10, 10, -30, 30);
    _cam->setProjectionMatrixAsPerspective(10, 2., 1, 30);
    _cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    _cam->setViewport(0., 0., 1024, 512);
    _cam->setViewMatrixAsLookAt(osg::Vec3d(-1., 1., 3.), osg::Vec3d(0., 0., 0.), osg::Vec3d(0., 0., 1.));
    //_cam->setViewMatrixAsLookAt(osg::Vec3d(3., -3., 10.), osg::Vec3d(0., 0., 0.), osg::Vec3d(0., 0., 1.));
    _cam->setRenderOrder(osg::Camera::PRE_RENDER);
    _cam->setRenderTargetImplementation(osg::Camera::PIXEL_BUFFER);

    // set viewport 1024 512
    _cam->attach(osg::Camera::COLOR_BUFFER, _image);
    _cam->setPostDrawCallback(new CamCallback(this));

    _r = new osg::Group;
    _r->addChild(_cam);
    _r->addChild(_root);

    //_cam->addChild(sim->visWorld()->sceneRoot());
    //sim->visWorld()->viewer()->getCamera()->addChild(_cam);
    //((osg::Group *)sim->visWorld()->sceneRoot())->addChild(_cam);

    /*
    _cam->setRenderer(new osgViewer::Renderer(_cam));
    {
        osg::Geode *g = new osg::Geode;
        g->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3d(0., 0., 0.), 1.)));
        _cam->addChild(g);
    }
    */
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

    //_viewer->setSceneData(_root);
    //_viewer->setSceneData(_cam);
    _viewer->setSceneData(_r);


    if (_window){
        if (!_viewer->getCameraManipulator()){
            //_viewer->setCameraManipulator(new osgGA::TrackballManipulator());
            _viewer->setCameraManipulator(new VisWorldManip());

            {
                osg::Vec3d eye, at, up;
                _viewer->getCameraManipulator()->getHomePosition(eye, at, up);
                DBG(DBGV(eye));
                DBG(DBGV(at));
                DBG(DBGV(up));
            }
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

}
