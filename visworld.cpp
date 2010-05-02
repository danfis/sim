#include <osgGA/TrackballManipulator>

#include "visworld.hpp"

namespace sim {


VisWorld::VisWorld()
{
    _viewer = new osgViewer::Viewer();
    _root = new osg::Group();
}

VisWorld::~VisWorld()
{
    delete _viewer;
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
    _viewer->setUpViewInWindow(0, 0, 1200, 800);

    _viewer->setSceneData(_root);

    if (!_viewer->getCameraManipulator()){
        _viewer->setCameraManipulator(new osgGA::TrackballManipulator());
    }

    _viewer->realize();
}

void VisWorld::destroy()
{
}

void VisWorld::step()
{
    _viewer->frame(0.);
}

bool VisWorld::done()
{
    return _viewer->done();
}

}
