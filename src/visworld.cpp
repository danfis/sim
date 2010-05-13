#include <osgGA/TrackballManipulator>

#include "visworld.hpp"
#include "msg.hpp"

namespace sim {

VisWorld::VisWorld()
    : _window(true)
{
    _viewer = new osgViewer::Viewer();
    _root = new osg::Group();
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

}
