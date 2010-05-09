#include "sim.hpp"
#include "msg.hpp"

namespace sim {

    /*
class SimKeyboard : public osgGA::GUIEventHandler {
    Sim *_sim;
  public:
    SimKeyboard(Sim *sim) : _sim(sim){}
    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
    virtual void accept(osgGA::GUIEventHandlerVisitor& v) { v.visit(*this); };
};

bool SimKeyboard::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    int key;

    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN){
        key = ea.getKey();
        return _sim->pressedKey(key);
    }
    return false;
}
*/

Sim::Sim()
    : _world(0), _visworld(0)
{
}

Sim::~Sim()
{
    if (_world)
        delete _world;
    if (_visworld)
        delete _visworld;
}


void Sim::setWorld(World *w)
{
    _world = w;

    if (_visworld)
        w->setVisWorld(_visworld);
}

void Sim::setVisWorld(VisWorld *w)
{
    _visworld = w;

    if (_world)
        _world->setVisWorld(w);
}

void Sim::init()
{
    if (_world)
        _world->init();
    if (_visworld)
        _visworld->init();
}

void Sim::step()
{
    if (_world)
        _world->step();
    if (_visworld)
        _visworld->step();

    usleep(10000);
}

bool Sim::done()
{
    if (_visworld)
        return _visworld->done();
    if (_world)
        return _world->done();
    return false;
}

void Sim::finish()
{
    if (_world)
        _world->finish();
    if (_visworld)
        _visworld->finish();
}

void Sim::run()
{
    init();
    while (!done()){
        step();
    }
    finish();
}

}
