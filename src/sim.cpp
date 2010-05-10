#include "sim.hpp"
#include "common.hpp"
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

Sim::Sim(World *world, VisWorld *visworld)
    : _world(world), _visworld(visworld)
{
    if (!_visworld)
        visworld = new VisWorld();
}

Sim::~Sim()
{
    Component *c;

    if (_world)
        delete _world;
    if (_visworld)
        delete _visworld;

    // remove and delete all components
    while (!_cs.empty()){
        c = _cs.front();
        rmComponent(c);
        delete c;
    }
}


void Sim::setWorld(World *w)
{
    if (_world)
        delete _world;

    _world = w;

    if (_visworld)
        w->setVisWorld(_visworld);
}

void Sim::setVisWorld(VisWorld *w)
{
    if (_visworld)
        delete _visworld;

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
    _cbPreStep();

    if (_world)
        _world->step();
    if (_visworld)
        _visworld->step();

    _cbPostStep();

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


void Sim::addComponent(Component *c)
{
    if (!_hasComponent(c)){
        _cs.push_back(c);
        c->init(this);
    }
}

void Sim::rmComponent(Component *c)
{
    if (_hasComponent(c)){
        c->finish();
        _cs.remove(c);

        // remove it also from callback lists
        _cs_pre.remove(c);
        _cs_post.remove(c);
    }
}

void Sim::regPreStep(Component *c)
{
    if (_hasComponent(c) && !_hasPreStep(c)){
        _cs_pre.push_back(c);
    }
}

void Sim::regPostStep(Component *c)
{
    if (_hasComponent(c) && !_hasPostStep(c)){
        _cs_post.push_back(c);
    }
}



bool Sim::_hasComponent(const Component *c) const
{
    return _isComponentInList(c, _cs);
}

bool Sim::_hasPreStep(const Component *c) const
{
    return _isComponentInList(c, _cs_pre);
}

bool Sim::_hasPostStep(const Component *c) const
{
    return _isComponentInList(c, _cs_post);
}

bool Sim::_isComponentInList(const Component *c,
                             const std::list<Component *> &list) const
{
    const_cit_t it, it_end;
    it = list.begin();
    it_end = list.end();

    return std::find(it, it_end, c) != it_end;
}

void Sim::_cbPreStep()
{
    for_each(cit_t, _cs_pre){
        (*it)->cbPreStep();
    }
}

void Sim::_cbPostStep()
{
    for_each(cit_t, _cs_post){
        (*it)->cbPostStep();
    }
}

}
