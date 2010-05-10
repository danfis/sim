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

SimComponentMessageRegistry::SimComponentMessageRegistry()
{
}

SimComponentMessageRegistry::~SimComponentMessageRegistry()
{
    for_each(_map_t::iterator, _map){
        delete it->second;
    }
    _map.clear();

    for_each(std::list<Message *>::iterator, _msgs){
        delete *it;
    }
    _msgs.clear();
}

void SimComponentMessageRegistry::regComponent(Component *c, unsigned long type)
{
    DBG(c << " " << type);

    _list_t *cs;
    _map_t::iterator it = _map.find(type);

    // Create new list if does not exist
    if (it == _map.end()){
        cs = new _list_t;
        DBG("new: " << cs);
        _map.insert(_map_t::value_type(type, cs));
    }else{
        cs = it->second;
        DBG(cs);
    }

    cs->insert(_list_t::value_type(c, c));
}

void SimComponentMessageRegistry::unregComponent(Component *c, unsigned long type)
{
    _map_t::iterator it = _map.find(type);

    if (it != _map.end()){
        it->second->erase(c);
    }
}

void SimComponentMessageRegistry::assignMessage(Message *m)
{
    DBG(m);

    // get list of Components registered to this message type
    _map_t::iterator cur = _map.find(m->type());

    if (cur != _map.end()){
        Component *c;
        _list_t &l = *cur->second;

        for_each(_list_t::iterator, l){
            c = it->second;
            DBG(c);

            // if this is first message to deliver to this Component
            // Component isn't (for sure) in list of active Components
            // because all Components are removed from _active list during
            // deliverMessages() method.
            if (c->__msgs_to_deliver.size() == 0){
                DBG(" -> _activate[" << c->prio() << "]");
                _active[c->prio()].push_back(it->second);
            }

            // append message to be delivered to Component
            c->__msgs_to_deliver.push_back(m);
        }
    }

    // store message in list of messages - to be able to delete it later
    _msgs.push_back(m);
}

void SimComponentMessageRegistry::deliverMessages()
{
    DBG("");

    // first deliver to Components with highest priority - O(1)
    for (int i = Component::PRIO_MAX - 1; i >= 0; i--){
        // check if there is any Component with pending messages - O(1)
        if (_active[i].size() > 0){
            // deliver all messages to Components
            //  O(num of active comps) * O(deliverAssignedMessages())
            for_each(std::list<Component *>::iterator, _active[i]){
                deliverAssignedMessages(*it);
            }

            // all Components processed - clear list
            _active[i].clear();
        }
    }

    // all messages delivered - delete them all
    for_each(std::list<Message *>::iterator, _msgs){
        delete *it;
    }
    _msgs.clear();
}

void SimComponentMessageRegistry::deliverAssignedMessages(Component *c)
{
    DBG(c);

    // call processMessage() on each message in assigned to Component
    //  O(num of assigned messages)
    for_each(std::list<const Message *>::iterator,
             c->__msgs_to_deliver){
        c->processMessage(**it);
    }

    // all messages delivered - clear list
    c->__msgs_to_deliver.clear();
}


Sim::Sim(World *world, VisWorld *visworld)
    : _world(world), _visworld(visworld)
{
    if (!_visworld)
        _visworld = new VisWorld();
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

    _cbMessages();

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

void Sim::sendMessage(Message *msg)
{
    _reg.assignMessage(msg);
}

void Sim::regPreStep(Component *c)
{
    if (_hasComponent(c) && !_hasPreStep(c)){
        _cs_pre.push_back(c);
    }
}

void Sim::unregPreStep(Component *c)
{
    if (_hasPreStep(c)){
        _cs_pre.remove(c);
    }
}

void Sim::regPostStep(Component *c)
{
    if (_hasComponent(c) && !_hasPostStep(c)){
        _cs_post.push_back(c);
    }
}

void Sim::unregPostStep(Component *c)
{
    if (_hasPostStep(c)){
        _cs_post.remove(c);
    }
}

void Sim::regMessage(Component *c, unsigned long msg_type)
{
    DBG("");
    if (_hasComponent(c)){
        _reg.regComponent(c, msg_type);
    }
}

void Sim::unregMessage(Component *c, unsigned long msg_type)
{
    _reg.unregComponent(c, msg_type);
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

void Sim::_cbMessages()
{
    _reg.deliverMessages();
}

}
