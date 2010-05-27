#include "sim.hpp"
#include "common.hpp"
#include "msg.hpp"

namespace sim {

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

SimComponentMessageRegistry::SimComponentMessageRegistry()
    : _counter(0)
{
}

SimComponentMessageRegistry::~SimComponentMessageRegistry()
{
    // delete all allocated maps of components
    for_each(_map_t::iterator, _map){
        delete it->second;
    }
    _map.clear();

    // delete all remaining Messages
    for_each(std::list<Message *>::iterator, _msgs[0]){
        delete *it;
    }
    for_each(std::list<Message *>::iterator, _msgs[1]){
        delete *it;
    }
    _msgs[0].clear();
    _msgs[1].clear();
}

void SimComponentMessageRegistry::regComponent(Component *c, unsigned long type)
{
    _list_t *cs;
    _map_t::iterator it = _map.find(type);

    // Create new list if does not exist
    if (it == _map.end()){
        cs = new _list_t;
        _map.insert(_map_t::value_type(type, cs));
    }else{
        cs = it->second;
    }

    // cs is std::map - it prevents from adding duplicate values (i.e. two
    // same components can't be registered to on msg type). For more info
    // see man for std::map::insert().
    cs->insert(_list_t::value_type(c, c));
}

void SimComponentMessageRegistry::unregComponent(Component *c, unsigned long type)
{
    _map_t::iterator it = _map.find(type);

    if (it != _map.end()){
        it->second->erase(c);
    }
}
void SimComponentMessageRegistry::unregComponentFromAll(Component *c)
{
    for_each(_map_t::iterator, _map){
        it->second->erase(c);
    }
}

void SimComponentMessageRegistry::assignMessage(Message *m)
{
    // get list of Components registered to this message type
    _map_t::iterator cur = _map.find(m->type());
    // compute index of next step
    unsigned char count = (_counter + 1) % 2;

    if (cur != _map.end()){
        Component *c;
        _list_t &l = *cur->second;

        for_each(_list_t::iterator, l){
            c = it->second;

            // if this is first message to deliver to this Component
            // Component isn't (for sure) in list of active Components
            // because all Components are removed from _active list during
            // deliverMessages() method.
            if (c->__msgs_to_deliver[count][m->prio()].size() == 0){
                _active[count][c->prio()][m->prio()].push_back(it->second);
            }

            // append message to be delivered to Component
            c->__msgs_to_deliver[count][m->prio()].push_back(m);
        }
    }

    // store message in list of messages - to be able to delete it later
    _msgs[count].push_back(m);
}

void SimComponentMessageRegistry::deliverMessages()
{
    // first deliver to Components with highest priority - O(1)
    for (int i = Component::PRIO_MAX - 1; i >= 0; i--){
        for (int j = Message::PRIO_MAX - 1; j >= 0; j--){
            // check if there is any Component with pending messages - O(1)
            if (_active[_counter][i][j].size() > 0){
                // deliver all messages to Components
                //  O(num of active comps) * O(deliverAssignedMessages())
                for_each(std::list<Component *>::iterator, _active[_counter][i][j]){
                    deliverAssignedMessages(*it, (Message::Priority)j);
                }

                // all Components processed - clear list
                _active[_counter][i][j].clear();
            }
        }
    }

    // all messages delivered - delete them all
    for_each(std::list<Message *>::iterator, _msgs[_counter]){
        delete *it;
    }
    _msgs[_counter].clear();

    _counter = (_counter + 1) % 2;
}

void SimComponentMessageRegistry::deliverAssignedMessages(Component *c,
                                                          Message::Priority prio)
{
    // call processMessage() on each message in assigned to Component
    //  O(num of assigned messages)
    for_each(std::list<const Message *>::iterator,
            c->__msgs_to_deliver[_counter][prio]){
        c->processMessage(**it);
    }

    // all messages delivered - clear list
    c->__msgs_to_deliver[_counter][prio].clear();
}


Sim::Sim(World *world, VisWorld *visworld)
    : _world(world), _visworld(visworld),
      _time_step(0, 20000000), _time_substeps(10),
      _simulate(true)
{
    if (!_visworld)
        _visworld = new VisWorld();

    _timer_real.start();
}

Sim::~Sim()
{
    Component *c;

    // remove and delete all components
    while (!_cs.empty()){
        c = _cs.front();
        rmComponent(c);
        delete c;
    }

    if (_world)
        delete _world;
    if (_visworld)
        delete _visworld;
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
    if (_visworld && _visworld->window() && _visworld->viewMain()){
        DBG("");
        _visworld->viewMain()->addEventHandler(new SimKeyboard(this));
    }

    _initComponents();

    std::cerr << "Real time / Simulated time: " << std::endl;
}

void Sim::step()
{
    if (_simulate)
        _stepWorld();
    _stepVisWorld();

    std::cerr << timeReal() << " / " << timeSimulated() << "\r";

    if (timeSimulated() > timeReal()){
        Time tdiff = Time::diff(timeReal(), timeSimulated());
        Time::sleep(tdiff);
    }
}

void Sim::_stepWorld()
{
    if (!_world){
        ERR("I have no World! Can't simulate physics!");
        return;
    }

    // simulation is stalled - simply silently skip the step
    if (!_simulate)
        return;

    // call pre step callbacks
    if (_cs_pre.size() > 0)
        _cbPreStep();

    // deliver all messages
    _cbMessages();

    // recompute real time
    timeRealNow();

    // perform simulation
    _world->step(_time_step, _time_substeps);
    _time_simulated += _time_step;

    // recompute real time
    timeRealNow();

    // call post step callbacks
    if (_cs_post.size() > 0)
        _cbPostStep();
}

void Sim::_stepVisWorld()
{
    if (_visworld)
        _visworld->step();
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
    std::cerr << std::endl;

    _finishComponents();

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

bool Sim::pressedKey(int key)
{
    DBG("Pressed key: " << key << " " << (char)key);

    if (key == 'w'){
        if (_visworld)
            _visworld->toggleWireframe();
	} else if (key == 'a') {
		if (_visworld) {
			_visworld->toggleAlpha();
		}
	} else if (key == 'x') {
		if (_visworld) {
			_visworld->toggleAxis();
		}
	} else if (key == 'p'){
        toggleSimulation();
    }else{
        sendMessage(new MessageKeyPressed(key));
    }

    return false;
}


void Sim::addComponent(Component *c)
{
    if (!_hasComponent(c)){
        _cs.push_back(c);
        _cs_uninit.push_back(c);
    }
}

void Sim::rmComponent(Component *c)
{
    if (_hasComponent(c)){
        _cs.remove(c);
        _cs_uninit.remove(c);

        // remove it also from callback lists
        _cs_pre.remove(c);
        _cs_post.remove(c);
        _reg.unregComponentFromAll(c);
    }
}

void Sim::sendMessage(Message *msg)
{
    _reg.assignMessage(msg);
}

void Sim::sendMessage(Message *msg, Message::Priority prio)
{
    msg->setPrio(prio);
    sendMessage(msg);
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
    if (_hasComponent(c)){
        _reg.regComponent(c, msg_type);
    }
}

void Sim::unregMessage(Component *c, unsigned long msg_type)
{
    _reg.unregComponent(c, msg_type);
}

void Sim::pauseSimulation()
{
    _simulate = false;
    _timer_real.pause();
}

void Sim::continueSimulation()
{
    _simulate = true;
    _timer_real.unpause();
}

void Sim::_initComponents()
{
    Component *c;

    while (_cs_uninit.size() > 0){
        c = _cs_uninit.front();
        _cs_uninit.pop_front();
        c->init(this);
    }
}

void Sim::_finishComponents()
{
    for_each(std::list<Component *>::iterator, _cs){
        (*it)->finish();
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

        if (_cs_uninit.size() > 0){
            _initComponents();
        }
    }
}

void Sim::_cbPostStep()
{
    for_each(cit_t, _cs_post){
        (*it)->cbPostStep();

        if (_cs_uninit.size() > 0){
            _initComponents();
        }
    }
}

void Sim::_cbMessages()
{
    _reg.deliverMessages();

    if (_cs_uninit.size() > 0){
        _initComponents();
    }
}

}
