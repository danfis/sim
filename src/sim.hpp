/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*!
\page dev_messaging Messaging Internals
\section dev_messaging_overview General Overview
Messaging is one method how \ref sim::Component "Components" can comunicate
with each other or how to notify Components from outside.

\ref sim::Component "Component" class has virtual method
\ref sim::Component::processMessage "processMessage()" (which can be
overloaded in child classes) that process all
\ref sim::Message "Messages" delivered by simulator.
Each Message is defined by its type and any Component can be registered for
receiving Messages with some type.

Each Component has assign level of priority and so have each Message.
Priority is defined in 5 levels (lowest, lower, normal, higher, highest).
Messages are delivered to Components (through method
\ref sim::Component::processMessage "processMessage()") in order from
highest to lowest priority. First are delivered all Messages to Components
with highest priority then to higher and so on, and first are delivered
Messages with highest priority then with higher and so on.


\section dev_messaging_impl Implementation
Whole messaging algorithm is implmented in
\ref sim::SimComponentMessageRegistry "SimComponentMessageRegistry" class.

Implementation can be divided into 3 logical operations:
- \ref dev_messaging_impl_reg "Component registration to message type"
- \ref dev_messaging_impl_assign "Assigning message to registered Components"
- \ref dev_messaging_impl_deliver "Delivering all assigned Messages to Component"

\subsection dev_messaging_impl_reg Component registration
Each \ref sim::Message "Message" is defined by its type (which is
accessible, for example, by static member ::Type). Any
\ref sim::Component "Component" can be registered to recieve any type of
Messages. Type of Message is simply one unsigned long number and number of
different types can be very much and number can be determined during
compilation - so registration must be dynamic and it should be fast. This
leads to use some sort of hash table for mapping between message type
(unsigned long) to list of Components.

Standard library has already implemented this type of container called
<a href="http://www.cplusplus.com/reference/stl/map">std::map</a>. Type of
message (unsigned long) is key of std::map and value must be list of
Components. This list can't contain duplicate references to Component which
leads to checking whole list everytime Component is registered. List must
be searched during unregistering of Component. Because of this all, also
std::map is used for implementation of list of Components.

So for message type -> Component mapping is used:
\code
std::map<unsigned long, std::map<sim::Component *, sim::Component *> *>
\endcode
See types
\ref sim::SimComponentMessageRegistry::_list_t
\ref sim::SimComponentMessageRegistry::_map_t
and member
\ref sim::SimComponentMessageRegistry::_map.

\ref sim::SimComponentMessageRegistry "SimComponentMessageRegistry" class
has these methods operating with mapping:
- \ref sim::SimComponentMessageRegistry::regComponent "regComponent()" -
registers given Component to message type. Complexity is
\f$O(log(N) + log(M))\f$, where \f$N\f$ is number of all registered types
and \f$M\f$ is number of Components registered to specified message type.
- \ref sim::SimComponentMessageRegistry::unregComponent "unregComponent()" -
unregisters given Component from specified message type. Complexity is same
as regComponent().
- \ref sim::SimComponentMessageRegistry::unregComponentFromAll "unregComponentFromAll()" -
unregisters given Component from all message types. Complexity is
\f$O(N . log(M))\f$, where \f$N\f$ is number of registered message types
and \f$M\f$ is number of registered Components.


\subsection dev_messaging_impl_assign Message assignement
\ref sim::Sim "Sim" class provides method
\ref sim::Sim::sendMessage "sendMessage()" for sending Messages to
Components. Everytime sendMessage() is called given Message is assigned to
all Components that are currently registered to recieving this type of
message but Message is not delivered immadiatelly. Instead of that all
sent Messages are delivered in batch as soon as possible (in next step).
This is because Messages are usually sent from Components as response for
other Message - it prevents dead lock by ping-ponging of Messages.

\ref sim::SimComponentMessageRegistry "SimComponentMessageRegistry" class
stores Messages directly to Component's member
\ref sim::Component::__msgs_to_deliver "__msgs_to_deliver" and this member
should NOT be touch from within Component class itself, only
SimComponentMessageRegistry can touch it.
\ref sim::Component::__msgs_to_deliver "__msgs_to_deliver"
is two arrays of std::lists:
- Two arrays are used to distinguish between
Messages registered to for current step and Messages registered to next
step.
- Array of std::lists has same length as is number of Messages' priorities.
It is because Messages must be delivered to Components in order of theirs
priorities.
- <a href="http://www.cplusplus.com/reference/stl/list">std::list</a> is
used because it has constant time for appending Message.

Type of
\ref sim::Component::__msgs_to_deliver "__msgs_to_deliver"
is thus:
\code
    std::list<const Message *> __msgs_to_deliver[2][Message::PRIO_MAX];
\endcode

\ref sim::SimComponentMessageRegistry "SimComponentMessageRegistry" class
also needs to know which Components have pending Messages to deliver to
prevent iterating over all Components. For this propose is used
\ref sim::SimComponentMessageRegistry "SimComponentMessageRegistry's"
member
\ref sim::SimComponentMessageRegistry::_active "_active" of type:
\code
    std::list<Component *> _active[2][Component::PRIO_MAX][Message::PRIO_MAX];
\endcode
This member holds for every pair of Component priority and Message priority
list of active Components (those that have pending Messages).

Correct filling of previously described variables allow to process messages
as effecient as possible:
- Message can be assigned to Component in constant time. List of Components
registered to this type of Message can be found in \f$O(log(N))\f$ time
(\f$N\f$ - number of message types - see \ref dev_messaging_impl_reg).
- All Components that have pending Messages are stored (and ordered by
priority) in
\ref sim::SimComponentMessageRegistry::_active "_active"
variable in constant time
and the rest of Components (those without any pending Messages) are not
touched at all.

See
\ref sim::SimComponentMessageRegistry::assignMessage "assignMessage"
method.


\subsection dev_messaging_impl_deliver Message delivering
From subsection above should be clear that this operation is very simple a
fast. It is enough to iterate over all active Components stored in
\ref sim::SimComponentMessageRegistry::_active "_active"
member (other Components are untouched!) and call
\ref sim::Component::processMessage "processMessage()"
method on these components with Messages as argument stored in theirs
member
\ref sim::Component::__msgs_to_deliver "__msgs_to_deliver"
member.

It should be clear that number of operations during delivering is one for
each pair of Component-Message that has to be delivered which is _minumum_
possible.

See
\ref sim::SimComponentMessageRegistry::deliverMessages "deliverMessages"
and
\ref sim::SimComponentMessageRegistry::deliverAssignedMessages "deliverAssignedMessages"
methods.
*/


#ifndef _SIM_SIM_HPP_
#define _SIM_SIM_HPP_

#include "world.hpp"
#include "visworld.hpp"
#include "component.hpp"
#include "message.hpp"
#include "time.hpp"

namespace sim {

/**
 * Registry connecting Messages and Components - this class is responsible
 * for Component registrations to Messages and Message delivering.
 * See \ref dev_messaging for more info.
 */
class SimComponentMessageRegistry {
    typedef std::map<Component *, Component *> _list_t;
    typedef std::map<unsigned long, _list_t *> _map_t;

    /**
     * Registry holding what components are registered to what type of
     * messages.
     */
    _map_t _map;

    unsigned char _counter;

    /**
     * List of Components with pending Messages.
     */
    std::list<Component *> _active[2][Component::PRIO_MAX][Message::PRIO_MAX];

    /**
     * List of all messages.
     */
    std::list<Message *> _msgs[2];

  public:
    SimComponentMessageRegistry();
    virtual ~SimComponentMessageRegistry();

    /**
     * Register Component to messages of specified type.
     * Complexity: O(log(num of all already registered types))
     */
    void regComponent(Component *c, unsigned long msg_type);
    void unregComponent(Component *c, unsigned long msg_type);

    /**
     * Unregister component from all message types.
     * Complexity: O(N * log(M)), N - number of msg types, M number of
     * registered components.
     */
    void unregComponentFromAll(Component *c);

    /**
     * Assign Message to registered Components.
     * Complexity: O(log(num of all already registered types) + num of
     * components registered to msg's type)
     */
    void assignMessage(Message *m);

    /**
     * Deliver all Messages.
     */
    void deliverMessages();

    /**
     * Deliver all Messages assigned to specified Component.
     */
    void deliverAssignedMessages(Component *c, Message::Priority prio);
};


/**
 * Simulator.
 */
class Sim {
  protected:
    World *_world;
    VisWorld *_visworld;
    pthread_mutex_t _step_lock; //!< Sync lock for World/VisWorld steps
    pthread_t _th_step_world; //!< Thread for World's steps
    pthread_t _th_step_visworld; //!< Thread for VisWorld's steps

    std::list<Component *> _cs; //!< List of all components
    std::list<Component *> _cs_uninit; //!< List of uninitialized components

    /**
     * List of components registered for preStep callback
     * (see Component::cbPreStep()).
     */
    std::list<Component *> _cs_pre;
    std::list<Component *> _cs_pre_to_unreg; //!< List of components meant to unregister

    /**
     * List of components registered for postStep callback
     * (see Component::cbPostStep()).
     */
    std::list<Component *> _cs_post;
    std::list<Component *> _cs_post_to_unreg;

    bool _in_cb; //!< True if in _cbPostStep()/_cbPreStep() methods

    SimComponentMessageRegistry _reg;

    Timer _timer_real;
    Time _time_simulated;

    Time _time_step; //!< Length of simulation steps
    unsigned int _time_substeps; //!< Number of sim. substeps
    Time _vis_time_step; //!< Delay between VisWorld's steps

    bool _simulate; //!< True if simulation is running
    bool _simulate_real; //!< True if simulator should try to synchorinze
                         //!< real time with simulated

    Time _time_limit; //!< Max simulated time
    bool _time_limit_enabled; //!< True if _time_limit is considered

  protected:
    typedef std::list<Component *>::iterator cit_t; //!< Component list iterator
    typedef std::list<Component *>::const_iterator const_cit_t;

  public:
    Sim(World *world = 0, VisWorld *visworld = 0);
    virtual ~Sim();

    World *world() { return _world; }
    const World *world() const { return _world; }
    VisWorld *visWorld() { return _visworld; }
    const VisWorld *visWorld() const { return _visworld; }

    void setWorld(World *w);
    void setVisWorld(VisWorld *w);

    const Time &timeStep() const { return _time_step; }
    unsigned int timeSubSteps() const { return _time_substeps; }
    void setTimeStep(const Time &t) { _time_step = t; }
    void setTimeSubSteps(unsigned int ss) { _time_substeps = ss; }

    const Time &visTimeStep() const { return _vis_time_step; }

    /**
     * Sets up delay between VisWorld's steps.
     */
    void setVisTimeStep(const Time &t) { _vis_time_step = t; }


    const Time &timeLimit() const { return _time_limit; }
    bool timeLimitEnabled() const { return _time_limit_enabled; }
    void setTimeLimit(const Time &t) { _time_limit = t; _time_limit_enabled = true; }
    void resetTimeLimit() { _time_limit_enabled = false; }

    virtual void init();
    virtual void finish();
    virtual bool done();

    /**
     * Run simulation.
     */
    virtual void run();

    /**
     * Is called when key is pressed.
     * Overload this method if you need to.
     */
    virtual bool pressedKey(int key);

    /**
     * Adds component to simulator if it wasn't already added.
     *
     * Component's init() function is called.
     */
    void addComponent(Component *c);

    /**
     * Removes component from simulator if it was added before.
     *
     * Component's finish() function is called.
     */
    void rmComponent(Component *c);

    /**
     * Sends message to all Components currently registered for receiving
     * Message of this type.
     */
    void sendMessage(Message *msg);

    /**
     * Same as sendMessage(Message *msg) but priority of Message can be
     * changed.
     */
    void sendMessage(Message *msg, Message::Priority prio);

    /**
     * Registers Component for cbPreStep() callback.
     * Component is registered only if it is already added using
     * addComponent() method.
     */
    void regPreStep(Component *c);
    void unregPreStep(Component *c);

    /**
     * Register Component for cbPostStep() callback.
     */
    void regPostStep(Component *c);
    void unregPostStep(Component *c);

    /**
     * Registers Component to receive messages with specified type.
     */
    void regMessage(Component *c, unsigned long msg_type);

    /**
     * Oposite of regMessage().
     */
    void unregMessage(Component *c, unsigned long msg_type);

    void timeRealRestart() { _timer_real.start(); }
    const Time &timeReal() const { return _timer_real.elapsedTime(); }
    const Time &timeRealNow() { return _timer_real.stop(); }
    const Time &timeSimulated() const { return _time_simulated; }

    void pauseSimulation();
    void continueSimulation();
    void toggleSimulation()
        { if (_simulate) pauseSimulation();
          else continueSimulation(); }

    bool simulateReal() const { return _simulate_real; }
    void setSimulateReal(bool yes = true) { _simulate_real = yes; }

  protected:
    void _initComponents();
    void _finishComponents();

    /**
     * Returns true if Component was added.
     */
    bool _hasComponent(const Component *c) const;

    /**
     * Returns true if Component is registered for cbPreStep() callback.
     */
    bool _hasPreStep(const Component *c) const;

    /**
     * Returns true if Component is registered for cbPreStep() callback.
     */
    bool _hasPostStep(const Component *c) const;

    /**
     * Returns true if Component is in list.
     */
    bool _isComponentInList(const Component *c,
                            const std::list<Component *> &list) const;


    /**
     * Performes simulation step of physical world.
     */
    void _stepWorld();

    /**
     * Performs step of visual world.
     */
    void _stepVisWorld();

    /**
     * Starts threads performing World and VisWorlds' steps.
     */
    void _runStepThreads();

    /**
     * Joins step threads started by _runStepThreads().
     */
    void _joinStepThreads();


    static void *_worldStepsThread(void *);
    static void *_visWorldStepsThread(void *);

    /**
     * Calls all Component::cbPreStep() methods on all registered
     * components.
     */
    void _cbPreStep();

    /**
     * Calls all Component::cbPostStep() methods on all registered
     * components.
     */
    void _cbPostStep();

    /**
     * Deliver all messages to Components.
     */
    void _cbMessages();

    void _unregPending();
};

}

#endif /* _SIM_SIM_HPP_ */
