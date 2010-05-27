#include "frequency.hpp"
#include <sim/msg.hpp>
#include <osgGA/GUIEventAdapter>
#include <sim/time.hpp>
#include <math.h>

namespace sim {

namespace comp {

Frequency::Frequency(sim::Joint *joint, const double amplitude, const double frequency, const double phase, const int type){
    _joint = joint;
    _amplitude = amplitude;
    _frequency = frequency;
    _phase = phase;
    _type = type;
}

Frequency::~Frequency(){
    _joint = NULL;
}

void Frequency::init(sim::Sim *sim) {
	_sim = sim;
    _joint->setParamFMax(10);
    _joint->setParamFMax2(10);
    _sim->regPreStep(this);

}

void Frequency::finish(){
}

void Frequency::cbPreStep() {

    



}

void Frequency::processMessage(const Message &msg) {
    
    sim::Time t = _sim->timeReal();
    const double ts = t.inSF();

    const double newVel = _amplitude*sin(ts*_frequency + _phase);

    DBG("Setting vel " << newVel);
    if (_type == 0) {
        _joint->setParamVel(newVel);
    } else {
        _joint->setParamVel2(newVel);
    }
}


}

}
