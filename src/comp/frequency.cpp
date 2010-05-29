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
    if (_type == 0) {
        _joint->setParamFMax(50);
        _joint->setParamVel(0);
    } else {
        _joint->setParamFMax2(50);
        _joint->setParamVel2(0);
    }
    _sim->regPreStep(this);

}

void Frequency::finish(){
}

void Frequency::cbPreStep() {

     sim::Time t = _sim->timeReal();
    const double ts = t.inSF();


    if (ts > 1) {
		double arg = ts*_frequency + _phase;
		if (arg > 2*M_PI) {
			arg = arg - (2*M_PI*floor(arg/(2*M_PI)));
		} 
		if (arg < -2*M_PI) {
			arg = -arg;
			arg = arg - (2*M_PI*floor(arg/(2*M_PI)));
			arg = -arg;

		}
		//const double newVel = _amplitude*sin(ts*_frequency + _phase);
		const double newVel = _amplitude*sin(arg);

		if (_type == 0) {
			_joint->setParamVel(newVel);
		} else {
			_joint->setParamVel2(newVel);
		}
    }
   
}

void Frequency::processMessage(const Message &msg) {
    
}


}

}
