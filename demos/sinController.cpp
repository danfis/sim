#ifndef SIN_CONTROLLER_H
#define SIN_CONTROLLER_H

#include "sinController.hpp"

#include <iostream>


SinController::SinController(sim::robot::SSSA *robot, const double frequency, const double amplitude, const double phase, const int name):
    _robot(robot),_freq(frequency),_amplitude(amplitude),_phase(phase),_name(name) {

}

SinController::~SinController() {
}

void SinController::init(sim::Sim *sim) {
    _sim = sim;
    sim->regPreStep(this);
}

void SinController::cbPreStep() {
    sim::Time t = _sim->timeReal();
    const double ts = t.inSF();

    double arg = ts*_freq + _phase;
    if (arg > 2*M_PI) {
        arg = arg - (2*M_PI*floor(arg/(2*M_PI)));
    } 
    if (arg < -2*M_PI) {
        arg = -arg;
        arg = arg - (2*M_PI*floor(arg/(2*M_PI)));
        arg = -arg;
    }
    const double newAngle = _amplitude*sin(arg);
    _robot->setVelArm(newAngle);

    std::cout << "Robot" << _name << " " << _robot->pos()[0] << " " << _robot->pos()[1] << " " << _robot->pos()[2] << "\n";
}
#endif

