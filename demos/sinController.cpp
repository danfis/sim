#ifndef SIN_CONTROLLER_H
#define SIN_CONTROLLER_H

#include "sinController.hpp"

#include <iostream>


SinController::SinController(sim::robot::SSSA *robot, const double frequency, const double amplitude, const double phase):
    _robot(robot),_freq(frequency),_amplitude(amplitude),_phase(phase) {

}

SinController::~SinController() {
}

void SinController::init(sim::Sim *sim) {
    sim->regPreStep(this);
}

void SinController::cbPreStep() {
    std::cerr << "ahoj\n";
}
#endif

