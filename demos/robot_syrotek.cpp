#include "robot_syrotek.hpp"
#include <sim/msg.hpp>
#include <sim/sensor/camera.hpp>
using sim::Vec3;
using sim::Quat;
using sim::Scalar;


RobotSyrotekComp::RobotSyrotekComp()
    : sim::Component(), _sim(0), _robot(0)
{
}

RobotSyrotekComp::~RobotSyrotekComp()
{
    if (_robot)
        delete _robot;
}

void RobotSyrotekComp::init(sim::Sim *sim)
{
    Vec3 pos(0., 0., 0.08);

    _sim = sim;
    _robot = new sim::robot::Syrotek(sim->world(), pos,
                                     osg::Vec4(0.7, 0.1, 0., 0.6));

    sim::sensor::Camera *cam = new sim::sensor::Camera;
    //cam->attachToBody(_robot->chasis(), Vec3(0.13, 0., 0.18), Quat(Vec3(0., 0., 1.), M_PI / 2.));
    cam->attachToBody(_robot->chasis(), Vec3(0.13, 0., 0.15));
    cam->visBodyEnable();
    cam->setWidthHeight(300, 300);
    cam->setBgColor(0., 0., 0., 1.);
    cam->enableDump("cam-syrotek/");
    cam->enableView();
    sim->addComponent(cam);

    _robot->activate();

    /*
    if (_joystick)
        sim->regPreStep(this);
    */
    {
        sim::comp::Joystick *j = new sim::comp::Joystick(0);
        sim->addComponent(j);
        sim->regMessage(this, sim::comp::JoystickMessage::Type);
    }

    sim->regMessage(this, sim::MessageKeyPressed::Type);
}

void RobotSyrotekComp::finish()
{
}

void RobotSyrotekComp::cbPreStep()
{
    /*
    if (!_joystick)
        return;

    if (sim::Time::diff(_joystick_last, sim::Time::cur()) < _joystick_delay)
        return;

    _joystick_last = sim::Time::cur();

    SDL_JoystickUpdate();

    if (SDL_JoystickGetButton(_joystick, 6)){
        _robot->addVelLeft(-0.1);
    }else if (SDL_JoystickGetButton(_joystick, 4)){
        _robot->addVelLeft(0.1);
    }

    if (SDL_JoystickGetButton(_joystick, 7)){
        _robot->addVelRight(-0.1);
    }else if (SDL_JoystickGetButton(_joystick, 5)){
        _robot->addVelRight(0.1);
    }
    DBG("Velocity: " << _robot->velLeft() << " " << _robot->velRight());
    */
}

void RobotSyrotekComp::processMessage(const sim::Message &msg)
{
    if (msg.type() == sim::MessageKeyPressed::Type){
        _keyPressed((const sim::MessageKeyPressed &)msg);
    }else if (msg.type() == sim::comp::JoystickMessage::Type){
        _joystick((const sim::comp::JoystickMessage &)msg);
    }
}

void RobotSyrotekComp::_keyPressed(const sim::MessageKeyPressed &msg)
{
    int key = msg.key();

    //DBG("Component: " << this << " - key pressed: " << msg.key());

    if (key == 'h'){
        _robot->addVelLeft(0.1);
    }else if (key == 'j'){
        _robot->addVelLeft(-0.1);
    }else if (key == 'k'){
        _robot->addVelRight(0.1);
    }else if (key == 'l'){
        _robot->addVelRight(-0.1);
    }
    DBG("Velocity: " << _robot->velLeft() << " " << _robot->velRight());
}

void RobotSyrotekComp::_joystick(const sim::comp::JoystickMessage &msg)
{
    if (msg.button(0)){
        _robot->setVelLeft(0.);
        _robot->setVelRight(0.);
    }else if (msg.button(2)){
        _robot->addVelLeft(0.1);
        _robot->addVelRight(0.1);
    }else if (msg.button(3)){
        _robot->addVelLeft(-0.1);
        _robot->addVelRight(-0.1);
    }else if (msg.button(6)){
        _robot->addVelLeft(0.1);
    }else if (msg.button(7)){
        _robot->addVelLeft(-0.1);
    }else if (msg.button(5)){
        _robot->addVelRight(0.1);
    }else if (msg.button(4)){
        _robot->addVelRight(-0.1);
    }
    DBG("Velocity: " << _robot->velLeft() << " " << _robot->velRight());
}
