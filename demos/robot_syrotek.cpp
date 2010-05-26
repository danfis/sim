#include "robot_syrotek.hpp"
#include <sim/msg.hpp>
#include <sim/sensor/camera.hpp>
using sim::Vec3;
using sim::Quat;
using sim::Scalar;


RobotSyrotekComp::RobotSyrotekComp()
    : sim::Component(), _sim(0), _robot(0),
      _joystick(0)
{
    int initialized = false;

    if (SDL_WasInit(SDL_INIT_JOYSTICK) == 0){
        if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) != 0){
            ERR("Can't initialize Joystick!");
        }else{
            initialized = true;
        }
    }else{
        initialized = true;
    }

    if (initialized){
        _joystick_delay = sim::Time(0, 100000000UL);
        _joystick_last = sim::Time::cur();

        _joystick = SDL_JoystickOpen(0);
        SDL_JoystickEventState(SDL_IGNORE);

#ifndef NDEBUG
        {
            DBG("Joystick:");
            int num;
            num = SDL_NumJoysticks();
            DBG("  " << num << " joysticks found");
            for (int i = 0; i < num; i++){
                DBG("    " << i << ": " << SDL_JoystickName(i));
            }
            DBG("  Num axes: " << SDL_JoystickNumAxes(_joystick));
            DBG("  Num buttons: " << SDL_JoystickNumButtons(_joystick));
            DBG("  Num hats: " << SDL_JoystickNumHats(_joystick));
            DBG("  Num balls: " << SDL_JoystickNumBalls(_joystick));
        }
#endif /* NDEBUG */
    }
}

RobotSyrotekComp::~RobotSyrotekComp()
{
    if (_robot)
        delete _robot;

    if (_joystick){
        SDL_JoystickClose(_joystick);
        SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
    }
}

void RobotSyrotekComp::init(sim::Sim *sim)
{
    Vec3 pos(0., 0., 0.08);

    _sim = sim;
    _robot = new sim::robot::Syrotek(sim->world(), pos);

    sim::sensor::Camera *cam = new sim::sensor::Camera;
    //cam->attachToBody(_robot->chasis(), Vec3(0.13, 0., 0.18), Quat(Vec3(0., 0., 1.), M_PI / 2.));
    cam->attachToBody(_robot->chasis(), Vec3(0.13, 0., 0.15));
    cam->visBodyEnable();
    cam->setWidthHeight(300, 300);
    cam->setBgColor(0., 0., 0., 1.);
    cam->enableDump("cam/");
    cam->enableView();
    sim->addComponent(cam);

    _robot->activate();

    if (_joystick)
        sim->regPreStep(this);

    sim->regMessage(this, sim::MessageKeyPressed::Type);
}

void RobotSyrotekComp::finish()
{
}

void RobotSyrotekComp::cbPreStep()
{
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
}

void RobotSyrotekComp::processMessage(const sim::Message &_msg)
{
    if (_msg.type() == sim::MessageKeyPressed::Type){
        const sim::MessageKeyPressed &msg = (const sim::MessageKeyPressed &)_msg;
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
}
