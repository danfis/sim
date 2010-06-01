#include "syrotek.hpp"
#include <sim/msg.hpp>

namespace sim {
namespace comp {

Syrotek::Syrotek(const Vec3 &pos, const osg::Vec4 &color)
    : sim::Component(), _sim(0), _pos(pos), _color(color),
      _robot(0),
      _cam(0), _range_finder(0), _joystick(0)
{
}

Syrotek::~Syrotek()
{
    if (_robot)
        delete _robot;
}

void Syrotek::init(sim::Sim *sim)
{
    _sim = sim;

    _robot = new sim::robot::Syrotek(_sim->world(), _pos, _color);
    _robot->activate();
}

void Syrotek::finish()
{
    //TODO: _robot->deactivate();
}


void Syrotek::processMessage(const sim::Message &msg)
{
    if (msg.type() == sim::MessageKeyPressed::Type){
        _keyPressedMsg((const sim::MessageKeyPressed &)msg);
    }else if (msg.type() == sim::comp::JoystickMessage::Type){
        _joystickMsg((const sim::comp::JoystickMessage &)msg);
    }
}

void Syrotek::_keyPressedMsg(const sim::MessageKeyPressed &msg)
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

void Syrotek::_joystickMsg(const sim::comp::JoystickMessage &msg)
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


void Syrotek::_useCamera(int width, int height)
{
    _cam = new sim::sensor::Camera;
    _cam->attachToBody(_robot->chasis(), Vec3(0.13, 0., 0.15));
    _cam->setWidthHeight(width, height);
    _cam->setBgColor(0., 0., 0., 1.);

    //_cam->visBodyEnable();
    //_cam->enableDump("cam-syrotek/");
    //_cam->enableView();

    _sim->addComponent(_cam);
}

void Syrotek::_useRangeFinder()
{

    _range_finder = new sim::sensor::RangeFinder(1., 181, M_PI / 2.);
    _range_finder->attachToBody(_robot->chasis(), Vec3(0.13, 0., 0.05));
    _range_finder->enableVis();
    _sim->addComponent(_range_finder);
}

void Syrotek::_useJoystick()
{
    _joystick = new sim::comp::Joystick(0);
    _sim->addComponent(_joystick);
    _sim->regMessage(this, sim::comp::JoystickMessage::Type);
}

void Syrotek::_useKeyboard()
{
    _sim->regMessage(this, sim::MessageKeyPressed::Type);
}

}
}
