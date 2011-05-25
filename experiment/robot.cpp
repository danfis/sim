#include "robot.hpp"

Robot::Robot(const Vec3 &pos, const Quat &rot, bool use_cam)
        : sim::comp::SSSA(pos, rot), _cam(0)
{
    if (use_cam){
        _cam = new sim::sensor::Camera();
    }
}


void Robot::init(sim::Sim *sim)
{
    sim::comp::SSSA::init(sim);
    sim->regMessage(this, sim::MessageKeyPressed::Type);

    if (_cam){
        _cam->attachToBody(_robot->arm(),
                           Vec3(-0.07, 0., 0.4),
                           Quat(Vec3(0, 0, 1), M_PI));
        _cam->setWidthHeight(640, 480);
        _cam->setBgColor(0., 0., 0., 1.);

        _cam->visBodyEnable(true);
        _cam->enableView(true);

        sim->addComponent(_cam);
    }

    _robot->setVelLeft(1.);
    _robot->setVelRight(1.);
}

void Robot::processMessage(const sim::Message &msg)
{
    if (msg.type() == sim::MessageKeyPressed::Type){
        _keyPressedMsg((const sim::MessageKeyPressed &)msg);
    }
}

void Robot::_keyPressedMsg(const sim::MessageKeyPressed &msg)
{
    int key = msg.key();

    DBG("Component: " << this << " - key pressed: " << msg.key());

    if (key == 'h'){
        _robot->addVelLeft(0.1);
    }else if (key == 'j'){
        _robot->addVelLeft(-0.1);
    }else if (key == 'k'){
        _robot->addVelRight(0.1);
    }else if (key == 'l'){
        _robot->addVelRight(-0.1);

    }else if (key == 'n'){
        _robot->addVelArm(0.1);
    }else if (key == 'm'){
        _robot->addVelArm(-0.1);
    }else if (key == ','){
        _robot->fixArm();
    }else if (key == '.'){
        _robot->unfixArm();
    }else if (key == 'v'){
        _robot->reachArmAngle(M_PI / 4.);
    }else if (key == 'b'){
        _robot->reachArmAngle(-M_PI / 4.);
    }
    DBG("Velocity: " << _robot->velLeft() << " " << _robot->velRight() << " " << _robot->velArm());
}
