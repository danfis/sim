#include "robot.hpp"

Robot::Robot(const Vec3 &pos, const Quat &rot, bool use_cam)
        : sim::comp::SSSA(pos, rot), _cam(0)
{
    if (use_cam){
        _cam = new sim::sensor::Camera();
    }

    _counter = 0;
    ferVec3Set(&_s, INIT_S);
    ferVec3Set(&_h, INIT_H);
    ferVec3Set(&_a, 0, 0, 0);

    ferMat3Set(&_K1, K1);
    ferMat3Set(&_K2, K2);
    ferMat3Set(&_K3, K3);
    ferMat3Set(&_K4, K4);
    ferMat3Set(&_K5, K5);
    ferMat3Set(&_K6, K6);
    ferMat3Set(&_A, A);
}


void Robot::init(sim::Sim *sim)
{
    sim::comp::SSSA::init(sim);
    sim->regMessage(this, sim::MessageKeyPressed::Type);
    sim->regPreStep(this);

    if (_cam){
        _cam->attachToBody(_robot->arm(),
                           Vec3(-0.07, 0., 0.4),
                           Quat(Vec3(0, 0, 1), M_PI));
        _cam->setWidthHeight(320, 240);
        _cam->setBgColor(0., 0., 0., 1.);

        _cam->visBodyEnable(true);
        //_cam->enableView(true);
        //_cam->enableDump("a");

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

void Robot::cbPreStep()
{
    if (_cam){
    }

    _gatherInput();
}

void Robot::_gatherInput()
{
    osg::Image *img;

    _counter++;

    if (_counter < FPS)
        return;

    _counter = 0;

    if (!_cam || !_cam->image()->valid()){
        ferVec3Set(&_s, 0, 0, 0);
        return;
    }

    DBG("");
    img = _cam->image();
}
