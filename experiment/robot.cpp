#include "robot.hpp"
#include "sim.hpp"

void setMatrix(gsl_matrix *m, ...)
{
    va_list ap;
    double val;
    int i, j;

    va_start(ap, m);

    for (i = 0; i < 7; i++){
        for (j = 0; j < 7; j++){
            val = va_arg(ap, double);
            gsl_matrix_set(m, i, j, val);
        }
    }

    va_end(ap);
}


Robot::Robot(const Vec3 &pos, const Quat &rot, bool use_cam)
        : sim::comp::SSSA(pos, rot), _sim(0), _cam(0),
          _finder(0), _finder2(0), _last_pos(pos)
{
    blobf::rgb_t rgb;

    if (use_cam){
        _cam = new sim::sensor::Camera();

        _finder = blobf::finderNew(WIDTH, HEIGHT);
        /*
           rgb.r = 255 * 0.7;
           rgb.g = 255 * 0.7;
           rgb.b = 255 * 0.1;
           blobf::finderAddPixel(_finder, rgb);
         */
        rgb.r = 255;
        rgb.g = 255;
        rgb.b = 49;
        blobf::finderAddPixel(_finder, rgb);

        _finder2 = blobf::finderNew(WIDTH, HEIGHT);
        rgb.r = 225;
        rgb.g = 35;
        rgb.b = 35;
        blobf::finderAddPixel(_finder2, rgb);
    }

    _s = gsl_vector_alloc(7);
    _h = gsl_vector_alloc(7);
    _a = gsl_vector_alloc(7);

    gsl_vector_set_zero(_s);
    gsl_vector_set_zero(_h);
    gsl_vector_set_zero(_a);

    _K1 = gsl_matrix_alloc(7, 7);
    _K2 = gsl_matrix_alloc(7, 7);
    _K3 = gsl_matrix_alloc(7, 7);
    _K4 = gsl_matrix_alloc(7, 7);
    _K5 = gsl_matrix_alloc(7, 7);
    _K6 = gsl_matrix_alloc(7, 7);
    _A  = gsl_matrix_alloc(7, 7);

    setMatrix(_K1, K1);
    setMatrix(_K2, K2);
    setMatrix(_K3, K3);
    setMatrix(_K4, K4);
    setMatrix(_K5, K5);
    setMatrix(_K6, K6);
    setMatrix(_A, A);

    gsl_vector_set_zero(_s);
    gsl_vector_set_zero(_h);
    //gsl_vector_set(_h, 6, INIT_POWER);

    _counter = 0;
    _wait_for_robot = 0;
}

Robot::~Robot()
{
    gsl_vector_free(_s);
    gsl_vector_free(_h);
    gsl_vector_free(_a);

    gsl_matrix_free(_K1);
    gsl_matrix_free(_K2);
    gsl_matrix_free(_K3);
    gsl_matrix_free(_K4);
    gsl_matrix_free(_K5);
    gsl_matrix_free(_K6);
    gsl_matrix_free(_A);

    if (_finder)
        blobf::finderDel(_finder);
    if (_finder2)
        blobf::finderDel(_finder2);
}

void Robot::init(sim::Sim *sim)
{
    sim::comp::SSSA::init(sim);

    _sim = (Sim *)sim;

    sim->regMessage(this, sim::MessageKeyPressed::Type);
    sim->regPreStep(this);

    if (_cam){
        _cam->attachToBody(_robot->arm(),
                           //Vec3(-0.07, 0., 0.4),
                           Vec3(-0.07, 0., 0.),
                           Quat(Vec3(0, 0, 1), M_PI));
        _cam->setWidthHeight(WIDTH, HEIGHT);
        _cam->setBgColor(0., 0., 0., 1.);

        //_cam->visBodyEnable(true);
        _cam->enableView(true);
        //_cam->enableDump("cam/");

        sim->addComponent(_cam);
    }

    //_robot->setVelLeft(1.);
    //_robot->setVelRight(1.);

    _robot->setData(this);

    _sim->connectRobots();

    _decolorize();
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

    //DBG("Component: " << this << " - key pressed: " << msg.key());

    if (key == 's'){
        fprintf(stderr, ":: %lx (wfr: %d) ::        \n", (long)this, _wait_for_robot);
        DBG("Velocity: " << _robot->velLeft() << " " << _robot->velRight() << " " << _robot->velArm());
        DBG("_s: " << gsl_vector_get(_s, 0) << " "
                   << gsl_vector_get(_s, 1) << " "
                   << gsl_vector_get(_s, 2) << " "
                   << gsl_vector_get(_s, 3) << " "
                   << gsl_vector_get(_s, 4) << " "
                   << gsl_vector_get(_s, 5) << " "
                   << gsl_vector_get(_s, 6));
        DBG("_h: " << gsl_vector_get(_h, 0) << " "
                   << gsl_vector_get(_h, 1) << " "
                   << gsl_vector_get(_h, 2) << " "
                   << gsl_vector_get(_h, 3) << " "
                   << gsl_vector_get(_h, 4) << " "
                   << gsl_vector_get(_h, 5) << " "
                   << gsl_vector_get(_h, 6));
        DBG("_a: " << gsl_vector_get(_a, 0) << " "
                   << gsl_vector_get(_a, 1) << " "
                   << gsl_vector_get(_a, 2) << " "
                   << gsl_vector_get(_a, 3) << " "
                   << gsl_vector_get(_a, 4) << " "
                   << gsl_vector_get(_a, 5) << " "
                   << gsl_vector_get(_a, 6));
    }

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
    //DBG("Velocity: " << _robot->velLeft() << " " << _robot->velRight() << " " << _robot->velArm());
}

void Robot::cbPreStep()
{
    int call_sim;

    _counter++;

    if (_counter == FPS){
        //fprintf(stderr, ":: %lx (wfr: %d) ::        \n", (long)this, _wait_for_robot);
        _gatherInput();
        _updateHormone();
        _updateActions();

        if (gsl_vector_get(_h, 6) > WAIT_FOR_ROBOT_TRESHOLD){
            call_sim = (_wait_for_robot == 0);
            _wait_for_robot = 1;
            _colorize();
            if (call_sim)
                _sim->waitForRobot(this);
        }else{
            //call_sim = (_wait_for_robot == 1);
            _wait_for_robot = 0;
            _decolorize();
            //if (call_sim)
            //    _sim->waitForRobot(this);
        }
        //DBG("wfr: " << _wait_for_robot);

        _counter = 0;

        if (!robot()->connectedRobot() && gsl_vector_get(_s, 2) > 60000)
            _sim->connectRobots();
    }
}

//#include <osgDB/WriteFile>
void Robot::_gatherInput()
{
    osg::Image *img;
    blobf::segment_t seg, seg2;

    if (!_cam || !_cam->image()->valid()){
        gsl_vector_set_zero(_s);
        gsl_vector_set(_s, 6, 1);
        return;
    }

    if (_wait_for_robot){
        gsl_vector_set_zero(_s);
    }else{
        img = _cam->image();

        seg = blobf::finderFindSegment(_finder, img);

        if (seg.size > 0){
            gsl_vector_set(_s, 0, (seg.x - (WIDTH / 2.)) );
            gsl_vector_set(_s, 1, ((HEIGHT / 2.) - seg.y));
            gsl_vector_set(_s, 2, seg.size);
        }else{
            gsl_vector_set(_s, 0, 0);
            gsl_vector_set(_s, 1, 0);
            gsl_vector_set(_s, 2, 0);
        }

        seg2 = blobf::finderFindSegment(_finder2, img);
        seg2.y = HEIGHT - seg2.y;

        if (seg2.size > 1000){
            if (seg2.x >= seg.x+10){
                gsl_vector_set(_s, 3, (seg2.x-WIDTH));
            }else if (seg2.x <= seg.x-10){
                gsl_vector_set(_s, 3, (seg2.x-0));
            }else{
                gsl_vector_set(_s, 3, 0);
            }

            if (seg2.y >= seg.y){
                gsl_vector_set(_s, 4, (HEIGHT - seg2.y));
            }else if (seg2.y <= seg.y){
                gsl_vector_set(_s, 4, (seg2.y - HEIGHT));
            }else{
                gsl_vector_set(_s, 4, 0.);
            }
        }else{
            gsl_vector_set(_s, 3, 0);
            gsl_vector_set(_s, 4, 0);
            gsl_vector_set(_s, 5, 0);
        }

//	gsl_vector_set(_s, 0,gsl_vector_get(_s,0)*1*seg.size*(seg.size+4*seg2.size));
//	gsl_vector_set(_s, 3,gsl_vector_get(_s,3)*4*seg2.size*(seg.size+4*seg2.size));

        //gsl_vector_set(_s, 6, (_last_pos - _robot->pos()).length());
        //gsl_vector_set(_s, 6, gsl_vector_get(_s, 6) + 1);
        gsl_vector_set(_s, 6, 1);
    }

    _last_pos = _robot->pos();

    /*
    DBG("_s: " << gsl_vector_get(_s, 0) << " "
               << gsl_vector_get(_s, 1) << " "
               << gsl_vector_get(_s, 2) << " "
               << gsl_vector_get(_s, 3));
    */
    //DBG((long)this << ":: seg.x: " << seg.x << ", .y: " << seg.y << ", .size: " << seg.size);
    /*
    {
        static int ___c = 0;
        char fn[100];
        sprintf(fn, "a/%06d.png", ___c++);
        osgDB::writeImageFile(*img, fn);
        DBG(fn);
    }
    */
}

void Robot::_updateHormone()
{
    gsl_vector *v, *w, *h_1;
    //fer_vec3_t a, b, h;
    sim::robot::SSSA *robot;
    Robot *comp;

    v = gsl_vector_alloc(7);
    w = gsl_vector_alloc(7);
    h_1 = gsl_vector_alloc(7);
    gsl_vector_memcpy(h_1, _h);

    // input
    gsl_blas_dgemv(CblasNoTrans, 1., _K1, _s, 0., v);
    gsl_vector_add(_h, v);

    // actions
    gsl_blas_dgemv(CblasNoTrans, 1., _K2, _a, 0., v);
    gsl_vector_add(_h, v);

    // robot on ball
    robot = _robot->connectedRobot();
    if (robot){
        comp = (Robot *)robot->data();
        gsl_vector_memcpy(w, h_1);
        gsl_vector_sub(w, comp->hormone());
        gsl_blas_dgemv(CblasNoTrans, 1., _K3, w, 0., v);
        gsl_vector_add(_h, v);
    }

    // robot on socket[0]
    robot = _robot->connectedRobotSocket(0);
    if (robot){
        comp = (Robot *)robot->data();
        gsl_vector_memcpy(w, h_1);
        gsl_vector_sub(w, comp->hormone());
        gsl_blas_dgemv(CblasNoTrans, 1., _K4, w, 0., v);
        gsl_vector_add(_h, v);
    }

    // robot on socket[1]
    robot = _robot->connectedRobotSocket(1);
    if (robot){
        comp = (Robot *)robot->data();
        gsl_vector_memcpy(w, h_1);
        gsl_vector_sub(w, comp->hormone());
        gsl_blas_dgemv(CblasNoTrans, 1., _K5, w, 0., v);
        gsl_vector_add(_h, v);
    }

    // robot on socket[2]
    robot = _robot->connectedRobotSocket(2);
    if (robot){
        comp = (Robot *)robot->data();
        gsl_vector_memcpy(w, h_1);
        gsl_vector_sub(w, comp->hormone());
        gsl_blas_dgemv(CblasNoTrans, 1., _K6, w, 0., v);
        gsl_vector_add(_h, v);
    }

    gsl_vector_free(v);
    gsl_vector_free(w);
    gsl_vector_free(h_1);

    /*
    DBG("_h: " << gsl_vector_get(_h, 0) << " "
               << gsl_vector_get(_h, 1) << " "
               << gsl_vector_get(_h, 2) << " "
               << gsl_vector_get(_h, 3));
    */
}

void Robot::_updateActions()
{
    if (_wait_for_robot || !_cam){
        gsl_vector_set_zero(_a);

        _robot->setVelLeft(0);
        _robot->setVelRight(0);
        _robot->setVelArm(0);
    }else{
        gsl_blas_dgemv(CblasNoTrans, 1., _A, _h, 0., _a);

        if (gsl_vector_get(_s, 2) > 60000){
            _robot->setVelLeft(0.1 * (VEL_OFFSET + gsl_vector_get(_a, 0)));
            _robot->setVelRight(0.1 * (VEL_OFFSET + -gsl_vector_get(_a, 0)));
            _robot->setVelArm(0.1 * gsl_vector_get(_a, 1));
        }else{
            _robot->setVelLeft(VEL_OFFSET + gsl_vector_get(_a, 0));
            _robot->setVelRight(VEL_OFFSET + -gsl_vector_get(_a, 0));
            _robot->setVelArm(gsl_vector_get(_a, 1));
        }

        /*
        if (_robot->connectedRobot()){
            _robot->setVelArm(30 * gsl_vector_get(_a, 1));
        }else{
            _robot->setVelArm(gsl_vector_get(_a, 1));
        }
        */
    }


    /*
    DBG("_a: " << gsl_vector_get(_a, 0) << " "
               << gsl_vector_get(_a, 1) << " "
               << gsl_vector_get(_a, 2) << " "
               << gsl_vector_get(_a, 3));
    */
}

void Robot::_colorize()
{
    _robot->setChasisColor2(osg::Vec4(0.7, 0.1, 0.1, 1.), 1);
    _robot->setChasisColor2(osg::Vec4(0.7, 0.7, 0.1, 1.), 3);
}

void Robot::_decolorize()
{
    _robot->setChasisColor(osg::Vec4(0., 0.1, 0.7, 0.6));
}
