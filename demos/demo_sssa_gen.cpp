/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <sys/time.h>

#include <sim/sim.hpp>
#include <sim/world.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/comp/sssa.hpp>
#include <sim/rand.hpp>
#include <sim/msg.hpp>

using sim::Scalar;
using sim::Vec3;
using sim::Quat;
using sim::Time;
using namespace std;

struct State {
    Scalar vel_left;
    Scalar vel_right;
    Scalar vel_arm;
};

struct Solution {
    std::vector<std::vector<State> > states;
    double fitness;
};


const char *fn_prefix = "sssa_gen/";
const size_t fn_len = 100;

size_t STEPS = 300;
Scalar maxvel = 5.;
Scalar minvel = -5.;
Scalar maxstep = 0.5;
size_t population_size = 300;
size_t maxgen = 400;
size_t gen = 0;
std::vector<Solution *> population(population_size);
size_t mutation_num = 20;

bool use_ode = false;
size_t step = 0;

class SSSA;

/**
 * Initializes states by random values.
 */
static void randStates(std::vector<State> *states);

static void crossover(Solution *a, Solution *b, Solution *out);
static void mutation(Solution *sol, size_t num = 1);
static void readStates(std::vector<State> *states, const char *fn);
static void makeFnState(int gen, int indiv, int id, char *fn);
static void makeFnFitness(int gen, int indiv, char *fn);
static double fitness(SSSA *r);

static int mainMain(int argc, char *argv[]);

class SSSA : public sim::comp::SSSA {
    std::vector<State> _states;
    int _id;

  public:
    SSSA(int id, int gen, int pop, const Vec3 &pos, const Quat &rot = Quat(0., 0., 0., 1.))
        : sim::comp::SSSA(pos, rot), _states(STEPS), _id(id)
    {
        char *fn = new char[fn_len];
        makeFnState(gen, pop, id, fn);
        readStates(&_states, fn);
        delete fn;
    }

    std::vector<State> &states() { return _states; }

    void init(sim::Sim *sim)
    {
        sim::comp::SSSA::init(sim);
        sim->regPreStep(this);
    }

    void cbPreStep()
    {
        State state = _states[step];
        //DBG(step << " " << state.vel_left << " " << state.vel_right << " " << state.vel_arm);
        robot()->addVelLeft(state.vel_left);
        robot()->addVelRight(state.vel_right);
        robot()->addVelArm(state.vel_arm);

        if (robot()->velLeft() > maxvel)
            robot()->setVelLeft(maxvel);
        if (robot()->velLeft() < minvel)
            robot()->setVelLeft(minvel);

        if (robot()->velRight() > maxvel)
            robot()->setVelRight(maxvel);
        if (robot()->velRight() < minvel)
            robot()->setVelRight(minvel);

        if (robot()->velArm() > maxvel)
            robot()->setVelArm(maxvel);
        if (robot()->velArm() < minvel)
            robot()->setVelArm(minvel);
    }
};

class RobotsManager : public sim::Component {
    sim::Sim *_sim;
    std::vector<SSSA *> sssas;

  public:
    RobotsManager()
    {
    }

    void init(sim::Sim *sim)
    {
        _sim = sim;

        std::list<sim::Component *> comps;
        sim->components(&comps);

        std::list<sim::Component *>::iterator it, it_end;
        SSSA *sssa;

        it = comps.begin();
        it_end = comps.end();
        for (; it != it_end; ++it){
            sssa = dynamic_cast<SSSA *>(*it);

            if (sssa && sssa->robot()){
                sssas.push_back(sssa);
            }
        }

        for (size_t i = 0; i < sssas.size(); i++){
            for (size_t j = i + 1; j < sssas.size(); j++){
                if (!sssas[i]->robot()->connectTo(*sssas[j]->robot())){
                    sssas[j]->robot()->connectTo(*sssas[i]->robot());
                }
            }
        }

        sim->regPostStep(this);
    }

    void cbPostStep()
    {
        //DBG("Step " << step);

        if (step >= STEPS){
            _sim->terminateSimulation();
        }

        step++;
    }
};

class S : public sim::Sim {
    int _gen, _pop;
    SSSA *_center_robot;

  public:
    S(int gen, int pop)
        : Sim(), _gen(gen), _pop(pop)
    {
        if (use_ode){
            initODE();
        }else{
            initBullet();
        }

        setTimeStep(Time::fromMs(10));
        setTimeSubSteps(2);

        setSimulateReal(false);

        createArena();
        createRobot(gen, pop);
    }

    void finish()
    {
        char *fn = new char[fn_len];
        double f;

        f = fitness(_center_robot);
        makeFnFitness(_gen, _pop, fn);

        std::ofstream fout(fn);
        fout << f << std::endl;
        fout.close();

        delete fn;
    }

    void initBullet()
    {
#ifdef SIM_HAVE_BULLET
        DBG("Using Bullet");

        sim::bullet::World *w = new sim::bullet::World();
        setWorld(w);
#endif /* SIM_HAVE_BULLET */
    }

    void initODE()
    {
#ifdef SIM_HAVE_ODE
        DBG("Using ODE");

        sim::ode::World *w = new sim::ode::World();

        setWorld(w);

        w->setCFM(0.0001);
        w->setERP(0.8);
        w->setStepType(sim::ode::World::STEP_TYPE_QUICK);
        w->setAutoDisable(0.01, 0.01, 5, 0.);

        w->setContactApprox1(true);
        w->setContactApprox2(true);
        w->setContactBounce(0.1, 0.1);
#endif /* SIM_HAVE_ODE */
    }

    void createArena()
    {
        osg::Vec4 color(0., 0.7, 0.1, 1.);
        sim::Body *c;
        int id;
        sim::World *w = world();

        c = w->createBodyCompound();
        id = c->addBox(Vec3(20., 20., 0.1));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, 20., 4.), SIM_BODY_DEFAULT_VIS, Vec3(-10, 0., 2.));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(0.1, 20., 4.), SIM_BODY_DEFAULT_VIS, Vec3(10, 0., 2.));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(20., .1, 4.), SIM_BODY_DEFAULT_VIS, Vec3(0., 10, 2.));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");
        id = c->addBox(Vec3(20., .1, 4.), SIM_BODY_DEFAULT_VIS, Vec3(0., -10, 2.));
        c->visBody(id)->setColor(color);
        c->visBody(id)->setTexture("wood.ppm");

        id = c->addCube(.2, SIM_BODY_DEFAULT_VIS, Vec3(0.5, 0.5, .1));
        c->visBody(id)->setColor(0., .1, .8, 1.);
        c->visBody(id)->setTexture("wood.ppm");
        c->activate();
    }

    void createRobot(int gen, int pop)
    {
        SSSA *rob;

        rob = new SSSA(0, gen, pop, Vec3(2., 2., .6));
        _center_robot = rob;
        addComponent(rob);

        rob = new SSSA(1, gen, pop, Vec3(.746, 2., .6));
        addComponent(rob);

        rob = new SSSA(2, gen, pop, Vec3(-.508, 2., .6));
        addComponent(rob);

        rob = new SSSA(3, gen, pop, Vec3(3.254, 2., .6));
        addComponent(rob);

        rob = new SSSA(4, gen, pop, Vec3(4.508, 2., .6));
        addComponent(rob);

        rob = new SSSA(5, gen, pop, Vec3(2., 3.254, .6), Quat(Vec3(0., 0., 1.), M_PI / 2.));
        addComponent(rob);

        rob = new SSSA(6, gen, pop, Vec3(2., 4.508, .6), Quat(Vec3(0., 0., 1.), M_PI / 2.));
        addComponent(rob);

        rob = new SSSA(7, gen, pop, Vec3(2., 0.746, .6), Quat(Vec3(0., 0., 1.), -M_PI / 2.));
        addComponent(rob);

        rob = new SSSA(8, gen, pop, Vec3(2., -.508, .6), Quat(Vec3(0., 0., 1.), -M_PI / 2.));
        addComponent(rob);

        // and as last add component which will connect all robots
        addComponent(new RobotsManager());
    }

};


static void randStates(std::vector<State> *states)
{
    static sim::Rand r;

    // resize vector if needed
    if (states->size() != STEPS)
        states->resize(STEPS);

    for (size_t i = 0; i < STEPS; i++){
        (*states)[i].vel_left = r.randF(-maxstep / 2., maxstep / 2.);
        (*states)[i].vel_right = r.randF(-maxstep / 2., maxstep / 2.);
        (*states)[i].vel_arm = r.randF(-maxstep / 2., maxstep / 2.);
    }
}

static void crossover(Solution *a, Solution *b, Solution *out)
{
    sim::Rand r;
    int cut, swap;
    std::vector<State> *st[2];

    for (size_t i = 0; i < 9; i++){
        swap = r.rand();
        if (swap % 2 == 0){
            st[0] = &a->states[i];
            st[1] = &b->states[i];
        }else{
            st[0] = &b->states[i];
            st[1] = &a->states[i];
        }
        //DBG("0: " << st[0] << ", 1: " << st[1]);

        cut = r.rand(0, STEPS);
        //DBG("cut: " << cut);

        if (out->states[i].size() != STEPS)
            out->states[i].resize(STEPS);

        int oi = 0;
        for (int j = 0; j < cut; j++, oi++){
            out->states[i][oi] = (*(st[0]))[j];
        }
        for (int j = cut; j < (int)STEPS; j++, oi++){
            out->states[i][oi] = (*(st[1]))[j];
        }
    }
}

static void mutation(Solution *sol, size_t num)
{
    sim::Rand r;
    int pos;

    for (size_t i = 0; i < 9; i++){
        for (size_t j = 0; j < num; j++){
            pos = r.rand(0, STEPS);
            sol->states[i][pos].vel_left  = r.randF(-maxstep / 2., maxstep / 2.);
            sol->states[i][pos].vel_right = r.randF(-maxstep / 2., maxstep / 2.);
            sol->states[i][pos].vel_arm   = r.randF(-maxstep / 2., maxstep / 2.);
        }
    }
}

static void readStates(std::vector<State> *states, const char *fn)
{
    std::ifstream fin(fn);
    Scalar l, r, a;

    if (states->size() != STEPS)
        states->resize(STEPS);

    for (size_t i = 0; i < STEPS && fin.good(); i++){
        fin >> l >> r >> a;
        (*states)[i].vel_left  = l;
        (*states)[i].vel_right = r;
        (*states)[i].vel_arm   = a;
    }

    fin.close();
}

static void writeStates(std::vector<State> &states, const char *fn)
{
    std::ofstream fout(fn);

    for (size_t i = 0; i < STEPS && i < states.size(); i++){
        fout << states[i].vel_left << " ";
        fout << states[i].vel_right << " ";
        fout << states[i].vel_arm << std::endl;
    }

    fout.close();
}

static void makeFnState(int gen, int indiv, int id, char *fn)
{
    sprintf(fn, "%s%06d-%03d-%03d.state", fn_prefix, gen, indiv, id);
}

static void makeFnFitness(int gen, int indiv, char *fn)
{
    sprintf(fn, "%s%06d-%03d.fitness", fn_prefix, gen, indiv);
}

static void makeCommand(const char *bin, int gen, int pop, char *comm)
{
    sprintf(comm, "%s -- %d %d 1>/dev/null 2>&1", bin, gen, pop);
    //sprintf(comm, "%s -- %d %d", bin, gen, pop);
}

static double fitness(SSSA *r)
{
    Vec3 p = r->robot()->pos();
    Vec3 goal = Vec3(0., 0., 0.);

    p[2] = 0;

    return (p - goal).length();
}

static bool popCmp(Solution *s1, Solution *s2)
{
    if (s1->fitness < s2->fitness)
        return true;
    return false;
}

static int mainMain(int argc, char *argv[])
{
    char *fn = new char[fn_len];
    char *comm = new char[fn_len];
    double f;

    for (; gen < maxgen; gen++){
        fprintf(stderr, "Generation: %06d.\n", gen);

        if (gen == 0){
            // generate random states
            for (size_t p = 0; p < population_size; p++){
                Solution *sol = new Solution;
                sol->states.resize(9);
                sol->fitness = 999999;

                for (size_t r = 0; r < 9; r++){
                    fprintf(stderr, "  -- gen states -- Individual: %03d, Robot: %03d, random...\r", p, r);
                    randStates(&sol->states[r]);
                }

                population[p] = sol;
            }
            fprintf(stderr, "\n");
        }else{
            sim::Rand r;

            // sort population
            std::sort(population.begin(), population.end(), popCmp);

            // crossover upper half into lower half and mutate lower half
            size_t newi = population_size / 4;
            size_t a, b;
            while (newi < population_size){
                fprintf(stderr, "  -- gen states -- Individual: %03d, ", newi);
                a = r.rand(0, population_size / 4);
                do {
                    b = r.rand(0, population_size / 4);
                } while (a == b);

                population[newi]->fitness = 999999;

                fprintf(stderr, "crossover (%03d, %03d), ", a, b);
                crossover(population[a], population[b], population[newi]);

                fprintf(stderr, "mutation(%d)\r", mutation_num);
                mutation(population[newi], mutation_num);

                newi++;
            }
            fprintf(stderr, "\n");
        }

        // print states
        for (size_t p = 0; p < population_size; p++){
            Solution *sol = population[p];

            for (size_t r = 0; r < 9; r++){
                fprintf(stderr, "  -- print states -- Individual: %03d, Robot: %03d\r", p, r);
                makeFnState(gen, p, r, fn);
                writeStates(sol->states[r], fn);
            }
        }
        fprintf(stderr, "\n");

        // test states
        for (size_t p = 0; p < population_size; p++){
            fprintf(stderr, "  -- testing -- Individual: %03d", p);
            makeCommand(argv[0], gen, p, comm);
            system(comm);

            // pick up fitness
            makeFnFitness(gen, p, fn);

            std::ifstream fin(fn);
            if (fin.good()){
                fin >> f;
                population[p]->fitness = f;
                fprintf(stderr, ", fitness: %f", f);
            }
            fin.close();

            fprintf(stderr, "\r");
        }
        fprintf(stderr, "\n");
    }

    delete fn;
    delete comm;

    for (size_t i = 0; i < population_size; i++){
        delete population[i];
    }
    population.clear();

    return 0;
}

static int mainTest(int argc, char *argv[])
{
    for (int i = 3; i < argc; i++){
        if (strcmp(argv[i], "--ode") == 0){
            use_ode = true;
        }else if (strcmp(argv[i], "--bullet") == 0){
            use_ode = false;
        }
    }

    int gen = atoi(argv[2]);
    int pop = atoi(argv[3]);

    S s(gen, pop);
    s.run();

    return 0;
}

int main(int argc, char *argv[])
{
    if (argc > 1 && strcmp(argv[1], "--") == 0){
        return mainTest(argc, argv);
    }else{
        return mainMain(argc, argv);
    }
}

