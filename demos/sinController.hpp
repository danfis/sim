#include <sim/sim.hpp>
#include <sim/robot/sssa.hpp>

/** Example of a component. The component controls speed of main arm using a simple sinus, hence the velocity
  in simuation time 't' is v = A*sin(f*t + p),
  where f is frequency,
        t is actual time
        p is phase and
        A is amplitude.

  The control is applied in function cbPreStep, which is called in before each simulation step.
  */
class SinController : public sim::Component {

    public:
        SinController(sim::robot::SSSA *robot, const double frequency, const double amplitude, const double phase,const int name);
        ~SinController();

        void cbPreStep();
        void init(sim::Sim *sim);

    private:
        sim::robot::SSSA *_robot;
        const double _freq;
        const double _amplitude;
        const double _phase;
        sim::Sim *_sim;
        int _name;
};
