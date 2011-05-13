#include <sim/sim.hpp>
#include <sim/robot/sssa.hpp>

class SinController : public sim::Component {

    public:
        SinController(sim::robot::SSSA *robot, const double frequency, const double amplitude, const double phase);
        ~SinController();

        void cbPreStep();
        void init(sim::Sim *sim);

    private:
        sim::robot::SSSA *_robot;
        const double _freq;
        const double _amplitude;
        const double _phase;
};
