#include "cu.h"

#include "component.hpp"
#include "message.hpp"
#include "time.hpp"


TEST_SUITES{
    TEST_SUITE_ADD(TSComponent),
    TEST_SUITE_ADD(TSMessage),
    TEST_SUITE_ADD(TSTime),

    TEST_SUITES_CLOSURE
};

int main(int argc, char *argv[])
{
    CU_SET_OUT_PREFIX("regressions/");
    CU_RUN(argc, argv);

    return 0;
}
