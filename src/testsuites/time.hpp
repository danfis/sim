#ifndef TIME_HPP
#define TIME_HPP

TEST(timeSetUp);
TEST(timeTearDown);

TEST(timeCmp);
TEST(timeDiff);
TEST(timeClock);
TEST(timeFrom);

TEST_SUITE(TSTime) {
    TEST_ADD(timeSetUp),

    TEST_ADD(timeCmp),
    TEST_ADD(timeDiff),
    TEST_ADD(timeClock),
    TEST_ADD(timeFrom),

    TEST_ADD(timeTearDown),
    TEST_SUITE_CLOSURE
};

#endif
