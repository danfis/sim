#ifndef COMPONENT_HPP_
#define COMPONENT_HPP_

TEST(compSetUp);
TEST(compTearDown);
TEST(compPrePostStep);
TEST(compMsg);

TEST_SUITE(TSComponent) {
    TEST_ADD(compSetUp),

    TEST_ADD(compPrePostStep),
    TEST_ADD(compMsg),

    TEST_ADD(compTearDown),
    TEST_SUITE_CLOSURE
};

#endif
