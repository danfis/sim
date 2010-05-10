#ifndef MESSAGE_HPP
#define MESSAGE_HPP

TEST(messageSetUp);
TEST(messageTearDown);
TEST(messageID);

TEST_SUITE(TSMessage) {
    TEST_ADD(messageSetUp),

    TEST_ADD(messageID),

    TEST_ADD(messageTearDown),
    TEST_SUITE_CLOSURE
};

#endif
