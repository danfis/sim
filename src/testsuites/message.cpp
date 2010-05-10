#include <iostream>

#include "cu.h"
#include "sim/message.hpp"

class Message1 : public sim::Message {
    SIM_MESSAGE_INIT(1001)
  public:
    Message1() : sim::Message() {}
};

class Message2 : public Message1 {
    SIM_MESSAGE_INIT2(1002, 1)
  public:
    Message2() : Message1() {}
};

class Message3 : public Message1 {
    SIM_MESSAGE_INIT(1003)
  public:
    Message3() : Message1() {}
};

class Message4 : public Message3 {
    SIM_MESSAGE_INIT2(1003, -1)
  public:
    Message4() : Message3() {}
};

class Message5 : public Message4 {
    SIM_MESSAGE_INIT2(-1, 2)
};

TEST(messageSetUp)
{
}

TEST(messageTearDown)
{
}

TEST(messageID)
{
    Message1 *m1 = new Message1();
    Message2 *m2 = new Message2();
    Message3 *m3 = new Message3();
    Message4 *m4 = new Message4();
    Message5 *m5 = new Message5();

    assertEquals(m1->type(), Message1::Type);
    assertEquals(m2->type(), Message2::Type);
    assertEquals(m3->type(), Message3::Type);
    assertEquals(m4->type(), Message4::Type);

    assertEquals(((Message1 *)m2)->type(), Message2::Type);
    assertEquals(((Message1 *)m3)->type(), Message3::Type);
    assertEquals(((Message1 *)m4)->type(), Message4::Type);

    assertEquals(m1->typeMajor(), 1001);
    assertEquals(m1->typeMinor(), 0);
    assertEquals(m2->typeMajor(), 1002);
    assertEquals(m2->typeMinor(), 1);
    assertEquals(m3->typeMajor(), 1003);
    assertEquals(m3->typeMinor(), 0);
    assertEquals(m4->typeMajor(), 1003);
    assertEquals(m4->typeMinor(), 0xffffUL);

    assertEquals(m5->type(), 0xffff0002UL);
    assertEquals(m5->typeMajor(), 0xffffUL);
    assertEquals(m5->typeMinor(), 2);

    {
        Message3 *m = m4->cast<Message3>(m4);
        assertEquals((long)m, (long)m4);
    }
    {
        Message3 *m = m2->cast<Message3>(m2);
        assertEquals((long)m, 0L);
    }

    delete m1;
    delete m2;
    delete m3;
    delete m4;
    delete m5;
}
