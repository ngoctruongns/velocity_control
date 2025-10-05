
#include "VelocityMsgPublisher.h"
#include "VelocityMsgSubscriber.h"
// #include <iostream>

int main(int argc, char **argv)
{

    VelocityMsgSubscriber mysub;
    if (mysub.init()) {
        mysub.run();
    }

    return 0;
}
