// file: send_message.cpp
//
// LCM example program.
//
// compile with:
//  $ g++ -o send_message send_message.cpp -llcm
//
// On a system with pkg-config, you can also use:
//  $ g++ -o send_message send_message.cpp `pkg-config --cflags --libs lcm`

#include <lcm/lcm-cpp.hpp>

#include "../sensorlcm/sensors_t.hpp"

int main(int argc, char **argv)
{
    lcm::LCM lcm;
    if (!lcm.good())
        return 1;

    sensorlcm::sensors_t my_data;
    my_data.id = 0;

    my_data.distance = 1;
    my_data.ranges[0] = 2.2;

    lcm.publish("EXAMPLE", &my_data);

    return 0;
}
