# This Makefile was tested with GNU Make
CXX=g++

# Use pkg-config to lookup the proper compiler and linker flags for LCM
CFLAGS=`pkg-config --cflags lcm`
LDFLAGS=`pkg-config --libs lcm`

msg_types=../lcm/*.lcm

all: ../sensorlcm/sensors_t.hpp \
	send-message \

send-message: send_message.o
	$(CXX) -o $@ $^ $(LDFLAGS)

%.o: %.cpp ../sensorlcm/sensors_t.hpp
	$(CXX) $(CFLAGS) -I. -o $@ -c $< 

exlcm/%.hpp:
	lcm-gen -x ../sensors_t.lcm

clean:
	rm -f listener send-message read-log
	rm -f *.o 
	rm -f ../sensorlcm/*.hpp
