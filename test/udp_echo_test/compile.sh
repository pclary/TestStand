g++ -std=c++11 udp_comms.cpp main.cpp -I../../src/Communication/CoalesceCodegen -Wl,-rpath,../../src/Communication/CoalesceCodegen -o udp_test -L../../src/Communication/CoalesceCodegen -lCassieUDP
