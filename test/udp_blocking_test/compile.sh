g++ -std=c++11 udp_comms.cpp main_tx.cpp -I../../src/Communication/CoalesceCodegen -Wl,-rpath,../../src/Communication/CoalesceCodegen -o udp_tx -L../../src/Communication/CoalesceCodegen -lCassieUDP
g++ -std=c++11 udp_comms.cpp main_rx.cpp -I../../src/Communication/CoalesceCodegen -Wl,-rpath,../../src/Communication/CoalesceCodegen -o udp_rx -L../../src/Communication/CoalesceCodegen -lCassieUDP
