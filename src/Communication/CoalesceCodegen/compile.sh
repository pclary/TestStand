gcc -fPIC -std=gnu11 -c cassie_user_in_t.c -o cassie_user_in_t.o
gcc -fPIC -std=gnu11 -c cassie_out_t.c -o cassie_out_t.o
gcc -shared -o libCassieUDP.so cassie_user_in_t.o cassie_out_t.o

