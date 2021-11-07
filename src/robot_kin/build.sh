g++ -c -fPIC invkin.cpp -o invkin.o
g++ -shared -Wl,-soname,libinvkin.so -o libinvkin.so invkin.o