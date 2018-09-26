soft-trunk: ./src/soft_trunk.cpp ./src/mpa/mpa.cpp ./src/MiniPID/MiniPID.cpp
	g++ $^ -o soft_trunk -std=c++11  -I/usr/include/python2.7 -lpython2.7 `pkg-config --libs --cflags libmodbus`

clean: soft_trunk
	rm $^
