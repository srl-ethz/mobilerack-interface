test_valve: ./src/test_valve.cpp ./src/mpa/mpa.cpp ./src/MiniPID/MiniPID.cpp
	g++ $^ -o test_valve -std=c++11  -I/usr/include/python2.7 -lpython2.7 `pkg-config --libs --cflags libmodbus`

clean: soft_trunk
	rm $^

forceController: ./src/forceController.cpp ./src/mpa/mpa.cpp ./src/MiniPID/MiniPID.cpp
	g++  $^ -o forceController -std=c++11 `pkg-config --libs --cflags libmodbus` -pthread

example_forceController: ./src/example_forceController.cpp ./src/forceController.cpp ./src/mpa/mpa.cpp ./src/MiniPID/MiniPID.cpp
	g++  $^ -o example_forceController -std=c++11 -I/usr/include/python2.7 -lpython2.7 `pkg-config --libs --cflags libmodbus` -pthread
