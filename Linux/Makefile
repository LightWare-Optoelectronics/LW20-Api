build_folder := $(shell mkdir -p bin)

output: ./bin/main.o
	g++ ./bin/main.o -o ./bin/lw20api_example -lrt

./bin/main.o: main.cpp ../lw20api.h
	g++ -O3 -c main.cpp -o ./bin/main.o

clean: rm ./bin/*.o

