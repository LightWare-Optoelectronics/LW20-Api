output: main.o
	g++ main.o -o lw20api_example -lrt

main.o: main.cpp ../lw20api.h
	g++ -O3 -c main.cpp

clean: rm*.o

