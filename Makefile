all: map_parser.o log_parser.o main.cpp particle_filter.o
	g++ -std=c++0x main.cpp log_parser.o map_parser.o -o main

particle_filter.o: MonteCarloLocalization.cpp MonteCarloLocalization.h
	g++ -std=c++0x -c MonteCarloLocalization.cpp -o particle_filter.o

log_parser.o: loadLog.cpp loadLog.h
	g++ -std=c++0x -c loadLog.cpp -o log_parser.o

map_parser.o: bee-map.c bee-map.h
	g++ -std=c++0x -c bee-map.c -o map_parser.o

particle_filter_unit: MonteCarloLocalization.cpp MonteCarloLocalization.h
	g++ -std=c++0x -c MonteCarloLocalization.cpp -o particle_filter_unit

map_parser_unit: bee-map.c bee-map.h
	g++ -std=c++0x -g -D MAP_UNIT_TEST bee-map.c -o map_parser_unit

log_parser_unit: loadLog.cpp loadLog.h
	g++ -std=c++0x -g -D LOG_UNIT_TEST loadLog.cpp -o log_parser_unit

clean:
	rm -f main *.o *~
