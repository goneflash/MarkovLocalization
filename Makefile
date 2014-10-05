all: map_parser.o log_parser.o main.cpp particle_filter.o
	g++ -std=c++0x -g -D VIZ main.cpp log_parser.o map_parser.o particle_filter.o -o main `pkg-config --cflags --libs opencv`

particle_filter.o: MonteCarloLocalization.cpp MonteCarloLocalization.h
	g++ -std=c++0x -c -D VIZ MonteCarloLocalization.cpp -o particle_filter.o

log_parser.o: loadLog.cpp loadLog.h
	g++ -std=c++0x -c loadLog.cpp -o log_parser.o

map_parser.o: bee-map.c bee-map.h
	g++ -std=c++0x -c bee-map.c -o map_parser.o

particle_filter_unit: MonteCarloLocalization.cpp bee-map.c MonteCarloLocalization.h bee-map.h 
	g++ -std=c++0x -g -D PF_UNIT_TEST -D VIZ MonteCarloLocalization.cpp bee-map.c -o particle_filter_unit `pkg-config --cflags --libs opencv`

map_parser_unit: bee-map.c bee-map.h
	g++ -std=c++0x -g -D MAP_UNIT_TEST bee-map.c -o map_parser_unit `pkg-config --cflags --libs opencv`

log_parser_unit: loadLog.cpp loadLog.h
	g++ -std=c++0x -g -D LOG_UNIT_TEST loadLog.cpp -o log_parser_unit

clean:
	rm -f main *.o *~
