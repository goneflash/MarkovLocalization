#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
	int resolution, size_x, size_y;
	float offset_x, offset_y;
	int min_x, max_x, min_y, max_y;
	float **cells;
} map_type;

void new_hornetsoft_map(map_type *map, int size_x, int size_y);
int read_beesoft_map(const char *mapName, map_type *map);