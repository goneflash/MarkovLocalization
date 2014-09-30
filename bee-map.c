#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef MAP_UNIT_TEST
	#include <iostream>
	#include <opencv2/core/core.hpp>
	#include <opencv2/highgui/highgui.hpp>
	using namespace std;
	using namespace cv;
#endif

#include "bee-map.h"

// #define DEBUG

void new_hornetsoft_map(map_type *map, int size_x, int size_y)
{
	int i;
	
	map->cells = (float **)calloc(size_x, sizeof(float *));
	for(i = 0; i < size_x; i++)
		map->cells[i] = (float *)calloc(size_y, sizeof(float));
}

int read_beesoft_map(const char *mapName, map_type *map)
{
	int x, y, count;
	float temp;
	char line[256];
	FILE *fp;
	
	if((fp = fopen(mapName, "rt")) == NULL) {
		fprintf(stderr, "# Could not open file %s\n", mapName);
		return -1;
	}
	fprintf(stderr, "# Reading map: %s\n", mapName);
	while((fgets(line, 256, fp) != NULL)
		  && (strncmp("global_map[0]", line , 13) != 0)) {
		if(strncmp(line, "robot_specifications->resolution", 32) == 0)
			if(sscanf(&line[32], "%d", &(map->resolution)) != 0)
#ifdef DEBUG
				printf("# Map resolution: %d cm\n", map->resolution);
#endif		
		if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
			if(sscanf(&line[35], "%g", &(map->offset_x)) != 0) {
				map->offset_x = map->offset_x;
#ifdef DEBUG
				printf("# Map offsetX: %g cm\n", map->offset_x);
#endif			
			}
		if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0) {
			if (sscanf(&line[35], "%g", &(map->offset_y)) != 0) {
				map->offset_y = map->offset_y;
#ifdef DEBUG
				printf("# Map offsetY: %g cm\n", map->offset_y);
#endif			
			}
		}
	}
	
	if(sscanf(line,"global_map[0]: %d %d", &map->size_y, &map->size_x) != 2) {
		fprintf(stderr, "ERROR: corrupted file %s\n", mapName);
		fclose(fp);
		return -1;
	}
#ifdef DEBUG
	printf("# Map size: %d %d\n", map->size_x, map->size_y);
#endif	
	new_hornetsoft_map(map, map->size_x, map->size_y);
	
	map->min_x = map->size_x;
	map->max_x = 0;
	map->min_y = map->size_y;
	map->max_y = 0;
	count = 0;
	for(x = 0; x < map->size_x; x++)
		for(y = 0; y < map->size_y; y++, count++) {
//			if(count % 10000 == 0)
//				fprintf(stderr, "\r# Reading ... (%.2f%%)",
//						count / (float)(map->size_x * map->size_y) * 100);
			
			fscanf(fp,"%e", &temp);
			if(temp < 0.0)
				map->cells[x][y] = -1;
			else {
				if(x < map->min_x)
					map->min_x = x;
				else if(x > map->max_x)
					map->max_x = x;
				if(y < map->min_y)
					map->min_y = y;
				else if(y > map->max_y)
					map->max_y = y;
				map->cells[x][y] = 1 - temp;	   
			}
#ifdef DEBUG
			printf("%e ",map->cells[x][y]);
#endif		
		}
#ifdef DEBUG
		printf("\n");
#endif
//	fprintf(stderr, "\r# Reading ... (%.2f%%)\n\n",
//			count / (float)(map->size_x * map->size_y) * 100);
	fclose(fp);
	return 0;
}

// Unit test
#ifdef MAP_UNIT_TEST

int main(int argc, char **argv)
{
	map_type map;
	
	string map_name = "../data/map/wean.dat";
	read_beesoft_map(map_name.c_str(), &map);

	// Check Map information
	cout << "Map information: " << endl;
	cout << "Resolution " << map.resolution << " SizeX " << map.size_x << " SizeY " << map.size_y << endl;
	cout << "Min_Max X " << map.min_x << " " << map.max_x << endl;
	cout << "Min_Max Y " << map.min_y << " " << map.max_y << endl;
	cout << "Offset X " << map.offset_x << " Offset Y " << map.offset_y << endl;

	// visualize
	Mat image = Mat::zeros( map.size_x, map.size_y, CV_32FC1 );

	cout << "Image Property" << endl;
	cout << "Row: " << image.rows << " Col: " << image.cols << endl;
	cout << "Step: " << image.step << " Dim: " << image.dims << endl;
	cout << "ElemSize: " << image.elemSize() << " Depth: " << image.depth() << endl;
	cout << "Channels: " << image.channels() << endl;

	// unsigned char *imgMat = (unsigned char*)(image.data);
	for (unsigned int i = 0; i < image.rows; i++)
		for (unsigned int j = 0; j < image.cols; j++){
			if (map.cells[i][j] > 0.0)
				image.at<float>(i, j) = map.cells[i][j]; 
		} 
  	// Draw a circle 
  	// circle( image, Point( 200, 200 ), 32.0, Scalar( 0, 0, 255 ), 1, 8 );
  	imshow("Image",image);
	waitKey( 0 );
	return 0;
}

#endif