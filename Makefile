# Makefile for Writing Make Files Example
# *****************************************************
# Variables to control Makefile operation

# Highly
# BASED ON :
# https://www.softwaretestinghelp.com/cpp-makefile-tutorial/
# https://www.tutorialspoint.com/makefile/makefile_dependencies.htm

CC = g++
CFLAGS = -Wall -g
DIR = RPLIDAR
INC = RPLIDAR/RPLidar_lib/include
SRC = RPLIDAR/RPLidar_lib/src
IN =  RPLIDAR/RPLidar_lib

# ****************************************************

main: main.o rplidar_retrieve_data.o
	$(CC) $(CFLAGS) -o main main.o rplidar_retrieve_data.o

main.o: main.cpp $(DIR)/rplidar_retrieve_data.cpp $(INC)/sl_lidar.h $(INC)/sl_lidar_driver.h
	$(CC) $(CFLAGS) -c main.cpp

rplidar_retrieve_data.o: $(DIR)/rplidar_retrieve_data.cpp $(IN)/sl_lidar.h $(IN)/sl_lidar_driver.h 
	$(CC) $(CFLAGS) -c $(DIR)/rplidar_retrieve_data.cpp $(IN)/sl_lidar.h $(IN)/sl_lidar_driver.h
#clean
#	rm *.o

