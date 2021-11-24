# Makefile for Writing Make Files Example
# *****************************************************
# Variables to control Makefile operation

# Highly
# BASED ON :
# https://www.softwaretestinghelp.com/cpp-makefile-tutorial/
# https://www.tutorialspoint.com/makefile/makefile_dependencies.htm

CC = g++
CFLAGS = -Wall -g
INC = RPLidar_lib/include
SRC = RPLidar_lib/src
IN =  RPLidar_lib

# ****************************************************

main: rplidar_retrieve_data.o
	$(CC) $(CFLAGS) -o test rplidar_retrieve_data.o

rplidar_retrieve_data.o: rplidar_retrieve_data.cpp $(IN)/sl_lidar.h $(IN)/sl_lidar_driver.h 
	$(CC) $(CFLAGS) -c rplidar_retrieve_data.cpp $(IN)/sl_lidar.h $(IN)/sl_lidar_driver.h

#clean
#	rm *.o

