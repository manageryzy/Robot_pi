#
# Makefile:
#	wiringPi - Wiring Compatable library for the Raspberry Pi
#	https://projects.drogon.net/wiring-pi
#
#	Copyright (c) 2012 Gordon Henderson
#################################################################################
# This file is part of wiringPi:
#	Wiring Compatable library for the Raspberry Pi
#
#    wiringPi is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    wiringPi is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
#################################################################################


#DEBUG	= -g -O0
DEBUG	= -O3
CC	= g++
INCLUDE	= -I/usr/local/include
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe 

LDFLAGS	= -L/usr/local/lib
LDLIBS    = -lwiringPi -lwiringPiDev -lpthread -lm 

# Should not alter anything below this line
###############################################################################

SRC	=	

OBJ	=	$(SRC:.c=.o)

BINS	=	$(SRC:.c=)

all:	
	@cat README.TXT
	@echo ""


really-all:	$(BINS)

serialOut.o: serialOut.c serialOut.h header.h
	@echo now complecting serialOut
	@$(CC)  -c $(CFLAGS) serialOut.c -o  serialOut.o -lwiringPi 
	
confReader.o: confReader.cpp confReader.h header.h
	@echo now complecting confReader
	@$(CC) -c  confReader.cpp -o confReader.o
	
robot.o:robot.cpp robot.h header.h
	@echo now complecting robot
	@$(CC) -c -ggdb `pkg-config --cflags opencv` robot.cpp -o robot.o 
	
opencv.o:opencv.cpp 
	@echo now complecting opencv
	@$(CC) -c  -ggdb `pkg-config --cflags opencv` opencv.cpp -o opencv.o


robot:	serialOut.o confReader.o robot.o opencv.o
	@echo [link]
	@$(CC) -o $@ serialOut.o confReader.o opencv.o robot.o $(LDFLAGS) $(LDLIBS)  `pkg-config --libs opencv`
	@cp robot ../robot

.c.o:
	@echo [CC] $<
	@$(CC) -c $(CFLAGS) $<make -o $@

clean:
	@echo "[Clean]"
	@rm -f $(OBJ) *~ core tags $(BINS)

tags:	$(SRC)
	@echo [ctags]
	@ctags $(SRC)


# DO NOT DELETE
