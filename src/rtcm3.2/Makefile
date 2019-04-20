#!bin/bash
main:main.o serial_open.o librtcm.so
	gcc -g -o main main.o serial_open.o librtcm.so -lrtcm -L. -Wl,-rpath=. 
main.o:main.c
	gcc -c main.c
serial_open.o:serial_open.c serial_open.h
	gcc -c serial_open.c

.PHONY:clean
clean:
	rm *.o

