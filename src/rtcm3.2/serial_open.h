#ifndef SERIAL_OPEN_H_
#define SERIAL_OPEN_H_

#include<fcntl.h>
#include<unistd.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>

#define READ_NUM 256
#define ARR_MAX 20

#define FALSE -1
#define TRUE   0
#define MAX_TRY 100
#define pi 3.1415926


void set_speed(int, int);
int set_Parity(int, int, int, int);


void test_re(int para);
void is_open(int fd, const char *filename);  // open file
int is_null(char *str);  //string is null?
int dev_open(const char dev[]);

#endif
