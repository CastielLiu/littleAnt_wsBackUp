#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>


 
int main(int argc,char** argv)
{
    int keys_fd ,mouse_fd;
    char ret[2];
    struct input_event keyEvent,mouseEvent;
    keys_fd=open("/dev/input/event1",O_RDONLY|O_NONBLOCK);//只读&非阻塞MODE
    mouse_fd=open("/dev/input/event2",O_RDONLY|O_NONBLOCK);//只读&非阻塞 MODE
    if(keys_fd<=0 ||mouse_fd<=0)
    {   
        printf("error\n");
        return -1; 
    }   
    while(1)
    {   
        read(keys_fd,&keyEvent,sizeof(struct input_event));
        read(mouse_fd,&mouseEvent,sizeof(struct input_event));
        
        if(keyEvent.type!=0)
            printf("type %i  code  %i  value %i \n",keyEvent.type ,keyEvent.code,keyEvent.value);
    	if(mouseEvent.type!=0)
    		printf("type = %i  mouse %i state %i \n",mouseEvent.type,mouseEvent.code,mouseEvent.value);
         usleep(3000);
        // std::cout << EV_KEY <<std:: endl;
    }   
    close(keys_fd);
    return 0;
}

