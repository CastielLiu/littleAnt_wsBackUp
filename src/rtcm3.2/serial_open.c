#include"serial_open.h"


int speed_arr[] =
    {B230400, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300 };
int name_arr[] = {230400, 115200, 38400, 19200, 9600, 4800, 2400, 1200, 300 };


int dev_open(char dev[])
{
    int fd, flag;
    struct termios term;
    struct timeval timeout;
    speed_t baud_rate_i, baud_rate_o;

    fd = open(dev, O_RDWR | O_NONBLOCK);
    if (fd == -1)
	printf("can not open the %s !\n",dev);
    else
	printf("open %s ok!\n",dev);

    flag = tcgetattr(fd, &term);
    baud_rate_i = cfgetispeed(&term);
    baud_rate_o = cfgetospeed(&term);
    //printf("设置之前的输入波特率是%d，输出波特率是%d\n",baud_rate_i, baud_rate_o);

    set_speed(fd, 115200);

    flag = tcgetattr(fd, &term);
    baud_rate_i = cfgetispeed(&term);
    baud_rate_o = cfgetospeed(&term);
    //printf("设置之后的输入波特率是%d，输出波特率是%d\n",baud_rate_i, baud_rate_o);

    if (set_Parity(fd, 8, 1, 'N') == FALSE) {
	printf("Set Parity Error\n");
	exit(1);
	
    }
    return fd;
}

void set_speed(int fd, int speed)
{
    unsigned int i;
    int status;
    struct termios Opt;
    tcgetattr(fd, &Opt);
    for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
	if (speed == name_arr[i]) {
	    tcflush(fd, TCIOFLUSH);
	    cfsetispeed(&Opt, speed_arr[i]);
	    cfsetospeed(&Opt, speed_arr[i]);
	    status = tcsetattr(fd, TCSANOW, &Opt);
	    if (status != 0) {
		perror("tcsetattr fd1");
		return;
	    }
	    tcflush(fd, TCIOFLUSH);
	}
    }
}



/**
*@brief   设置串口数据位，停止位和效验位
*@param fd     类型 int 打开的串口文件句柄*
*@param databits 类型 int 数据位   取值 为 7 或者8*
*@param stopbits 类型 int 停止位   取值为 1 或者2*
*@param parity 类型 int 效验类型 取值为N,E,O,,S
*/
int set_Parity(int fd, int databits, int stopbits, int parity)
{
    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
	perror("SetupSerial 1");
	return (FALSE);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits) {		/*设置数据位数 */
    case 7:
	options.c_cflag |= CS7;
	break;
    case 8:
	options.c_cflag |= CS8;
	break;
    default:
	fprintf(stderr, "Unsupported data size\n");
	return (FALSE);
    }
    switch (parity) {
    case 'n':
    case 'N':
//        options.c_cflag &= ~PARENB;   /* Clear parity enable */
//        options.c_iflag &= ~INPCK;     /* Enable parity checking */
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	/*Input */
	options.c_oflag &= ~OPOST;	/*Output */
	break;
    case 'o':
    case 'O':
	options.c_cflag |= (PARODD | PARENB);	/* 设置为奇效验 */
	options.c_iflag |= INPCK;	/* Disnable parity checking */
	break;
    case 'e':
    case 'E':
	options.c_cflag |= PARENB;	/* Enable parity */
	options.c_cflag &= ~PARODD;	/* 转换为偶效验 */
	options.c_iflag |= INPCK;	/* Disnable parity checking */
	break;
    case 'S':
    case 's':			/*as no parity */
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	break;
    default:
	fprintf(stderr, "Unsupported parity\n");
	return (FALSE);
    }
/* 设置停止位*/
    switch (stopbits) {
    case 1:
	options.c_cflag &= ~CSTOPB;
	break;
    case 2:
	options.c_cflag |= CSTOPB;
	break;
    default:
	fprintf(stderr, "Unsupported stop bits\n");
	return (FALSE);
    }
/* Set input parity option */
    if ((parity != 'n') && (parity != 'N'))
	options.c_iflag |= INPCK;

    options.c_cc[VTIME] = 5;	// 0.5 seconds
    options.c_cc[VMIN] = 1;

    options.c_cflag &= ~HUPCL;
    options.c_iflag &= ~INPCK;
    options.c_iflag |= IGNBRK;
    options.c_iflag &= ~ICRNL;
    options.c_iflag &= ~IXON;
    options.c_lflag &= ~IEXTEN;
    options.c_lflag &= ~ECHOK;
    options.c_lflag &= ~ECHOCTL;
    options.c_lflag &= ~ECHOKE;
    options.c_oflag &= ~ONLCR;

    tcflush(fd, TCIFLUSH);	/* Update the options and do it NOW */
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
	perror("SetupSerial 3");
	return (FALSE);
    }

    return (TRUE);
}


//
void test_re(int para)
{
}


// open file
void is_open(int fd, char *filename)
{
	if(fd == -1)
	{
		printf("Not open %s\n",filename);
		exit(1);
	}
	else
		printf("open %s sccussful!\n",filename);
}


//string is null?
int is_null(char *str)
{
	int len = strlen(str);
	if (len == 0)
		return 0;
	else
		return 1;
}

