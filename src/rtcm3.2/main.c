#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>

#include "qxwz_rtcm.h"
#include "serial_open.h"
//#include "qxlog.h"

#undef QXLOGI
#define QXLOGI printf

//#define _QXWZ_TEST_START_STOP
int fd_rtcm;

qxwz_account_info *p_account_info = NULL;
void  get_qxwz_sdk_account_info(void);

//unsigned char buf_[] = "wendao\r\n";

void qxwz_rtcm_response_callback(qxwz_rtcm data){
    //printf("QXWZ_RTCM_DATA:%s\n",data.buffer);
    QXLOGI("QXWZ_RTCM_DATA:%s\n",data.buffer);
    QXLOGI("QXWZ_RTCM_DATA:%ld\n",data.length);
   write(fd_rtcm, data.buffer, data.length);
//write(fd_rtcm, buf_,8);
}


void qxwz_status_response_callback(qxwz_rtcm_status code){
    //printf("QXWZ_RTCM_STATUS:%d\n",code);
    QXLOGI("QXWZ_RTCM_STATUS:%d\n",code);
	struct tm *ptr = NULL;
	//test account expire
	if(code == QXWZ_STATUS_OPENAPI_ACCOUNT_TOEXPIRE){
		get_qxwz_sdk_account_info();
	}
}

void  get_qxwz_sdk_account_info(void)
{
	p_account_info = getqxwzAccount();
	if(p_account_info->appkey != NULL) {
		printf("appkey=%s\n",p_account_info->appkey);
	}
	if(p_account_info->deviceID != NULL) {
		printf("deviceID=%s\n",p_account_info->deviceID);
	}
	if(p_account_info->deviceType != NULL) {
		printf("deviceType=%s\n",p_account_info->deviceType);
	}

	if(p_account_info->NtripUserName != NULL) {
		printf("NtripUserName=%s\n",p_account_info->NtripUserName);
	}
	if(p_account_info->NtripPassword != NULL) {
		printf("NtripPassword=%s\n",p_account_info->NtripPassword);
	}
	printf("expire_time=%d\n",p_account_info->expire_time);
}



//void getAccountExpireDate(void);


#ifdef _QXWZ_TEST_START_STOP
pthread_t qxwz_rtcm_test;
void test_qxwz_rtcm_start_stop(void);
#endif

unsigned char gpggaMsg[200];

int main(int argc, const char * argv[]) {

	fd_rtcm = dev_open("/dev/ttyS4");
    //设置appKey和appSecret
    //apapKey申请详细见说明文档
    qxwz_config config;
    //RTD
    config.appkey="549565";
    config.appSecret="a9fa0d817d5ad7f7641d73d0cdc510288d97fb05003553640f07565d04ee9c5e";
    config.deviceId="78:0c:b8:d7:2f:17";
    config.deviceType="test";

    qxwz_setting(&config);
    //启动rtcm sdk
    qxwz_rtcm_start(qxwz_rtcm_response_callback,qxwz_status_response_callback);

	#ifdef _QXWZ_TEST_START_STOP
    pthread_create(&qxwz_rtcm_test,NULL,test_qxwz_rtcm_start_stop,NULL);
	#endif
    //demo测试10秒发送gga
    //每秒发送gga以获取最新的rtcm数据流
    int i;
    while(1){
       // qxwz_rtcm_sendGGAWithGGAString("$GPGGA,104853.4,3153.29183,N,11848.61485,E,1,03,20.0,6.8,M,1.1,M,,*53\r\n");
		//qxwz_rtcm_sendGGAWithGGAString("$GPGGA,102957.00,3148.1359,N,12000.7208,E,1,23,0.7,8.35,M,5.30,M,,*54\r\n");
        //printf("Send GGA done\r\n");
        QXLOGI("Send GGA done\r\n");
		//getAccountExpireDate();
		get_qxwz_sdk_account_info();
		
		int len = read(fd_rtcm,gpggaMsg,199);
		
		printf("..........len:%d\r\n",len);  /71?
		
		gpggaMsg[len]='\r';
		gpggaMsg[len+1] = '\n';
		gpggaMsg[len+2] = '\0';
		
		qxwz_rtcm_sendGGAWithGGAString(gpggaMsg);
		printf("%s",gpggaMsg);
		
        sleep(1);
    }
    QXLOGI("qxwz_rtcm_stop here\r\n");
//    //关闭rtcm sdk
    qxwz_rtcm_stop();
    QXLOGI("qxwz_rtcm_stop done\r\n");
    return 0;
}
#if 0
void getAccountExpireDate(void)
{
	struct tm *ptr = NULL;
	expire_time = qxwz_get_account_expire_time();
	//printf("expire_time=%d,date=",expire_time.expire_time);
	QXLOGI("expire_time=%d,date=",expire_time.expire_time);
	ptr = &expire_time.expire_date;

	QXLOGI("year:%d,month:%d,mday:%d,hour:%d,minute:%d,second:%d\n", \
		ptr->tm_year+1900,ptr->tm_mon+1,ptr->tm_mday,ptr->tm_hour,ptr->tm_min,ptr->tm_sec);
}
#endif

#ifdef _QXWZ_TEST_STARTSTOP
void test_qxwz_rtcm_start_stop(void)
{
	//sleep(2);
	qxwz_rtcm_stop();
    while(1)
    {
    	qxwz_rtcm_start(qxwz_rtcm_response_callback,qxwz_status_response_callback);
		sleep(50);
	    time_t time_stop_begin = time(NULL);
	    qxwz_rtcm_stop();
	    time_t time_stop_end = time(NULL);
	    QXLOGI("time_stop_begin:%d,time_stop_end:%d\n",time_stop_begin,time_stop_end);
		sleep(1);
    }
}
#endif
