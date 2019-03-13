#include "msgHandler.h"


int main(int argc,char**argv)
{
	MsgHandler msgHandler;
	
	if(msgHandler.init(argc,argv))
		msgHandler.run();
	
	
	return 0;
}
