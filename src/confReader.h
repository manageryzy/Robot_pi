#include "header.h"
using namespace std;

class confReader{
	char fileReadBuffer[1024];
	map<string,string> conf;
	FILE * fp;
public :
	confReader(char * fileName);
	string getConf(string item);
	
};