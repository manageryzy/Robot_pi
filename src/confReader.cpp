#include "confReader.h"

confReader::confReader(const char * fileName)
{
	char confItemName[256];
	char confItemValue[1024];
	
	string * ItemName,* ItemValue;
	fp =  fopen(fileName,"r");
	if(fp!=NULL)
	{
		while(fgets(fileReadBuffer,sizeof(fileReadBuffer),fp)!=fileReadBuffer)
		{
			if(fileReadBuffer[0]=='#')
				continue;
			if(sscanf(fileReadBuffer,"%s %s",confItemName,confItemValue)==EOF)
			{
				puts("Some thing error while reading config file!");
				continue;
			}
			
			ItemName = new string(confItemName);
			ItemValue = new string(confItemValue);
			
			#ifdef __DEBUG__
				puts(ItemName->c_str());
				puts(ItemValue->c_str());
			#endif

			conf.insert(pair<string,string>(*ItemName,*ItemValue));
		}
		puts("finish reading configure file!");
	}
	else
	{
		puts("error in reading configure file!");
	}
}

string confReader ::  getConf(string item)
{
	map<string,string>::iterator iter;
	iter = conf.find(item);
	if(iter==conf.end())
	{
		return string("null");
	}
	else
	{
		return iter->second;
	}
}
