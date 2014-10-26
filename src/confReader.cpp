#include "confReader.h"

confReader::confReader(const char * fileName)
{
	char confItemName[256];
	char confItemValue[1024];
	
	string * ItemName,* ItemValue;
	fp =  fopen(fileName,"r");
	if(fp!=NULL)
	{
		puts("start to read configure file");
		while(fgets(fileReadBuffer,sizeof(fileReadBuffer),fp)==fileReadBuffer)
		{
			#ifdef __DEBUG__
				puts("line readed:");
				puts(fileReadBuffer);
			#endif

			if(fileReadBuffer[0]=='#')
				continue;
			if(fileReadBuffer[0]== 0)
				continue;
			if(fileReadBuffer[0]==';')
				continue;

			if(sscanf(fileReadBuffer,"%s %s",confItemName,confItemValue)==EOF)
			{
				puts("Some thing error while reading config file!May be this is NULL line");
				continue;
			}
			
			ItemName = new string(confItemName);
			ItemValue = new string(confItemValue);
			
			#ifdef __DEBUG__
				puts("Item Name Got:");
				puts(ItemName->c_str());
				puts("Item Value Got:");
				puts(ItemValue->c_str());
			#endif

			conf.insert(pair<string,string>(*ItemName,*ItemValue));
		}
		puts("finish reading configure file!");
	}
	else
	{
		puts("error in reading configure file!\ncan't open file!");
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
