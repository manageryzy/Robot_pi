#include "confReader.h"

confReader::confReader(char * fileName)
{
	char confItemName[256];
	char confItemValue[1024];
	
	string * ItemName,* ItemValue;
	fp =  fopen(fileName,"r");
	if(fp!=NULL)
	{
		while(!fgets(fileReadBuffer,sizeof(fileReadBuffer),fp)||ferror(fp)||feof(fp))
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
			
			conf.insert(pair<string,string>(*ItemName,*ItemValue));
		}
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