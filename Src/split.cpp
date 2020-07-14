#include <stdlib.h>
#include <string.h>
#include <stdio.h>

int GetInputs(char* Command, int* inputs);

int main()
{

char Command[20]="set 0 10 20\r";
int inputs[20];

int n = GetInputs(Command, inputs);

for (int i=0; i<n; i++)
{
printf("%d\n",inputs[i]);
}

}

int GetInputs(char* Command, int* inputs)
{
	char buffer [20];
	int n,cnt;

	char* str = Command;
	char* pch;
	pch = strtok (str," ,\r");
	int num=0;
	cnt=0;
	while (pch != NULL)
	{
	  n=sprintf(buffer, "%s",pch);
          pch = strtok (NULL, " ,\r");
          if (num!=0)
          {
    	    inputs[cnt] = atoi(buffer);
    	    cnt++;
          }
          num=1;
	}
	return cnt;
}
