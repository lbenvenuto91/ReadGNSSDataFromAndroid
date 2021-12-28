#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<stdbool.h>
#include<math.h>

#define MAXCHAR 1024
#define SPEED_OF_LIGHT 299792458.0  // [m/s]
#define GPS_WEEKSEC 604800  // Number of seconds in a week
#define NS_TO_S 1e-9 

typedef struct {

	char typemeas [3];
	long long int utcTimeMillis;
	long long int TimeNanos;
	signed long long int FullBiasNanos;
	float BiasNanos;
	int TimeOffsetNanos;
	int Svid;

}androidgnssmeas;




float computePseudorange(androidgnssmeas gnssdata, float psdrgBias) 

{
	float psrange = 0;

	int gpsweek = 0;

	gpsweek = floor(-gnssdata.FullBiasNanos * NS_TO_S / GPS_WEEKSEC);
	
	return psrange;
}




void printValues(androidgnssmeas values[])
{
	int nelem = 0;
	nelem=sizeof(values) / sizeof(values[0]);

	for (int i = 0; i < 129; i++)
	{

		printf("TypeMeas= %s, utcTimeMillis= %lli, TimeNanos= %lli, FullBiasNanos= %lli, BiasNanos= %f, TimeOffsetNanos= %i\n", values[i].typemeas, values[i].utcTimeMillis, values[i].TimeNanos, values[i].FullBiasNanos, values[i].BiasNanos, values[i].TimeOffsetNanos);
		//printf("TypeMeas= %s\n", values[i].typemeas);
	}
}

int main() {

	
	FILE *gnssfile = fopen("C:\\Users\\gterg\\Documents\\GitHub\\readGNSSandroid\\index.csv", "r");

	if (!gnssfile)
	{
		printf("Error in opening file!\n");
		return 1;
	}

	char buff[1024]; //store the first 1024 lines into buff
	int row_count = 0;
	int col_count = 0;

	androidgnssmeas fgnssand[129];  //array to store values

	int i = 0;
	while (fgets(buff, 1024, gnssfile))
	{
		col_count = 0;
		row_count++;
		if(row_count==1)
		{
			continue; //skip the first line
		}

		char *col = strtok(buff, ",");//separate buff with commas
		while (col) 
		{
			if (col_count == 0)
				strcpy(fgnssand[i].typemeas, col);
			if (col_count == 1)
				fgnssand[i].utcTimeMillis = atoll(col);
			if (col_count == 2)
				fgnssand[i].TimeNanos = atoll(col);
			if (col_count == 3)
				fgnssand[i].FullBiasNanos = atoll(col);
			if (col_count == 4)
				fgnssand[i].BiasNanos = atof(col);
			if (col_count == 7)
				fgnssand[i].Svid = atoi(col);
			if (col_count == 8)
				fgnssand[i].TimeOffsetNanos = atoi(col);

			col = strtok(NULL, ","); //update field value
			col_count++;

		}
		i++;
	}
	fclose(gnssfile);

	float num = 0;

	num = sizeof(fgnssand) / sizeof(fgnssand[0]);

	printf("numero elementi: %f", num);


	//printValues(fgnssand);
	float psdrgBias = 0;

	for (int i = 0; i < 129; i++)
	{

		computePseudorange(fgnssand[i], psdrgBias);
	}


	return 0;
}



