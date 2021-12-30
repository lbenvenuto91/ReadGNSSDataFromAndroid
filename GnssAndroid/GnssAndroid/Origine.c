#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<stdbool.h>
#include<math.h>
#include<time.h>

#define MAXCHAR 1024
#define SPEED_OF_LIGHT 299792458.0  // [m/s]
#define GPS_WEEKSEC 604800  // Number of seconds in a week
#define NS_TO_S 1e-9 
#define BDST_TO_GPST 14 //Leap seconds difference between BDST and GPST
#define DAYSEC 86400
#define CURRENT_GPS_LEAP_SECOND 18
#define GLOT_TO_UTC 10800  // Time difference between GLOT and UTC in seconds

//#define GPSTIME newDateTime(1980, 01, 06, 00, 00, 00)

typedef struct {

	char typemeas [3];
	long long int utcTimeMillis;
	long long int TimeNanos;
	signed long long int FullBiasNanos;
	double BiasNanos;
	int TimeOffsetNanos;
	int Svid;
	int ConstellationType;
	long long int ReceivedSvTimeNanos;

}androidgnssmeas;


time_t newDateTime(const int year, const int month, const int date, const int hrs, const int min, const int sec) {
	time_t time;
	struct tm tmStruct = { 0 };
	tmStruct.tm_mday = date;
	tmStruct.tm_mon = month - 1;
	tmStruct.tm_year = year - 1900;
	tmStruct.tm_hour = hrs;
	tmStruct.tm_min = min;
	tmStruct.tm_sec = sec;
	time = mktime(&tmStruct);
	if (time == -1) {
		printf("Data non supportata.");
		exit(1);
	}
	return time;
}


char* formatDateTime(const time_t mytime) {
	/*Used to printf date object*/
	char* dateTimeStr = malloc(20 * sizeof(char));
	struct tm date = *localtime(&mytime);
	sprintf(dateTimeStr, "%.2d/%.2d/%4d %.2d:%.2d:%.2d", date.tm_mday, date.tm_mon + 1, date.tm_year + 1900, date.tm_hour, date.tm_min, date.tm_sec);
	return dateTimeStr;
}

double check_week_crossover(double tRxSeconds, double tTxSeconds) 
{
	/*
	Checks time propagation time for week crossover
		: param tRxSeconds : received time in seconds of week
		: param tTxSeconds : transmitted time in seconds of week
		: return : corrected propagation time
	*/
	int del_sec = 0;
	double tau = 0, rho_sec=0;
	tau = tRxSeconds - tTxSeconds;
	if (tau > GPS_WEEKSEC / 2) 
	{
		del_sec = round(tau / GPS_WEEKSEC) * GPS_WEEKSEC;
		rho_sec = tau - del_sec;
		if (rho_sec > 10){
			tau = 0.0;
		}
		else {
			tau = rho_sec;
		}
	}
	return tau;
}


double glot_to_gpst(time_t gpst_current_epoch, double tod_seconds) 
{
	/*
	 Converts GLOT to GPST
    :param gpst_current_epoch: Current epoch of the measurement in GPST
    :param tod_seconds: Time of days as number of seconds
    :return: Time of week in seconds	
	*/

	// Get the GLONASS epoch given the current GPS time
	struct tm tmStruct, tmStructtod;
	double tod_sec_frac, tod_sec, tow_sec;
	time_t glo_epoch,glo_td, glo_tod;
	int day_of_week_sec;

	tod_sec_frac = modf(tod_seconds, &tod_sec);

	tmStruct = *localtime(&gpst_current_epoch);
	tmStruct.tm_hour += 3;
	tmStruct.tm_sec -= CURRENT_GPS_LEAP_SECOND;
	glo_epoch= mktime(&tmStruct); 

	tmStruct = *localtime(&glo_epoch);
	tmStruct.tm_hour = 0;
	tmStruct.tm_min = 0;
	tmStruct.tm_sec = 0;
	tmStruct.tm_sec += tod_sec;
	glo_tod = mktime(&tmStruct); //LB: maybe this passage doent have sense
	
	//The day of week in seconds needs to reflect the time passed before the current day starts
	day_of_week_sec = tmStruct.tm_wday * DAYSEC;

	tow_sec = day_of_week_sec + tod_seconds - GLOT_TO_UTC + CURRENT_GPS_LEAP_SECOND;

	return tow_sec;

}

double computePseudorange(androidgnssmeas gnssdata, float psdrgBias) 

{
	
	int gpsweek = 0;
	double psrange, local_est_GPS_time = 0, gpssow = 0, T_Rx_seconds = 0, T_Tx_seconds=0, tau=0,Tod_secs=0 ;
	struct tm tmStruct;
	time_t gpst_epoch,gpstime;
	gpstime = newDateTime(1980, 01, 06, 00, 00, 00);

	//compute Receiver time
	gpsweek = floor(-gnssdata.FullBiasNanos * NS_TO_S / GPS_WEEKSEC);
	local_est_GPS_time = gnssdata.TimeNanos - (gnssdata.FullBiasNanos + gnssdata.BiasNanos);

	gpssow = local_est_GPS_time * NS_TO_S - gpsweek * GPS_WEEKSEC;

	tmStruct = *localtime(&gpstime);
	tmStruct.tm_mday+=gpsweek*7;
	tmStruct.tm_sec += gpssow;
	gpst_epoch = mktime(&tmStruct); //LB: NS not handled
	
	if (gpst_epoch == -1) {
		printf("Data non supportata.");
		exit(1);
	}
	
	T_Rx_seconds = gpssow - gnssdata.TimeOffsetNanos * NS_TO_S;

	//compute satellite emission time

	//split cases depending on different constellations
	
	if (gnssdata.ConstellationType == 2) 
	{
		printf("ERROR: Pseudorange computation not supported for SBAS\n");
		return -1;
	}
	else if (gnssdata.ConstellationType == 3) 
	{
		//GLONASS
		Tod_secs = gnssdata.ReceivedSvTimeNanos * NS_TO_S;
    	T_Tx_seconds = glot_to_gpst(gpst_epoch, Tod_secs);
	}
	else if (gnssdata.ConstellationType == 5) 
	{
		//BEIDOU
		T_Tx_seconds = gnssdata.ReceivedSvTimeNanos * NS_TO_S + BDST_TO_GPST;

	}
	else if (gnssdata.ConstellationType == 1 || gnssdata.ConstellationType == 6) 
	{
		//GPS and GALILEO
		T_Tx_seconds = gnssdata.ReceivedSvTimeNanos * NS_TO_S;
	}
	else
	{
		printf("Case not implemented\n");
		return -1;
	}

	tau = check_week_crossover(T_Rx_seconds, T_Tx_seconds);
	psrange = tau * SPEED_OF_LIGHT;
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

int main(void) {

	
	FILE *gnssfile = fopen("C:\\Users\\gterg\\Documents\\GitHub\\ReadGNSSDataFromAndroid\\index.csv", "r");


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
				//fgnssand[i].BiasNanos = strtod()
				sscanf(col, "%lf", &fgnssand[i].BiasNanos);
			if (col_count == 7)
				fgnssand[i].Svid = atoi(col);
			if (col_count == 8)
				fgnssand[i].TimeOffsetNanos = atoi(col);
			if (col_count == 20)
				fgnssand[i].ConstellationType = atoi(col);
			if (col_count == 10)
				fgnssand[i].ReceivedSvTimeNanos = atoll(col);
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



