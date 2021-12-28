import sys,os
import numpy as np
import math
from datetime import datetime, timedelta
import time


# Flags to check wether the measurement is correct or not
# https://developer.android.com/reference/android/location/GnssMeasurement.html#getState()
STATE_2ND_CODE_LOCK = int(0x00010000)
STATE_BDS_D2_BIT_SYNC = int(0x00000100)
STATE_BDS_D2_SUBFRAME_SYNC = int(0x00000200)
STATE_BIT_SYNC = int(0x00000002)
STATE_CODE_LOCK = int(0x00000001)
STATE_GAL_E1BC_CODE_LOCK = int(0x00000400)
STATE_GAL_E1B_PAGE_SYNC = int(0x00001000)
STATE_GAL_E1C_2ND_CODE_LOCK = int(0x00000800)
STATE_GLO_STRING_SYNC = int(0x00000040)
STATE_GLO_TOD_DECODED = int(0x00000080)
STATE_GLO_TOD_KNOWN = int(0x00008000)
STATE_MSEC_AMBIGUOUS = int(0x00000010)
STATE_SBAS_SYNC = int(0x00002000)
STATE_SUBFRAME_SYNC = int(0x00000004)
STATE_SYMBOL_SYNC = int(0x00000020)
STATE_TOW_DECODED = int(0x00000008)
STATE_TOW_KNOWN = int(0x00004000)
STATE_UNKNOWN = int(0x00000000)

ADR_STATE_UNKNOWN = int(0x00000000)
ADR_STATE_VALID = int(0x00000001)
ADR_STATE_RESET = int(0x00000002)
ADR_STATE_HALF_CYCLE_RESOLVED = int(0x00000008)
ADR_STATE_HALF_CYCLE_REPORTED = int(0x00000010)
ADR_STATE_CYCLE_SLIP = int(0x00000004)




#CONSTANTS

global c
c=299792458.0 #m/s
global NanoSecondsWeek
NanoSecondsWeek=604800000000000 # Number of nanoseconds in a week
global NanoSecondsDay
NanoSecondsDay=86400000000000
global CurrentLeapSenconds
CurrentLeapSeconds=18


# Define constants
SPEED_OF_LIGHT = 299792458.0  # [m/s]
GPS_WEEKSECS = 604800  # Number of seconds in a week
NS_TO_S = 1.0e-9
NS_TO_M = NS_TO_S * SPEED_OF_LIGHT  # Constant to transform from nanoseconds to meters
BDST_TO_GPST = 14  # Leap seconds difference between BDST and GPST
GLOT_TO_UTC = 10800  # Time difference between GLOT and UTC in seconds
# Origin of the GPS time scale
GPSTIME = datetime(1980, 1, 6)
DAYSEC = 86400  # Number of seconds in a day
CURRENT_GPS_LEAP_SECOND = 18


EPOCH_STR = 'epoch'

GLO_L1_CENTER_FREQ = 1.60200e9
GLO_L1_DFREQ = 0.56250e6



def get_rnx_band_from_freq(frequency):
    """
    Obtain the frequency band
    >>> get_rnx_band_from_freq(1575420030.0)
    1
    >>> get_rnx_band_from_freq(1600875010.0)
    1
    >>> get_rnx_band_from_freq(1176450050.0)
    5
    >>> get_rnx_band_from_freq(1561097980.0)
    2
    """
    # Backwards compatibility with empty fields (assume GPS L1)
    ifreq = 154 if frequency == '' else round(frequency / 10.23e6)
    # QZSS L1 (154), GPS L1 (154), GAL E1 (154), and GLO L1 (156)
    if ifreq >= 154:
        return 1
    # QZSS L5 (115), GPS L5 (115), GAL E5 (115)
    elif ifreq == 115:
        return 5
    # BDS B1I (153)
    elif ifreq == 153:
        return 2
    else:
        raise ValueError("Cannot get Rinex frequency band from frequency [ {0} ]. "
        "Got the following integer frequency multiplier [ {1:.2f} ]\n".format(frequency, ifreq))
    return ifreq

def get_frequency(header,gnssdata):
    v = float(gnssdata[header['CarrierFrequencyHz']])
    return 154 * 10.23e6 if v == '' else v

def get_rnx_attr(band, constellation='G', state=0x00):

    """
    Generate the RINEX 3 attribute from a given band. Assumes 'C' for L1/E1
    frequency and 'Q' for L5/E5a frequency. For E5a it assumes Q tracking.
    """
    attr = 'C'

    # Make distinction between GAL E1C and E1B code
    if band == 1 and constellation[1] == 'E':
        if (state & STATE_GAL_E1C_2ND_CODE_LOCK) == 0 and (state & STATE_GAL_E1B_PAGE_SYNC) != 0:
            attr = 'B'

    # GAL E5, QZSS L5, and GPS L5 (Q)
    if band == 5:
        attr = 'Q'

    # BDS B1I
    if band == 2 and constellation[1] == 'C':
        attr = 'I'

    return attr
    

def get_band_from_freq():
    pass

def get_obs_code(header,gnssdata):
    band = get_rnx_band_from_freq(get_frequency(header,gnssdata))
    ConstellationType=int(gnssdata[header['ConstellationType']])
    constellation=getConstellation(ConstellationType)
    attr = get_rnx_attr(band, constellation,state=int(gnssdata[header['State']]))
    return'{}{}'.format(band,attr)

def readHeader(filename,category):
    '''
    returns a dict with the fields contained in the 
    specified category
    For raw GNSS meas the category is Raw
    '''
    header={}

    with open(filename,'r') as infile:
        for line in infile.readlines():
            if line.startswith('# {}'.format(category)):
                l=line.split(',')
                for i in range(len(l)):
                    header[l[i]]=i
    return header

def readGNSSAndroid(filename,category):
    '''
    returns a list with only Raw GNSS information
    contained in a Android file
    '''
    header=readHeader(filename,category)
    dati=[]
    with open(filename,'r') as infile:
        for line in infile.readlines():
            if line.startswith('Raw'):
                dati.append(line)
    return header,dati


def getConstellation(constellationType):

    constellation={}
    constellation[1]=('GPS','G')
    constellation[2]=('SBAS','S')
    constellation[3]=('GLONASS','R')
    constellation[4]=('QZSS','J')
    constellation[5]=('BeiDou','C')
    constellation[6]=('Galileo','E')
    constellation[7]=('IRNSS','I')
    if type(constellationType)!=int:
        print('Error: Invalid Constellation Type')
        return
    elif constellationType<1 or constellationType>7:
        print('Error: Invalid Constellation Type')
        return
    else:
        return constellation[constellationType]

def getUTCtime(header,gnssdata):
    '''
    Returns utcTimeMillis in a UTC datetime object
    '''
    if gnssdata[0] != 'Raw':
        print('Error: invalid input data!')
        return  
    utcTimeMillis=int(gnssdata[header['utcTimeMillis']])
    return datetime.utcfromtimestamp(utcTimeMillis/1000)

def getGPSwn(date):
    '''
    Calculate the number of weeks since 6th January 1980
    '''
    epoch=datetime(1980,1,6) #sunday 6th Jan 1980
    i=0
    while i<=6:
        if date.weekday()==6:
            date_sunday=date
            break
        else:
            date -= timedelta(days=1)
        i+=1         
    wn=int((date_sunday-epoch).days/7)
    return wn

def getNanosecondsDay1980(date):
    '''
    Returns the number of day in nanoseconds since 6th Jan 1980
    '''
    epoch=datetime(1980,1,6) #sunday 6th Jan 1980
    ndays=(date-epoch).days
    return ndays*NanoSecondsDay


def getSatID(header,gnssdata):
    '''
    Returns utcTimeMillis in a UTC datetime object
    '''
    if gnssdata[0] != 'Raw':
        print('Error: invalid input data!')
        return  
    Svid=int(gnssdata[header['Svid']])
    ConstellationType=int(gnssdata[header['ConstellationType']])
    c=getConstellation(ConstellationType)
    return '{}{}'.format(c[1],Svid)


def glot_to_gpst(gpst_current_epoch, tod_seconds):
    """
    Converts GLOT to GPST
    :param gpst_current_epoch: Current epoch of the measurement in GPST
    :param tod_seconds: Time of days as number of seconds
    :return: Time of week in seconds
    """
    (tod_sec_frac, tod_sec) = math.modf(tod_seconds);
    tod_sec = int(tod_sec)

    # Get the GLONASS epoch given the current GPS time
    glo_epoch = datetime(year=gpst_current_epoch.year,
                         month=gpst_current_epoch.month,
                         day=gpst_current_epoch.day,
                         hour=gpst_current_epoch.hour,
                         minute=gpst_current_epoch.minute,
                         second=gpst_current_epoch.second)\
                         + timedelta(
                                hours=3,
                                seconds=-CURRENT_GPS_LEAP_SECOND)
    # Adjust the GLONASS time with the TOD measurements
    glo_tod = datetime(year=glo_epoch.year,
                        month=glo_epoch.month,
                        day=glo_epoch.day) + timedelta(seconds=tod_sec)

    # The day of week in seconds needs to reflect the time passed before the current day starts
    day_of_week_sec = (glo_tod.isoweekday()) * DAYSEC

    # Compute time of week in seconds
    tow_sec = day_of_week_sec + tod_seconds - GLOT_TO_UTC + CURRENT_GPS_LEAP_SECOND

    return tow_sec


def check_week_crossover(tRxSeconds, tTxSeconds):
    """
    Checks time propagation time for week crossover
    :param tRxSeconds: received time in seconds of week
    :param tTxSeconds: transmitted time in seconds of week
    :return: corrected propagation time

    """

    tau = tRxSeconds - tTxSeconds
    if tau > GPS_WEEKSECS / 2:
        del_sec = round(tau/GPS_WEEKSECS)*GPS_WEEKSECS
        rho_sec = tau - del_sec

        if rho_sec > 10:
            tau = 0.0
        else:
            tau = rho_sec

    return tau

def check_trck_state(header,gnssdata):
    """
    Checks if measurement is valid or not based on the Sync bits
    """
    # Obtain state, constellation type and frquency value to apply proper sync state
    state = int(gnssdata[header['State']])
    constellation = int(gnssdata[header['ConstellationType']])
    frequency = get_frequency(header,gnssdata)
    frequency_band = get_rnx_band_from_freq(frequency)

    # Filtering measurements for GPS constellation (common and non optional for L1 and L5 signals)
    if constellation == 1:
        if (state & STATE_CODE_LOCK) == 0:
            #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_CODE_LOCK))
            print("State [ 0x{0:2x} {0:8b} ] has STATE_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_CODE_LOCK))
        if (state & STATE_TOW_DECODED) == 0:
            #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_TOW_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_TOW_DECODED))
            print("State [ 0x{0:2x} {0:8b} ] has STATE_TOW_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_TOW_DECODED))
        if (state & STATE_MSEC_AMBIGUOUS) != 0:
            #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_MSEC_AMBIGUOUS))
            print("State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_MSEC_AMBIGUOUS))
    # Filtering measurements for SBAS constellation
    elif constellation == 2:
        if (state & STATE_CODE_LOCK) == 0:
            raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_CODE_LOCK))

        if (state & STATE_TOW_DECODED) == 0:
            raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_TOW_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_TOW_DECODED))

        if (state & STATE_MSEC_AMBIGUOUS) != 0:
            raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_MSEC_AMBIGUOUS))

    # Filtering measurements for GLO constellation
    elif constellation == 3:
        if (state & STATE_CODE_LOCK) == 0:
            #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_CODE_LOCK))
            print("State [ 0x{0:2x} {0:8b} ] has STATE_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_CODE_LOCK))
        if (state & STATE_GLO_TOD_DECODED) == 0:
            #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_GLO_TOD_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_GLO_TOD_DECODED))
            print("State [ 0x{0:2x} {0:8b} ] has STATE_GLO_TOD_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_GLO_TOD_DECODED))
        if (state & STATE_MSEC_AMBIGUOUS) != 0:
            #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_MSEC_AMBIGUOUS))
            print("State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_MSEC_AMBIGUOUS))
    elif constellation == 4:
        if (state & STATE_CODE_LOCK) == 0:
            #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_CODE_LOCK))
            print("State [ 0x{0:2x} {0:8b} ] has STATE_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_CODE_LOCK))
        if (state & STATE_TOW_DECODED) == 0:
            #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_TOW_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_TOW_DECODED))
            print("State [ 0x{0:2x} {0:8b} ] has STATE_TOW_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_TOW_DECODED))
        if (state & STATE_MSEC_AMBIGUOUS) != 0:
            #raise ValueError(
            #    "State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state,
            #                                                                                              STATE_MSEC_AMBIGUOUS))
            print("State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state,
                                                                                                          STATE_MSEC_AMBIGUOUS))

    elif constellation == 5:
        if (state & STATE_CODE_LOCK) == 0:
            #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_CODE_LOCK))
            print("State [ 0x{0:2x} {0:8b} ] has STATE_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_CODE_LOCK))
        if (state & STATE_TOW_DECODED) == 0:
            #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_TOW_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_TOW_DECODED))
            print("State [ 0x{0:2x} {0:8b} ] has STATE_TOW_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_TOW_DECODED))
        if (state & STATE_MSEC_AMBIGUOUS) != 0:
            #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_MSEC_AMBIGUOUS))
            print("State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_MSEC_AMBIGUOUS))
    elif constellation == 6:
        if frequency_band == 1:
            if (state & STATE_GAL_E1BC_CODE_LOCK) == 0:
                #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_GAL_E1BC_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_GAL_E1BC_CODE_LOCK))
                print("State [ 0x{0:2x} {0:8b} ] has STATE_GAL_E1BC_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_GAL_E1BC_CODE_LOCK))
            # State value indicates presence of E1B code
            if (state & STATE_GAL_E1C_2ND_CODE_LOCK) == 0:
                if (state & STATE_TOW_DECODED) == 0:
                    #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_TOW_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_TOW_DECODED))
                    print("State [ 0x{0:2x} {0:8b} ] has STATE_TOW_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_TOW_DECODED))
                if (state & STATE_MSEC_AMBIGUOUS) != 0:
                    # raise ValueError(
                    #     "State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state,
                    #                                                                                               STATE_MSEC_AMBIGUOUS))
                    print("State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state,
                                                                                                                  STATE_MSEC_AMBIGUOUS))
            # State value indicates presence of E1C code
            else:
                if (state & STATE_GAL_E1C_2ND_CODE_LOCK) == 0:
                    # raise ValueError(
                    #     "State [ 0x{0:2x} {0:8b} ] has STATE_GAL_E1BC_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(
                    #         state, STATE_GAL_E1C_2ND_CODE_LOCK))
                    print("State [ 0x{0:2x} {0:8b} ] has STATE_GAL_E1BC_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(
                            state, STATE_GAL_E1C_2ND_CODE_LOCK))
                if (state & STATE_MSEC_AMBIGUOUS) != 0:
                    # raise ValueError(
                    #     "State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state,
                    #                                                                                               STATE_MSEC_AMBIGUOUS))
                    print("State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state,
                                                                                                                  STATE_MSEC_AMBIGUOUS))
        # Measurement is E5a
        elif frequency_band == 5:
            if (state & STATE_CODE_LOCK) == 0:
                #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_CODE_LOCK))
                print("State [ 0x{0:2x} {0:8b} ] has STATE_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_CODE_LOCK))
            if (state & STATE_TOW_DECODED) == 0:
                #raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_TOW_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_TOW_DECODED))
                print("State [ 0x{0:2x} {0:8b} ] has STATE_TOW_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_TOW_DECODED))
            if (state & STATE_MSEC_AMBIGUOUS) != 0:
                # raise ValueError(
                #     "State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state,
                #                                                                                               STATE_MSEC_AMBIGUOUS))
                print("State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state,
                                                                                                              STATE_MSEC_AMBIGUOUS))
    elif constellation <1 or constellation>7:
        if (state & STATE_CODE_LOCK) == 0:
            raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_CODE_LOCK [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_CODE_LOCK))

        if (state & STATE_TOW_DECODED) == 0:
            raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_TOW_DECODED [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_TOW_DECODED))

        if (state & STATE_MSEC_AMBIGUOUS) != 0:
            raise ValueError("State [ 0x{0:2x} {0:8b} ] has STATE_MSEC_AMBIGUOUS [ 0x{1:2x} {1:8b} ] not valid".format(state, STATE_MSEC_AMBIGUOUS))

    else:
        raise ValueError("ConstellationType [ 0x{0:2x} {0:8b} ] is not valid".format(constellation))

    return True


def computePseudorange(header,gnssdata,FullBiasNanos=None):
    '''
    Computes the pseudorange values starting from android data
    Input: list with values contained in Raw Lines
    '''
    if gnssdata[0] != 'Raw':
        print('Error: invalid input data!')
        return  
    
    obscode=get_obs_code(header,gnssdata)

    #RECEIVER TIME
    #TimeNanos quantity: GNSS receiver’s internal hardware clock provided as an integer number of nanoseconds
    try:
        TimeNanos=float(gnssdata[header['TimeNanos']])
    except ValueError:
        raise ValueError("WARNING: Invalid value of TimeNanos for satellite {}".format(getSatID(header,gnssdata)))
    
    #FullBiasNanos: difference between the TimeNanos inside the GPS receiver and the true GPS time since the 6th of January 1980,
    #In order to keep FullBiasNanos and not upate it call the computePseudorange function with a value of FullBiasNanos
    FullBiasNanos=float(gnssdata[header['FullBiasNanos']]) if FullBiasNanos is None else FullBiasNanos

    #BiasNanos: the clock’s sub-nanosecond bias (varying between 0 and 1), which allows getting a more accurate timing
    try:
        BiasNanos=float(gnssdata[header['BiasNanos']])
    except ValueError:
        BiasNanos=0.0
        print("WARNING: Invalid value of BiasNanos for satellite {}. Used BiasNanos=0.0".format(getSatID(header,gnssdata)))

    # Compute the GPS week number and reception time (i.e. clock epoch)
    gpsweek = math.floor(-FullBiasNanos * NS_TO_S / GPS_WEEKSECS)
    local_est_GPS_time = TimeNanos - (FullBiasNanos + BiasNanos)
    gpssow = local_est_GPS_time * NS_TO_S - gpsweek * GPS_WEEKSECS
    gpst_epoch = GPSTIME + timedelta(weeks=gpsweek, seconds=gpssow)

    # Furthermore, for each observation, the time offset at which the measurement was taken
    # w.r.t. the TimeNanos has to be considered. This quantity is given by the TimeOffsetNanos .
    # Accordingly for a specific measurement, a certain epoch, a receiver r , a satellite s
    # and frequency j 

    try:
        TimeOffsetNanos=float(gnssdata[header['TimeOffsetNanos']])
    except ValueError:
        TimeOffsetNanos=0.0
        print('WARNING: Invalid value of TimeOffsetNanos for satellite {}. Used TimeOffsetNanos=0.0'.format(getSatID(header,gnssdata)))

    # Compute the reception times
    t_Rx_seconds = gpssow - TimeOffsetNanos * NS_TO_S

    #SATELLITE TRANSMISSION TIME
    # The received satellite time (ReceivedSvTimeNanos) is relative to the beginning of
    # the system week for all constellations except for GLONASS, where it is relative to the beginning
    # of the GLONASS system day. Therefore, depending on the GNSS involved, the condition of
    # reliability of the received satellite time is satisfied in the following cases:

    check_trck_state(header,gnssdata)

    ConstellationType=int(gnssdata[header['ConstellationType']])

    if ConstellationType == 3: #GLONASS
        # GLOT is given as TOD, need to change to TOW
        Tod_secs=int(gnssdata[header['ReceivedSvTimeNanos']])*NS_TO_S
        T_Tx_seconds=glot_to_gpst(gpst_epoch,Tod_secs)

    elif ConstellationType == 5:
        #BDST uses different epoch as GPS
        T_Tx_seconds=int(gnssdata[header['ReceivedSvTimeNanos']])*NS_TO_S+ BDST_TO_GPST
    else:
        #GPS, GALILEO, QZSS and SBAS share the same epoch time
        T_Tx_seconds=int(gnssdata[header['ReceivedSvTimeNanos']])*NS_TO_S

   
    # Once the receiver and satellite time have been computed, the pseudorange observation
    # can be reconstructed. 

    tau=check_week_crossover(t_Rx_seconds,T_Tx_seconds)
    range=tau*SPEED_OF_LIGHT
    
    return round(range,3)

def computeCarrierPhase(header,gnssdata):
    if gnssdata[0] != 'Raw':
        print('Error: invalid input data!')
        return  

    wavelength = SPEED_OF_LIGHT / get_frequency(header,gnssdata)
    #Checks if measurement is valid or not based on the Sync bits
    # Obtain state, constellation type and frquency value to apply proper sync state
    state = int(gnssdata[header['AccumulatedDeltaRangeState']])

    if (state & ADR_STATE_VALID) == 0:
        print("ADR State [ 0x{0:2x} {0:8b} ] has ADR_STATE_VALID [ 0x{1:2x} {1:8b} ] not valid".format(state, ADR_STATE_VALID))
        cphase=0.0

    else:
        # Process the accumulated delta range (i.e. carrier phase). This
        # needs to be translated from meters to cycles (i.e. RINEX format
        # specification)
        cphase = float(gnssdata[header['AccumulatedDeltaRangeMeters']]) / wavelength

    return round(cphase,3)

def computeDoppler(header, gnssdata):
    PseudorangeRateMetersPerSecond=float(gnssdata[header['PseudorangeRateMetersPerSecond']])
    wavelength = SPEED_OF_LIGHT / get_frequency(header,gnssdata)
    return round(-PseudorangeRateMetersPerSecond/wavelength,3)

def main():
    header,gnssraw=readGNSSAndroid('readgnsstest.txt','Raw')
    
    for line in gnssraw:
        if line.split(',')[0]=='Raw':
            a=getUTCtime(header,line.split(','))

            b=getSatID(header,line.split(','))
            c=computePseudorange(header,line.split(','))
            d=computeCarrierPhase(header,line.split(','))
            e=computeDoppler(header,line.split(','))
            cn0=round(float(line.split(',')[header['Cn0DbHz']]),3)
            print(b,c,d,e,cn0)
            time.sleep(1)

if __name__=="__main__":
    main()