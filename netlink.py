#
#    Copyright (c) 2012 Tom Keffer <tkeffer@gmail.com>
#
#    See the file LICENSE.txt for your full rights.
#
#    $Revision: 1459 $
#    $Author: mwall $
#    $Date: 2013-10-08 17:44:50 -0700 (Tue, 08 Oct 2013) $
#
"""Classes and functions for interfacing with an Arduino/Oregon Scientific Sensor Combination

	This driver relies on a stream of network socket connections from the Arduino which is listening
	to the Oregon Scientific Sensors 433 MHz transmissions. There is nothing Arduino specific about the
	implementation, the driver simply waits for ascii strings on network port 2029 which consist of
	keyword value pairs separated by colons.  eg. windSpeed:  4.68:windDir: 22.50:gustWindSpeed:  5.04:barometer:1009.80:

	Very roughly based on the hackulink driver, which in turn was based on the wmr100 driver.


    Changes
    -------

    12-3-2013  First Public Release

"""

import io
import operator
import socket
import syslog
import time

import weeutil.weeutil
import weewx.abstractstation
import weewx.units
import weewx.wxformulas

def loader(config_dict, engine):

    # The WMR driver needs the altitude in meters. Get it from the Station data
    # and do any necessary conversions.
    altitude_t = weeutil.weeutil.option_as_list(config_dict['Station'].get('altitude', (None, None)))
    # Form a value-tuple:
    altitude_vt = (float(altitude_t[0]), altitude_t[1], "group_altitude")
    # Now convert to meters, using only the first element of the returned value-tuple:
    altitude_m = weewx.units.convert(altitude_vt, 'meter')[0]
    
    station = Netlink(altitude=altitude_m, **config_dict['Netlink'])
    
    return station
        
class Netlink(weewx.abstractstation.AbstractStation):
    """Driver for the Netlink station."""
    
    def __init__(self, **stn_dict) :
        """Initialize an object of type Netlink.
        
        NAMED ARGUMENTS:
        
        altitude: The altitude in meters. Required.
        
        stale_wind: Max time wind speed can be used to calculate wind chill
        before being declared unusable. [Optional. Default is 30 seconds]
        
        timeout: How long to wait, in seconds, before giving up on a response from the
        USB port. [Optional. Default is 15 seconds]
        
        wait_before_retry: How long to wait before retrying. [Optional.
        Default is 5 seconds]

        max_tries: How many times to try before giving up. [Optional.
        Default is 3]
        """
        
        self.altitude          = stn_dict['altitude']
        # TODO: Consider changing this so these go in the driver loader instead:
        self.record_generation = stn_dict.get('record_generation', 'software')
        self.timeout           = float(stn_dict.get('timeout', 20.0))
        self.wait_before_retry = float(stn_dict.get('wait_before_retry', 10.0))
        self.max_tries         = int(stn_dict.get('max_tries', 3))

        self.sensor_id         = stn_dict.get('sensor_id', '02222')
        self.host_ip           = stn_dict.get('host_ip', '127.0.0.1')
        self.host_port         = int(stn_dict.get('host_port', 2029))

        self.last_rain = None
        self.last_totalRain = None
        self.last_windSpeed = None

        #DLT Get labels for tower sensors, if specified
        self.sensors            = stn_dict.get('Sensors', {})

        self.port = None
        self.openPort()

    def openPort(self):
        try:
            print 'Opening Socket'
            print self.host_port
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.bind (('', self.host_port))
            self.socket.listen (5)
        except (socket.error, socket.timeout, socket.herror), ex:
            syslog.syslog(syslog.LOG_ERR, "Netlink: Socket error while opening port %d to ethernet host %s." % (self.host_port, self.host_ip))
            # Reraise as a weewx I/O error:
            raise weewx.WeeWxIOError(ex)
        except:
            syslog.syslog(syslog.LOG_ERR, "Netlink: Unable to connect to ethernet host %s on port %d." % (self.host_ip, self.host_port))
            raise
        syslog.syslog(syslog.LOG_DEBUG, "Netlink: Opened up ethernet host %s on port %d" % (self.host_ip, self.host_port))

  
    def closePort(self):
        self.port.close()
        
    def genLoopPackets(self):
        """Generator function that continuously returns loop packets"""

        for _packet in self.genPackets():
            yield _packet
                
    def genPackets(self):
        #Generate measurement packets.
        try:
           (client, addr) = self.socket.accept()
        except (socket.timeout, socket.error), ex:
            syslog.syslog(syslog.LOG_ERR, "Netlink: timed out reading packet");
            yield self._process_message("")
            return
        
        #self.socket.settimeout(None)
        self.port = client.makefile('r',0)

        while True:
            try:
                _line = self.port.readline(120)
                if _line == '':
                    print('EOF')
                    break
                yield self._process_message(_line)
                break
            except (socket.timeout, socket.error), ex:
                syslog.syslog(syslog.LOG_ERR, "Netlink: timed out reading line");
                yield self._process_message("")
                return
            if _line == None:
                print 'none'
                break



             
    @property
    def hardware_name(self):
        return "Arduino based"
    
    #===============================================================================
    #                         LOOP record decoding functions
    #===============================================================================

    def _process_message(self, message):
        _packet = {}
        syslog.syslog(syslog.LOG_DEBUG, "Netlink:Packet %s" % (message))
        #separate line into a dict
        _l = message.split( ':' )
        #print _l
       
        #   _d = dict( [ i.split( '=' ) for i in _l ] )

        for i in range (0,len(_l)-2):
            if _l[i] == "rainTotal":
                if self.last_totalRain != None:
                    _packet['rain'] = float(_l[i+1])/10 - self.last_totalRain;
                self.last_totalRain = float(_l[i+1])/10

            if _l[i] == "humidity":
                _packet['outHumidity'] = float(_l[i+1])
                
            if _l[i] == "temperature":
                _packet['outTemp'] = float(_l[i+1])
                T = _packet['outTemp']
                R = _packet['outHumidity']
                _packet['dewpoint']  = weewx.wxformulas.dewpointC(T, R)
                _packet['heatindex'] = weewx.wxformulas.heatindexC(T, R)
                if self.last_windSpeed != None:
                    _packet['windchill'] = weewx.wxformulas.windchillC(T, self.last_windSpeed)
                    
            if _l[i] == "windSpeed":
                _packet['windSpeed'] = float(_l[i+1])
                self.last_windSpeed = _packet['windSpeed'] 
                
            if _l[i] == "gustWindSpeed":
                _packet['windGust'] = float(_l[i+1])

                
            if _l[i] == "windDir":                
                # Don't record a wind direction if windspeed is below 2kmh
                if _packet['windSpeed'] < 2.0 :
                    _packet['windDir'] = None
                else:
                    _packet['windDir'] = float(_l[i+1])
                    
            if _l[i] == "barometer":
                _packet['barometer'] = float(_l[i+1])
                
            if _l[i] == "rssi":
                _packet['rssi'] = float(_l[i+1])
            
        #if _d['mt'] == "pressure":
        #    _packet.update( self._barometer_reading(_d) )
        #else:  #DLT Removed check for type "tower"
        #    for _k, _v in _d.items():
        #        if _k in Netlink._dispatch_dict:
        #            _packet.update( [ Netlink._dispatch_dict[_k](self, _v) ] )


        _packet['dateTime'] = int(time.time())
        _packet['usUnits'] = weewx.METRIC
        print _packet
        return _packet


    def _rain_reading(self, reading):
        # example: A0000254
        # measured rainfall in mm/1000 since last reading
        # 36s reporting interval
        
        # rainRate [in, cm]_per_hour
        #return ( 'rainRate', None ) # TODO Not doing anything with this yet, because it requires historical data
        return ('rain', float(reading[2:])/10000)

    def _temperature_reading(self, reading):
        # example: A018000000
        # index 1 : = 0 for positive, - for negative
        # index 2-10 temp in C with a decimal before the last digit
        # 36s reporting interval

        t = float(reading[2:10])/1000000
        if reading[1] == '-':
            t = -1 * t
        # outTemp: degree_[F, C]
        return ( 'outTemp', t )
    
    def _humidity_reading(self, reading):
        # input example: A0590
        # index 1-5 humidity % , with a decimal before the last. 
        # 36s reporting interval
        
        # outHumidity: percent
        return ( 'outHumidity', float(reading[2:5])/10 )   
    
    def _windspeed_reading(self, reading):
        # example A000970000
        # index 2-6 speed in centimeters per second
        # 18s reporting interv

        # windSpeed: kmh
        return ( 'windSpeed', (float(reading[2:6])*(60*60)/(1000*100)) )

    def _winddir_reading(self, reading):
        # example: A
        # A single HEX digit. Assumption that 0=N
        # shouldn't be populated when there is low/no wind
        # 36s reporting interval

        # windDir: degree_compass
        return ( 'windDir', Netlink._windmap_dict[reading] )

    def _battery_reading(self, reading):
        # example: normal

        # txBatteryStatus
        return ( 'txBatteryStatus', int(reading == 'normal') )
        #return ( 'txBatteryStatus', None )

    def _rssi_reading(self, reading):
        # ranges from 0 to 4

        # rxCheckPercent: percent (may not be valid for loop packets)
        return ( 'rxCheckPercent', int(reading)*25 )
        #return ( 'rxCheckPercent', None )

    def _barometer_reading(self, reading):
        # example id=24C86E010EAD&mt=pressure&C1=4978&C2=0E3F&C3=0148&C4=03B5&C5=80F4&C6=1744&C7=09C4&A=07&B=15&C=06&D=09&PR=9C7E&TR=7F0B
        # Believed to be this formulae: http://www.hoperf.com/upload/sensor/HP03S.pdf
        # Programming Guide: http://www.hoperf.com/upload/sensor/HP03_code.pdf
        #
        #   Example
        #       Key     Value   Range (Hex)             Range(Dec)
        #       ---     -----   ---------------         -------------
        #       A:      07      0x01 -- 0x3F             1 -- 63
        #       B:      15      0x01 -- 0x3F             1 -- 63
        #       C:      06      0x01 -- 0x0F             1 -- 15
        #       C1:     4978    0x100 -- 0xFFFF          256 -- 65535
        #       C2:     0E3F    0x00  -- 0x1FFF          0   -- 8191
        #       C3:     0148    0x00 -- 0x400            0   -- 3000
        #       C4:     03B5    0x00 -- 0x1000           0   -- 4096
        #       C5:     80F4    0x1000 -- 0xFFFF        4096 -- 65535
        #       C6:     1744    0x00 -- 0x4000           0   -- 16384
        #       C7:     09C4    0x960 -- 0xA28          2400 -- 2600
        #       D:      09      0x01 -- 0x0F             1 -- 15
        #       PR:     9C7D    0x00 -- 0xFFFF           0 -- 65535
        #       TR:     7F0F    0x00 -- 0xFFFF           0 -- 65535
        

        A, B, C, C1, C2, C3, C4, C5, C6, C7, D, D1, D2 = [ int(reading[i], 16) for i in ("A", "B", "C", "C1", "C2",
                                                                                 "C3", "C4", "C5", "C6", 
                                                                                 "C7", "D", "PR", "TR") ]

        if D2 >= C5:
            COEF = A
        else:
            COEF = B       

        dUT     = D2-C5-((D2-C5)/2**7)*((D2-C5)/2**7)*COEF/2**C
        OFF     = (C2+(C4-1024)*dUT/2**14)*4
        SENS    = C1+C3*dUT/2**10
        X       = SENS*(D1-7168)/2**14-OFF
        P       = (X*10/2**5)+C7
        T       = 250 + (dUT*C6/2**16)-dUT/2**D
        
        # divide by ten because the formulas given seem to be fixed point
        # Not bothering with correcting this, for now.
        return [('barometer', P/10.0), ( 'pressure', P/10.0 ), ('inTemp', T/10.0)]
    
    # Dictionary that maps a measurement code, to a function that can decode it:
    _dispatch_dict = {'rainfall': _rain_reading,
                      'temperature': _temperature_reading,
                      'pressure': _barometer_reading,
                      'humidity': _humidity_reading,
                      'windspeed': _windspeed_reading,
                      'winddir': _winddir_reading,
                      'battery': _battery_reading,
                      'rssi': _rssi_reading}

    #dictionary for decoding wind direction                  
    _windmap_dict = {'5': 0.0,
                     '7': 22.5,
                     '3': 45.0,
                     '1': 67.5,
                     '9': 90.0,
                     'B': 112.5,
                     'F': 135.0,
                     'D': 157.5,
                     'C': 180.0,
                     'E': 202.5,
                     'A': 225.0,
                     '8': 247.5,
                     '0': 270.0,
                     '2': 292.5,
                     '6': 315.0,
                     '4': 337.5}
