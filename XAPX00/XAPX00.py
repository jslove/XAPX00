"""
XAPX00 Control.

Adapted to python and XAP 800
Jay Love (jsloveiv@gmail.com)
(C) 2017

Based on the javascript library ap800js.lib:
Gentner/ClearOne AP800 Automatic Mixer Control Library
Copyright (C) 2005 Kade Consulting, LLC.  All rights reserved.
Author: Dan Rudman (dantelope)

This script may be freely copied and modified so long as it carries a
reference to the above copyright.  Any modifications to this script
must use this license and, thus, must itself be freely copyable and
modifiable under the same terms.


Issues:
Matrix Routing:
  Wildcard is not accepted for channels, why not?
  Add ability to turn off all matrix entries
  Is matrix retained after poweroff? Add ability to clear by default?
"""

__version__ = '0.2.8.2'

import serial
import logging
import math
import time
import warnings
import time
import string
from threading import Lock
from functools import wraps

testing = 0


_LOGGER = logging.getLogger(__name__)
if 0:
    import sys
    out_hdlr = logging.StreamHandler(sys.stdout)
    out_hdlr.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
    out_hdlr.setLevel(logging.DEBUG)
    _LOGGER.addHandler(out_hdlr)
    _LOGGER.setLevel(logging.DEBUG)

# Global Constants
XAP800_CMD = "#5"
XAP400_CMD = "#7"
XAP800TYPE = "XAP800"
XAP400TYPE = "XAP400"
EOM = "\r"
DEVICE_MAXMICS = "Max Number of Microphones"
matrixGeo = {'XAP800': 12, 'XAP400': 8}
nogainGroups = ('E')

def stereo(func):
    """
    Call as stereo.

    This will take every method that starts with get or set
    and call it twice, the second time incrementing the second argument
    by 1, which should be the channel argument. If the method is a matrix
    method, both the input and output channels (params 1 & 2) will be
    incremented.
    """
    @wraps(func)
    def stereoFunc(*args, **kwargs):
        # trying to find a way to have a method
        # calling another method not do the stereo repeat
        # so if calling an internal func from a stereo func,
        # add the stereo kw arg to call
        # it will be removed before calling underlying func

        if 'stereo' in kwargs.keys():
            _stereo = kwargs['stereo']
            del(kwargs['stereo'])
        else:
            _stereo = 1
        res = func(*args, **kwargs)
        if args[0].stereo and _stereo:  # self.stereo
            _LOGGER.debug("Stereo Command {}:{}".format(func.__name__, args))
            largs = list(args)
            if type(largs[1]) == str:
                largs[1] = chr(ord(largs[1])+1)
            else:
                largs[1] = largs[1] + 1
            if func.__name__[3:9] == "Matrix":  # do stereo on input and output
                _LOGGER.debug("Matrix Stereo Command {}".format(func.__name__))
                if type(largs[2]) == str:
                    largs[2] = chr(ord(largs[2])+1)
                else:
                    largs[2] = largs[2] + 1
            res2 = func(*largs, **kwargs)
            if res != res2:
                _LOGGER.debug("Stereo out of sync {} : {}".format(res, res2))
                warnings.warn("Stereo out of sync", RuntimeWarning)
        if res is not None:
            return res
    return stereoFunc


def is_number(s):
    """ Returns True if string is a number. """
    return s.replace('.','',1).replace('-','',1).isdigit()

def db2linear(db, maxref=0):
    """Convert a db level to a linear level of 0-1.

    If maxref is provided, the return value is a proportion of maxref
    """
    return (10.0 ** ((float(db) + 0.0000001 - maxref) / 20.0))

def linear2db(gain, maxref=0):
    """Convert a linear volume level of 0-1 to db.

    if maxref is provided, then gain parameter provided is used
    as a proportion of maxref. This is to allow the use of the
    maxgain level.
    """
    dbdiff = 20.0 * math.log10(float(gain) + 0.000001)
    return min(max(maxref + dbdiff, -99), 99)


class XAPCommError(Exception):
    """ Communications Error """
    pass

class XAPRespError(Exception):
    """ XAP Responded With Error """
    pass

class XAPX00(object):
    """XAPX000 Module."""

    def __init__(self, comPort="/dev/ttyUSB0", baudRate=38400,
                 stereo=0, XAPType=XAP800TYPE):
        """init: no parameters required."""
        _LOGGER.info("XAPX00 version: {}".format(__version__))
        self.comPort      = comPort
        self.baudRate     = baudRate
        self.byteLength   = 8
        self.stopBits     = 1
        self.parity       = "N"
        self.timeout      = 1
        self.stereo       = stereo
        self.XAPType      = XAPType
        self.matrixGeo    = matrixGeo[self.XAPType] 
        self.XAPCMD       = XAP800_CMD if XAPType == XAP800TYPE else XAP400_CMD
        self.connected    = 0
        self.input_range  = range(1, 13)
        self.output_range = range(1, 13)
        self.convertDb    = 1  # translate levels between linear(0-1) and db
        self._lastcall    = time.time()
        self._maxtime     = 60 * 60 * 1  # 1 hour
        self._maxrespdelay = 5
        self._sleeptime   = 0.25
        self._waiting_response = 0
        self.ExpansionChannels = string.ascii_uppercase[string.ascii_uppercase.find('O'):]
        self.ProcessingChannels = string.ascii_uppercase[:string.ascii_uppercase.find('H')]
        self._commlock = Lock()
#        self.connect(check=True)

    def connect(self, check=False):
        """Open serial port and check connection."""
        _LOGGER.info("connect called, shouldn't be")
        if self.connected:
            return
        self.serial = serial.Serial(self.comPort, self.baudRate,
                                    timeout=self.timeout)
        if check:
            _LOGGER.info("Connecting to XAPX00 at " + str(self.baudRate) +
                         " baud...")
            # Ensure connectivity by requesting the UID of the first unit
            self.serial.reset_input_buffer()
            self.serial.write(("%s0 SERECHO 1 %s" % (self.XAPCMD,EOM)).encode())
            self.serial.readlines(3) #clear response
            uid = self.getUniqueId(0)
            _LOGGER.info("Connected, got uniqueID %s", str(uid))
        self.connected = 1

    def disconnect(self):
        """Disconnect from serial port"""
        _LOGGER.info("disconnect called shouldn't be")
        if self.connected:
            self.serial.close()
            self.connected = 0

    def send(self, data):
        """Send the specified data string to the XAP800

        Returns:
            number of bytes sent
        """
        
        _LOGGER.info("Old style send called")
        self._commlock.acquire()
        serialconn = serial.Serial(self.comPort, self.baudRate,
                                   timeout=self.timeout)
        _LOGGER.debug("Sending: %s", data)
        if not testing:
            # self.serial.reset_input_buffer()
            bytessent = self.serial.write(data.encode())
            serialconn.close()
            return bytessent
        else:
            return len(data)

    def readResponse(self, numElements=1, serial_conn=None):
        """Get response from unit.

        Args:
        numElements: How many response components to return,
                     starts from end of resposne

        Returns:
            response string from unit
        """
        if not serial_conn:
            serialconn = serial.Serial(self.comPort, self.baudRate,
                                       timeout=self.timeout)
            self._commlock.acquire()
        else:
            serialconn = serial_conn

        while 1:
            resp = serialconn.readline().decode()
            if len(resp) > 5 and resp[0:5] == "ERROR":
                self._waiting_response = 0
                raise XAPRespError(resp)
            _LOGGER.debug("Response %s" % resp)
            if resp.find('#') > -1:
                break
            if resp == '':
                # nothing coming, have read too many lines
                return None
        if serial_conn is None:
            serialconn.close()
            self._commlock.release()
        respitems = resp.split("#",maxsplit=1)[1].split()
        if numElements == 1:
            return respitems[-1]
        else:
            return respitems[-numElements:]
        
    def XAPCommand(self, command, *args, **kwargs):
        """Call command and return value"""

        self._commlock.acquire()
        serialconn = serial.Serial(self.comPort, self.baudRate,
                                   timeout=self.timeout)

        unitCode = kwargs.get('unitCode',0)
        rtnCount = kwargs.get('rtnCount',1)
        args = [str(x) for x in args]

        xapstr = "%s%s %s %s %s" % ( self.XAPCMD, unitCode, command, " ".join(args),  EOM)
        _LOGGER.debug("sending command: {}".format(xapstr))
        serialconn.write(xapstr.encode())
        # self._waiting_response = 1
        try:
            res = self.readResponse(numElements = rtnCount, serial_conn=serialconn)
        finally:
            self._commlock.release()
            serialconn.close()
        return res

    def test_connection(self):
        self._commlock.acquire()
        serialconn = serial.Serial(self.comPort, self.baudRate,
                                   timeout=self.timeout)
        _LOGGER.info("Connecting to XAPX00 at " + str(self.baudRate) +
                         " baud...")
            # Ensure connectivity by requesting the UID of the first unit
        serialconn.write(("%s0 SERECHO 1 %s" % (self.XAPCMD,EOM)).encode())
        resp = serialconn.readlines() #clear response
        self._commlock.release()
        id = self.getUniqueId(0)
        return isinstance(id,str)
    
    def reset(self):
        """Reset connection."""
        warnings.warn("Clearing Serial Connection")
        self.serial.reset_input_buffer()

    def getUniqueId(self, unitCode=0):
        """Requests the unique ID of the target XAP800.

        This is a hex value preprogrammed at the factory.

        unitCode - the unit code of the target XAP800
        """
        res = self.XAPCommand("UID", unitCode=unitCode)
        return res
    
    @stereo
    def setDecayRate(self, channel, decayRate, unitCode=0):
        """Modify the decay rate for the specified XAP800.

        unitCode:   the unit code of the target XAP800
        channel:    1-8
        decayRate:  the rate of decay
                       1 = slow, 2 = medium, 3 = fast
        """
        # Ensure compliance with level boundary conditions
        if decayRate < 1:
                decayRate = 1
        elif decayRate > 3:
                decayRate = 3
        res = self.XAPCommand("DECAY", channel, decayRate, unitCode=unitCode)
        return int(res)


    @stereo
    def getDecayRate(self, channel, unitCode=0):
        """Request the decay rate for the specified XAP800.

        channel: 1-8
        unitCode - the unit code of the target XAP800
        """
        res = self.XAPCommand("DECAY", channel, unitCode=unitCode)
        return int(res)

    @stereo
    def setEchoCanceller(self, channel, isEnabled=True, unitCode=0):
        """Enable/Disable the echo canceller.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        isEnabled - true to enable the channel, false to disable
        """
        if self.XAPType == XAP800TYPE: ec="AEC"
        else: ec = "EC"
        res = self.XAPCommand(ec, channel, "1" if isEnabled else "0", unitCode=unitCode)
        return int(res)


    @stereo
    def getEchoCanceller(self, channel, unitCode=0):
        """Enable or disable the echo canceller for the channel-unit.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        isEnabled - true to enable the channel, false to disable
        """
        if self.XAPType == XAP800TYPE: ec="AEC"
        else: ec = "EC"
        res = self.XAPCommand(ec, channel, unitCode=unitCode)
        return int(res)

    @stereo
    def getMaxGain(self, channel, group="I", unitCode=0, **kwargs):
        """Get max gain setting for a channel."""
        if group in nogainGroups: #E is expansion, GAIN is set on source unit, so return max
            raise Exception('Gain not available on Expansion Bus')
        resp = self.XAPCommand("MAX", channel, group, unitCode=unitCode)
        if is_number(resp):
            return float(resp)
        else: raise XAPCommError

    @stereo
    def setMaxGain(self, channel, gain, group="I", unitCode=0):
        """Set max gain setting for a channel."""
        if group in nogainGroups: #E is expansion, GAIN is set on source unit, so return max
            raise Exception('Gain not available on Expansion Bus')
        resp = self.XAPCommand("MAX", channel, group, gain, unitCode=unitCode)
        if is_number(resp):
            return resp
        else:
            raise XAPCommError

    @stereo
    def getPropGain(self, channel, group="I", unitCode=0):
        """Get gain level for a channel relative to that channel's maxgain setting

        Returned as linear scale of 0-1, with 1=maxgain
        """
        if group in nogainGroups: #E is expansion, GAIN is set on source unit, so return max
            raise Exception('Gain not available on Expansion Bus')
        maxdb = self.getMaxGain(channel, group=group, unitCode=unitCode,
                                stereo=0)
        resp = self.XAPCommand("GAIN", channel, group, unitCode=unitCode, rtnCount=2)[0]
        if is_number(resp):
            resp = db2linear(resp, maxdb)
            return resp
        else:
            raise XAPCommError("resp={}".format(resp))

    @stereo
    def setPropGain(self, channel, gain, isAbsolute=1, group="I", unitCode=0):
        """Set gain level for a channel relative to that channel's maxgain setting.

        Gain input is on a linear scale of 0-1, with 1=maxgain
        """
        if group in nogainGroups: #E is expansion, GAIN is set on source unit, so return max
            raise Exception('Gain not available on Expansion Bus')
        maxdb = self.getMaxGain(channel, group, unitCode, stereo=0)
        dbgain = linear2db(gain, maxdb)  # if self.convertDb else gain
        dbgain = "{0:.4f}".format(dbgain)
        _LOGGER.debug("setPropGain: linear:{}, max:{}, db:{}".format( gain, maxdb, dbgain)) 
        resp = self.XAPCommand("GAIN", channel, group, dbgain, "A" if isAbsolute == 1 else "R",
                               unitCode=unitCode, rtnCount=2)[0] 
        if is_number(resp):
            return db2linear(resp, maxdb)
        else:
            raise XAPCommError

    @stereo
    def getGain(self, channel, group="I", unitCode=0):
        """Get gain level for a channel/group, 0 - 1"""
        if group == 'E': #E is expansion, GAIN is set on source unit, so return max
            raise Exception('Gain not available on Expansion Bus')
        else:
            resp = self.XAPCommand("GAIN", channel, group, unitCode = unitCode,
                                   rtnCount=2)[0]
        if is_number(resp):
            return db2linear(resp) if self.convertDb else resp
        else:
            raise XAPCommError

    @stereo
    def setGain(self, channel, gain, isAbsolute=1, group="I", unitCode=0):
        """Sets the gain on the specified channel for the specified XAP800.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, A-D, 1-2, or * for all)
        channelType - the type of channel (I for input, O for output,
                                           S for subbus)
        gain - the amount of gain to apply, 0 - 1.
        isAbsolute - true if the level specified is an absolute setting,
                     false if it's a relative movement.
        """
        if group in nogainGroups: #E is expansion, GAIN is set on source unit, so return max
            raise Exception('Gain not available on Expansion Bus')
        gain = linear2db(gain) if self.convertDb else gain
        gain = "{0:.4f}".format(gain)
        if group in ('E'):  # can't set GAIN on expansion bus
            resp = 20.0 # or raise error?????
        resp = self.XAPCommand("GAIN", channel, group, gain, "A" if isAbsolute == 1 else "R",
                               unitCode=unitCode, rtnCount=2)[0]
        if is_number(resp):
            return db2linear(resp) if self.convertDb else resp
        else:
            raise XAPCommError

    @stereo
    def getLevel(self, channel, group="I", stage="I", unitCode=0):
        """Requests the db level of the target channel on the specified XAP800.

        read-only

        channel - the target channel (1-8, A-D)
        unitCode - the unit code of the target XAP800
        group - the target channel type
        stage - See documentation
        """
        resp = self.XAPCommand("LVL", channel, group, stage, unitCode=unitCode)
        if is_number(resp):
            return float(resp)
        else:
            raise XAPCommError

    def getLabel(self, channel, group, unitCode=0):
        """Retrieve the text label assigned to an inpout or ouput"""
        resp = self.XAPCommand("LABEL", channel, group, unitCode=unitCode)
        return resp
    
    @stereo
    def setMatrixRouting(self, inChannel, outChannel, state=1, inGroup="I",
                         outGroup="O", unitCode=0):
        """Set the routing matrix for the target channel.

        unitCode - the unit code of the target XAP800
        inChannel - the target input channel or group (1-25 see
                    page 64 of the XAP800 manual for
                    details)
        outChannel - the target output channel or group (1-25 see
                    page 64 of the XAP800 manual for
                    details)
        inGroup - input group (I-inlts, M=mics, L=line, ...
        outGroup - otput group (O=all Outputs 1-12, P=processing A-H, ...
        state - 0=off, 1=on (line inputs only), 2=toggle (line only),
               3=Non-Gated (mic only), 4=Gated (mic only), Null=currrent mode
        """
        res = self.XAPCommand("MTRX",inChannel, inGroup,
                   outChannel, outGroup, state, unitCode=unitCode)
        return res

    @stereo
    def getMatrixRouting(self, inChannel, outChannel, inGroup="I",
                         outGroup="O", unitCode=0, **kwargs):
        """Gets the routing matrix for the target channel

        Args:
            unitCode - the unit code of the target XAP800
            inChannel - the target input channel (1-25 see
                    page 64 of the XAP800 manual for
                    details)
            outChannel - a hex value specifying the output mix as a
                 series of bit flags (again, see pg 64).
            inGroup - input group (I-inlts, M=mics, L=line, ...
            outGroup - otput group (O=all Outputs 1-12, P=processing A-H, ...

        Return:
            state: 0=off, 1=on (line inputs only), 2=toggle (line only),
               3=Non-Gated (mic only), 4=Gated (mic only), Null=currrent mode
        """
        resp = self.XAPCommand("MTRX",inChannel, inGroup,
                   outChannel, outGroup, unitCode=unitCode)
        return resp

    def getMatrixRoutingReport(self, unitCode=0):
        """Returns a matrix of levels as a list of lists"""
        routingMatrix = []
        for x in range(0, self.matrixGeo):
            routingMatrix.append([])
            for y in range(0, self.matrixGeo):
                routingMatrix[x].append(self.getMatrixRouting(
                    x + 1, y + 1, unitCode=unitCode, stereo=0))
        return routingMatrix

    @stereo
    def setMatrixLevel(self, inChannel, outChannel, level=0,
                       isAbsolute=1, inGroup="I", outGroup="O", unitCode=0):
        """Sets the matrix level at the crosspoint.

        unitCode:   the unit code of the target XAP800
        inChannel:  the target input channel or group (1-25 see
                    page 64 of the XAP800 manual for
                    details)
        outChannel: the target output channel or group (1-25 see
                    page 64 of the XAP800 manual for
                    details)
        inGroup:  input group (I-inlts, M=mics, L=line, ...
        outGroup: output group (O=all Outputs 1-12, P=processing A-H, ...
        level:    0 - 1
        """
        level = linear2db(level) if self.convertDb else level
        resp = self.XAPCommand("MTRXLVL", inChannel, inGroup,
                   outChannel, outGroup, level, "A" if isAbsolute == 1
                               else "R", unitCode=unitCode, rtnCount=2)[0]

        return db2linear(resp) if self.convertDb else resp

    @stereo
    def getMatrixLevel(self, inChannel, outChannel, inGroup="I",
                       outGroup="O", unitCode=0, **kwargs):
        """
        Gets the matrix level at the crosspoint

        unitCode - the unit code of the target XAP800
        inChannel - the target input channel (1-12 see
                page 64 of the XAP800 manual for
                details)
        outChannel - target output channel, 1-12
        inGroup - input group (I-inlts, M=mics, L=line, ...
        outGroup - otput group (O=all Outputs 1-12, P=processing A-H, ...
        """
        resp = self.XAPCommand("MTRXLVL", inChannel, inGroup,
                   outChannel, outGroup, unitCode=unitCode, rtnCount=2)[0]

        return db2linear(resp) if self.convertDb else resp

    def getMatrixLevelReport(self, unitCode=0):
        """Returns a matrix of levels as a list of lists"""
        levelMatrix = []
        for x in range(0, self.matrixGeo):
            levelMatrix.append([])
            for y in range(0, self.matrixGeo):
                levelMatrix[x].append(self.getMatrixLevel(
                    x + 1, y + 1, unitCode=unitCode, stereo=0))
        return levelMatrix

    @stereo
    def setMute(self, channel, isMuted=1, group="I", unitCode=0):
        """Mutes the target channel on the specified XAP800.

        Args:
        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8 or *, A-D, 1-2)
        group -   I=Input, O=Output, M=Mike, etc
        isMuted - 1 to mute, 0 to unmute, 2 to toggle
        """
        resp = self.XAPCommand("MUTE", channel, group, str(isMuted),
                   unitCode=unitCode)
        return int(resp)

    @stereo
    def getMute(self, channel, group="I", unitCode=0):
        """Mutes the target channel on the specified XAP800.

        Args:
        unitCode - the unit code of the target XAP800
       channel - the target channel (1-8 or *, A-D, 1-2)
        group -   I=Input, O=Output, M=Mike, etc
        Return:
            isMuted - 1=muted, 0 = unmute
        """
        resp = self.XAPCommand("MUTE", channel, group, unitCode=unitCode)
        return int(resp)

    @stereo
    def setRamp(self, channel, group, rate, target, unitCode=0):
        """Ramp the gain.

        Ramp gain at the specified rate in db/sec to the target
        or to max / min if target is blank (space).
        """
        resp = self.XAPCommand("Ramp", channel, group, rate, target, unitCode=unitCode)
        return int(resp)

#  below here not tested very much ###
    def setAdaptiveAmbient(self, channel, group="M", isEnabled=1, unitCode=0):
        """Modifies the state of the adaptive ambient for the specified microphone(s).

        Args:
            unitCode - the unit code of the target XAP800
            channel - 1-8 for specific mic channel, or * for all mics
            group:  "M"ike, "O"utput, "I"nput, etc
            isEnabled - 1 to enable, 0 to disable
        """
        resp = self.XAPCommand("AAMB", channel, group, ("1" if isEnabled else "0"), unitCode=unitCode)
        return int(resp)

    def getAdaptiveAmbient(self, channel, group="M", unitCode=0):
        """Requests the state of the adaptive ambient for the specified mic(s).

        unitCode - the unit code of the target XAP800
        channel - 1-8 for specific mic channel, or * for all mics
        """
        resp = self.XAPCommand("AAMB", channel, group, unitCode=unitCode)
        return int(resp)

    def setAutoGainControl(self, channel, isEnabled, group="I", unitCode=0):
        """Modifies the state of the automatic gain control (AGC) for the mic(s).

        unitCode - the unit code of the target XAP800
        channel - 1-8 for specific mic channel, or * for all mics
        isEnabled - true to enable, false to disable
        """
        resp = self.XAPCommand("AGC", channel, group, ("1" if isEnabled else "0"), unitCode=unitCode)
        return bool(resp)

    def getAutoGainControl(self, channel, group="I", unitCode=0):
        """Requests the state of the automatic gain control (AGC) for the mic.

        unitCode - the unit code of the target XAP800
        channel - 1-8 for specific mic channel, or * for all mics
        """
        resp = self.XAPCommand("AGC", channel, group, unitCode=unitCode)
        return bool(resp)
    
    def setAmbientLevel(self, channel, levelInDb, unitCode=0):
        """Sets the fixed ambient level of the specified XAP800.

        This value will only be set if adaptive ambient is disabled.

        Args:
            unitCode - the unit code of the target XAP800
            levelInDb - the ambient level in dB (0 to -70)
        """
        # Ensure compliance with level boundary conditions
        if levelInDb > 0:
                levelInDb = 0
        elif levelInDb < -70:
                levelInDb = -70
        resp = self.XAPCommand("AMBLVL", channel, levelInDb, unitCode=unitCode)
        return float(resp)

    def getAmbientLevel(self, channel, unitCode=0):
        """Requests the fixed ambient level of the specified XAP800.

        Args:
            channel: mic channel 1-9
            unitCode - the unit code of the target XAP800
        """
        resp = self.XAPCommand("AMBLVL", channel, unitCode=unitCode)
        return float(resp)

    def getSetBaudRate(self, baudRate, unitCode=0):
        """Set the baud rate for the RS232 port on the specified XAP800.

        unitCode - the unit code of the target XAP
        baudRate - the baud rate (9600, 19200, or 38400)
        """
        if self.XAPType == XAP400TYPE:
            baudRateCode = {9600: 1, 19200: 2, 38400: 3, " ": " ", "":" "}
            rateBaudCode = {v: k for k, v in baudRateCode.items()}
            baud = baudRateCode.get(baudRate,3)
        else:
            baud = baudRate
        res = self.XAPCommand("BAUD", baud, unitCode=unitCode)
        if self.XAPType == XAP400TYPE:
            res = rateBaudCode.get(res, 0)
        return res

    def setChairmanOverride(self, channel, isEnabled=0, unitCode=0):
        """Modifies the state of the chairman override for the specified microphone(s).

        unitCode - the unit code of the target XAP800
        channel - 1-8 for specific mic channel, or * for all mics
        isEnabled - 0=off, 1=om, 2=toggle
        """
        resp = self.XAPCommand("CHAIRO", channel, "1" if isEnabled else "0", unitCode=unitCode)
        return int(resp)

    def getChairmanOverride(self, channel, isEnabled=0, unitCode=0):
        """Modifies the state of the chairman override a microphone(s).

        unitCode - the unit code of the target XAP800
        channel - 1-8 for specific mic channel, or * for all mics
        isEnabled - 0=off, 1=om, 2=toggle
        """
        resp = self.XAPCommand("CHAIRO", channel, unitCode=unitCode)
        return int(resp)

    def setPreset(self, preset, state=1, unitCode=0):
        """Run the preset.

        state: 0: state=off
        state: 1: execute preset, set state=on
        state: 2: execute preset, set state=off
        """
        resp = self.XAPCommand("PRESET", state, unitCode=unitCode)
        return int(resp)

    def getPreset(self, preset, state=1, unitCode=0):
        """Get the preset state"""
        resp = self.XAPCommand("PRESET", unitCode=unitCode)
        return int(resp)

    def usePreset(self, preset, unitCode=0):
        """Cause the XAP800 to swap its settings for the given preset.

        Args:
            unitCode - the unit code of the target XAP800
            preset - the preset to switch to (1-6)
        """
        resp = self.XAPCommand("PRESET", preset, unitCode=unitCode)
        return int(resp)

    def requestPreset(self, preset, unitCode=0):
        """Request the current preset in use for the specified XAP800.

        Args:
            unitCode - the unit code of the target XAP800
        """
        resp = self.XAPCommand("PRESET", unitCode=unitCode)
        return int(resp)

    def getEchoReturnLoss(self, channel, unitCode=0):
        """Request the status of the echo return loss.

        unitCode - the unit code of the target XAP800
        channel - the target channel
        """
        resp = self.XAPCommand("ERL", channel, unitCode=unitCode)
        return int(resp)

    def getEchoReturnLossEnhancement(self, channel, unitCode=0):
        """Request the status of the echo return loss enhancement.

        unitCode - the unit code of the target XAP800
        channel - the target channel
        """
        resp = self.XAPCommand("ERLE", channel, unitCode=unitCode)
        return int(resp)

    def enableEqualizer(self, channel, isEnabled=True, unitCode=0):
        """Enable or disable the equalizer for channel-unit.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        isEnabled - true to enable the channel, false to disable
        """
        resp = self.XAPCommand("EQ", channel, "1" if isEnabled else "0", unitCode=unitCode)
        return bool(resp)

    def setDefaultMeter(self, channel, isInput, unitCode=0):
        """Modify the default meter.

        unitCode - the unit code of the target XAP800
        channel - the input or output to set as the meter
                  (1-8 or A-D)
        isInput - true if the channel is an input, false
                  if the channel is an output.
        """
        self.send(XAP800_CMD + unitCode + " DFLTM " + channel + " " +
                  ("I" if isInput else "O") + EOM)
        return int(self.readResponse())

    def getDefaultMeter(self, unitCode=0):
        """Request the default meter.

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " DFLTM " + EOM)
        return int(self.readResponse())

    def toggleEqualizer(self, channel, unitCode=0):
        """Toggle the equalizer for the specified channel-unit.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        """
        self.send(XAP800_CMD + unitCode + " EQ " + channel + " 2" + EOM)
        return int(self.readResponse())

    def requestEqualizer(self, channel, unitCode=0):
        """Request the status of the equalizer for channel.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        """
        self.send(XAP800_CMD + unitCode + " EQ " + channel + EOM)
        return int(self.readResponse())

    def enableHardwareFlowControl(self, isEnabled=True, unitCode=0):
        """Enables or disables hardware flow control for the specified XAP800.

        unitCode - the unit code of the target XAP800
        isEnabled - true to enable the channel, false to disable
        """
        self.send(XAP800_CMD + unitCode + " FLOW " +
                  ("1" if isEnabled else "0") + EOM)
        return int(self.readResponse())

    def requestHardwareFlowControl(self, unitCode=0):
        """Request the status of the hardware flow control for the specified XAP800.

        unitCode - the unit code of the target XAP800
        isEnabled - true to enable the channel, false to disable
        """
        self.send(XAP800_CMD + unitCode + " FLOW " + EOM)
        return int(self.readResponse())

    def enableFirstMicPriorityMode(self, isEnabled=True, unitCode=0):
        """Enable or disable first microphone priority mode.

        unitCode - the unit code of the target XAP800
        isEnabled - true to enable the channel, false to disable
        """
        self.send(XAP800_CMD + unitCode + " FMP " +
                  ("1" if isEnabled else "0") + EOM)
        return int(self.readResponse())

    def requestFirstMicPriorityMode(self, unitCode=0):
        """Request the first microphone priority mode.

        unitCode - the unit code of the target XAP800
        isEnabled - true to enable the channel, false to disable
        """
        self.send(XAP800_CMD + unitCode + " FMP " + EOM)
        return int(self.readResponse())

    def setFrontPanelPasscode(self, passcode, unitCode=0):
        """Set the front panel passcode for the specified XAP800.

        unitCode - the unit code of the target XAP800
        passcode - the passcode to use for the front panel
        """
        self.send(XAP800_CMD + unitCode + " FPP " + passcode + EOM)
        return int(self.readResponse())

    def requestFrontPanelPasscode(self, unitCode=0):
        """Request the front panel passcode for the specified XAP800.

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " FPP " + EOM)
        return int(self.readResponse())

    def requestGate(self, unitCode):
        """Request the gating status for the specified XAP800.

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " GATE " + EOM)
        return int(self.readResponse())

    def setGatingMode(self, channel, mode, unitCode=0):
        """Set the gating mode on the specified channel for the specified XAP800.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        mode - the gating mode
               1 = AUTO
               2 = MANUAL ON
               3 = MANUAL OFF
               4 = OVERRIDE ON
               5 = OVERRIDE OFF
        """
        self.send(XAP800_CMD + unitCode + " GMODE " + channel + " " +
                  mode + EOM)
        return int(self.readResponse())

    def requestGatingMode(self, channel, unitCode=0):
        """Request the gating mode on the specified channel for the specified XAP800.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        """
        self.send(XAP800_CMD + unitCode + " GMODE " + channel + EOM)
        return int(self.readResponse())

    def setGateRatio(self, gateRatioInDb, unitCode=0):
        """Set the gating ratio for the specified XAP800.

        The ratio is limited to 0-50 values outside the boundary
        will be anchored to the boundaries.

        unitCode - the unit code of the target XAP800
        gateRatioInDb - the gating ratio in dB (0-50)
        """
        # Ensure compliance with level boundary conditions
        if gateRatioInDb < 0:
            gateRatioInDb = 0
        elif gateRatioInDb > 50:
            gateRatioInDb = 50
        self.send(XAP800_CMD + unitCode + " GRATIO " + " " +
                  gateRatioInDb + EOM)
        return float(self.readResponse())

    def requestGateRatio(self, unitCode=0):
        """Request the gating ratio for the specified XAP800.

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " GRATIO" + EOM)
        return float(self.readResponse())

    def setHoldTime(self, holdTimeInMs, unitCode=0):
        """Set the hold time for the specified XAP800.

        The value is limited to 100-8000 values outside the boundary
        will be anchored to the boundaries.

        unitCode - the unit code of the target XAP800
        holdTimeInMs - the hold time in milliseconds (100-8000)
        """
        # Ensure compliance with level boundary conditions
        holdTimeInMs  = max(min(holdTimeInMs, 8000), 100)
        self.send(XAP800_CMD + unitCode + " HOLD " + holdTimeInMs + EOM)
        return int(self.readResponse())

    def setFrontPanelLock(self, isLocked=True, unitCode=0):
        """Lock or unlock the front panel of the specifid XAP800

        unitCode - the unit code of the target XAP800
        isLocked - true to lock, false to unlock
        """
        self.send(XAP800_CMD + unitCode + " LFP " +
                  ("1" if isLocked else "0") + EOM)
        return bool(self.readResponse())

    def toggleFrontPanelLock(self, unitCode=0):
        """Toggles the front panel lock of the specified XAP800

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " LFP 2" + EOM)
        return int(self.readResponse())

    def requestFrontPanelLock(self, unitCode):
        """
        Requests the front panel lock status of the specified XAP800

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " LFP" + EOM)
        return int(self.readResponse())

    def setLastMicOnMode(self, mode, unitCode=0):
        """Set the last microphone on mode of the specified XAP800

        unitCode - the unit code of the target XAP800
        mode - the last mic on mode
                                 0 = OFF
                                 1 = Microphone #1
                                 2 = Last Microphone On
        """
        self.send(XAP800_CMD + unitCode + " LMO " + mode + EOM)
        return int(self.readResponse())

    def requestLastMicOnMode(self, unitCode=0):
        """Request the last microphone on mode of the specified XAP800

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " LMO" + EOM)
        return int(self.readResponse())

    def setMasterMode(self, mode, unitCode=0):
        """Set the master mode of the specified XAP800.

        unitCode - the unit code of the target XAP800
        mode - the master mode
                                 1 = Master Single
                                 2 = Dual Mixer
                                 3 = Slave
                                 4 = Master Linked
        """
        self.send(XAP800_CMD + unitCode + " MASTER " + mode + EOM)
        return int(self.readResponse())

    def requestMasterMode(self, unitCode=0):
        """Request the master mode of the specified XAP800.

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " MASTER " + EOM)
        return int(self.readResponse())

    def enableModemMode(self, unitCode, isEnabled=True):
        """Enable or disable the modem mode of the specified XAP800

        unitCode - the unit code of the target XAP800
        isEnabled - true to enable, false to disable
        """
        self.send(XAP800_CMD + unitCode + " MDMODE " +
                  ("1" if isEnabled else "0") + EOM)
        return bool(self.readResponse())

    def setMicEqualizerAdjustment(self, unitCode, channel, band, eqValue):
        """Set the microphone equalizer adjustment of the target channel.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        band - H=High, M=Medium, L=Low
        eqValue - the eq value (-12 to 12)
        """
        self.send(XAP800_CMD + unitCode + " MEQ " + channel + " " + band +
                  " " + eqValue + EOM)
        return float(self.readResponse())

    def requestMicEqualizerAdjustment(self, channel, band, unitCode=0):
        """Requests the microphone equalizer adjustment of the target channel.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        band - H=High, M=Medium, L=Low
        """
        self.send(XAP800_CMD + unitCode + " MEQ " + channel + " " + band + EOM)
        return float(self.readResponse())

    def enableMicHighPassFilter(self, channel, isEnabled=True, unitCode=0):
        """Enable or disables the microphone high pass filter of the target channel.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        isEnabled - true to enable, false to disable
        """
        self.send(XAP800_CMD + unitCode + " MHP " + channel + " " +
                  ("1" if isEnabled else "0") + EOM)
        return bool(self.readResponse())

    def requestMicHighPassFilter(self, channel, unitCode=0):
        """Request the microphone high pass filter of the target channel.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        """
        self.send(XAP800_CMD + unitCode + " MHP " + channel + " " + EOM)
        return bool(self.readResponse())

    def setModemInitString(self, initString, unitCode=0):
        """Set the modem initialization string of the specified XAP800.

        unitCode - the unit code of the target XAP800
        initString - the string to use when initializing the modem
        """
        self.send(XAP800_CMD + unitCode + " MINIT " + initString + EOM)
        return self.readResponse()

    def setMicInputGain(self, unitCode, channel, gain):
        """Set the microphone input gain for the target channel.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        gain - the gain value to use
                                 1 = 55dB
                                 2 = 25dB
                                 3 = 0dB (line level)
        """
        self.send(XAP800_CMD + unitCode + " MLINE " + channel +
                  " " + gain + EOM)
        return float(self.readResponse())

    def requestMicInputGain(self, channel, unitCode=0):
        """Request the microphone input gain for the target channel.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        """
        self.send(XAP800_CMD + unitCode + " MLINE " + channel + EOM)
        return float(self.readResponse())

    def setMaxActiveMics(self, maxMics, unitCode=0):
        """Set the maxmimum number of active microphones.

        unitCode - the unit code of the target XAP800
        maxMics - the maximum number of mics that will be active
              at the same time (1-8, or 0 for no limit)

        """
        if maxMics < 0:
            maxMics = 0
        elif maxMics > 8:
            maxMics = 8
        self.send(XAP800_CMD + unitCode + " MMAX " + maxMics + EOM)
        return int(self.readResponse())

    def requestMaxActiveMics(self, unitCode=0):
        """Request the maxmimum number of active microphones

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " MMAX" + EOM)
        return int(self.readResponse())

    def setModemModePassword(self, modemPassword, unitCode=0):
        """Set the modem password for the specified XAP800.

        unitCode - the unit code of the target XAP800
        modemPassword - the modem password
        """
        self.send(XAP800_CMD + unitCode + " MPASS " + modemPassword + EOM)
        return self.readResponse()

    def setMicEchoCancellerReference(self, channel, ecRef, unitCode=0):
        """Set the microphone echo canceller reference channel

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        ecRef - the reference channel
                1 = EC Ref 1
                2 = EC Ref 2
        """
        self.send(XAP800_CMD + unitCode + " MREF " + channel +
                  " " + ecRef + EOM)
        return int(self.readResponse())

    def requestMicEchoCancellerReference(self, channel, unitCode=0):
        """Request the microphone echo canceller reference channel.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        """
        self.send(XAP800_CMD + unitCode + " MREF " + channel + EOM)
        return int(self.readResponse())

    def setNonlinearProcessingMode(self, channel, nlpMode, unitCode=0):
        """Set the nonlinear processing (NLP) mode of the target channel.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        nlpMode - the NLP mode to use
            0 = OFF
            1 = Soft
            2 = Medium
            3 = Aggressive
        """
        self.send(XAP800_CMD + unitCode + " NLP " + channel + " " +
                  nlpMode + EOM)
        return int(self.readResponse())

    def requestNonlinearProcessingMode(self, channel, unitCode=0):
        """Request the nonlinear processing (NLP) mode of the channel-unit.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        """
        self.send(XAP800_CMD + unitCode + " NLP " + channel + EOM)
        return int(self.readResponse())

    def enableNumberOpenMicsAttenuation(self, channel, isEnabled=True,
                                        unitCode=0):
        """Enable or disable NOM attenuation for the target channel-unit.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        isEnabled - true to enable NOM, false to disable
        """
        self.send(XAP800_CMD + unitCode + " NOM " + channel + " " +
                  ("1" if isEnabled else "0") + EOM)
        return bool(self.readResponse())

    def requestNumberOpenMicsAttenuation(self, channel, unitCode=0):
        """Request the NOM attenuation for the target channel.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        """
        self.send(XAP800_CMD + unitCode + " NOM " + channel + EOM)
        return float(self.readResponse())

    def setOffAttenuation(self, attenuation, unitCode=0):
        """Set the off attenuation for the specified XAP800.

        The value is limited to 0-50 values outside the boundary
        will be anchored to the boundaries.

        unitCode - the unit code of the target XAP800
        attenuation - the attenuation in dB (0-50)
        """
        if attenuation < 0:
            attenuation = 0
        elif attenuation > 50:
            attenuation = 50
        self.send(XAP800_CMD + unitCode + " OFFA " + attenuation + EOM)
        return int(self.readResponse())

    def requestOffAttenuation(self, unitCode=0):
        """Request the off attenuation for the specified XAP800.

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " OFFA" + EOM)
        return int(self.readResponse())

    def enablePaAdaptiveMode(self, isEnabled=True, unitCode=0):
        """Enable or disable PA adaptive mode.

        unitCode - the unit code of the target XAP800
        isEnabled - true to enable, false to disable
        """
        self.send(XAP800_CMD + unitCode + " PAA " +
                  ("1" if isEnabled else "0") + EOM)
        return bool(self.readResponse())

    def requestPaAdaptiveMode(self, unitCode=0):
        """Request the PA adaptive mode.

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " PAA" + EOM)
        return self.readResponse()

    def setControlPinCommand(self, pinLocation, command, unitCode=0):
        """Specifies the command to be executed when the GPIO pin/control occurs.

        unitCode - the unit code of the target XAP800
        pinLocation - the pin location (see pg 67 of the
                  XAP800 manual for specifications)
        command - the command to execute (any of LFP,
                                    PRESET, MUTE, GAIN, AGC, EQ, GMODE,
                                          or CHAIRO)
        """
        self.send(XAP800_CMD + unitCode + " PCMD " + pinLocation +
                  " " + command + EOM)
        return self.readResponse()

    def clearControlPinCommand(self, pinLocation, unitCode=0):
        """Clear any commands set for the the GPIO pin/control specified.

        unitCode - the unit code of the target XAP800
        pinLocation - the pin location (see pg 67 of the
                  XAP800 manual for specifications)
        """
        self.setControlPinCommand(unitCode, pinLocation, "CLEAR")
        return int(self.readResponse())

    def requestControlPinCommand(self, pinLocation, unitCode=0):
        """Request the command to be executed when the GPIO pin/control occurs.

        unitCode - the unit code of the target XAP800
        pinLocation - the pin location (see pg 67 of the
                  XAP800 manual for specifications)
        """
        self.send(XAP800_CMD + unitCode + " PCMD " + pinLocation + EOM)
        return self.readResponse()

    def setStatusPinCommand(self, pinLocation, command, unitCode=0):
        """Specifie the command to be executed when the GPIO pin/status occurs.

        unitCode - the unit code of the target XAP800
        pinLocation - the pin location (see pg 67 of the
                  XAP800 manual for specifications)
        command - the command to execute (any of LFP,
                                    PRESET, MUTE, GAIN, AGC, EQ, GMODE,
                                          or CHAIRO)
        """
        self.send(XAP800_CMD + unitCode + " PEVNT " + pinLocation + " " +
                  command + EOM)
        return self.readResponse()

    def clearStatusPinCommand(self, pinLocation, unitCode=0):
        """Clear any commands set for the GPIO pin/status specified.

        unitCode - the unit code of the target XAP800
        pinLocation - the pin location (see pg 67 of the
                  XAP800 manual for specifications)
        """
        self.setStatusPinCommand(unitCode, pinLocation, "CLEAR")
        return int(self.readResponse())

    def requestStatusPinCommand(self, pinLocation, unitCode=0):
        """Request the command to be executed when the GPIO pin/status occurs.

        unitCode - the unit code of the target XAP800
        pinLocation - the pin location (see pg 67 of the
                  XAP800 manual for specifications)
        """
        self.send(XAP800_CMD + unitCode + " PEVNT " + pinLocation + EOM)
        return self.readResponse()

    def enablePhantomPower(self, channel, isEnabled=True, unitCode=0):
        """Enable or disable phantom power for the target channel.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        isEnabled - true to enable, false to disable
        """
        self.send(XAP800_CMD + unitCode + " PP " + channel +
                  ("1" if isEnabled else "0") + EOM)
        return bool(self.readResponse())

    def requestPhantomPower(self, channel, unitCode=0):
        """Request the enable status of phantom power.

        unitCode - the unit code of the target XAP800
        channel - the target channel (1-8, or * for all)
        """
        self.send(XAP800_CMD + unitCode + " PP " + channel + EOM)
        return bool(self.readResponse())

    def setMicEchoCancellerReferenceOutput(self, ecRef, output, unitCode=0):
        """Set the microphone echo canceller reference output

        unitCode - the unit code of the target XAP800
        ecRef - the desired reference channel
                                  1 = EC Ref 1
                                  2 = EC Ref 2
                                  3 = G-Link EC Ref bus
        output - the output channel to reference
        (1-8, A-D, E to select G-Link Ref Bus, or F
              to select NONE)
        """
        self.send(XAP800_CMD + unitCode + " REFSEL " + ecRef + " " +
                  output + EOM)
        return self.readResponse()

    def requestMicEchoCancellerReferenceOutput(self, ecRef, unitCode=0):
        """Request the microphone echo canceller reference output.

        unitCode - the unit code of the target XAP800
        ecRef - the desired reference channel
                                  1 = EC Ref 1
                                  2 = EC Ref 2
                                  3 = G-Link EC Ref bus
        """
        self.send(XAP800_CMD + unitCode + " REFSEL " + ecRef + EOM)
        return int(self.readResponse())

    def setScreenTimeout(self, timeInMinutes, unitCode=0):
        """Sets the screen timeout in minutes.

        The value is limited to 0-50 values outside the boundary
        will be anchored to the boundaries.

        unitCode - the unit code of the target XAP800
        timeInMinutes - the timeout value in minutes (1-15, or 0 to disable)
        """
        if timeInMinutes < 0:
            timeInMinutes = 0
        elif timeInMinutes > 15:
            timeInMinutes = 15
        self.send(XAP800_CMD + unitCode + " TOUT " + timeInMinutes + EOM)
        return int(self.readResponse())

    def requestScreenTimeout(self, unitCode=0):
        """Request the screen timeout in minutes.

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " TOUT" + EOM)
        return int(self.readResponse())

    def requestVersion(self, unitCode=0):
        """Request the firmware version of the target XAP800.

        unitCode - the unit code of the target XAP800
        """
        self.send(XAP800_CMD + unitCode + " VER" + EOM)
        return self.readResponse()

    errorDefs = {"ERROR 1": "Out of Memory",
                 "ERROR 2": "Could not extract a command from\
                 the string received",
                 "ERROR 3": "Unknown Command",
                 "ERROR 4": "N/A - reserved for later use",
                 "ERROR 5": "Invalid parameter",
                 "ERROR 6": "Unrecognized command",
                 "default": "Unknown error - no description found"
                 }

    def getHumanErrorDescription(self, errorMsg):
        """Translates ERROR replies from the XAP800 into a human-readable description of the problem."""
        return self.errorDefs.get(errorMsg, "Unknown Error")
