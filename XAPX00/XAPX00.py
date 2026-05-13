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

import asyncio
import logging
import math
import time
import warnings
import string
from functools import wraps

try:
    import serialx
    _SERIALX_AVAILABLE = True
except ImportError:
    serialx = None
    _SERIALX_AVAILABLE = False

try:
    import telnetlib3
    _TELNETLIB3_AVAILABLE = True
except ImportError:
    _TELNETLIB3_AVAILABLE = False

from . import __version__

_LOGGER = logging.getLogger(__name__)
if 0:
    import sys
    out_hdlr = logging.StreamHandler(sys.stdout)
    out_hdlr.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
    out_hdlr.setLevel(logging.DEBUG)
    _LOGGER.addHandler(out_hdlr)
    _LOGGER.setLevel(logging.DEBUG)

# Global Constants
XAP800_CMD  = "#5"
XAP400_CMD  = "#7"
CP880_CMD   = "#1"
CP880T_CMD  = "#D"
CP880TA_CMD = "#H"
XAP800TYPE  = "XAP800"
XAP400TYPE  = "XAP400"
CP880TYPE   = "CP880"
CP880TTYPE  = "CP880T"
CP880TATYPE = "CP880TA"
EOM = "\r"
DEVICE_MAXMICS = "Max Number of Microphones"
matrixGeo = {'XAP800': 12, 'XAP400': 8, 'CP880': 12, 'CP880T': 12, 'CP880TA': 12}
nogainGroups = ('E',)


def stereo(func):
    """Repeat a get/set command for the paired stereo channel."""
    @wraps(func)
    async def stereoFunc(*args, **kwargs):
        if 'stereo' in kwargs:
            _stereo = kwargs.pop('stereo')
        else:
            _stereo = 1
        res = await func(*args, **kwargs)
        if args[0].stereo and _stereo:
            _LOGGER.debug("Stereo Command %s:%s", func.__name__, args)
            largs = list(args)
            if isinstance(largs[1], str):
                largs[1] = chr(ord(largs[1]) + 1)
            else:
                largs[1] = largs[1] + 1
            if func.__name__[3:9] == "Matrix":
                _LOGGER.debug("Matrix Stereo Command %s", func.__name__)
                if isinstance(largs[2], str):
                    largs[2] = chr(ord(largs[2]) + 1)
                else:
                    largs[2] = largs[2] + 1
            res2 = await func(*largs, **kwargs)
            if res != res2:
                _LOGGER.debug("Stereo out of sync %s : %s", res, res2)
                warnings.warn("Stereo out of sync", RuntimeWarning)
        if res is not None:
            return res
    return stereoFunc


def is_number(s):
    """ Returns True if string is a number. """
    if isinstance(s, str):
        return s.replace('.', '', 1).replace('-', '', 1).isdigit()
    return False

def db2linear(db, maxref=0):
    """Convert a db level to a linear level of 0-1."""
    return 10.0 ** ((float(db) + 0.0000001 - maxref) / 20.0)

def linear2db(gain, maxref=0):
    """Convert a linear volume level of 0-1 to db."""
    dbdiff = 20.0 * math.log10(float(gain) + 0.000001)
    return min(max(maxref + dbdiff, -99), 99)


class XAPCommError(Exception):
    """ Communications Error """
    pass

class XAPRespError(Exception):
    """ XAP Responded With Error """
    pass


class XAPX00:
    """ClearOne XAP/Converge mixer controller — async interface."""

    def __init__(self, comPort="/dev/ttyUSB0", baudRate=38400,
                 stereo=0, XAPType=XAP800TYPE,
                 connection_type="serial", telnet_host=None, telnet_port=23,
                 telnet_username="clearone", telnet_password="converge"):
        """Set up attributes.  Call await obj.connect() or use XAPX00.create()."""
        _LOGGER.info("XAPX00 version: %s", __version__)
        self.connection_type  = connection_type
        self.stereo           = stereo
        self.XAPType          = XAPType
        self.matrixGeo        = matrixGeo[XAPType]
        if XAPType == XAP800TYPE:
            self.XAPCMD = XAP800_CMD
        elif XAPType == CP880TYPE:
            self.XAPCMD = CP880_CMD
        elif XAPType == CP880TTYPE:
            self.XAPCMD = CP880T_CMD
        elif XAPType == CP880TATYPE:
            self.XAPCMD = CP880TA_CMD
        else:
            self.XAPCMD = XAP400_CMD

        self.timeout          = 2
        self.connectionLive   = 0
        self.UID              = None
        self.input_range      = range(1, 13)
        self.output_range     = range(1, 13)
        self.convertDb        = 1
        self._lastcall        = time.time()
        self._maxtime         = 60 * 60 * 1
        self._maxrespdelay    = 5
        self._sleeptime       = 0.25
        self.ExpansionChannels   = string.ascii_uppercase[string.ascii_uppercase.find('O'):]
        self.ProcessingChannels  = string.ascii_uppercase[:string.ascii_uppercase.find('H')]
        self._commlock        = asyncio.Lock()
        self._last_attempt    = 0
        self._retry_interval  = 10
        self._reader          = None
        self._writer          = None

        # Serial params
        self.comPort  = comPort
        self.baudRate = baudRate

        # Telnet params
        self.telnet_host     = telnet_host
        self.telnet_port     = telnet_port
        self.telnet_username = telnet_username
        self.telnet_password = telnet_password

        if connection_type == "telnet" and telnet_host is None:
            raise ValueError("telnet_host is required when connection_type='telnet'")
        if connection_type == "serial" and not _SERIALX_AVAILABLE:
            raise ImportError("serialx is required for serial connections: pip install serialx")
        if connection_type == "telnet" and not _TELNETLIB3_AVAILABLE:
            raise ImportError("telnetlib3 is required for telnet connections: pip install telnetlib3")

    @classmethod
    async def create(cls, **kwargs):
        """Async factory — create and connect in one step."""
        obj = cls(**kwargs)
        await obj.connect()
        return obj

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    async def connect(self):
        """Open transport and verify connectivity."""
        if self.connection_type == "telnet":
            await self._open_telnet()
        else:
            await self._open_serial()
        await self.test_connection()

    async def _open_serial(self):
        _LOGGER.debug("Opening serial %s @ %s baud", self.comPort, self.baudRate)
        self._reader, self._writer = await serialx.open_serial_connection(
            url=self.comPort, baudrate=self.baudRate
        )
        _LOGGER.debug("Serial streams ready")

    async def _open_telnet(self):
        _LOGGER.debug("Connecting telnet %s:%s", self.telnet_host, self.telnet_port)
        reader, writer = await asyncio.wait_for(
            telnetlib3.open_connection(self.telnet_host, self.telnet_port),
            timeout=self.timeout,
        )
        self._reader = reader
        self._writer = writer
        _LOGGER.debug("Telnet connected")
        if self.telnet_username:
            await self._telnet_login()

    async def _read_until(self, prompt):
        """Read one character at a time until the given prompt suffix is seen."""
        buf = ''
        while True:
            ch = await asyncio.wait_for(self._reader.read(1), timeout=self.timeout)
            if not ch:
                break
            buf += ch
            if buf.endswith(prompt):
                break
        return buf

    async def _telnet_login(self):
        _LOGGER.debug("telnet: waiting for login prompt")
        prompt = await self._read_until('ser: ')
        _LOGGER.debug("telnet: got login prompt: %r", prompt)
        self._writer.write(self.telnet_username + '\r')
        _LOGGER.debug("telnet: waiting for password prompt")
        prompt = await self._read_until('pass: ')
        _LOGGER.debug("telnet: got password prompt: %r", prompt)
        self._writer.write((self.telnet_password or '') + '\r')
        _LOGGER.debug("telnet: sent password, consuming banner")
        await asyncio.sleep(0.5)
        banner = await asyncio.wait_for(self._reader.read(4096), timeout=1)
        _LOGGER.debug("telnet: post-login banner: %r", banner)

    async def disconnect(self):
        """Close the transport."""
        if self._writer is not None:
            try:
                self._writer.close()
            except Exception:
                pass
            self._reader = None
            self._writer = None
        self.connectionLive = 0

    async def reconnect(self):
        """Tear down and re-establish the connection."""
        await self.disconnect()
        await self.connect()

    # ------------------------------------------------------------------
    # Low-level I/O
    # ------------------------------------------------------------------

    async def _readline(self):
        """Read until \\r (XAP EOM).  Returns str without \\r, or '' on EOF."""
        buf = []
        while True:
            ch = await asyncio.wait_for(self._reader.read(1), timeout=self.timeout)
            if not ch:
                return ''
            # telnetlib3 yields str; serialx yields bytes
            if isinstance(ch, (bytes, bytearray)):
                ch = ch.decode(errors='replace')
            if ch == '\r':
                break
            if ch != '\n':
                buf.append(ch)
        return ''.join(buf)

    async def _write(self, data: bytes):
        if self.connection_type == 'telnet':
            self._writer.write(data.decode())
            await self._writer.drain()
        else:
            self._writer.write(data)
            await self._writer.drain()

    # ------------------------------------------------------------------
    # Protocol
    # ------------------------------------------------------------------

    async def readResponse(self, numElements=1):
        """Read lines until one contains '#'; return the parsed value(s)."""
        blank = 0
        while True:
            resp = await self._readline()
            _LOGGER.debug("readResponse raw: %r", resp)
            if 'ERROR' in resp:
                raise XAPRespError(resp)
            if resp == '':
                blank += 1
                if blank > 2:
                    raise XAPCommError('No response from device')
                continue
            if '#' in resp:
                break
        respitems = resp.split('#', maxsplit=1)[1].split()
        if numElements == 1:
            return respitems[-1]
        return respitems[-numElements:]

    async def XAPCommand(self, command, *args, **kwargs):
        """Send a command and return the parsed response.

        Raises XAPCommError on transport failure, XAPRespError on device error.
        """
        async with self._commlock:
            unitCode = kwargs.get('unitCode', 0)
            rtnCount = kwargs.get('rtnCount', 1)
            str_args = [str(x) for x in args]
            xapstr = "%s%s %s %s %s" % (
                self.XAPCMD, unitCode, command, " ".join(str_args), EOM)
            _LOGGER.debug("sending command: %s", xapstr.strip())
            try:
                await self._write(xapstr.encode())
                try:
                    res = await self.readResponse(numElements=rtnCount)
                except XAPCommError:
                    if self.connection_type == 'telnet':
                        _LOGGER.warning("No response; reconnecting and retrying %s", command)
                        await self.reconnect()
                        await self._write(xapstr.encode())
                        res = await self.readResponse(numElements=rtnCount)
                    else:
                        raise
                self.connectionLive = 1
                return res
            except XAPRespError:
                raise
            except (OSError, TimeoutError, asyncio.TimeoutError) as e:
                self.connectionLive = 0
                raise XAPCommError("Command {} failed: {}".format(command, e)) from e

    async def test_connection(self):
        """Send a UID command and verify a response.  Returns True/False."""
        _LOGGER.debug("test_connection")
        now = time.time()
        if not self.connectionLive and (now - self._last_attempt) < self._retry_interval:
            return False
        self._last_attempt = now
        try:
            str_to_write = "%s0 UID %s" % (self.XAPCMD, EOM)
            _LOGGER.debug("writing: %s", str_to_write.strip())
            await self._write(str_to_write.encode())
            resp = await self.readResponse()
            _LOGGER.debug("test_connection response: %s", resp)
            self.connectionLive = 1
            if self.UID is None:
                self.UID = resp
            _LOGGER.debug("connected, UID: %s", resp)
            return True
        except (XAPCommError, OSError, TimeoutError, asyncio.TimeoutError) as e:
            _LOGGER.debug("test_connection failed: %s", e)
            self.connectionLive = 0
            return False

    async def reset(self):
        """Drain input buffer."""
        if self._reader is not None:
            try:
                await asyncio.wait_for(self._reader.read(4096), timeout=0.1)
            except (asyncio.TimeoutError, TimeoutError):
                pass

    # ------------------------------------------------------------------
    # Device commands
    # ------------------------------------------------------------------

    async def getUniqueId(self, unitCode=0):
        """Return the factory-programmed unique ID (hex string)."""
        return await self.XAPCommand("UID", unitCode=unitCode)

    async def getVersion(self, unitCode=0):
        """Return the firmware version string."""
        return await self.XAPCommand("VER", unitCode=unitCode)

    @stereo
    async def setDecayRate(self, channel, decayRate, unitCode=0):
        """Set decay rate: 1=slow, 2=medium, 3=fast."""
        decayRate = max(1, min(3, decayRate))
        res = await self.XAPCommand("DECAY", channel, decayRate, unitCode=unitCode)
        return int(res)

    @stereo
    async def getDecayRate(self, channel, unitCode=0):
        res = await self.XAPCommand("DECAY", channel, unitCode=unitCode)
        return int(res)

    @stereo
    async def setEchoCanceller(self, channel, isEnabled=True, unitCode=0):
        ec = "AEC" if self.XAPType == XAP800TYPE else "EC"
        res = await self.XAPCommand(ec, channel, "1" if isEnabled else "0", unitCode=unitCode)
        return int(res)

    @stereo
    async def getEchoCanceller(self, channel, unitCode=0):
        ec = "AEC" if self.XAPType == XAP800TYPE else "EC"
        res = await self.XAPCommand(ec, channel, unitCode=unitCode)
        return int(res)

    @stereo
    async def getMaxGain(self, channel, group="I", unitCode=0, **kwargs):
        if group in nogainGroups:
            raise Exception('Gain not available on Expansion Bus')
        resp = await self.XAPCommand("MAX", channel, group, unitCode=unitCode)
        if is_number(resp):
            return float(resp)
        raise XAPCommError

    @stereo
    async def setMaxGain(self, channel, gain, group="I", unitCode=0):
        if group in nogainGroups:
            raise Exception('Gain not available on Expansion Bus')
        resp = await self.XAPCommand("MAX", channel, group, gain, unitCode=unitCode)
        if is_number(resp):
            return resp
        raise XAPCommError

    @stereo
    async def getPropGain(self, channel, group="I", unitCode=0):
        if group in nogainGroups:
            raise Exception('Gain not available on Expansion Bus')
        maxdb = await self.getMaxGain(channel, group=group, unitCode=unitCode, stereo=0)
        resp = (await self.XAPCommand("GAIN", channel, group, unitCode=unitCode, rtnCount=2))[0]
        if is_number(resp):
            return db2linear(resp, maxdb)
        raise XAPCommError("resp={}".format(resp))

    @stereo
    async def setPropGain(self, channel, gain, isAbsolute=1, group="I", unitCode=0):
        if group in nogainGroups:
            raise Exception('Gain not available on Expansion Bus')
        maxdb = await self.getMaxGain(channel, group, unitCode, stereo=0)
        dbgain = "{0:.4f}".format(linear2db(gain, maxdb))
        _LOGGER.debug("setPropGain: linear:%s, max:%s, db:%s", gain, maxdb, dbgain)
        resp = (await self.XAPCommand("GAIN", channel, group, dbgain,
                                      "A" if isAbsolute == 1 else "R",
                                      unitCode=unitCode, rtnCount=2))[0]
        if is_number(resp):
            return db2linear(resp, maxdb)
        raise XAPCommError

    @stereo
    async def getGain(self, channel, group="I", unitCode=0):
        if group == 'E':
            raise Exception('Gain not available on Expansion Bus')
        resp = (await self.XAPCommand("GAIN", channel, group,
                                      unitCode=unitCode, rtnCount=2))[0]
        if is_number(resp):
            return db2linear(resp) if self.convertDb else resp
        raise XAPCommError

    @stereo
    async def setGain(self, channel, gain, isAbsolute=1, group="I", unitCode=0):
        if group in nogainGroups:
            raise Exception('Gain not available on Expansion Bus')
        gain = linear2db(gain) if self.convertDb else gain
        gain = "{0:.4f}".format(gain)
        resp = (await self.XAPCommand("GAIN", channel, group, gain,
                                      "A" if isAbsolute == 1 else "R",
                                      unitCode=unitCode, rtnCount=2))[0]
        if is_number(resp):
            return db2linear(resp) if self.convertDb else resp
        raise XAPCommError

    @stereo
    async def getLevel(self, channel, group="I", stage="I", unitCode=0):
        resp = await self.XAPCommand("LVL", channel, group, stage, unitCode=unitCode)
        if is_number(resp):
            return float(resp)
        raise XAPCommError

    async def getLabel(self, channel, group, unitCode=0):
        return await self.XAPCommand("LABEL", channel, group, unitCode=unitCode)

    @stereo
    async def setMatrixRouting(self, inChannel, outChannel, state=1,
                               inGroup="I", outGroup="O", unitCode=0):
        return await self.XAPCommand("MTRX", inChannel, inGroup,
                                     outChannel, outGroup, state, unitCode=unitCode)

    @stereo
    async def getMatrixRouting(self, inChannel, outChannel,
                               inGroup="I", outGroup="O", unitCode=0, **kwargs):
        return await self.XAPCommand("MTRX", inChannel, inGroup,
                                     outChannel, outGroup, unitCode=unitCode)

    async def getMatrixRoutingReport(self, unitCode=0):
        routingMatrix = []
        for x in range(self.matrixGeo):
            routingMatrix.append([])
            for y in range(self.matrixGeo):
                routingMatrix[x].append(
                    await self.getMatrixRouting(x + 1, y + 1, unitCode=unitCode, stereo=0))
        return routingMatrix

    @stereo
    async def setMatrixLevel(self, inChannel, outChannel, level=0,
                             isAbsolute=1, inGroup="I", outGroup="O", unitCode=0):
        level = linear2db(level) if self.convertDb else level
        resp = (await self.XAPCommand("MTRXLVL", inChannel, inGroup,
                                      outChannel, outGroup, level,
                                      "A" if isAbsolute == 1 else "R",
                                      unitCode=unitCode, rtnCount=2))[0]
        return db2linear(resp) if self.convertDb else resp

    @stereo
    async def getMatrixLevel(self, inChannel, outChannel,
                             inGroup="I", outGroup="O", unitCode=0, **kwargs):
        resp = (await self.XAPCommand("MTRXLVL", inChannel, inGroup,
                                      outChannel, outGroup,
                                      unitCode=unitCode, rtnCount=2))[0]
        return db2linear(resp) if self.convertDb else resp

    async def getMatrixLevelReport(self, unitCode=0):
        levelMatrix = []
        for x in range(self.matrixGeo):
            levelMatrix.append([])
            for y in range(self.matrixGeo):
                levelMatrix[x].append(
                    await self.getMatrixLevel(x + 1, y + 1, unitCode=unitCode, stereo=0))
        return levelMatrix

    @stereo
    async def setMute(self, channel, isMuted=1, group="I", unitCode=0):
        resp = await self.XAPCommand("MUTE", channel, group, str(isMuted), unitCode=unitCode)
        return int(resp)

    @stereo
    async def getMute(self, channel, group="I", unitCode=0):
        resp = await self.XAPCommand("MUTE", channel, group, unitCode=unitCode)
        return int(resp)

    @stereo
    async def setRamp(self, channel, group, rate, target, unitCode=0):
        resp = await self.XAPCommand("Ramp", channel, group, rate, target, unitCode=unitCode)
        return int(resp)

    async def setAdaptiveAmbient(self, channel, group="M", isEnabled=1, unitCode=0):
        resp = await self.XAPCommand("AAMB", channel, group,
                                     "1" if isEnabled else "0", unitCode=unitCode)
        return int(resp)

    async def getAdaptiveAmbient(self, channel, group="M", unitCode=0):
        resp = await self.XAPCommand("AAMB", channel, group, unitCode=unitCode)
        return int(resp)

    async def setAutoGainControl(self, channel, isEnabled, group="I", unitCode=0):
        resp = await self.XAPCommand("AGC", channel, group,
                                     "1" if isEnabled else "0", unitCode=unitCode)
        return bool(resp)

    async def getAutoGainControl(self, channel, group="I", unitCode=0):
        resp = await self.XAPCommand("AGC", channel, group, unitCode=unitCode)
        return bool(resp)

    async def setAmbientLevel(self, channel, levelInDb, unitCode=0):
        levelInDb = max(-70, min(0, levelInDb))
        resp = await self.XAPCommand("AMBLVL", channel, levelInDb, unitCode=unitCode)
        return float(resp)

    async def getAmbientLevel(self, channel, unitCode=0):
        resp = await self.XAPCommand("AMBLVL", channel, unitCode=unitCode)
        return float(resp)

    async def setChairmanOverride(self, channel, isEnabled=0, unitCode=0):
        resp = await self.XAPCommand("CHAIRO", channel,
                                     "1" if isEnabled else "0", unitCode=unitCode)
        return int(resp)

    async def getChairmanOverride(self, channel, unitCode=0):
        resp = await self.XAPCommand("CHAIRO", channel, unitCode=unitCode)
        return int(resp)

    async def setPreset(self, preset, state=1, unitCode=0):
        resp = await self.XAPCommand("PRESET", preset, state, unitCode=unitCode)
        return int(resp)

    async def getPreset(self, preset, unitCode=0):
        resp = await self.XAPCommand("PRESET", preset, unitCode=unitCode)
        return int(resp)

    async def getEchoReturnLoss(self, channel, unitCode=0):
        resp = await self.XAPCommand("ERL", channel, unitCode=unitCode)
        return int(resp)

    async def getEchoReturnLossEnhancement(self, channel, unitCode=0):
        resp = await self.XAPCommand("ERLE", channel, unitCode=unitCode)
        return int(resp)

    async def enableEqualizer(self, channel, isEnabled=True, unitCode=0):
        resp = await self.XAPCommand("EQ", channel, "1" if isEnabled else "0", unitCode=unitCode)
        return bool(resp)

    async def setNonlinearProcessingMode(self, channel, nlpMode, unitCode=0):
        resp = await self.XAPCommand("NLP", channel, nlpMode, unitCode=unitCode)
        return int(resp)

    async def getNonlinearProcessingMode(self, channel, unitCode=0):
        resp = await self.XAPCommand("NLP", channel, unitCode=unitCode)
        return int(resp)

    async def setMaxActiveMics(self, maxMics, unitCode=0):
        maxMics = max(0, min(8, maxMics))
        resp = await self.XAPCommand("MMAX", maxMics, unitCode=unitCode)
        return int(resp)

    async def getMaxActiveMics(self, unitCode=0):
        resp = await self.XAPCommand("MMAX", unitCode=unitCode)
        return int(resp)

    async def enablePhantomPower(self, channel, isEnabled=True, unitCode=0):
        resp = await self.XAPCommand("PP", channel,
                                     "1" if isEnabled else "0", unitCode=unitCode)
        return bool(resp)

    async def getPhantomPower(self, channel, unitCode=0):
        resp = await self.XAPCommand("PP", channel, unitCode=unitCode)
        return bool(resp)

    async def setGatingMode(self, channel, mode, unitCode=0):
        resp = await self.XAPCommand("GMODE", channel, mode, unitCode=unitCode)
        return int(resp)

    async def getGatingMode(self, channel, unitCode=0):
        resp = await self.XAPCommand("GMODE", channel, unitCode=unitCode)
        return int(resp)

    async def setGateRatio(self, gateRatioInDb, unitCode=0):
        gateRatioInDb = max(0, min(50, gateRatioInDb))
        resp = await self.XAPCommand("GRATIO", gateRatioInDb, unitCode=unitCode)
        return float(resp)

    async def getGateRatio(self, unitCode=0):
        resp = await self.XAPCommand("GRATIO", unitCode=unitCode)
        return float(resp)

    async def setHoldTime(self, holdTimeInMs, unitCode=0):
        holdTimeInMs = max(100, min(8000, holdTimeInMs))
        resp = await self.XAPCommand("HOLD", holdTimeInMs, unitCode=unitCode)
        return int(resp)

    async def getSetBaudRate(self, baudRate, unitCode=0):
        if self.XAPType == XAP400TYPE:
            baudRateCode = {9600: 1, 19200: 2, 38400: 3, " ": " ", "": " "}
            rateBaudCode = {v: k for k, v in baudRateCode.items()}
            baud = baudRateCode.get(baudRate, 3)
        else:
            baud = baudRate
        res = await self.XAPCommand("BAUD", baud, unitCode=unitCode)
        if self.XAPType == XAP400TYPE:
            res = rateBaudCode.get(res, 0)
        return res

    errorDefs = {
        "ERROR 1": "Out of Memory",
        "ERROR 2": "Could not extract a command from the string received",
        "ERROR 3": "Unknown Command",
        "ERROR 4": "N/A - reserved for later use",
        "ERROR 5": "Invalid parameter",
        "ERROR 6": "Unrecognized command",
        "default": "Unknown error - no description found",
    }

    def getHumanErrorDescription(self, errorMsg):
        return self.errorDefs.get(errorMsg, "Unknown Error")
