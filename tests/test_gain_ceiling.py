"""MAXGAIN is enforced here or nowhere.

The hardware does not enforce it: GAIN's only documented bound is the internal
-65..20 range, and MAX carries that same range. So the ceiling means whatever
this library makes it mean. Before these tests it meant nothing on the write
path - setPropGain(gain=10.0) against a 0 dB ceiling wrote +20 dB - and the only
reason that never bit in Home Assistant is that HA validates volume_level into
[0,1] before the entity sees it. That is the caller's property, not this
library's.
"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from XAPX00.XAPX00 import (  # noqa: E402
    XAPX00, GAIN_CLAMP_TOLERANCE_DB, linear2db,
)


class FakeXAP(XAPX00):
    """A XAPX00 with the wire replaced, built without opening anything.

    __init__ calls test_connection(), so it is bypassed deliberately.
    """

    def __init__(self, ceilings, convertDb=True):
        self.stereo = 0
        self.convertDb = convertDb
        self._maxgain_cache = {}
        self._ceilings = ceilings
        self.current_db = 0.0
        self.written = []

    def XAPCommand(self, command, *args, unitCode=0, rtnCount=1):
        if command == "MAX":
            channel, group = args[0], args[1]
            return "{0:.2f}".format(self._ceilings[(unitCode, group, str(channel))])
        if command == "GAIN" and len(args) == 2:      # a read
            return [str(self.current_db)]
        if command == "GAIN":
            channel, group, value, mode = args[0], args[1], args[2], args[3]
            self.written.append({"unit": unitCode, "channel": channel, "group": group,
                                 "db": float(value), "mode": mode})
            return [value]
        raise AssertionError("unexpected command " + command)


def make(ceiling_db=0.0, group="O", channel=7):
    return FakeXAP({(0, group, str(channel)): ceiling_db})


# --- setPropGain ----------------------------------------------------------

def test_a_proportion_above_the_ceiling_is_held_at_it():
    """setPropGain(10.0) against a 0 dB ceiling used to write +20 dB."""
    xap = make(ceiling_db=0.0)
    xap.setPropGain(7, 10.0, group="O", stereo=0)
    assert xap.written[0]["db"] == pytest.approx(0.0, abs=1e-3)


def test_the_overshoot_is_logged(caplog):
    xap = make(ceiling_db=0.0)
    with caplog.at_level("WARNING"):
        xap.setPropGain(7, 1.5, group="O", stereo=0)
    assert "above its 0.00 dB MAXGAIN" in caplog.text
    assert "3.52 dB above" in caplog.text


def test_a_level_below_the_ceiling_is_untouched():
    xap = make(ceiling_db=0.0)
    xap.setPropGain(7, 0.5, group="O", stereo=0)
    assert xap.written[0]["db"] == pytest.approx(-6.02, abs=0.01)


def test_a_channel_exactly_on_its_ceiling_does_not_warn(caplog):
    """The reason the tolerance exists.

    db2linear applies _GAIN_EPSILON, so a channel sitting exactly on its ceiling
    does not read back a clean 1.0. A bare `> ceiling` test would log a 0.00 dB
    overshoot on every startup for a channel that is where it was asked to be.
    """
    xap = make(ceiling_db=0.0)
    with caplog.at_level("WARNING"):
        xap.setPropGain(7, 1.0, group="O", stereo=0)
    assert caplog.text == ""
    assert abs(xap.written[0]["db"]) < GAIN_CLAMP_TOLERANCE_DB


def test_the_ceiling_is_the_channels_own_not_a_constant():
    xap = FakeXAP({(0, "O", "7"): -15.0, (0, "O", "8"): 0.0})
    xap.setPropGain(7, 10.0, group="O", stereo=0)
    xap.setPropGain(8, 10.0, group="O", stereo=0)
    assert xap.written[0]["db"] == pytest.approx(-15.0, abs=1e-3)
    assert xap.written[1]["db"] == pytest.approx(0.0, abs=1e-3)


# --- setGain --------------------------------------------------------------

def test_setgain_is_clamped_too():
    """setGain never consulted MAXGAIN at all before this."""
    xap = make(ceiling_db=-15.0)
    xap.convertDb = False
    xap.setGain(7, -5.0, group="O", stereo=0)
    assert xap.written[0]["db"] == pytest.approx(-15.0, abs=1e-3)


def test_setgain_below_the_ceiling_is_untouched():
    xap = make(ceiling_db=-15.0)
    xap.convertDb = False
    xap.setGain(7, -20.0, group="O", stereo=0)
    assert xap.written[0]["db"] == pytest.approx(-20.0, abs=1e-3)


# --- scope ----------------------------------------------------------------

def test_inputs_are_enforced_as_well_as_outputs():
    """Group I too - #29's case, closed on the write path."""
    xap = FakeXAP({(0, "I", "1"): 0.0})
    xap.setPropGain(1, 10.0, group="I", stereo=0)
    assert xap.written[0]["db"] == pytest.approx(0.0, abs=1e-3)


def test_a_relative_write_is_bounded_too():
    """The path where the ceiling matters most.

    Current -18, asked to go up 6, ceiling -15: the result is -15, not -12.
    """
    xap = make(ceiling_db=-15.0)
    xap.convertDb = False
    xap.current_db = -18.0
    xap.setGain(7, 6.0, isAbsolute=0, group="O", stereo=0)
    assert xap.written[0]["db"] == pytest.approx(-15.0, abs=1e-3)


def test_a_relative_write_becomes_absolute():
    """Which is what makes it safe under XAPCommand's retry.

    A retried RELATIVE write applies its delta twice - current + 2 * delta, back
    over the ceiling. Shrinking the delta does not fix that; resolving it to an
    absolute write does, because the write is then idempotent.
    """
    xap = make(ceiling_db=-15.0)
    xap.convertDb = False
    xap.current_db = -18.0
    xap.setGain(7, 1.0, isAbsolute=0, group="O", stereo=0)
    assert xap.written[0]["mode"] == "A"
    assert xap.written[0]["db"] == pytest.approx(-17.0, abs=1e-3)


def _apply(level_db, wire):
    """What the unit does with one command off the wire."""
    return wire["db"] if wire["mode"] == "A" else level_db + wire["db"]


def test_the_wire_command_is_idempotent_under_retry():
    """The property the conversion buys.

    XAPCommand retries after a telnet no-response, replaying the SAME command -
    so what matters is what a second application of that command does. A relative
    delta applies twice; the resolved absolute does not. This asserts the wire
    command directly rather than calling setGain twice, which would be a second
    user request, not a retry.
    """
    xap = make(ceiling_db=-15.0)
    xap.convertDb = False
    xap.current_db = -18.0
    xap.setGain(7, 1.0, isAbsolute=0, group="O", stereo=0)
    wire = xap.written[0]

    once = _apply(-18.0, wire)
    twice = _apply(once, wire)
    assert once == pytest.approx(-17.0, abs=1e-3)
    assert twice == pytest.approx(once, abs=1e-3), "a retry moved the level again"


def test_an_unresolved_relative_delta_would_have_doubled():
    """Why shrinking the delta is not sufficient - the case being avoided."""
    unresolved = {"db": 1.0, "mode": "R"}
    once = _apply(-18.0, unresolved)
    twice = _apply(once, unresolved)
    assert once == pytest.approx(-17.0)
    assert twice == pytest.approx(-16.0), "this is the overshoot the resolve prevents"


def test_a_relative_write_below_the_ceiling_is_untouched():
    xap = make(ceiling_db=-15.0)
    xap.convertDb = False
    xap.current_db = -30.0
    xap.setGain(7, 6.0, isAbsolute=0, group="O", stereo=0)
    assert xap.written[0]["db"] == pytest.approx(-24.0, abs=1e-3)


def test_the_ceiling_read_is_cached_not_refetched():
    """Enforcement must not add a round trip to the volume path."""
    xap = make(ceiling_db=0.0)
    reads = []
    inner = xap.XAPCommand

    def counting(command, *args, **kwargs):
        if command == "MAX":
            reads.append(args[:2])
        return inner(command, *args, **kwargs)

    xap.XAPCommand = counting
    for _ in range(5):
        xap.setPropGain(7, 0.5, group="O", stereo=0)
    assert len(reads) == 1, f"MAXGAIN re-read {len(reads)} times; cache not used"
