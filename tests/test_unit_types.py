"""A chain can mix models, and each model answers only to its own type prefix.

Found 2026-09-18: two Converge 880Ts ("#D0", "#D1") gained a plain Converge 880
set to device ID 2. The library sent "#D2 VER" and nothing answered - the 880
listens for "#12". One XAPType per connection cannot express that.
"""

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from XAPX00.XAPX00 import XAPX00, EOM, parseUnitTypes  # noqa: E402


class WireXAP(XAPX00):
    """A XAPX00 whose serial link is a list, built without opening anything."""

    def __init__(self, XAPType="CP880T", unit_types=None):
        # Only what XAPCommand's send path touches.
        import threading
        self.XAPType = XAPType
        self.XAPCMD = {"CP880T": "#D", "CP880": "#1", "XAP800": "#5"}[XAPType]
        self.unit_types = parseUnitTypes(unit_types)
        self._commlock = threading.Lock()
        self._maxgain_cache = {}
        self.connection_type = "serial"
        self.sent = []

        outer = self

        class Conn:
            def open(self): pass
            def close(self): pass
            def write(self, data): outer.sent.append(data.decode())
            def readline(self): return b"OK> #D0 VER 4.4.0.31\r\n"
        self._serialconn = Conn()
        self.connectionLive = 0
        self._lastcall = 0


def test_default_prefix_is_the_connection_type():
    x = WireXAP("CP880T")
    x.XAPCommand("VER", unitCode=1)
    assert x.sent == ["#D1 VER  " + EOM]


def test_listed_unit_gets_its_own_model_prefix():
    x = WireXAP("CP880T", unit_types={2: "CP880"})
    x.XAPCommand("VER", unitCode=2)
    x.XAPCommand("VER", unitCode=0)
    assert x.sent[0].startswith("#12 VER"), x.sent
    assert x.sent[1].startswith("#D0 VER"), x.sent


def test_unit_prefix_accepts_string_keys():
    # Config from a text field arrives as {"2": "CP880"}.
    x = WireXAP("CP880T", unit_types={"2": "CP880"})
    assert x.unitPrefix(2) == "#1"
    assert x.unitPrefix("2") == "#1"


def test_unknown_type_is_rejected_at_construction():
    with pytest.raises(ValueError):
        WireXAP("CP880T", unit_types={2: "CP999"})
