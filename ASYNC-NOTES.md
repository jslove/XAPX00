# async-library — a proof-of-concept that was set aside

This branch is a full async rewrite of XAPX00 (serialx + telnetlib3, every
command method `async def`, `await XAPX00.create(...)` as the factory). It works.
It is **not** the live line — `master` is.

## Why it was stopped

**1. Async is contagious for library users outside Home Assistant.** The whole
point of XAPX00 is that it is a plain Python library for controlling a ClearOne
mixer. This branch forces every caller to have an event loop: a script that said
`xap.setGain(...)` has to become `asyncio.run(...)`. The complexity belongs in
the Home Assistant integration, not in the library that other people use.

**2. The rewrite also dropped ~46 public methods.** Not deliberately — they were
lost by attrition, because `xap_controller` does not call them and so nothing
noticed. Gone here but present on master: `usePreset`, `setMicInputGain`,
`setMicEqualizerAdjustment`, `setMicEchoCancellerReference(Output)`,
`setOffAttenuation`, `setMasterMode`, `setFrontPanelLock`/`Passcode`,
`toggleEqualizer`, `get`/`setDefaultMeter`, the control- and status-pin commands,
the modem-mode set, and most of the `request*` readers. So this is not "the same
library, async" — it is a smaller library that is also async. Two breaking
changes, not one.

**3. Home Assistant never needed it.** Wrapping a synchronous library in
`hass.async_add_executor_job` is the sanctioned HA pattern, not a workaround, and
that is what `xap_controller` master does. Removing the executor hop saves
microseconds against a ~10 ms device round trip — nothing measurable.

If async ever is worth revisiting, the way to do it without breaking anyone is an
async core with thin synchronous wrappers, so both audiences are served from one
codebase.

## State as of 2026-09-03

- Pairs with `xap_controller`'s `async-library` branch, which calls
  `await XAPX00.create(...)` and awaits this library directly, with the executor
  and lock wrappers removed. That branch pins tag `2026.05.13-async`, which is
  this branch's head — the two are installable together as designed.
- **`master` has since moved ahead of this branch** (MAXGAIN caching, the
  db<->linear epsilon fix, version 2026.09.03). Reviving this branch means
  porting that work forward first.
- Locking is internal here (`asyncio.Lock` held inside `XAPCommand`), which is
  why the component's external `_lock` monkey-patch is absent on its side.
