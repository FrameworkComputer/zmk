# Dahlia Input Deck — Vendor Command Protocol

Interface for the EC to query the input-deck STM32 and set BIOS-driven
policy bits. Runs over the existing I²C HID transport that already carries
keyboard, consumer, and LED reports — no new I²C channel needed.

## Transport summary

| | |
|---|---|
| I²C address (STM32) | `0x15` |
| HID Report ID | `0x20` (Feature type) |
| Total report size on wire | 32 bytes (1 byte Report ID + 31 payload bytes) |
| Direction | Bidirectional — EC uses HID `SET_REPORT` to write a command, then HID `GET_REPORT` to read the response |

The vendor collection is declared in the device's HID Report Descriptor
with Usage Page `0xFF00` (Vendor-Defined), Application Usage `0x03`,
Report ID `0x20`, Feature bytes marked `BUFFERED_BYTES` (opaque to
non-Framework hosts, which will parse-and-ignore).

## Command / response frame

Every request the EC writes has the form:

```
byte 0     opcode
byte 1..N  opcode-specific arguments
```

Every response the EC reads has the form:

```
byte 0     opcode  (echoed from the last write)
byte 1     status  (0 = OK; non-zero = error)
byte 2..N  opcode-specific response data
```

Unused bytes in the 31-byte payload are unspecified — the EC should look
at only the bytes each opcode defines.

### Status values

| Value | Meaning |
|-------|---------|
| `0x00` | OK |
| `0x01` | Invalid argument (e.g. missing bytes in the request) |
| `0xFF` | Unknown opcode |

## Sequence per command

Because HID Feature reports are inherently bidirectional, one command is
always **two I²C-HID transactions**:

```
1. EC → device:  SET_REPORT(type=Feature, id=0x20, data=[opcode, args…])
   Device runs the command synchronously and stores the response.
2. EC → device:  GET_REPORT(type=Feature, id=0x20)
   Device returns the last-stored response bytes.
```

The device holds only the **last** response. If the EC sends two `SET`s
without an intervening `GET`, the first response is lost.

Recommended pacing: a few milliseconds between the `SET` and the `GET` is
plenty; the device runs each command in interrupt handler + one worker
kick, well under a millisecond in practice.

## Opcodes

### `0x01 — GET_FW_VERSION`

**Request** (1 byte)
```
[0x01]
```

**Response** (13 bytes)
```
[0x01, 0x00,
 version:u16,  // LE. Same value as HID wVersionID (DT version-id).
               // Nibble-encoded: 0x1234 displays as "12.3.4".
 dirty:u8,     // 0 = clean, 1 = built from a dirty working tree
 hash:u8[8]]   // first 8 bytes of the zmk-private git commit hash
```

### `0x02 — GET_HW_INFO`

**Request** (1 byte)
```
[0x02]
```

**Response** (20 bytes, all little-endian)
```
[0x02, 0x00,
 chip_id:u16,   // STM32 DEV_ID from DBGMCU (e.g. 0x0489 for STM32U073)
 flash_kb:u16,  // Total on-chip flash in KB (from factory FLASHSIZE register)
 ram_kb:u16,    // Total on-chip SRAM in KB (40 for STM32U073: 32K SRAM1 + 8K SRAM2)
 uid:u8[12]]    // 96-bit factory unique ID, byte-for-byte from silicon
```

`chip_id` matches the value in ST's reference manuals (RM0503 §63.7.2 for
STM32U0). `uid` is guaranteed by silicon to be unique per die; suitable
as a per-unit serial number.

### `0x10 — SET_SWAP_FN_LCTRL`

Enable or disable the Fn/LCtrl key swap.

**Request** (2 bytes)
```
[0x10, value:u8]
```

- `value = 0` → swap disabled (default). Physical LCtrl sends LCtrl,
  physical Fn activates the FN layer.
- `value = 1` → swap enabled. Physical LCtrl activates the FN layer,
  physical Fn sends LCtrl.

**Response** (2 bytes)
```
[0x10, status]
```

- `status = 0x00` on success.
- `status = 0x01` if the request was less than 2 bytes.

**Persistence**: **none**. The setting lives in the STM32's RAM only.
Cold boot / kbreset restores it to `0`. If the BIOS-owned setting is
persistent across reboots, the EC must re-apply it every time the input
deck (re-)enumerates.

### `0x11 — GET_SWAP_FN_LCTRL`

Query the current swap state.

**Request** (1 byte)
```
[0x11]
```

**Response** (3 bytes)
```
[0x11, 0x00, value:u8]
```

- `value = 0` → swap is disabled.
- `value = 1` → swap is enabled.

### `0x12 — SET_FLAGS`

Change some of the device's state bits.

**Request** (9 bytes, little-endian)
```
[0x12, mask:u32, value:u32]
```

Only the bits set in `mask` are touched; their new values come from
`value`. Bits outside `mask` are left alone, so a caller never has to
read-modify-write — which matters, because the EC issues vendor writes
of its own and the device stores only the most recent response.

**Response** (6 bytes)
```
[0x12, status, flags:u32]
```

`flags` is the resulting state, reported whether or not the set
succeeded, so no follow-up `GET_FLAGS` is needed.

- `status = 0x00` on success.
- `status = 0x01` if the request was short, or if `mask` names a bit that
  is unknown or read-only. Nothing is changed in that case.
- `status = 0x02` if the device failed to apply the change.

Setting `FN_LOCK` writes NVS. That is normally a fast append, but an
occasional garbage collection erases a page, and this part executes from
flash — so allow for the device NAKing for a few tens of milliseconds
after such a set, and retry. Setting a bit to the value it already has
does nothing and costs no write.

### `0x13 — GET_FLAGS`

Read that state back as one bitmask.

**Request** (1 byte)
```
[0x13]
```

**Response** (14 bytes, little-endian)
```
[0x13, 0x00, present:u32, flags:u32, writable:u32]
```

- `present` — the bits this firmware implements.
- `flags` — their current values. Only meaningful within `present`.
- `writable` — the subset of `present` that `SET_FLAGS` will accept.

`present` is what lets a host support several firmware versions without
a table mapping version to feature set: without it, a bit that exists
and reads 0 is indistinguishable from one that does not exist. Combined
with the `0xFF` "unknown opcode" status, which covers whole commands, a
host can discover everything it needs by asking.

Bit assignments are append-only. A retired setting drops out of
`present`, but its bit is never reused — a host that knows the old
meaning would silently misread the new one. A host should likewise
ignore bits set in `present` that it does not recognise: that is newer
firmware, not an error.

| Bit | Name | Access | Backing |
|-----|------|--------|---------|
| 0 | `FN_LOCK` | R/W | FNLK layer, persisted in NVS under `dahlia/flags` |
| 1 | `FN_CTRL_SWAP` | R/W | RAM only; the same state as `0x10`/`0x11` |
| 2 | `UPDATE_PROTECT` | R/O | misc-flags slot 0 in flash |

`UPDATE_PROTECT` is read-only by design: it is set when the firmware is
built or flashed, never while it runs. See "Update protection" in
`board.c`.

## Example transactions

### Read firmware version
```
EC → dev:  SET_REPORT (Feature, id=0x20)  [0x01]
EC ← dev:  GET_REPORT (Feature, id=0x20)  [0x01, 0x00, 0x13, 0x00, 0x00,
                                           0xa4, 0xb2, 0x75, 0xd3,
                                           0x8c, 0x01, 0x9e, 0x42]
Parse:  version 0x0013 → v0.1.3, clean, hash a4b275d38c019e42
```

> **Note on wire framing** — every byte layout in this doc is the payload
> as your HID API sees it (opcode-first). On the raw I²C-HID wire, both
> directions also carry a 2-byte length prefix and the Report ID byte
> (`0x20`) before your payload — but any standard HID-over-I²C driver
> strips those automatically. Your test code should pass and parse only
> the opcode-first bytes.

### Read chip UID
```
EC → dev:  SET_REPORT (Feature, id=0x20)  [0x02]
EC ← dev:  GET_REPORT (Feature, id=0x20)  [0x02, 0x00,
                                           0x89, 0x04,          // chip_id = 0x0489
                                           0x40, 0x00,          // flash_kb = 64
                                           0x28, 0x00,          // ram_kb = 40
                                           XX XX XX XX XX XX
                                           XX XX XX XX XX XX]   // uid[12]
```

### Read state, then turn Fn Lock on
```
EC → dev:  SET_REPORT (Feature, id=0x20)  [0x13]
EC ← dev:  GET_REPORT (Feature, id=0x20)  [0x13, 0x00,
                                           0x07, 0x00, 0x00, 0x00,   // present:  all three bits
                                           0x04, 0x00, 0x00, 0x00,   // flags:    UPDATE_PROTECT
                                           0x03, 0x00, 0x00, 0x00]   // writable: FN_LOCK|FN_CTRL_SWAP
Parse:  Fn Lock off, swap off, update protected; Fn Lock and swap settable

EC → dev:  SET_REPORT (Feature, id=0x20)  [0x12,
                                           0x01, 0x00, 0x00, 0x00,   // mask  = FN_LOCK
                                           0x01, 0x00, 0x00, 0x00]   // value = FN_LOCK
EC ← dev:  GET_REPORT (Feature, id=0x20)  [0x12, 0x00,
                                           0x05, 0x00, 0x00, 0x00]   // now FN_LOCK|UPDATE_PROTECT
```

### Try to clear update protect (refused)
```
EC → dev:  SET_REPORT (Feature, id=0x20)  [0x12,
                                           0x04, 0x00, 0x00, 0x00,   // mask = UPDATE_PROTECT
                                           0x00, 0x00, 0x00, 0x00]
EC ← dev:  GET_REPORT (Feature, id=0x20)  [0x12, 0x01,
                                           0x05, 0x00, 0x00, 0x00]   // unchanged
```

### Enable and confirm the Fn/LCtrl swap
```
EC → dev:  SET_REPORT (Feature, id=0x20)  [0x10, 0x01]        // enable swap
EC ← dev:  GET_REPORT (Feature, id=0x20)  [0x10, 0x00]        // OK

EC → dev:  SET_REPORT (Feature, id=0x20)  [0x11]              // query
EC ← dev:  GET_REPORT (Feature, id=0x20)  [0x11, 0x00, 0x01]  // enabled
```

After the enable, physical LCtrl activates the FN layer and physical Fn
sends the LCtrl modifier. User-observable check: press physical LCtrl+F1
→ Mute fires (because that's the Fn+F1 binding).

### Probe for unknown opcodes
```
EC → dev:  SET_REPORT (Feature, id=0x20)  [0x99]              // undefined
EC ← dev:  GET_REPORT (Feature, id=0x20)  [0x99, 0xFF]        // "unknown opcode"
```

## Suggested EC test coverage

1. **GET_HW_INFO round-trip** — read `chip_id`, verify it equals `0x0489`
   for STM32U073. Any other value means the wrong device is on the bus.
2. **GET_HW_INFO uid stability** — read UID twice, verify it's identical.
   Read on ten different units, verify they're all distinct.
3. **GET_FW_VERSION** — verify `major`/`minor` match the flashed firmware
   image.
4. **SET_SWAP_FN_LCTRL round-trip** — write `1`, read back via `0x21`,
   assert value is `1`. Write `0`, read back, assert `0`.
5. **Cold-boot persistence check** — enable swap, cold-boot the deck
   (kbreset), read back via `0x21`, assert `0`. Confirms non-persistence.
6. **Physical effect verification** — enable swap, simulate a physical
   LCtrl press-and-release, verify a `MODIFIER FN` / FN-layer-scoped
   scancode appears on the wire (rather than LCtrl). Disable, repeat,
   verify LCtrl comes back.
7. **Unknown opcode robustness** — send `[0xFF]`, verify response is
   `[0xFF, 0xFF]`. Send zero-length command, verify no crash.
8. **Response overwrite** — send two `SET`s back-to-back with no `GET`
   between (`[0x01]` then `[0x02]`), verify a subsequent `GET` returns
   the `0x02` response (fw version response is lost).

## Adding new opcodes (future)

Opcodes `0x03`–`0x0F` are reserved for future device-info / diagnostic
queries. Opcodes `0x14`–`0x7F` are reserved for future config-set/get
pairs. Opcodes `0x80`–`0xFE` are reserved for higher-bandwidth operations
(e.g. keycode remapping, macro upload). Opcode `0xFF` is reserved for
"unknown opcode" (never assign it to a real command).

New opcodes should:
- Keep the same frame layout (opcode, status, data).
- Fit request and response in 31 bytes each (single Feature report).
- Add a matching case to `zmk_i2c_hid_vendor_set_report` in
  `app/boards/framework/dahlia_tp_module/board.c`.

If a future opcode needs more than 31 bytes of payload, we'll need a
multi-report protocol (offset + chunk index in the header). Not
implemented yet — flag it and we'll design it.
