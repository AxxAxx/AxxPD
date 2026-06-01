# Local modifications to vendored pdsink

Vendored snapshot of https://github.com/pdsink/pdsink (MIT license, see
`pdsink/LICENSE`). Modifications made in this repository are listed here
so the upstream diff is discoverable.

## Current modifications

### `src/pd/dpm.cpp` — RDO EPR capability bit mirror

Upstream `DPM::fill_rdo_flags()` extracts bit 22 of source `PDO1` to mirror
into the RDO's `epr_capable` flag. Per USB-PD 3.1 Table 6.9 (Fixed Supply
PDO – Source), the "EPR Mode Capable" flag is at **bit 23**, not bit 22
(bit 22 is reserved). pdsink's own `PDO_FIXED` union in `data_objects.h`
confirms this. Reading bit 22 always returned 0, so the outgoing RDO never
advertised EPR capability, and strict sources (Anker A2697 observed)
replied to `EPR_Mode(Enter)` with `Enter_Failed, reason = 0x03 (RDO)`.

Patch: read bit 23 instead of bit 22.

This is a candidate for an upstream PR to the pdsink project.

### `src/pd/utils/dobj_utils.cpp` — EPR AVS match_limits unit error

`match_limits()` for `APDO_EPR_AVS` computed implied current as
`pdp * 1000 / mv`, which gives amps (not milliamps).  The correct
formula is `pdp * 1000000 / mv`.  With the old formula, a 140W source
at 23V produced `implied_ma = 6`, failing the `implied_ma >= ma` check
against any non-zero requested current — causing the DPM to silently
skip the AVS PDO and fall back to PDO1 (5V).

Selftest was unaffected because it passes `ma = 0`, which short-circuits
`match_limits` before reaching the buggy formula.

### `src/pd/dpm.cpp` — EPR AVS ma_limit unit error

Same unit bug as above in `get_request_data_object()`: `ma_limit` for
AVS PDOs was computed as `pdp * 1000 / mv` (amps, not milliamps).
Fixed to `pdp * 1000000 / mv`.  Without this fix, the RDO current
would be clamped to ~6 mA instead of the correct ~6000 mA.

Both fixes are candidates for upstream PRs to the pdsink project.

## Non-code additions

- `pdsink/LICENSE` — MIT license text from the upstream repository, kept
  here to satisfy the MIT "include the copyright notice in all copies"
  requirement. Not present in the upstream `src/` tree; copied from the
  project root.
