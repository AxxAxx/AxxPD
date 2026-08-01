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

### `src/pd/pe.cpp` — source caps buffer overflow clamp (2026-08-01)

`PE_SNK_Evaluate_Capability::on_enter_state()` copied
`rx_emsg.size_to_pdo_count()` PDOs into `port.source_caps`
(`etl::vector<uint32_t, MaxPdoObjects>`, capacity 11) with no bounds
check. ETL push/pop checks are compiled out, so a malicious or buggy
charger sending more than 11 PDOs overflowed the vector and corrupted
adjacent `Port` members. The `validate_source_caps()` size check ran
only *after* the overflow had already happened.

Patch: clamp the loop count to `MaxPdoObjects` before copying. Marked
`[AxxPD fix 2026-08-01]`.

### `src/pd/pe.cpp` — tEnterEPR not restarted in EPR entry wait state (2026-08-01)

`PE_SNK_EPR_Mode_Entry_Wait_For_Response` relies solely on
`is_expired(tEnterEPR)` as its timeout, but the previous state
(`PE_SNK_Send_EPR_Mode_Entry`) stops `tEnterEPR` in its `on_exit`, and a
stopped timer never reports expired (`TimerPack` marks it *disabled*,
and `is_expired()` returns false for disabled timers). A source that
ACKed EPR entry (`Enter_Acknowledged`) and then went silent hung EPR
entry forever.

Patch: start `PD_TIMEOUT::tEnterEPR` in the state's `on_enter_state`;
the `on_exit` stop is kept. Marked `[AxxPD fix 2026-08-01]`.

### `src/pd/prl.cpp` — leaked tSenderResponse from RCH chunking (2026-08-01)

The RCH (chunked receive) FSM aliases the PE's `tSenderResponse` timer:
`RCH_Requesting_Chunk::on_enter` stops it and
`RCH_Waiting_Chunk::on_enter` (re)starts it for every chunk, but nothing
stopped it on normal completion or on a chunking error. After a
multi-chunk message (e.g. `EPR_Source_Capabilities`) the timer was left
running while the PE moved on to `PE_SNK_Select_Capability`, whose run
loop checks `is_expired(tSenderResponse)` — the stale timer (33 ms)
could then fire before the source's `Accept` arrived, causing a
premature Hard Reset mid-EPR negotiation.

Patch: stop `PD_TIMEOUT::tSenderResponse` in the RCH completion path
(`RCH_Pass_Up_Message::on_enter_state`) and in the RCH error path
(`RCH_Report_Error::on_enter_state`). In both paths the PE is being
handed either the awaited response or a PRL error notification, so the
response timer is obsolete at that point. Marked
`[AxxPD fix 2026-08-01]`.

All three fixes are candidates for upstream PRs to the pdsink project.

## Non-code additions

- `pdsink/LICENSE` — MIT license text from the upstream repository, kept
  here to satisfy the MIT "include the copyright notice in all copies"
  requirement. Not present in the upstream `src/` tree; copied from the
  project root.
