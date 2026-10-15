# 0002 — Raw echo output, no matched filter in the sensor

- Status: Accepted
- Date: 2026-10-15
- REQs: REQ-009, REQ-010

## Context

The reference plugin had a `range_compressed` option that applied the matched filter inside the
sensor. Range compression is a processing step, and doing it in the sensor hides the raw signal from
downstream algorithms.

## Decision

The plugin publishes only the raw received complex baseband signal plus additive noise. The
`range_compressed` field is removed from `SasPing`. The message carries all waveform parameters
(`center_frequency`, `bandwidth`, `pulse_length`, `sample_rate`, `t0`) so downstream can rebuild the
chirp and range-compress.

## Consequences

- Validation scripts (V1–V3) perform their own offline matched filter.
- Slightly more downstream compute; no loss of information.
