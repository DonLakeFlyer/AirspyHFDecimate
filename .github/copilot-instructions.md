# Repository Instructions

This repository is part of a 3-repo signal pipeline. Keep code and docs aligned across all three:

1. AirspyHFDecimate (this repo)
   - https://github.com/DonLakeFlyer/AirspyHFDecimate
2. ZeroMQ IQ source
   - https://github.com/DonLakeFlyer/airspyhf-zeromq
3. MAVLink controller
   - https://github.com/DonLakeFlyer/MavlinkTagController2

## System contract (cross-repo)

- `airspyhf-zeromq` publishes IQ over ZeroMQ PUB with the documented packet header and float32 IQ payload.
- `AirspyHFDecimate` subscribes to that ZeroMQ stream, validates header/sequence/rate, decimates, and emits UDP packets in the downstream expected format.
- `MavlinkTagController2` consumes downstream telemetry/detection products and must remain compatible with stream timing and packet continuity assumptions.

## Change policy

When changing protocol, timing, sample-rate assumptions, or packet fields in this repo:

- Treat it as a cross-repo contract change.
- Update README and CLI docs in this repo.
- Call out required companion changes in:
  - `airspyhf-zeromq` (publisher side), and/or
  - `MavlinkTagController2` (consumer/controller side).
- Prefer backward-compatible transitions when practical (feature flags, version checks, soft warnings before hard-fail).

## Integration checks to preserve

- Detect and log dropped/out-of-order ZeroMQ packets.
- Detect and log bad incoming sample rates (header and measured).
- Keep decimated output packet framing and timestamp behavior stable unless explicitly versioned.

## Coding focus

- Keep edits minimal and protocol-safe.
- Do not introduce silent behavior changes in stream contracts.
- If ambiguity exists between repos, document assumptions in PR description and README notes.

## Versioning policy

- Any incompatible change to packet fields, framing, timestamp semantics, or required rate assumptions must be explicitly versioned.
- Use soft migration when practical: add support for both old/new behavior first, then deprecate.
- Do not remove backward compatibility in the same change that introduces a new version unless explicitly requested.

## Wire-format and ABI contract

- Treat field sizes/order, integer signedness, endianness, and float format assumptions as a strict ABI contract.
- Avoid compiler-dependent struct packing as the sole serialization mechanism; prefer explicit field serialization/parsing.
- If a layout assumption changes, update docs and call out required publisher/consumer updates.

## Rate and timing invariants

- Preserve stream continuity assumptions: sequence progression, packet cadence, and timestamp monotonicity.
- Keep sample-rate handling explicit: expected rate source, tolerance checks, and strict-mode behavior.
- Any change to timing origin or timestamp interpretation must be documented as contract-impacting.

## Failure behavior matrix

- Define and keep consistent behavior for each anomaly type:
   - malformed packet/header,
   - sequence gap,
   - out-of-order/duplicate packet,
   - sample-rate mismatch (header and measured).
- For each anomaly, specify whether behavior is log-only, packet drop, degrade-mode, or hard-fail.

## Observability requirements

- Maintain stable, machine-parseable counters/keys for continuity and rate anomalies.
- Keep periodic health/performance logging so operators can confirm sustained-rate operation.
- Do not silently suppress repeated errors; use bounded-rate logging with totals.

## Cross-repo change checklist

When changing this repo, verify and document impact on:

- `airspyhf-zeromq`: header version/fields, sequence semantics, timestamp source semantics, sample-rate expectations.
- `MavlinkTagController2`: UDP framing/timestamp assumptions, continuity expectations, downstream timing sensitivity.
- This repo README/CLI docs: new flags, defaults, strict-vs-soft behavior, migration notes.

## Test requirements by change type

- Protocol/wire change: add or update parser/format tests and cross-version compatibility checks.
- Timing/rate change: validate measured rate, timestamp monotonicity, and continuity counters.
- Error-handling change: verify each anomaly path (malformed, dropped, out-of-order, rate mismatch).

## Performance and backpressure constraints

- Avoid unbounded buffering in hot paths.
- Preserve predictable behavior under slow-consumer conditions.
- If introducing retries/recovery logic, ensure it does not hide sustained data loss.

## PR expectations

- Include a cross-repo impact section in PR descriptions.
- State whether behavior is backward compatible and, if not, the migration path.
- Include sample logs or metrics for changed anomaly/health behavior when relevant.
