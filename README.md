# AirspyHFDecimate

[![CI](https://github.com/DonLakeFlyer/AirspyHFDecimate/actions/workflows/ci.yml/badge.svg)](https://github.com/DonLakeFlyer/AirspyHFDecimate/actions/workflows/ci.yml)

Utility that consumes complex IQ samples from the `airspyhf-zeromq` publisher stream, performs a three-stage FIR decimation (factors 8, 5, 5) using Hamming-windowed low-pass filters, and republishes the reduced-rate stream over UDP in the format required by the `uavrt_detection` pipeline.

## Related repositories

- AirspyHFDecimate: https://github.com/DonLakeFlyer/AirspyHFDecimate
- ZeroMQ IQ source (`airspyhf_zeromq_rx`): https://github.com/DonLakeFlyer/airspyhf-zeromq
- MAVLink controller (`MavlinkTagController2`): https://github.com/DonLakeFlyer/MavlinkTagController2

Protocol, packet-format, timing, and sample-rate assumption changes must be coordinated across all three repositories.

## Build

```
git submodule update --init --recursive
cmake -S . -B build
cmake --build build
```

## Test

```
git submodule update --init --recursive
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## Usage

```
./build/airspyhf_decimator [options]
```

| Option | Default | Description |
| --- | --- | --- |
| `--input-rate <Hz>` | `0` | Expected incoming complex sample rate from ZeroMQ packet headers; `0` auto-learns from first valid packet. |
| `--strict-input-rate` | off | Exit immediately when header/measured sample-rate mismatches exceed `--rate-tol-ppm`. |
| `--shift-khz <kHz>` | `10` | Shift the IQ stream by this amount before decimation: positive values shift up, negative values shift down. |
| `--frame <samples>` | `1024` | Total complex samples per UDP packet (timestamp + payload). |
| `--zmq-endpoint <uri>` | `tcp://127.0.0.1:5555` | ZeroMQ `SUB` endpoint exported by `airspyhf_zeromq_rx`. |
| `--rate-tol-ppm <ppm>` | `5000` | Allowed sample-rate error before warning logs are emitted. |
| `--ip <addr>` | `127.0.0.1` | Destination IPv4 address. |
| `--ports <p0,p1>` | `10000,10001` | Comma-separated UDP ports that each receive identical packets. |
| `--help` |  | Print help text. |

Run with `airspyhf_zeromq_rx`:

```
airspyhf_zeromq_rx -f 148.525 -a 768000 -Z -I 127.0.0.1 -P 5555
./build/airspyhf_decimator --zmq-endpoint tcp://127.0.0.1:5555 --input-rate 768000
```

Where `-f` sets frequency in MHz, `-a` sets sample rate in Hz, `-Z` enables ZeroMQ output, and `-I`/`-P` set the publisher bind host/port.

Auto-learn rate from stream and run strict mismatch enforcement:

```
./build/airspyhf_decimator --zmq-endpoint tcp://127.0.0.1:5555 --strict-input-rate
```

## ZeroMQ input validation

Incoming packets are validated against the `airspyhf-zeromq` wire format header (magic/version/header size/sequence/sample count/payload bytes). The decimator logs:

- malformed packets (invalid header or payload layout),
- dropped packets (sequence gaps),
- out-of-order or duplicate packets,
- bad incoming sample-rate fields (header sample rate outside `--rate-tol-ppm`),
- bad measured incoming sample rates (observed samples/second outside `--rate-tol-ppm`).

## Packet format

Each UDP datagram contains exactly `frame` complex `float32` samples:

1. **Sample 0 (timestamp)** – the real and imaginary halves carry the Unix seconds and nanoseconds fields, respectively, by reinterpreting the raw `uint32` values as IEEE-754 floats. This matches how `uavrt_detection` decodes packets (`timeStampSec = typecast(real(header),'uint32')`).
2. **Samples 1..(frame-1)** – IQ payload at the decimated rate (input rate ÷ 200). Samples are interleaved complex floats matching MATLAB's `single` complex layout. With the default `frame = 1024`, the payload provides 1023 IQ samples per packet.

When the first frame is about to be sent, the encoder samples the host wall clock to create `t_0`. All subsequent packet timestamps advance deterministically as $t_n = t_0 + (n \cdot (\text{frame}-1))/F_s$, mirroring the `airspy_channelize` behavior.

The application keeps a running sample counter so that consecutive packets have contiguous timestamps even if the host clock jitters. The timestamp represents the first payload sample in the packet.

## Flow

1. Receive ZeroMQ packets from `airspyhf_zeromq_rx`.
2. Validate packet header/payload integrity and monitor sequence continuity.
3. Parse interleaved `float32` IQ payload to complex samples.
4. Shift the complex stream by `--shift-khz` (positive = up, negative = down; default 10 kHz) to dodge the HF DC spur.
5. Run the samples through three cascaded FIR decimators (8×, 5×, 5×) with automatically designed Hamming-window filters.
6. Buffer decimated samples until `frame - 1` IQs are available.
7. Emit a timestamp+payload frame (total `frame` samples) to each configured UDP port.
