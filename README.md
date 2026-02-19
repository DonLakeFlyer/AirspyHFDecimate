# AirspyHFDecimate

Utility that consumes raw complex IQ samples from `airspyhf_rx` via `stdin`, performs a three-stage FIR decimation (factors 8, 5, 5) using Hamming-windowed low-pass filters, and republishes the reduced-rate stream over UDP in the format required by the `uavrt_detection` pipeline.

## Build

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Usage

```
./build/airspyhf_decimator [options]
```

| Option | Default | Description |
| --- | --- | --- |
| `--input-rate <Hz>` | `768000` | Incoming complex sample rate from `airspyhf_rx`. |
| `--shift-khz <kHz>` | `10` | Mix the IQ stream down by this amount before decimation (e.g., 10 kHz shifts a 146 MHz tone to 145.99 MHz). |
| `--frame <samples>` | `1024` | Total complex samples per UDP packet (timestamp + payload). |
| `--chunk <samples>` | `16384` | Number of complex samples pulled from `stdin` per iteration before decimation. |
| `--ip <addr>` | `127.0.0.1` | Destination IPv4 address. |
| `--ports <p0,p1>` | `10000,10001` | Comma-separated UDP ports that each receive identical packets. |
| `--help` |  | Print help text. |

Pipe IQ from `airspyhf_rx`:

```
airspyhf_rx -f 148.525 -a 768000 -r stdout | ./build/airspyhf_decimator --input-rate 768000
```

## Packet format

Each UDP datagram contains exactly `frame` complex `float32` samples:

1. **Sample 0 (timestamp)** – the real and imaginary halves carry the Unix seconds and nanoseconds fields, respectively, by reinterpreting the raw `uint32` values as IEEE-754 floats. This matches how `uavrt_detection` decodes packets (`timeStampSec = typecast(real(header),'uint32')`).
2. **Samples 1..(frame-1)** – IQ payload at the decimated rate (input rate ÷ 200). Samples are interleaved complex floats matching MATLAB's `single` complex layout. With the default `frame = 1024`, the payload provides 1023 IQ samples per packet.

When the first frame is about to be sent, the encoder samples the host wall clock to create `t_0`. All subsequent packet timestamps advance deterministically as $t_n = t_0 + (n \cdot (\text{frame}-1))/F_s$, mirroring the `airspy_channelize` behavior.

The application keeps a running sample counter so that consecutive packets have contiguous timestamps even if the host clock jitters. The timestamp represents the first payload sample in the packet.

## Flow

1. Read little-endian 16-bit signed IQ pairs from `stdin`.
2. Normalize to `[-1, 1)` floats.
3. Mix the complex stream left by `--shift-khz` (default 10 kHz) to dodge the HF DC spur.
4. Run the samples through three cascaded FIR decimators (8×, 5×, 5×) with automatically designed Hamming-window filters.
5. Buffer decimated samples until `frame - 1` IQs are available.
6. Emit a timestamp+payload frame (total `frame` samples) to each configured UDP port.
