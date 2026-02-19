#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <complex>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

constexpr double kTotalDecimation = 8.0 * 5.0 * 5.0;
constexpr std::size_t kBytesPerIQ = 4; // 2 * int16
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 6.28318530717958647692;

struct Options {
  double inputRate = 768000.0;
  std::size_t packetSamples = 1024;
  std::size_t chunkSamples = 16384;
  std::string ip = "127.0.0.1";
  std::vector<uint16_t> ports = {10000, 10001};
  double shiftKhz = 10.0;
};

struct ArgsError : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

void printUsage(const char *argv0) {
  std::cerr << "Usage: " << argv0 << " [options]\n"
            << "  --input-rate <Hz>     Incoming IQ sample rate (default 768000)\n"
            << "  --shift-khz <kHz>     Mix signal down by this amount before decimation (default 10)\n"
            << "  --frame <samples>     Total complex samples per UDP packet, including the timestamp (default 1024)\n"
            << "  --chunk <samples>     Complex samples pulled per stdin read (default 16384)\n"
            << "  --ip <addr>           Destination IPv4 address (default 127.0.0.1)\n"
            << "  --ports <p0,p1,...>   Comma-separated UDP ports (default 10000,10001)\n"
            << "  --help                Show this message\n";
}

Options parseArgs(int argc, char **argv) {
  Options opts;
  for (int i = 1; i < argc; ++i) {
    std::string_view arg(argv[i]);
    if (arg == "--help") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--input-rate") {
      if (++i >= argc) {
        throw ArgsError("--input-rate requires a value");
      }
      opts.inputRate = std::stod(argv[i]);
    } else if (arg == "--frame") {
      if (++i >= argc) {
        throw ArgsError("--frame requires a value");
      }
      opts.packetSamples = static_cast<std::size_t>(std::stoul(argv[i]));
    } else if (arg == "--chunk") {
      if (++i >= argc) {
        throw ArgsError("--chunk requires a value");
      }
      opts.chunkSamples = static_cast<std::size_t>(std::stoul(argv[i]));
    } else if (arg == "--ip") {
      if (++i >= argc) {
        throw ArgsError("--ip requires a value");
      }
      opts.ip = argv[i];
    } else if (arg == "--shift-khz") {
      if (++i >= argc) {
        throw ArgsError("--shift-khz requires a value");
      }
      opts.shiftKhz = std::stod(argv[i]);
    } else if (arg == "--ports") {
      if (++i >= argc) {
        throw ArgsError("--ports requires a value");
      }
      opts.ports.clear();
      std::string value(argv[i]);
      std::size_t start = 0;
      while (start < value.size()) {
        std::size_t comma = value.find(',', start);
        auto token = value.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
        if (!token.empty()) {
          opts.ports.push_back(static_cast<uint16_t>(std::stoul(token)));
        }
        if (comma == std::string::npos) {
          break;
        }
        start = comma + 1;
      }
      if (opts.ports.empty()) {
        throw ArgsError("--ports requires at least one port number");
      }
    } else {
      throw ArgsError("Unknown option: " + std::string(arg));
    }
  }
  if (opts.inputRate <= 0.0) {
    throw ArgsError("input-rate must be positive");
  }
  if (opts.packetSamples < 2) {
    throw ArgsError("frame must be at least 2 samples (timestamp + payload)");
  }
  if (opts.chunkSamples == 0) {
    throw ArgsError("chunk size must be positive");
  }
  return opts;
}

std::vector<float> designLowpass(std::size_t taps, float cutoff) {
  if (taps < 3) {
    taps = 3;
  }
  if ((taps % 2) == 0) {
    ++taps;
  }
  constexpr float pi = 3.14159265358979323846f;
  std::vector<float> coeffs(taps);
  const float M = static_cast<float>(taps - 1);
  for (std::size_t n = 0; n < taps; ++n) {
    const float m = static_cast<float>(n) - M / 2.0f;
    const float window = 0.54f - 0.46f * std::cos(2.0f * pi * static_cast<float>(n) / M);
    float sinc = 0.0f;
    if (std::abs(m) < 1e-6f) {
      sinc = 2.0f * cutoff;
    } else {
      sinc = std::sin(2.0f * pi * cutoff * m) / (pi * m);
    }
    coeffs[n] = window * sinc;
  }
  float sum = 0.0f;
  for (float c : coeffs) {
    sum += c;
  }
  if (sum != 0.0f) {
    for (float &c : coeffs) {
      c /= sum;
    }
  }
  return coeffs;
}

class FirDecimator {
 public:
  FirDecimator(int factor, std::size_t taps, float cutoff)
      : factor_(factor), taps_(designLowpass(taps, cutoff)), state_(taps_.size()) {}

  std::vector<std::complex<float>> process(const std::vector<std::complex<float>> &input) {
    std::vector<std::complex<float>> output;
    if (factor_ <= 0 || taps_.empty()) {
      return output;
    }
    output.reserve(input.size() / factor_ + 1);
    for (const auto &sample : input) {
      state_[writeIndex_] = sample;
      writeIndex_ = (writeIndex_ + 1) % state_.size();
      phase_ = (phase_ + 1) % factor_;
      if (phase_ == 0) {
        std::complex<float> acc{0.0f, 0.0f};
        std::size_t idx = writeIndex_;
        for (std::size_t k = 0; k < taps_.size(); ++k) {
          idx = (idx == 0) ? state_.size() - 1 : idx - 1;
          acc += state_[idx] * taps_[k];
        }
        output.push_back(acc);
      }
    }
    return output;
  }

 private:
  int factor_;
  std::vector<float> taps_;
  std::vector<std::complex<float>> state_;
  std::size_t writeIndex_ = 0;
  int phase_ = 0;
};

class FrequencyShifter {
 public:
  FrequencyShifter(double sampleRate, double shiftHz)
      : shiftHz_(shiftHz) {
    if (sampleRate <= 0.0) {
      sampleRate = 1.0;
    }
    step_ = (shiftHz_ == 0.0) ? 0.0 : (-kTwoPi * shiftHz_ / sampleRate);
  }

  void mix(std::vector<std::complex<float>> &samples) {
    if (shiftHz_ == 0.0 || samples.empty()) {
      return;
    }
    for (auto &sample : samples) {
      const auto c = static_cast<float>(std::cos(phase_));
      const auto s = static_cast<float>(std::sin(phase_));
      sample *= std::complex<float>(c, s);
      phase_ += step_;
      if (phase_ > kPi) {
        phase_ -= kTwoPi;
      } else if (phase_ < -kPi) {
        phase_ += kTwoPi;
      }
    }
  }

 private:
  double shiftHz_ = 0.0;
  double step_ = 0.0;
  double phase_ = 0.0;
};

class TimestampEncoder {
 public:
  explicit TimestampEncoder(double sampleRate) : sampleRate_(sampleRate) {}

  void reset() { anchored_ = false; }

  std::complex<float> headerForSample(uint64_t sampleIndex) {
    if (!anchored_) {
      anchorToWallClock();
    }
    double absolute = baseSeconds_ + static_cast<double>(sampleIndex) / sampleRate_;
    auto seconds = static_cast<uint32_t>(absolute);
    double fractional = absolute - static_cast<double>(seconds);
    auto nanoseconds = static_cast<uint32_t>(std::round(fractional * 1'000'000'000.0));
    if (nanoseconds >= 1'000'000'000U) {
      nanoseconds -= 1'000'000'000U;
      ++seconds;
    }
    float secBits = 0.0f;
    float nsecBits = 0.0f;
    std::memcpy(&secBits, &seconds, sizeof(uint32_t));
    std::memcpy(&nsecBits, &nanoseconds, sizeof(uint32_t));
    return {secBits, nsecBits};
  }

 private:
  void anchorToWallClock() {
    timespec ts {};
    clock_gettime(CLOCK_REALTIME, &ts);
    baseSeconds_ = static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
    anchored_ = true;
  }

  double sampleRate_;
  double baseSeconds_ = 0.0;
  bool anchored_ = false;
};

class UdpStreamer {
 public:
  UdpStreamer(std::string ip, const std::vector<uint16_t> &ports) {
    sockaddr_in templateAddr {};
    templateAddr.sin_family = AF_INET;
    if (inet_pton(AF_INET, ip.c_str(), &templateAddr.sin_addr) != 1) {
      throw std::runtime_error("Invalid IPv4 address");
    }
    for (auto port : ports) {
      if (port == 0) {
        continue;
      }
      int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
      if (fd < 0) {
        throw std::runtime_error("Failed to create UDP socket");
      }
      sockaddr_in addr = templateAddr;
      addr.sin_port = htons(port);
      sockets_.push_back({fd, addr});
    }
    if (sockets_.empty()) {
      throw std::runtime_error("No valid UDP ports configured");
    }
  }

  ~UdpStreamer() {
    for (const auto &socket : sockets_) {
      if (socket.fd >= 0) {
        ::close(socket.fd);
      }
    }
  }

  void send(const std::vector<std::complex<float>> &frame) const {
    const auto *raw = reinterpret_cast<const char *>(frame.data());
    const std::size_t bytes = frame.size() * sizeof(std::complex<float>);
    for (const auto &socket : sockets_) {
      ssize_t sent = ::sendto(socket.fd, raw, bytes, MSG_DONTWAIT,
                              reinterpret_cast<const sockaddr *>(&socket.addr), sizeof(sockaddr_in));
      if (sent < 0) {
        std::perror("sendto");
      }
    }
  }

 private:
  struct SocketSlot {
    int fd;
    sockaddr_in addr;
  };

  std::vector<SocketSlot> sockets_;
};

std::vector<std::complex<float>> convertToComplex(const std::vector<uint8_t> &bytes) {
  if (bytes.size() % kBytesPerIQ != 0) {
    throw std::runtime_error("Unaligned IQ byte stream");
  }
  std::vector<std::complex<float>> result;
  result.reserve(bytes.size() / kBytesPerIQ);
  constexpr float scale = 1.0f / 32768.0f;
  for (std::size_t offset = 0; offset + kBytesPerIQ <= bytes.size(); offset += kBytesPerIQ) {
    int16_t i = static_cast<int16_t>(static_cast<uint16_t>(bytes[offset]) |
                                     (static_cast<uint16_t>(bytes[offset + 1]) << 8));
    int16_t q = static_cast<int16_t>(static_cast<uint16_t>(bytes[offset + 2]) |
                                     (static_cast<uint16_t>(bytes[offset + 3]) << 8));
    result.emplace_back(static_cast<float>(i) * scale, static_cast<float>(q) * scale);
  }
  return result;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    std::signal(SIGPIPE, SIG_IGN);
    auto opts = parseArgs(argc, argv);
    const double outputRate = opts.inputRate / kTotalDecimation;

    FirDecimator stage1(8, 8 * 16, 0.45f / 8.0f);
    FirDecimator stage2(5, 5 * 16, 0.45f / 5.0f);
    FirDecimator stage3(5, 5 * 16, 0.45f / 5.0f);

    TimestampEncoder timestampEncoder(outputRate);
    UdpStreamer streamer(opts.ip, opts.ports);
    FrequencyShifter frequencyShifter(opts.inputRate, opts.shiftKhz * 1000.0);

    const std::size_t payloadSamples = opts.packetSamples - 1;

    std::vector<uint8_t> carry;
    carry.reserve(1024);

    std::vector<std::complex<float>> buffer;
    buffer.reserve(payloadSamples * 2);

    uint64_t samplesSent = 0;
    const std::size_t chunkBytes = opts.chunkSamples * kBytesPerIQ;

    while (std::cin.good()) {
      std::vector<uint8_t> chunk(chunkBytes);
      std::cin.read(reinterpret_cast<char *>(chunk.data()), static_cast<std::streamsize>(chunkBytes));
      std::streamsize bytesRead = std::cin.gcount();
      if (bytesRead <= 0) {
        break;
      }
      chunk.resize(static_cast<std::size_t>(bytesRead));

      std::vector<uint8_t> all;
      all.reserve(carry.size() + chunk.size());
      all.insert(all.end(), carry.begin(), carry.end());
      all.insert(all.end(), chunk.begin(), chunk.end());

      const std::size_t usableBytes = (all.size() / kBytesPerIQ) * kBytesPerIQ;
      std::vector<uint8_t> toConvert(all.begin(), all.begin() + usableBytes);
      carry.assign(all.begin() + usableBytes, all.end());
      if (toConvert.empty()) {
        continue;
      }

      auto stageInput = convertToComplex(toConvert);
      frequencyShifter.mix(stageInput);
      auto afterStage1 = stage1.process(stageInput);
      auto afterStage2 = stage2.process(afterStage1);
      auto decimated = stage3.process(afterStage2);

      if (!decimated.empty()) {
        buffer.insert(buffer.end(), decimated.begin(), decimated.end());
      }

      while (buffer.size() >= payloadSamples) {
        std::vector<std::complex<float>> frame;
        frame.reserve(opts.packetSamples);
        frame.push_back(timestampEncoder.headerForSample(samplesSent));
        frame.insert(frame.end(), buffer.begin(), buffer.begin() + payloadSamples);
        streamer.send(frame);
        buffer.erase(buffer.begin(), buffer.begin() + payloadSamples);
        samplesSent += payloadSamples;
      }
    }
  } catch (const ArgsError &err) {
    std::cerr << "Argument error: " << err.what() << "\n";
    printUsage(argv[0]);
    return 64;
  } catch (const std::exception &err) {
    std::cerr << "Fatal error: " << err.what() << "\n";
    return 1;
  }

  return 0;
}
