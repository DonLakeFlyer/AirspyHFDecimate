#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#define main airspyhf_decimator_program_main
#include "../src/main.cpp"
#undef main

namespace {

bool approxEqual(float left, float right, float epsilon = 1e-5f) {
  return std::fabs(left - right) <= epsilon;
}

uint32_t floatBitsToUint32(float value) {
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(uint32_t));
  return bits;
}

uint64_t extractTimeNs(const std::complex<float> &header) {
  const uint64_t seconds = static_cast<uint64_t>(floatBitsToUint32(header.real()));
  const uint64_t nanoseconds = static_cast<uint64_t>(floatBitsToUint32(header.imag()));
  return seconds * 1'000'000'000ULL + nanoseconds;
}

void decodeUavrtTimestamp(const std::complex<float> &header, uint32_t &seconds, uint32_t &nanoseconds) {
  seconds = floatBitsToUint32(header.real());
  nanoseconds = floatBitsToUint32(header.imag());
}

double estimateToneFrequencyHz(const std::vector<std::complex<float>> &samples, double sampleRateHz) {
  if (samples.size() < 2 || sampleRateHz <= 0.0) {
    return 0.0;
  }

  std::complex<double> sum{0.0, 0.0};
  for (std::size_t index = 1; index < samples.size(); ++index) {
    sum += static_cast<std::complex<double>>(samples[index]) *
           std::conj(static_cast<std::complex<double>>(samples[index - 1]));
  }

  const double avgPhaseStep = std::atan2(sum.imag(), sum.real());
  return (avgPhaseStep * sampleRateHz) / kTwoPi;
}

void testParseArgsDefaults() {
  char arg0[] = "airspyhf_decimator";
  char *argv[] = {arg0};
  const Options opts = parseArgs(1, argv);

  if (opts.inputRate != 768000.0 || opts.packetSamples != 1024 || opts.chunkSamples != 16384 ||
      opts.ip != "127.0.0.1" || opts.shiftKhz != 10.0 || opts.ports.size() != 2 || opts.ports[0] != 10000 ||
      opts.ports[1] != 10001) {
    throw std::runtime_error("parseArgs defaults mismatch");
  }
}

void testParseArgsCustom() {
  char arg0[] = "airspyhf_decimator";
  char arg1[] = "--input-rate";
  char arg2[] = "1024000";
  char arg3[] = "--frame";
  char arg4[] = "2048";
  char arg5[] = "--chunk";
  char arg6[] = "4096";
  char arg7[] = "--ip";
  char arg8[] = "127.0.0.2";
  char arg9[] = "--shift-khz";
  char arg10[] = "12.5";
  char arg11[] = "--ports";
  char arg12[] = "12000,12001,12002";
  char *argv[] = {arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12};

  const Options opts = parseArgs(static_cast<int>(sizeof(argv) / sizeof(argv[0])), argv);

  if (opts.inputRate != 1024000.0 || opts.packetSamples != 2048 || opts.chunkSamples != 4096 ||
      opts.ip != "127.0.0.2" || opts.shiftKhz != 12.5 || opts.ports.size() != 3 || opts.ports[0] != 12000 ||
      opts.ports[1] != 12001 || opts.ports[2] != 12002) {
    throw std::runtime_error("parseArgs custom values mismatch");
  }
}

void testParseArgsValidation() {
  char arg0[] = "airspyhf_decimator";
  char arg1[] = "--input-rate";
  char arg2[] = "0";
  char *argv[] = {arg0, arg1, arg2};

  bool threw = false;
  try {
    (void)parseArgs(static_cast<int>(sizeof(argv) / sizeof(argv[0])), argv);
  } catch (const ArgsError &) {
    threw = true;
  }

  if (!threw) {
    throw std::runtime_error("parseArgs should reject non-positive input rate");
  }
}

void testDesignLowpassNormalization() {
  const auto coeffs = designLowpass(10, 0.2f);
  if (coeffs.size() % 2 == 0) {
    throw std::runtime_error("designLowpass should produce odd tap count");
  }

  float sum = 0.0f;
  for (float value : coeffs) {
    sum += value;
  }

  if (!approxEqual(sum, 1.0f, 1e-3f)) {
    throw std::runtime_error("designLowpass coefficients should be normalized");
  }
}

void testConvertToComplexLittleEndian() {
  const std::vector<uint8_t> bytes = {
      0x00, 0x00, 0x80, 0xbf,
      0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x3f,
      0x00, 0x00, 0x80, 0xbf,
  };

  const auto samples = convertToComplex(bytes);
  if (samples.size() != 2) {
    throw std::runtime_error("convertToComplex sample count mismatch");
  }

  if (!approxEqual(samples[0].real(), -1.0f) || !approxEqual(samples[0].imag(), 0.0f) ||
      !approxEqual(samples[1].real(), 0.5f) || !approxEqual(samples[1].imag(), -1.0f)) {
    throw std::runtime_error("convertToComplex scaling mismatch");
  }
}

void testFrequencyShifterZeroShiftNoop() {
  std::vector<std::complex<float>> samples = {
      {0.25f, -0.5f},
      {-0.125f, 0.75f},
  };
  const auto original = samples;

  FrequencyShifter shifter(768000.0, 0.0);
  shifter.mix(samples);

  if (samples != original) {
    throw std::runtime_error("FrequencyShifter with 0 Hz should not modify samples");
  }
}

void testFrequencyShifterSignConvention() {
  constexpr double sampleRateHz = 96000.0;
  constexpr double inputToneHz = 5000.0;
  constexpr double shiftHz = 2000.0;
  constexpr std::size_t sampleCount = 4096;

  std::vector<std::complex<float>> base(sampleCount);
  const double inputPhaseStep = kTwoPi * inputToneHz / sampleRateHz;
  for (std::size_t index = 0; index < sampleCount; ++index) {
    const double phase = inputPhaseStep * static_cast<double>(index);
    base[index] = {static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
  }

  auto shiftedUp = base;
  FrequencyShifter shifterUp(sampleRateHz, shiftHz);
  shifterUp.mix(shiftedUp);

  auto shiftedDown = base;
  FrequencyShifter shifterDown(sampleRateHz, -shiftHz);
  shifterDown.mix(shiftedDown);

  const double upHz = estimateToneFrequencyHz(shiftedUp, sampleRateHz);
  const double downHz = estimateToneFrequencyHz(shiftedDown, sampleRateHz);

  if (std::abs(upHz - (inputToneHz + shiftHz)) > 60.0) {
    throw std::runtime_error("Positive shift should move tone up in frequency");
  }

  if (std::abs(downHz - (inputToneHz - shiftHz)) > 60.0) {
    throw std::runtime_error("Negative shift should move tone down in frequency");
  }
}

void testFirDecimatorOutputCount() {
  std::vector<std::complex<float>> samples(20, {1.0f, 0.0f});
  FirDecimator decimator(4, 17, 0.1f);
  const auto output = decimator.process(samples);

  if (output.size() != 5) {
    throw std::runtime_error("FirDecimator output size mismatch");
  }
}

void testTimestampEncoderMonotonicStep() {
  TimestampEncoder encoder(1000.0);

  const auto first = encoder.headerForSample(0);
  const auto second = encoder.headerForSample(1000);

  const uint64_t firstNs = extractTimeNs(first);
  const uint64_t secondNs = extractTimeNs(second);
  const uint64_t delta = (secondNs > firstNs) ? (secondNs - firstNs) : 0;

  if (delta < 999'999'000ULL || delta > 1'000'001'000ULL) {
    throw std::runtime_error("TimestampEncoder delta should track sample period");
  }
}

void testTimestampMatchesUavrtDetectionFormat() {
  constexpr double sampleRateHz = 3840.0;
  TimestampEncoder encoder(sampleRateHz);

  const auto header0 = encoder.headerForSample(0);
  const auto header1 = encoder.headerForSample(1);
  const auto headerMany = encoder.headerForSample(12345);

  uint32_t sec0 = 0;
  uint32_t nsec0 = 0;
  uint32_t sec1 = 0;
  uint32_t nsec1 = 0;
  uint32_t secMany = 0;
  uint32_t nsecMany = 0;

  decodeUavrtTimestamp(header0, sec0, nsec0);
  decodeUavrtTimestamp(header1, sec1, nsec1);
  decodeUavrtTimestamp(headerMany, secMany, nsecMany);

  if (nsec0 >= 1'000'000'000U || nsec1 >= 1'000'000'000U || nsecMany >= 1'000'000'000U) {
    throw std::runtime_error("uavrt timestamp nanoseconds must be < 1e9");
  }

  const uint64_t t0 = static_cast<uint64_t>(sec0) * 1'000'000'000ULL + static_cast<uint64_t>(nsec0);
  const uint64_t t1 = static_cast<uint64_t>(sec1) * 1'000'000'000ULL + static_cast<uint64_t>(nsec1);
  const uint64_t tm = static_cast<uint64_t>(secMany) * 1'000'000'000ULL + static_cast<uint64_t>(nsecMany);

  const uint64_t stepOneNs = static_cast<uint64_t>(std::llround(1'000'000'000.0 / sampleRateHz));
  const uint64_t expectedManyNs = static_cast<uint64_t>(std::llround(12345.0 * 1'000'000'000.0 / sampleRateHz));

  if (t1 <= t0 || tm <= t1) {
    throw std::runtime_error("uavrt timestamp fields must be monotonic in sample index");
  }

  const uint64_t observedStepOne = t1 - t0;
  const uint64_t observedMany = tm - t0;
  constexpr uint64_t oneSampleToleranceNs = 2'000ULL;
  constexpr uint64_t manySampleToleranceNs = 10'000ULL;

  if (observedStepOne + oneSampleToleranceNs < stepOneNs || observedStepOne > stepOneNs + oneSampleToleranceNs) {
    throw std::runtime_error("uavrt timestamp decode mismatch for one-sample increment");
  }

  if (observedMany + manySampleToleranceNs < expectedManyNs || observedMany > expectedManyNs + manySampleToleranceNs) {
    throw std::runtime_error("uavrt timestamp decode mismatch for multi-sample increment");
  }
}

void testPulseSurvivesShiftAndDecimation() {
  constexpr double inputRateHz = 768000.0;
  constexpr double rfCenterHz = 145990000.0;
  constexpr double pulseRfHz = 146000000.0;
  constexpr double toneOffsetHz = pulseRfHz - rfCenterHz;
  constexpr double shiftHz = -10000.0;
  constexpr double durationSeconds = 2.5;
  constexpr double pulseWidthSeconds = 0.015;
  constexpr double pulseIntervalSeconds = 2.0;
  constexpr double firstPulseStartSeconds = 0.25;
  constexpr float pulseAmplitude = 0.7f;

  const std::size_t inputSamples = static_cast<std::size_t>(durationSeconds * inputRateHz);
  std::vector<std::complex<float>> input(inputSamples, {0.0f, 0.0f});

  const double phaseStep = kTwoPi * toneOffsetHz / inputRateHz;
  for (std::size_t index = 0; index < inputSamples; ++index) {
    const double timeSeconds = static_cast<double>(index) / inputRateHz;
    bool inPulse = false;
    for (double pulseStart = firstPulseStartSeconds; pulseStart < durationSeconds; pulseStart += pulseIntervalSeconds) {
      if (timeSeconds >= pulseStart && timeSeconds < pulseStart + pulseWidthSeconds) {
        inPulse = true;
        break;
      }
    }

    if (inPulse) {
      const float i = pulseAmplitude * static_cast<float>(std::cos(phaseStep * static_cast<double>(index)));
      const float q = pulseAmplitude * static_cast<float>(std::sin(phaseStep * static_cast<double>(index)));
      input[index] = {i, q};
    }
  }

  FrequencyShifter shifter(inputRateHz, shiftHz);
  shifter.mix(input);

  FirDecimator stage1(8, 8 * 16, 0.45f / 8.0f);
  FirDecimator stage2(5, 5 * 16, 0.45f / 5.0f);
  FirDecimator stage3(5, 5 * 16, 0.45f / 5.0f);

  const auto afterStage1 = stage1.process(input);
  const auto afterStage2 = stage2.process(afterStage1);
  const auto output = stage3.process(afterStage2);
  if (output.empty()) {
    throw std::runtime_error("Decimation output is empty");
  }

  std::vector<float> power(output.size(), 0.0f);
  float maxPower = 0.0f;
  for (std::size_t index = 0; index < output.size(); ++index) {
    power[index] = std::norm(output[index]);
    if (power[index] > maxPower) {
      maxPower = power[index];
    }
  }

  if (maxPower <= 0.0f) {
    throw std::runtime_error("Pulse power was not detected");
  }

  const float threshold = maxPower * 0.35f;
  std::vector<std::size_t> regionStarts;
  std::vector<std::size_t> regionEnds;

  bool inRegion = false;
  std::size_t regionStart = 0;
  for (std::size_t index = 0; index < power.size(); ++index) {
    const bool above = power[index] >= threshold;
    if (above && !inRegion) {
      inRegion = true;
      regionStart = index;
    } else if (!above && inRegion) {
      inRegion = false;
      if (index - regionStart >= 10) {
        regionStarts.push_back(regionStart);
        regionEnds.push_back(index - 1);
      }
    }
  }
  if (inRegion && power.size() - regionStart >= 10) {
    regionStarts.push_back(regionStart);
    regionEnds.push_back(power.size() - 1);
  }

  if (regionStarts.size() < 2) {
    throw std::runtime_error("Expected at least two pulse regions after processing");
  }

  const std::size_t expectedGap = static_cast<std::size_t>(pulseIntervalSeconds * (inputRateHz / kTotalDecimation));
  const std::size_t observedGap = regionStarts[1] - regionStarts[0];
  const std::size_t tolerance = 200;

  if (observedGap + tolerance < expectedGap || observedGap > expectedGap + tolerance) {
    throw std::runtime_error("Pulse spacing after decimation is incorrect");
  }

  float peakInRegions = 0.0f;
  float maxOutsideRegions = 0.0f;
  for (std::size_t index = 0; index < power.size(); ++index) {
    bool insideAnyRegion = false;
    for (std::size_t region = 0; region < regionStarts.size(); ++region) {
      if (index >= regionStarts[region] && index <= regionEnds[region]) {
        insideAnyRegion = true;
        break;
      }
    }

    if (insideAnyRegion) {
      if (power[index] > peakInRegions) {
        peakInRegions = power[index];
      }
    } else if (power[index] > maxOutsideRegions) {
      maxOutsideRegions = power[index];
    }
  }

  if (peakInRegions <= 0.0f || maxOutsideRegions > peakInRegions * 0.5f) {
    throw std::runtime_error("Pulse contrast is too low after shift and decimation");
  }
}

void testNoisyPulseSurvivesShiftAndDecimation() {
  constexpr double inputRateHz = 768000.0;
  constexpr double rfCenterHz = 145990000.0;
  constexpr double pulseRfHz = 146000000.0;
  constexpr double toneOffsetHz = pulseRfHz - rfCenterHz;
  constexpr double shiftHz = -10000.0;
  constexpr double durationSeconds = 2.5;
  constexpr double pulseWidthSeconds = 0.015;
  constexpr double pulseIntervalSeconds = 2.0;
  constexpr double firstPulseStartSeconds = 0.25;
  constexpr float pulseAmplitude = 0.7f;
  constexpr float noiseStdDev = 0.12f;

  const std::size_t inputSamples = static_cast<std::size_t>(durationSeconds * inputRateHz);
  std::vector<std::complex<float>> input(inputSamples, {0.0f, 0.0f});

  std::mt19937 rng(42);
  std::normal_distribution<float> noise(0.0f, noiseStdDev);
  const double phaseStep = kTwoPi * toneOffsetHz / inputRateHz;

  for (std::size_t index = 0; index < inputSamples; ++index) {
    const double timeSeconds = static_cast<double>(index) / inputRateHz;
    bool inPulse = false;
    for (double pulseStart = firstPulseStartSeconds; pulseStart < durationSeconds; pulseStart += pulseIntervalSeconds) {
      if (timeSeconds >= pulseStart && timeSeconds < pulseStart + pulseWidthSeconds) {
        inPulse = true;
        break;
      }
    }

    float i = noise(rng);
    float q = noise(rng);
    if (inPulse) {
      i += pulseAmplitude * static_cast<float>(std::cos(phaseStep * static_cast<double>(index)));
      q += pulseAmplitude * static_cast<float>(std::sin(phaseStep * static_cast<double>(index)));
    }
    input[index] = {i, q};
  }

  FrequencyShifter shifter(inputRateHz, shiftHz);
  shifter.mix(input);

  FirDecimator stage1(8, 8 * 16, 0.45f / 8.0f);
  FirDecimator stage2(5, 5 * 16, 0.45f / 5.0f);
  FirDecimator stage3(5, 5 * 16, 0.45f / 5.0f);

  const auto afterStage1 = stage1.process(input);
  const auto afterStage2 = stage2.process(afterStage1);
  const auto output = stage3.process(afterStage2);
  if (output.empty()) {
    throw std::runtime_error("Noisy decimation output is empty");
  }

  std::vector<float> power(output.size(), 0.0f);
  float maxPower = 0.0f;
  float meanPower = 0.0f;
  for (std::size_t index = 0; index < output.size(); ++index) {
    power[index] = std::norm(output[index]);
    meanPower += power[index];
    if (power[index] > maxPower) {
      maxPower = power[index];
    }
  }
  meanPower /= static_cast<float>(power.size());

  if (maxPower <= 0.0f || meanPower <= 0.0f) {
    throw std::runtime_error("Noisy pulse power was not detected");
  }

  const float threshold = maxPower * 0.45f;
  std::vector<std::size_t> regionStarts;

  bool inRegion = false;
  std::size_t regionStart = 0;
  for (std::size_t index = 0; index < power.size(); ++index) {
    const bool above = power[index] >= threshold;
    if (above && !inRegion) {
      inRegion = true;
      regionStart = index;
    } else if (!above && inRegion) {
      inRegion = false;
      if (index - regionStart >= 8) {
        regionStarts.push_back(regionStart);
      }
    }
  }
  if (inRegion && power.size() - regionStart >= 8) {
    regionStarts.push_back(regionStart);
  }

  if (regionStarts.size() < 2) {
    throw std::runtime_error("Expected two pulse regions in noisy output");
  }

  const std::size_t expectedGap = static_cast<std::size_t>(pulseIntervalSeconds * (inputRateHz / kTotalDecimation));
  const std::size_t observedGap = regionStarts[1] - regionStarts[0];
  const std::size_t tolerance = 260;

  if (observedGap + tolerance < expectedGap || observedGap > expectedGap + tolerance) {
    throw std::runtime_error("Noisy pulse spacing after decimation is incorrect");
  }

  if (maxPower < meanPower * 3.0f) {
    throw std::runtime_error("Noisy pulse peak is not sufficiently above background");
  }
}

}  // namespace

int main() {
  struct Case {
    const char *name;
    void (*fn)();
  };

  const std::vector<Case> cases = {
      {"parseArgs defaults", testParseArgsDefaults},
      {"parseArgs custom", testParseArgsCustom},
      {"parseArgs validation", testParseArgsValidation},
      {"designLowpass normalization", testDesignLowpassNormalization},
      {"convertToComplex little-endian", testConvertToComplexLittleEndian},
      {"FrequencyShifter zero-shift", testFrequencyShifterZeroShiftNoop},
      {"FrequencyShifter sign convention", testFrequencyShifterSignConvention},
      {"FirDecimator output count", testFirDecimatorOutputCount},
      {"TimestampEncoder monotonic step", testTimestampEncoderMonotonicStep},
        {"Timestamp matches uavrt_detection format", testTimestampMatchesUavrtDetectionFormat},
      {"Pulse survives shift and decimation", testPulseSurvivesShiftAndDecimation},
      {"Noisy pulse survives shift and decimation", testNoisyPulseSurvivesShiftAndDecimation},
  };

  int failures = 0;
  for (const auto &testCase : cases) {
    try {
      testCase.fn();
      std::cout << "[PASS] " << testCase.name << "\n";
    } catch (const std::exception &err) {
      ++failures;
      std::cerr << "[FAIL] " << testCase.name << ": " << err.what() << "\n";
    }
  }

  if (failures != 0) {
    std::cerr << failures << " test(s) failed\n";
    return 1;
  }

  std::cout << "All tests passed\n";
  return 0;
}
