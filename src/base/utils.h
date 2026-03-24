#ifndef RAMULATOR_BASE_UTILS_H
#define RAMULATOR_BASE_UTILS_H

#include <string>
#include <vector>
#include <cstdint>
#include <functional>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "spdlog/spdlog.h"

namespace Ramulator {

extern uint32_t g_latency_factor;
static inline void set_latency_factor(uint32_t factor) {
  g_latency_factor = factor;
}
static inline uint32_t latmul(uint32_t latency) {
  return latency * g_latency_factor;
}

/************************************************
 *     Utility Functions for Parsing Configs
 ***********************************************/

/**
 * @brief    Parse capacity strings (e.g., KB, MB) into the number of bytes
 *
 * @param    size_str       A capacity string (e.g., "8KB", "64MB").
 * @return   size_t         The number of bytes.
 */
size_t parse_capacity_str(std::string size_str);

/**
 * @brief    Parse frequency strings (e.g., MHz, GHz) into MHz
 *
 * @param    size_str       A capacity string (e.g., "4GHz", "3500MHz").
 * @return   size_t         The number of bytes.
 */
size_t parse_frequency_str(std::string size_str);

/**
 * @brief Convert a timing constraint in nanoseconds into number of cycles according to JEDEC convention.
 *
 * @param t_ns      Timing constraint in nanoseconds
 * @param tCK_ps    Clock cycle in picoseconds
 * @return uint64_t Number of cycles
 */
uint64_t JEDEC_rounding(float t_ns, int tCK_ps);


/**
 * @brief Convert a timing constraint in nanoseconds into number of cycles according to JEDEC DDR5 convention.
 *
 * @param t_ns      Timing constraint in nanoseconds
 * @param tCK_ps    Clock cycle in picoseconds
 * @return uint64_t Number of cycles
 */
uint64_t JEDEC_rounding_DDR5(float t_ns, int tCK_ps);


/************************************************
 *       Bitwise Operations for Integers
 ***********************************************/

/**
 * @brief Calculate how many bits are needed to store val
 *
 * @tparam Integral_t
 * @param val
 * @return Integral_t
 */
template <typename Integral_t>
Integral_t calc_log2(Integral_t val) {
  static_assert(std::is_integral_v<Integral_t>, "Only integral types are allowed for bitwise operations!");

  Integral_t n = 0;
  while ((val >>= 1)) {
    n ++;
  }
  return n;
};

/**
 * @brief Slice the lest significant num_bits from addr and return these bits. The originial addr value is modified.
 *
 * @tparam Integral_t
 * @param addr
 * @param num_bits
 * @return Integral_t
 */
template <typename Integral_t>
Integral_t slice_lower_bits(Integral_t& addr, int num_bits) {
  static_assert(std::is_integral_v<Integral_t>, "Only integral types are allowed for bitwise operations!");

  Integral_t lbits = addr & ((1<<num_bits) - 1);
  addr >>= num_bits;
  return lbits;
};

template <typename T, typename Formatter>
void dump_raw_stats(const std::vector<T>& stats,
                    const std::string& file_name,
                    const std::string& header,
                    Formatter formatter)
{
  std::ofstream out(file_name);
  out << header << std::endl;
  for (const auto& stat : stats) {
    out << formatter(stat) << std::endl;
  }
  out.close();
}

struct Formatter2Tuple {
  template <typename T1, typename T2>
  std::string operator()(const std::tuple<T1, T2>& stat) const {
    return fmt::format("{}\t\t\t{}", std::get<0>(stat), std::get<1>(stat));
  }
};
inline constexpr Formatter2Tuple formatter_2tuple{};

struct Formatter3Tuple {
  template <typename T1, typename T2, typename T3>
  std::string operator()(const std::tuple<T1, T2, T3>& stat) const {
    return fmt::format("{}\t\t\t{}\t\t\t{}", std::get<0>(stat), std::get<1>(stat), std::get<2>(stat));
  }
};
inline constexpr Formatter3Tuple formatter_3tuple{};

/************************************************
 *                Tokenization
 ***********************************************/
void tokenize(std::vector<std::string>& tokens, std::string line, std::string delim);

}           // namespace Ramulator

#endif      // RAMULATOR_BASE_UTILS_H
