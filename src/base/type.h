#ifndef     RAMULATOR_BASE_TYPE_H
#define     RAMULATOR_BASE_TYPE_H

#include <vector>
#include <unordered_map>
#include <string>
#include <type_traits>


namespace Ramulator {

using Clk_t     = int64_t;            // Clock cycle
using Addr_t    = int64_t;            // Plain address as seen by the OS
using AddrVec_t = std::vector<int>;   // Device address vector as is sent to the device from the controller

// Yoav:
// extend Addr_t with an ASID/App ID. We need this to run different apps, each with it's
// own phys. addr trace, on the same machine

// ASID bit count, bit mask, and offset in phys. address
const uint64_t ADDR_ASID_BITS = 7;
const uint64_t ADDR_ASID_MAX = ((1<<ADDR_ASID_BITS)-1);
const uint64_t ADDR_ASID_MASK = ADDR_ASID_MAX;
const uint64_t ADDR_ASID_SHIFT = 56;

// we have 7b for ASID
static inline bool is_valid_asid(int64_t asid) {
    return (asid >= 0 && asid <= ADDR_ASID_MAX);
}

// extract the ASID from an address
static inline uint64_t addr_get_asid(Addr_t addr)
{
    return (addr >> ADDR_ASID_SHIFT) & ADDR_ASID_MASK;
}

// set the ASID in an address address
static inline uint64_t addr_set_asid(Addr_t addr, uint64_t asid)
{
    return (addr | (asid << ADDR_ASID_SHIFT));
}


template<typename T>
using Registry_t = std::unordered_map<std::string, T>;


// From WG21 P2098R1 Proposing std::is_specialization_of
template<class T, template<class...> class Primary>
struct is_specialization_of : std::false_type {};

template<template<class...> class Primary, class... Args>
struct is_specialization_of<Primary<Args...>, Primary> : std::true_type {};

template< class T, template<class...> class Primary>
inline constexpr bool is_specialization_of_v = is_specialization_of<T, Primary>::value;

}        // namespace Ramulator


#endif   // RAMULATOR_BASE_TYPE_H
