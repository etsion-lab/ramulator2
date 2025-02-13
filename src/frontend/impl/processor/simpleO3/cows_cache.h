#pragma once

#include <vector>
#include <list>
#include <bit>
#include <cassert>

#include "base/type.h"

namespace Ramulator {

template<class T>
static bool is_power_of_2(T val) {
    return (val & (val-1)) == 0;
}

class CoWsCache {

public:
    struct Config {
        Config() : fixed_dram_latency(0),
                   force_lookup_on_ASID_miss(false),
                   llc2cows_ratio(0),
                   assoc(0),
                   dram_page_bytes(0) {}

        int fixed_dram_latency;
        bool force_lookup_on_ASID_miss;
        uint32_t llc2cows_ratio;
        uint32_t assoc;
        uint32_t dram_page_bytes;
    };

    class Line {
        const uint32_t MAX_BLOCKS_PER_PAGE = 8192/64;

        private:
            bool valid;
            Addr_t page_phys_addr;
            Addr_t real_page_phys_addr;
            std::vector<bool> block_map;

        public:
            Line() : valid(false),
                     page_phys_addr(0),
                     block_map(MAX_BLOCKS_PER_PAGE, false) {}

            bool getBlockID(uint32_t block) const {
                assert(block < block_map.size());
                return block_map[block];
            }

            void setBlockID(uint32_t block, bool present) {
                assert(block < block_map.size());
                block_map[block] = present;
            }

            uint32_t getNumBlocks() {
                return (uint32_t)std::count_if(block_map.begin(), block_map.end(), [](bool b) { return b == true; });
            }
    };

private:
    const uint32_t m_nlines;
    const uint32_t m_nsets;
    const uint32_t m_assoc;
    const uint32_t m_cache_line_bytes;
    const uint32_t m_dram_page_bytes;

    const uint32_t m_dram_page_bit_offset;
    const Addr_t m_set_mask;
    const Addr_t m_set_offset;

    using CacheSet_t = std::list<Line>;   // LRU queue for the set. The head of the list is the least-recently-used way.
    std::vector<CacheSet_t> m_cache_sets;

public:
    CoWsCache(uint32_t nlines,
              uint32_t assoc,
              uint32_t cache_line_bytes,
              uint32_t dram_page_bytes);

    virtual ~CoWsCache() {}

    bool lookup(Addr_t block, Line& ret);
    void insert(Addr_t block, Addr_t real_phys_addr, Line& victim);
};

}; // namespace
