#pragma once

#include <vector>
#include <list>
#include <bit>
#include <cassert>
#include <unordered_map>

#include "base/type.h"
#include "base/debug.h"

namespace Ramulator {

template<class T>
static bool is_power_of_2(T val) {
    return (val & (val-1)) == 0;
}

class CoWsCache {

public:
    struct CoWsStats {
        std::string stats_fname;
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

            bool getBlockID(uint32_t block_id) const {
                assert(block_id < block_map.size());
                return block_map[block_id];
            }

            void setBlockID(uint32_t block_id, bool present) {
                DBG("block=%u, block_map.size()=%d", block_id, (int)block_map.size());

                assert(block_id < block_map.size());
                block_map[block_id] = present;
            }

            uint32_t getNumBlocks() {
                return (uint32_t)std::count_if(block_map.begin(), block_map.end(), [](bool b) { return b == true; });
            }

            Addr_t getRealAddr() { return real_page_phys_addr; }
            void setRealAddr(Addr_t addr) { real_page_phys_addr = addr; }
    };

private:
    const uint32_t m_nlines;
    const uint32_t m_nsets;
    const uint32_t m_assoc;
    const uint32_t m_cache_line_bytes;
    const uint32_t m_dram_page_bytes;

    const CoWsStats m_stats;

    const uint32_t m_dram_latency_on_translation;
    const uint32_t m_access_latency;

    const uint32_t m_dram_page_bit_offset;
    const Addr_t m_dram_page_mask;

    const Addr_t m_set_mask;
    const Addr_t m_set_offset;

//    using CacheSet_t = std::list<Line>;   // LRU queue for the set. The head of the list is the least-recently-used way.
//    std::vector<CacheSet_t> m_cache_sets;
    std::unordered_map<Addr_t, Line> m_cache;

    // track the number of valid lines in llc
    uint32_t m_lines_in_llc;
    std::vector<std::pair<uint32_t, uint32_t>> m_cows2llc_valid;
    bool miss_after_insert; // we get the miss notification from the llc after we insert the page to the cows cache. this helps us with stats.

public:
    CoWsCache(uint32_t nlines,
              uint32_t assoc,
              uint32_t cache_line_bytes,
              uint32_t dram_page_bytes,
              uint32_t access_latency,
              uint32_t dram_latency_on_translation,
              const CoWsStats& stats);

    virtual ~CoWsCache() {}

    // called when simulation finished to dump stats
    void fini();

    bool lookup(Addr_t baddr, Line*& ret);
    void insert(Addr_t baddr, Addr_t real_phys_addr);
    void erase(Addr_t baddr);

    void llc_hit(Addr_t baddr);
    void llc_miss(Addr_t baddr);
    uint32_t llc_evict(Addr_t baddr);

    uint32_t get_block_id(Addr_t baddr) { return (uint32_t)((baddr & ~m_dram_page_mask) / m_cache_line_bytes); }

    uint32_t get_dram_latency_on_translation(Addr_t addr) { return m_dram_latency_on_translation; }
    uint32_t get_access_latency() { return m_access_latency; }
private:
    Addr_t get_page_addr(Addr_t baddr) { return baddr & m_dram_page_mask; }
    uint32_t get_set_idx(Addr_t baddr) { return (uint32_t)((baddr & m_set_mask) >> m_set_offset); }
};


}; // namespace
