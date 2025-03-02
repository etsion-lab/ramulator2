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

            Addr_t getPageAddr() const {
                return page_phys_addr;
            }

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

    // perfect cache
    std::unordered_map<Addr_t, Line> m_perfect_cache;

    using CacheSet_t = std::list<Line>;   // LRU queue for the set. The head of the list is the least-recently-used way.
    std::vector<CacheSet_t> m_cache_sets;

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

    bool lookup(Addr_t page_addr, Line*& ret);
    void insert(Addr_t page_addr, Addr_t real_phys_addr);
    void erase(Addr_t page_addr);

    void llc_hit(Addr_t baddr);
    // this function returns the latency incurred by the cows cach access (fill + potential WB)
    uint32_t llc_miss(Addr_t baddr);
    uint32_t llc_evict(Addr_t baddr);

    uint32_t get_block_id(Addr_t baddr) { return (uint32_t)((baddr & ~m_dram_page_mask) / m_cache_line_bytes); }
    uint32_t get_dram_page_bytes() { return m_dram_page_bytes; }

    uint32_t get_dram_latency_on_translation(Addr_t addr) { return m_dram_latency_on_translation; }
    uint32_t get_access_latency() { return m_access_latency; }
private:
    Addr_t get_page_addr(Addr_t baddr) { return baddr & m_dram_page_mask; }
    uint32_t get_set_idx(Addr_t baddr) {
        uint32_t page_id = baddr >> m_dram_page_bit_offset;
        // we don't force the number of sets to be a power of 2, so we can scan different
        // cache sizes
        return (uint32_t)(page_id % m_nsets);
    }

    CacheSet_t::iterator find_in_set(CacheSet_t& set, Addr_t page_addr) {
        return std::find_if(set.begin(), set.end(),
                            [page_addr](Line l){return (l.getPageAddr() == page_addr);});
    }

    void line_to_XXXru(CacheSet_t& set, CacheSet_t::iterator& line_it, bool is_lru) {

        // The head of the list is the least-recently-used way.
        auto line = *line_it;

        set.erase(line_it);
        if(is_lru)
            set.push_back(line);
        else
            set.push_front(line);
    }

    void line_to_lru(CacheSet_t& set, CacheSet_t::iterator& line_it) {
        line_to_XXXru(set, line_it, true);
    }

    void line_to_mru(CacheSet_t& set, CacheSet_t::iterator& line_it) {
        line_to_XXXru(set, line_it, false);
    }
};


}; // namespace
