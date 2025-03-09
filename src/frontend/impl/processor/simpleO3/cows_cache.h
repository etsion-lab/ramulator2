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
            Clk_t ready_clk;
            Addr_t real_page_phys_addr;
            std::vector<bool> block_map;

        public:
            Line() : valid(false),
                     page_phys_addr(0),
                     ready_clk(0),
                     block_map(MAX_BLOCKS_PER_PAGE, false) {}

            Addr_t getTag() const {
                return page_phys_addr;
            }

            void setTag(Addr_t page_addr) {
                page_phys_addr = page_addr;
            }

            void setReadyClk(Clk_t when_ready) {
                ready_clk = when_ready;
            }

            Clk_t whenReady(Clk_t now) {
                if(now > ready_clk)
                    return 0;

                return (now - ready_clk);
            }

            void setValid(bool v) {
                valid = v;
            }

            bool getValid() const {
                return valid;
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

            std::string toString() const;
        };

public:
    // replacement policies
    using CacheSet_t = std::list<Line>;   // LRU queue for the set. The head of the list is the least-recently-used way.

    // The head of the list is the least-recently-used way (aka victim).
    class ReplPolicy {
        public:
        // all methods return true if there was a need to access the renaming table in DRAM
        virtual std::string name() const = 0;
        virtual bool llc_hit(CoWsCache& cows_cache, CacheSet_t& set, Addr_t page_addr) const = 0;
        virtual bool llc_miss(CoWsCache& cows_cache, CacheSet_t& set, Addr_t page_addr) const = 0;
        virtual bool llc_evict(CoWsCache& cows_cache, CacheSet_t& set, Addr_t page_addr) const = 0;

        protected:
        virtual CacheSet_t::iterator victim(CacheSet_t& set) const {
            return set.begin();
        }

        CacheSet_t::iterator find_in_set(CacheSet_t& set, Addr_t page_addr) const {
            return std::find_if(set.begin(), set.end(), [page_addr](Line l){ return (l.getTag() == page_addr);});
        }

        void move_to_mru(CacheSet_t& set, CacheSet_t::iterator it) const {
            // move the accessed line to MRU (tail of list)
            auto line = *it;
            set.erase(it);
            set.push_back(line);
        }
    };

    // The head of the list is the least-recently-used way.
    class LRU : public ReplPolicy  {
        std::string name() const { return "LRU"; }
        bool llc_hit(CoWsCache& cows_cache, CacheSet_t& set, Addr_t baddr) const;
        bool llc_miss(CoWsCache& cows_cache, CacheSet_t& set, Addr_t baddr) const;
        bool llc_evict(CoWsCache& cows_cache, CacheSet_t& set, Addr_t baddr) const { return false; };
    };

    // The head of the list is the least-recently-used way.
    class LRU_nohit : public ReplPolicy  {
        std::string name() const { return "LRU_nohit"; }
        bool llc_hit(CoWsCache& cows_cache, CacheSet_t& set, Addr_t baddr) const { return false; };
        bool llc_miss(CoWsCache& cows_cache, CacheSet_t& set, Addr_t baddr) const;
        bool llc_evict(CoWsCache& cows_cache, CacheSet_t& set, Addr_t baddr) const;
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

    std::vector<CacheSet_t> m_cache_sets;
    const ReplPolicy* m_policy;

    // track the number of valid lines in llc
    uint32_t m_lines_in_llc;
    std::vector<std::pair<uint32_t, uint32_t>> m_cows2llc_valid;

public:
    // public stats
    uint64_t s_hits = 0;
    uint64_t s_misses = 0;
    uint64_t s_access = 0;

    Clk_t s_avg_dram_lat_sum = 0;
    Clk_t s_avg_dram_lat_cnt = 0;

public:
    CoWsCache(uint32_t nlines,
              uint32_t assoc,
              uint32_t cache_line_bytes,
              uint32_t dram_page_bytes,
              uint32_t access_latency,
              uint32_t dram_latency_on_translation,
              const CoWsCache::ReplPolicy* policy,
              const CoWsStats& stats);

    virtual ~CoWsCache() {}

    // called when simulation finished to dump stats
    void fini();

    bool perfect_cache_lookup(Addr_t page_addr, Line*& ret);
    void perfect_cache_insert(Addr_t page_addr, Addr_t real_phys_addr);
    void perfect_cache_erase(Addr_t page_addr);

    void llc_hit(Addr_t baddr);
    // this function returns the latency incurred by the cows cach access (fill + potential WB)
    uint32_t llc_miss(Addr_t baddr);
    uint32_t llc_evict(Addr_t baddr);

    uint32_t get_block_id(Addr_t baddr) { return (uint32_t)((baddr & ~m_dram_page_mask) / m_cache_line_bytes); }
    uint32_t get_dram_page_bytes() { return m_dram_page_bytes; }

    uint32_t get_dram_latency_on_translation(Addr_t addr) {
        return m_dram_latency_on_translation;
    }
    uint32_t get_access_latency() { return m_access_latency; }

private:
    Addr_t get_page_addr(Addr_t baddr) { return baddr & m_dram_page_mask; }
    uint32_t get_set_idx(Addr_t page_addr) {
        uint32_t page_id = page_addr >> m_dram_page_bit_offset;
        // we don't force the number of sets to be a power of 2, so we can scan different
        // cache sizes
        return (uint32_t)(page_id % m_nsets);
    }
};


}; // namespace
