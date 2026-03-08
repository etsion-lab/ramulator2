#pragma once

#define VICTIM_ADDRESS 0x206840100

#include <vector>
#include <list>
#include <bit>
#include <cassert>
#include <unordered_map>
#include <utility>

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

    class AddrParser {
        public:
        static uint64_t s_dram_page_bytes;
        static uint64_t s_bytes_per_line;
        static uint32_t s_nsets;

        static inline uint32_t getNumBlocksPerPage() {
            return (uint32_t)(s_dram_page_bytes / s_bytes_per_line);
        }
        static inline uint64_t getDramPageBits() { // number of bits for offset within a dram page
            return std::countr_zero(s_dram_page_bytes);
        }
        static inline Addr_t getDramPageOffsetMask() { // mask to get offset inside a dram page address from memory address
            return ((((Addr_t)1)<<getDramPageBits()) - 1);
        }
        static inline Addr_t getDramPageAddrMask() { // mask to get dram page address from memory address
            return ~getDramPageOffsetMask();
        }
        static inline uint32_t getNumSets() {
            return s_nsets;
        }
        static inline uint64_t getSetBits() { // number of bits for offset within a dram page
            return std::countr_zero(getNumSets());
        }
        static inline Addr_t getSetMask() { // number of bits for offset within a dram page
            return (((Addr_t)1<<getSetBits()) - 1);
        }
        static inline uint32_t getBlockIdx(Addr_t baddr) {
            return (uint32_t)((baddr & getDramPageOffsetMask()) / s_bytes_per_line);
        }

        public:
        AddrParser() = delete;
        AddrParser(Addr_t addr) : m_addr(addr) {}

        inline Addr_t getPageAddr() const {
            return m_addr & getDramPageAddrMask();
        }
        inline Addr_t getPageNum() const {
            return m_addr >> getDramPageBits();
        }
        inline uint32_t getSetID() const {
            return (uint32_t)(getPageNum() & getSetMask());
        }
        inline Addr_t getTag() const {
            return (getPageNum() >> getSetBits());
        }

        std::string toString() const;

        private:
        const Addr_t m_addr;
    };

    class Line {
        public:
        static const uint32_t PHYS_ADDRESS_SPACE_BITS = 52; // ARMv8.2 supports a 52b physical address space
        static uint64_t bytes_per_line;

        static inline uint32_t getDataBitSize() {
            return AddrParser::getNumBlocksPerPage();
        }
        static inline uint32_t getTagBits() {
            return (PHYS_ADDRESS_SPACE_BITS - CoWsCache::AddrParser::getDramPageBits() - CoWsCache::AddrParser::getSetBits()) + 2; // dirty + valid bits
        }

        private:
            bool valid;
            bool dirty;

            Addr_t page_phys_addr;
            Addr_t tag;
            uint32_t set_id;

            Clk_t ready_clk;
            std::vector<bool> block_map;
            uint32_t max_present_count;

            const Addr_t NULL_ADDR = (Addr_t)-1;

        public:
            Line() : block_map(CoWsCache::AddrParser::getNumBlocksPerPage()) { reset(); }

            void reset() {
                valid = false;
                dirty = false;

                setPageAddr(NULL_ADDR);
                tag = NULL_ADDR;
                set_id=(uint32_t)-1;

                ready_clk = -1;
                std::fill(block_map.begin(), block_map.end(), false);
                max_present_count = 0;
            }

            Addr_t getTag() const {
                assert(page_phys_addr!=NULL_ADDR);
                return tag;
            }

            Addr_t getSetID() const {
                assert(page_phys_addr!=NULL_ADDR);
                return set_id;
            }

            void setPageAddr(Addr_t page_addr) {
                page_phys_addr = page_addr;
                AddrParser ap(page_addr);
                tag =ap.getTag();
                set_id = ap.getSetID();

                // verify address parser
                if(page_addr!=NULL_ADDR && (tag << (CoWsCache::AddrParser::getDramPageBits() + CoWsCache::AddrParser::getSetBits()) | set_id << CoWsCache::AddrParser::getDramPageBits()) != page_addr) {
                    std::cerr<<"Error: address/tag/set mismatch:"<<std::endl;
                    std::cerr<<ap.toString();
                    assert(0);
                }
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

            bool isValid() const {
                return valid;
            }

            void setDirty(bool d) {
                dirty = d;
            }

            bool isDirty() const {
                return dirty;
            }

            bool getBlockID(uint32_t block_id) const {
                assert(block_id < block_map.size());
                return block_map[block_id];
            }

            void setBlockID(uint32_t block_id, bool present) {
                DBG("block=%u, block_map.size()=%d", block_id, (int)block_map.size());

                assert(block_id < block_map.size());
                if(block_map[block_id] != present) {
                    if(present) max_present_count++;
                }
                block_map[block_id] = present;
            }

            uint32_t getNumBlocks() {
                return (uint32_t)std::count_if(block_map.begin(), block_map.end(), [](bool b) { return b == true; });
            }

            std::string toString() const;
        };

public:
    // replacement policies
    using CacheSet_t = std::list<Line*>;   // LRU queue for the set. The head of the list is the least-recently-used way.

    // The head of the list is the least-recently-used way (aka victim).
    class ReplPolicy {
        public:
        CoWsCache* m_cows_cache = nullptr;

        virtual ~ReplPolicy() {}

        void setCowsCache(CoWsCache* cows_cache) {
            assert(m_cows_cache == nullptr);
            m_cows_cache = cows_cache;
        }

        virtual std::string name() const = 0;
        // all methods return:
        // bool 1: hit/miss in the cows_cache
        // bool 2: was a block evicted from the cows_cache
        virtual std::pair<bool,bool> llc_hit(CacheSet_t& set, Line* page, uint32_t block_id, bool is_write) const = 0;
        virtual std::pair<bool,bool> llc_miss(CacheSet_t& set, Line* page, uint32_t block_id, bool is_write) const = 0;
        virtual std::pair<bool,bool> llc_evict(CacheSet_t& set, Line* page, uint32_t block_id) const = 0;

        protected:
        virtual CacheSet_t::iterator victim(CacheSet_t& set) const {
            auto victim = set.begin();

            return victim;
        }

        CacheSet_t::iterator find_in_set(CacheSet_t& set, Addr_t tag) const {
            auto match = [tag](Line* l){ return (l->getTag() == tag);};
            return std::find_if(set.begin(), set.end(), match);
        }

        std::pair<CacheSet_t::iterator, bool> alloc_line(CacheSet_t& set, Line* page) const {
            CacheSet_t::iterator it;
            bool do_evict = false;

            // set not full? add a new line
            if(set.size() < m_cows_cache->m_assoc) {
                set.push_back(page);
                it = std::prev(set.end());
            }
            else {
                do_evict = true;

                // set is full so get the a victim
                it = victim(set);
                auto victim = *it;
                set.erase(it);
                set.push_back(page);

                if(page->isDirty()) {
                    // TODO write back to memory
                    page->setDirty(false);
                }

                m_cows_cache->s_evicts++;
            }

            return { it, do_evict };
        }

        void move_to_mru(CacheSet_t& set, CacheSet_t::iterator it) const {
            // move the accessed line to MRU (tail of list)
            auto line = *it;
            set.erase(it);
            set.push_back(line);
        }

        public:
        static void dump_set(const CacheSet_t& set, Addr_t page_addr, const std::string &s) {
            printf("%s[dump_set]: page_addr=0x%lx, set.size=%zu\n", s.c_str(), (unsigned long)page_addr, set.size());
            for (auto line : set) {
                Addr_t tag = line->getTag();
                if (tag == page_addr)
                    printf("  * tag=0x%lx  <-- match\n", (unsigned long)tag);
                else
                    printf("    tag=0x%lx\n", (unsigned long)tag);
            }
        }

    };

    // The head of the list is the least-recently-used way.
    class LRU_nohit : public ReplPolicy  {
        public:
        virtual std::string name() const override { return "LRU_nohit"; }
        std::pair<bool,bool> llc_hit(CacheSet_t& set, Line* page, uint32_t block_id, bool is_write) const override;
        std::pair<bool,bool> llc_miss(CacheSet_t& set, Line* page, uint32_t block_id, bool is_write) const override;
        std::pair<bool,bool> llc_evict(CacheSet_t& set, Line* page, uint32_t block_id) const override;
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

    const bool m_always_miss;

    // perfect cache
    std::unordered_map<Addr_t, Line> m_perfect_cache;

    std::vector<CacheSet_t> m_cache_sets;
    ReplPolicy* m_policy;

    // track the number of valid lines in llc
    uint32_t m_lines_in_llc;
    std::vector<std::pair<uint32_t, uint32_t>> m_stats_cows2llc_valid;
    std::vector<Clk_t> m_stats_cows_miss_clk;
    std::vector<uint64_t> m_stats_set_access;
    std::vector<uint64_t> m_stats_set_miss;

public:
    // public stats
    uint64_t s_hits = 0;
    uint64_t s_misses = 0;
    uint64_t s_access = 0;
    uint64_t s_evicts = 0;

    uint64_t s_accesses_on_llc_hit = 0;
    uint64_t s_accesses_on_llc_miss = 0;
    uint64_t s_accesses_on_llc_evict = 0;
    uint64_t s_misses_on_llc_hit = 0;
    uint64_t s_misses_on_llc_miss = 0;
    uint64_t s_misses_on_llc_evict = 0;

    uint64_t s_cows_cycles = 0;
    uint64_t s_cows_cycles_self = 0;

public:
    CoWsCache(uint32_t nlines,
              uint32_t assoc,
              uint32_t cache_line_bytes,
              uint32_t dram_page_bytes,
              bool always_miss,
              uint32_t access_latency,
              uint32_t dram_latency_on_translation,
              CoWsCache::ReplPolicy* policy,
              const CoWsStats& stats);

    virtual ~CoWsCache() {}

    // called when simulation finished to dump stats
    void fini();

    std::pair<bool, Line*> perfect_cache_lookup(Addr_t page_addr);

    // these functions return the latency incurred by the cows cach access (fill + potential WB)
    std::pair<bool, uint32_t> llc_hit(Addr_t baddr, bool is_write, Clk_t clk, int total_llc_misses);
    std::pair<bool, uint32_t> llc_miss(Addr_t baddr, bool is_write, Clk_t clk, int total_llc_misses);
    std::pair<bool, uint32_t> llc_evict(Addr_t baddr, bool evict_dirty, Clk_t clk, int total_llc_misses);

    uint32_t get_dram_latency_on_translation(Addr_t addr) {
        return m_dram_latency_on_translation;
    }
    uint32_t get_access_latency() { return m_access_latency; }

private:
    void track_misses(Clk_t clk, int total_llc_misses) {
        static Clk_t last_print = 0;
        static uint64_t last_cows_miss_count = 0;
        static uint64_t last_llc_miss_count = 0;

#if 0
        if(clk - last_print >= 10000000) {
            Clk_t delta_clk = clk - last_print;
            uint64_t cows_delta_miss = s_misses - last_cows_miss_count;
            uint64_t llc_delta_miss = total_llc_misses - last_llc_miss_count;
            double misses_per_cycle = (double)cows_delta_miss / (double)delta_clk;
            double misses_per_llc_miss = (double)cows_delta_miss / (double)llc_delta_miss;

            printf("# COWs stats at %lu: misses/cycle %12.6lf, misses/llc %12.6lf "
                "(last_cows_miss_count=%lu, cows_delta_miss=%lu, last_llc_miss=%lu, llc_delta_miss=%lu)\n",
                    (uint64_t)clk, misses_per_cycle, misses_per_llc_miss,
                    last_cows_miss_count, cows_delta_miss, last_llc_miss_count, llc_delta_miss);

            last_print = clk;
            last_cows_miss_count = s_misses;
            last_llc_miss_count = (uint64_t)total_llc_misses;
        }
#endif

        m_stats_cows_miss_clk.push_back(clk);
    }
};


}; // namespace
