//#define DEBUG_COWS

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include "cows_cache.h"


namespace Ramulator {

// initialize variables to illegal values, so we can flush out bugs
uint64_t CoWsCache::Line::bytes_per_line = 0;
uint64_t CoWsCache::Line::dram_page_bytes = 0;

std::string
CoWsCache::Line::toString() const
{
    std::ostringstream oss;

    uint64_t lo=0;
    uint64_t hi=0;

    for(uint64_t i=0; i<64; i++)
        lo |= (block_map[i]) ? 0x1 : 0x0;

    for(uint64_t i=64; i<127; i++)
        hi |= (block_map[i]) ? 0x1 : 0x0;

    oss<<"page_phys_addr=0x"<<std::hex<<page_phys_addr
        <<", "<<"valid="<<valid
        <<", map=0x"
        << std::setfill('0') << std::setw(8) << std::right << std::hex << hi << "_"
        << std::setfill('0') << std::setw(8) << std::right << std::hex << lo;

    return oss.str();
}

bool CoWsCache::LRU_nohit::llc_hit(CoWsCache& cows_cache, CacheSet_t& set, Addr_t page_addr, uint32_t block_id, bool is_write) const
{
    // on writes we access the mapping in the cows cache to update the block map, so we need it.

    // miss? get the page's cows entry from memory
    bool miss = false;
    auto it = find_in_set(set, page_addr);
    if(it == set.end()) {
        miss = true;

        it = alloc_line(cows_cache, set, page_addr);
    }

    // block bit not set? set it and mark dirty
    if(!it->getBlockID(block_id)) {
        it->setBlockID(block_id, true);
        it->setDirty(true);
    }

    return miss;
}

bool CoWsCache::LRU_nohit::llc_miss(CoWsCache& cows_cache, CacheSet_t& set, Addr_t page_addr, uint32_t block_id, bool is_write) const
{
    bool miss = false;
    // on a miss we access the mapping the cows cache (and insert it if it's not there)
    auto it = find_in_set(set, page_addr);
    if(it == set.end()) {
        miss = true;

        it = alloc_line(cows_cache, set, page_addr);
    }

    // block bit not set? set it and mark dirty
    if(is_write && !it->getBlockID(block_id)) {
        it->setBlockID(block_id, true);
        it->setDirty(true);
    }

    // move the accessed line to MRU (tail of list)
    move_to_mru(set, it);

    return miss;
}

bool CoWsCache::LRU_nohit::llc_evict(CoWsCache& cows_cache, CacheSet_t& set, Addr_t page_addr, uint32_t block_id) const
{
    bool miss = false;
    auto it = find_in_set(set, page_addr);
    if(it == set.end()) {
        miss = true;

        it = alloc_line(cows_cache, set, page_addr);
    }

    // move the accessed line to MRU (tail of list)
    move_to_mru(set, it);

    return miss;
}

CoWsCache::CoWsCache(uint32_t nlines,
                     uint32_t assoc,
                     uint32_t cache_line_bytes,
                     uint32_t dram_page_bytes,
                     uint32_t access_latency,
                     uint32_t dram_latency_on_translation,
                     const CoWsCache::ReplPolicy* policy,
                     const CoWsStats& stats)
              : m_nlines(nlines),
                m_nsets(nlines/assoc),
                m_assoc(assoc),
                m_cache_line_bytes(cache_line_bytes),
                m_dram_page_bytes(dram_page_bytes),
                m_access_latency(access_latency),
                m_stats(stats),
                m_dram_latency_on_translation(dram_latency_on_translation),
                m_dram_page_bit_offset(std::countr_zero(dram_page_bytes)),
                m_dram_page_mask(~( (((Addr_t)1)<<m_dram_page_bit_offset) - 1) ),
                m_lines_in_llc(0),
                m_stats_cows2llc_valid(),
                m_stats_cows_miss_clk(),
                m_stats_set_access(m_nsets),
                m_stats_set_miss(m_nsets),
                m_perfect_cache(),
                m_cache_sets(m_nsets, CacheSet_t()),
                m_policy(policy)
{
    assert(is_power_of_2(nlines));
    assert(is_power_of_2(assoc));
    assert(is_power_of_2(m_cache_line_bytes));
    assert(is_power_of_2(m_dram_page_bytes));

    // set global cache line parameters
    CoWsCache::Line::dram_page_bytes = dram_page_bytes;
    CoWsCache::Line::bytes_per_line = cache_line_bytes;

    std::cerr<<"# COWs cache: bytes="<<(m_nsets*m_assoc*20)<<", nlines="<<m_nlines<<", sets="<<m_nsets<<", m_assoc="<<m_assoc<<std::endl;
}

bool CoWsCache::perfect_cache_lookup(Addr_t baddr, Line*& ret)
{
    Addr_t page_addr = get_page_addr(baddr);
    auto it = m_perfect_cache.find(page_addr);
    if(it == m_perfect_cache.end()) {
        ret = nullptr;
        return false;
    }

    ret = &it->second;
    return true;
}

void CoWsCache::perfect_cache_insert(Addr_t page_addr, Addr_t real_phys_addr)
{
#if 0 /* debug */
    if(m_perfect_cache.find(page_addr) != m_perfect_cache.end()) {
        auto it = m_perfect_cache.find(page_addr);
        std::cerr<<">>>>>> key=0x"<<std::hex<<it->first
                 <<", val="<<it->second.toString()
                 <<std::endl;
    }
#endif

    assert(m_perfect_cache.find(page_addr) == m_perfect_cache.end()); // make sure page is not already in the cache

    // insert new line and set mapping
    Line& line = m_perfect_cache[page_addr];
    line.setTag(page_addr);
    line.setValid(true);
    line.setRealAddr(real_phys_addr);
}

void CoWsCache::perfect_cache_erase(Addr_t baddr)
{
    Addr_t page_addr = get_page_addr(baddr);
    assert(m_perfect_cache.find(page_addr) != m_perfect_cache.end()); // make sure page is already in the cache

    // poison the Line before erasing it
    auto it = m_perfect_cache.find(page_addr);
    it->second.setValid(false);
    it->second.setTag(0xdeadbeef12345678L);
    m_perfect_cache.erase(it);
}

// this function is here as a placeholder for collecting statistics and calling the ReplPolicy hit method
uint32_t CoWsCache::llc_hit(Addr_t baddr, bool is_write, Clk_t clk, int total_llc_misses)
{
    // on read hit we don't need to access the cows cache at all.
    if(!is_write) {
        return 0;
    }

    Addr_t page_addr = get_page_addr(baddr);
    uint32_t block_id = get_block_id(baddr);
    auto it = m_perfect_cache.find(page_addr);

    // miss latency is at least a cows cache access latency
    uint32_t hit_lat = m_access_latency;

    //
    // perfect cache sanity
    //
    // it's a hit, so the dram page must already be available in the cows cache
    assert(it != m_perfect_cache.end());

    // it's a hit, so the block must already be available in the cows cache
    assert(it->second.getBlockID(block_id));

    //
    // access cows cache
    //
    // tell the replacement policy's we have an llc write hit
    auto set_idx = get_set_idx(page_addr);
    auto set = m_cache_sets[set_idx];
    auto cows_cache_miss = m_policy->llc_hit(*this, set, page_addr, block_id, is_write);

    // gather cows stats
    ++s_access;
    ++s_accesses_on_llc_hit;
    m_stats_set_access[set_idx]++;
    if(cows_cache_miss) {
        ++s_misses;
        ++s_misses_on_llc_hit;
        m_stats_set_miss[set_idx]++;

        track_misses(clk, total_llc_misses);

        // miss latency is at least a cows cache access latency
        s_cows_cycles += get_dram_latency_on_translation(page_addr);
        hit_lat += get_dram_latency_on_translation(page_addr);
    }
    else {
        ++s_hits;
    }

    return hit_lat;
}

uint32_t CoWsCache::llc_miss(Addr_t baddr, bool is_write, Clk_t clk, int total_llc_misses)
{
    Addr_t page_addr = get_page_addr(baddr);
    uint32_t block_id = get_block_id(baddr);
    // miss latency is at least a cows cache access latency
    uint32_t miss_latency = m_access_latency;

    // update stats
    m_lines_in_llc++;

    //
    // first update the perfect cache
    //
    auto it = m_perfect_cache.find(page_addr);
    if(it == m_perfect_cache.end()) {
        perfect_cache_insert(page_addr, page_addr + 1<<20);
        it = m_perfect_cache.find(page_addr);

        // update stats
        uint32_t lines_in_cows = (uint32_t)m_perfect_cache.size();
        m_stats_cows2llc_valid.push_back({lines_in_cows, m_lines_in_llc});
    }
    it->second.setBlockID(block_id, true);

    //
    // now update the real cache we had a miss
    //
    auto set_idx = get_set_idx(page_addr);
    auto& set = m_cache_sets[set_idx];
    auto cows_cache_miss = m_policy->llc_miss(*this, set, page_addr, block_id, is_write);

    ++s_access;
    ++s_accesses_on_llc_miss;
    m_stats_set_access[set_idx]++;
    if(cows_cache_miss) {
        ++s_misses;
        ++s_misses_on_llc_miss;
        m_stats_set_miss[set_idx]++;

        track_misses(clk, total_llc_misses);

        miss_latency += get_dram_latency_on_translation(page_addr);
    }
    else {
        ++s_hits;
    }

    s_cows_cycles += miss_latency;

    return miss_latency;
}

uint32_t CoWsCache::llc_evict(Addr_t baddr, bool evict_dirty, Clk_t clk, int total_llc_misses)
{
    Addr_t page_addr = get_page_addr(baddr);
    uint32_t block_id = get_block_id(baddr);
    uint32_t evict_latency = 0;

    //
    // update perfect cache
    //
    auto it = m_perfect_cache.find(page_addr);
    assert(it != m_perfect_cache.end());
    it->second.setBlockID(block_id, false);

    // update stats
    m_lines_in_llc--;

    // perfect cache: no more page lines in llc? evict
    auto cnt = it->second.getNumBlocks();
    if(cnt == 0) {
        perfect_cache_erase(page_addr);
    }

    //
    // now update the real cache we had a miss
    //
    // we only need to access the cows cache if we evict a dirty page (need real mapping)
    if(evict_dirty) {
        evict_latency += m_access_latency;

        auto set_idx = get_set_idx(page_addr);
        auto set = m_cache_sets[set_idx];
        auto cows_cache_miss = m_policy->llc_evict(*this, set, page_addr, block_id);

        ++s_access;
        ++s_accesses_on_llc_evict;
        m_stats_set_access[set_idx]++;
        if(cows_cache_miss) {
            ++s_misses;
            ++s_misses_on_llc_evict;
            m_stats_set_miss[set_idx]++;

            track_misses(clk, total_llc_misses);

            evict_latency += get_dram_latency_on_translation(page_addr);
        }
        else {
            ++s_hits;
        }
    }

    s_cows_cycles += evict_latency;

    return evict_latency;
}

void CoWsCache::fini()
{
    static char outbuf[1024];

    if(m_stats.stats_fname.size() == 0) // no stats
        return;

    // dump stat: fraction on entries in LLC that also exist in CoWs cache
    {
        auto of = std::ofstream(m_stats.stats_fname + ".frac-of-entries");

        std::cout<<"# Dumping CoWs stats"<<std::endl;
        of<<"# CoWs stats"<<std::endl;
        for(auto it : m_stats_cows2llc_valid) {
            auto cows = it.first;
            auto llc = it.second;
            float ratio = (1.0*cows) / llc;

            snprintf(outbuf, 1024, "%12u%12u%12.3f", cows, llc, ratio);
            of<<outbuf<<std::endl;
        }
    }

    // dump stat: cycles between misses
    {
        auto of = std::ofstream(m_stats.stats_fname + ".cycles-between-misses");

        std::cout<<"# Dumping CoWs stats"<<std::endl;
        of<<"# CoWs stats"<<std::endl;
        Clk_t prev_clk = 0;
        for(auto it : m_stats_cows_miss_clk) {
            auto clk = it;
            auto diff = clk - prev_clk;
            prev_clk = clk;

            snprintf(outbuf, 1024, "%12ld%12ld", clk, diff);
            of<<outbuf<<std::endl;
        }
    }

    // dump stat: set popularity
    {
        auto of = std::ofstream(m_stats.stats_fname + ".set-popularity");

        std::cout<<"# Dumping CoWs stats"<<std::endl;
        of<<"# CoWs stats"<<std::endl;
        of<<"# CoWs access: "<<s_access<<std::endl;
        of<<"# CoWs hits: "<<s_hits<<std::endl;
        of<<"# CoWs miss: "<<s_misses<<std::endl;
        for(uint32_t set=0; set<m_nsets; set++) {
            snprintf(outbuf, 1024, "%12u%12lu%12lu", set, m_stats_set_access[set], m_stats_set_miss[set]);
            of<<outbuf<<std::endl;
        }
    }
}

}; // namespace
