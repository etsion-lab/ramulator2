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

CoWsCache::CoWsCache(uint32_t nlines,
                     uint32_t assoc,
                     uint32_t cache_line_bytes,
                     uint32_t dram_page_bytes,
                     uint32_t access_latency,
                     uint32_t dram_latency_on_translation,
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
                m_cows2llc_valid(),
                m_perfect_cache(),
                m_cache_sets(m_nsets, std::list<Line>())
{

    std::cerr<<"# Creating CoWsCache"<<std::endl;

    assert(is_power_of_2(nlines));
    assert(is_power_of_2(assoc));
    assert(is_power_of_2(m_cache_line_bytes));
    assert(is_power_of_2(m_dram_page_bytes));
}

bool CoWsCache::lookup(Addr_t baddr, Line*& ret)
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

void CoWsCache::insert(Addr_t page_addr, Addr_t real_phys_addr)
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
    line.setPageAddr(page_addr);
    line.setValid(true);
    line.setRealAddr(real_phys_addr);
}

void CoWsCache::erase(Addr_t baddr)
{
    Addr_t page_addr = get_page_addr(baddr);
    assert(m_perfect_cache.find(page_addr) != m_perfect_cache.end()); // make sure page is already in the cache

    // poison the Line before erasing it
    auto it = m_perfect_cache.find(page_addr);
    it->second.setValid(false);
    it->second.setPageAddr(0xdeadbeef12345678L);
    m_perfect_cache.erase(it);
}

// this function is here as a placeholder for collecting statistics
void CoWsCache::llc_hit(Addr_t baddr)
{
    Addr_t page_addr = get_page_addr(baddr);
    uint32_t block_id = get_block_id(baddr);
    auto it = m_perfect_cache.find(page_addr);

    // it's a hit, so the dram page must already be available in the cows cache
    assert(it != m_perfect_cache.end());

    // it's a hit, so the block must already be available in the cows cache
    assert(it->second.getBlockID(block_id));
}

uint32_t CoWsCache::llc_miss(Addr_t baddr)
{
    Addr_t page_addr = get_page_addr(baddr);
    uint32_t block_id = get_block_id(baddr);
    // miss latency is at least a cows cache access latency
    uint32_t miss_latency = m_access_latency;

    // update stats
    m_lines_in_llc++;

    // first update the perfect cache
    auto it = m_perfect_cache.find(page_addr);
    if(it == m_perfect_cache.end()) {
        insert(page_addr, page_addr + 1<<20);
        it = m_perfect_cache.find(page_addr);

        // update stats
        uint32_t lines_in_cows = (uint32_t)m_perfect_cache.size();
        m_cows2llc_valid.push_back({lines_in_cows, m_lines_in_llc});

        miss_latency += get_dram_latency_on_translation(page_addr);
    }
    it->second.setBlockID(block_id, true);

    // now update the real cache
    #if 0
auto page_addr = get_page_addr(baddr);
auto block_id = get_block_id(baddr);

CacheSet_t& set = m_cache_sets[get_set_idx(page_addr)];
line.setBlockID(block_id, true);
auto line_it = find_in_set(set, page_addr);
#endif


    DBG("MISS: baddr=0x%lx, page_addr=0x%lx, block_id=0x%x",
        baddr, page_addr, block_id);

    return miss_latency;
}

uint32_t CoWsCache::llc_evict(Addr_t baddr)
{
    Addr_t page_addr = get_page_addr(baddr);
    uint32_t block_id = get_block_id(baddr);

    auto it = m_perfect_cache.find(page_addr);
    assert(it != m_perfect_cache.end());

    DBG("EVICT: baddr=0x%lx, page_addr=0x%lx, block_id=0x%x",
        baddr, page_addr, block_id);
    it->second.setBlockID(block_id, false);

    // update stats
    m_lines_in_llc--;

    auto cnt = it->second.getNumBlocks();

    // perfect cache: no more page lines in llc? evict
    if(cnt == 0) {
        erase(page_addr);
    }

    return cnt;
}

void CoWsCache::fini()
{
    static char outbuf[1024];

    if(m_stats.stats_fname.size() == 0) // no stats
        return;

    auto of = std::ofstream(m_stats.stats_fname + ".frac-of-entries");

    std::cout<<"# Dumping CoWs stats"<<std::endl;
    of<<"# CoWs stats"<<std::endl;
    for(auto it : m_cows2llc_valid) {
        auto cows = it.first;
        auto llc = it.second;
        float ratio = (1.0*cows) / llc;

        snprintf(outbuf, 1024, "%12u%12u%12.3f", cows, llc, ratio);
        of<<outbuf<<std::endl;
    }
}

}; // namespace
