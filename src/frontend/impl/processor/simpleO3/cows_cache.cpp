//#define DEBUG_COWS

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <iostream>
#include <fstream>

#include "cows_cache.h"


namespace Ramulator {

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
                m_set_mask((m_nsets-1)<<m_dram_page_bit_offset),
                m_set_offset(m_dram_page_bit_offset),
                m_lines_in_llc(0),
                m_cows2llc_valid(),
//                m_cache_sets(m_nsets, std::list<Line>())
                m_cache()
                {
                    assert(is_power_of_2(nlines));
                    assert(is_power_of_2(assoc));
                    assert(is_power_of_2(m_cache_line_bytes));
                    assert(is_power_of_2(m_dram_page_bytes));
                }

bool CoWsCache::lookup(Addr_t baddr, Line*& ret)
{
    Addr_t page_addr = get_page_addr(baddr);
    auto it = m_cache.find(page_addr);
    if(it == m_cache.end()) {
        ret = nullptr;
        return false;
    }

    ret = &it->second;
    return true;
}

void CoWsCache::insert(Addr_t baddr, Addr_t real_phys_addr)
{
    Addr_t page_addr = get_page_addr(baddr);
    uint32_t block_id = get_block_id(baddr);
    assert(m_cache.find(page_addr) == m_cache.end()); // make sure page is not already in the cache

    DBG("INSERT: baddr=0x%lx, page_addr=0x%lx, block_id=0x%x",
        baddr, page_addr, block_id);

    // insert new line and set params
    Line& line = m_cache[page_addr];
    line.setRealAddr(real_phys_addr);
    line.setBlockID(block_id, true);

    miss_after_insert = true;
}

void CoWsCache::erase(Addr_t baddr)
{
    Addr_t page_addr = get_page_addr(baddr);
    assert(m_cache.find(page_addr) != m_cache.end()); // make sure page is already in the cache

    m_cache.erase(page_addr);
}

void CoWsCache::llc_hit(Addr_t baddr)
{
    Addr_t page_addr = get_page_addr(baddr);
    uint32_t block_id = get_block_id(baddr);
    auto it = m_cache.find(page_addr);

    // it's a hit, so the dram page must already be available in the cows cache
    assert(it != m_cache.end());

    // it's a hit, so the block must already be available in the cows cache
    assert(it->second.getBlockID(block_id));
}

void CoWsCache::llc_miss(Addr_t baddr)
{
    Addr_t page_addr = get_page_addr(baddr);
    uint32_t block_id = get_block_id(baddr);
    auto it = m_cache.find(page_addr);

    //  the page should already be in the cache
    assert(it != m_cache.end());

    DBG("MISS: baddr=0x%lx, page_addr=0x%lx, block_id=0x%x",
        baddr, page_addr, block_id);

    it->second.setBlockID(block_id, true);

    // update stats
    m_lines_in_llc++;

    // just inserted this block? collect stats
    if(miss_after_insert) {
        // update stats
        uint32_t lines_in_cows = (uint32_t)m_cache.size();
        m_cows2llc_valid.push_back({lines_in_cows, m_lines_in_llc});

        miss_after_insert = false;
    }
}

uint32_t CoWsCache::llc_evict(Addr_t baddr)
{
    Addr_t page_addr = get_page_addr(baddr);
    uint32_t block_id = get_block_id(baddr);

    auto it = m_cache.find(page_addr);
    assert(it != m_cache.end());

    DBG("EVICT: baddr=0x%lx, page_addr=0x%lx, block_id=0x%x",
        baddr, page_addr, block_id);
    it->second.setBlockID(block_id, false);

    // update stats
    m_lines_in_llc--;

    return it->second.getNumBlocks();
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
