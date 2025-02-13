#include "cows_cache.h"

namespace Ramulator {

CoWsCache::CoWsCache(uint32_t nlines,
                     uint32_t assoc,
                     uint32_t cache_line_bytes,
                     uint32_t dram_page_bytes,
                     uint32_t dram_latency_on_translation)
              : m_nlines(nlines),
                m_nsets(nlines/assoc),
                m_assoc(assoc),
                m_cache_line_bytes(cache_line_bytes),
                m_dram_page_bytes(dram_page_bytes),
                m_dram_latency_on_translation(dram_latency_on_translation),
                m_dram_page_bit_offset(std::countr_zero(dram_page_bytes)),
                m_dram_page_mask(~( (((Addr_t)1)<<m_dram_page_bit_offset) - 1) ),
                m_set_mask((m_nsets-1)<<m_dram_page_bit_offset),
                m_set_offset(m_dram_page_bit_offset),
//                m_cache_sets(m_nsets, std::list<Line>())
                m_cache()
                {
                    assert(is_power_of_2(nlines));
                    assert(is_power_of_2(assoc));
                    assert(is_power_of_2(m_cache_line_bytes));
                    assert(is_power_of_2(m_dram_page_bytes));
                }

bool CoWsCache::lookup(Addr_t block, Line*& ret)
{
    Addr_t page_addr = get_page_addr(block);
    auto it = m_cache.find(page_addr);
    if(it == m_cache.end()) {
        ret = nullptr;
        return false;
    }

    ret = &it->second;
    return true;
}

void CoWsCache::insert(Addr_t block, Addr_t real_phys_addr)
{
    Addr_t page_addr = get_page_addr(block);
    assert(m_cache.find(page_addr) != m_lines.end());

    Line line;
    line.setRealAddr(real_phys_addr);
    line.setBlockID(get_block_id(block), true);

    m_cache.insert({page_addr, line});
}

void CoWsCache::erase(Addr_t block)
{
    Addr_t page_addr = get_page_addr(block);
    assert(m_cache.find(page_addr) == m_lines.end());

    m_cache.erase(page_addr);
}

void CoWsCache::llc_hit(Addr_t block)
{
    Addr_t page_addr = get_page_addr(block);
    uint32_t block_id = get_block_id(block);
    auto it = m_cache.find(page_addr);

    // it's a hit, so the dram page must already be available in the cows cache
    assert(it != m_cache.end());

    // it's a hit, so the block must already be available in the cows cache
    assert(it->second.getBlockID(block_id));
}

uint32_t CoWsCache::llc_evict(Addr_t block)
{
    Addr_t page_addr = get_page_addr(block);
    uint32_t block_id = get_block_id(block);

    auto it = m_cache.find(page_addr);
    assert(it != m_cache.end());

    DBG("block=0x%lx, page_addr=0x%lx, block_id=0x%x",
        block, page_addr, block_id);
    it->second.setBlockID(block_id, false);

    return it->second.getNumBlocks();
}

}; // namespace
