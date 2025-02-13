#include "cows_cache.h"

namespace Ramulator {

CoWsCache::CoWsCache(uint32_t nlines,
                     uint32_t assoc,
                     uint32_t cache_line_bytes,
                     uint32_t dram_page_bytes)
              : m_nlines(nlines),
                m_nsets(nlines/assoc),
                m_assoc(assoc),
                m_cache_line_bytes(cache_line_bytes),
                m_dram_page_bytes(dram_page_bytes),
                m_dram_page_bit_offset(std::countr_zero(dram_page_bytes)),
                m_set_mask((m_nsets-1)<<m_dram_page_bit_offset),
                m_set_offset(m_dram_page_bit_offset),
                m_cache_sets(m_nsets, std::list<Line>()) {
                    assert(is_power_of_2(nlines));
                    assert(is_power_of_2(assoc));
                    assert(is_power_of_2(m_cache_line_bytes));
                    assert(is_power_of_2(m_dram_page_bytes));
                }

bool CoWsCache::lookup(Addr_t block, Line& ret)
{
    return false;
}

void CoWsCache::insert(Addr_t block, Addr_t real_phys_addr, Line& victim)
{

}

};
