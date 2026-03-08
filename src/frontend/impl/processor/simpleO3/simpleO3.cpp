#include <functional>

#include "base/utils.h"
#include "frontend/frontend.h"
#include "translation/translation.h"
#include "frontend/impl/processor/simpleO3/core.h"
#include "frontend/impl/processor/simpleO3/llc.h"


namespace Ramulator {

class SimpleO3 final : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, SimpleO3, "SimpleO3", "Simple timing model OoO processor frontend.")

  private:
    ITranslation*  m_translation;

    int m_num_cores = -1;
    std::vector<SimpleO3Core*> m_cores;
    SimpleO3LLC* m_llc;
    CoWsCache* m_cows_cache;

    size_t m_num_expected_insts = 0;
    size_t m_warmup_percent = 0;

    std::string serialization_filename;

  public:
    void init() override {
      m_clock_ratio = param<uint>("clock_ratio").required();

      // Core params
      std::vector<std::string> trace_list = param<std::vector<std::string>>("traces").desc("A list of traces.").required();
      m_num_cores = trace_list.size();

      int ipc   = param<int>("ipc").desc("IPC of the SimpleO3 core.").default_val(4);
      int depth = param<int>("inst_window_depth").desc("Instruction window size of the SimpleO3 core.").default_val(128);
      uint32_t lat_factor = param<uint32_t>("latency_factor").desc("Multiple all memory latencies by that factor.").default_val(1);
      set_latency_factor(lat_factor);

      // LLC params
      int llc_latency           = param<int>("llc_latency").desc("Aggregated latency of the LLC.").default_val(47);
      int llc_linesize_bytes    = param<int>("llc_linesize").desc("LLC cache line size in bytes.").default_val(64);
      int llc_associativity     = param<int>("llc_associativity").desc("LLC set associativity.").default_val(8);
      int llc_capacity_per_core = parse_capacity_str(param<std::string>("llc_capacity_per_core").desc("LLC capacity per core.").default_val("2MB"));
      int llc_num_mshr_per_core = param<int>("llc_num_mshr_per_core").desc("Number of LLC MSHR entries per core.").default_val(16);

      bool cows_accel_zero_page = param<bool>("cows_accel_zero_page").desc("Accelerate zero page copy").required();

      bool cows_enable = param<bool>("cows_enable").desc("Enable CoWs.").required();
      uint32_t dram_latency_on_translation = param<int>("cows_dram_latency_on_translation").desc("DRAM latency for COWs mapping translations.").required();
      uint32_t cows_cache_access_latency = param<int>("cows_cache_access_latency").desc("No. of cycles to access the CoWs cache.").required();
      uint32_t llc2cows_ratio = param<uint32_t>("cows_cache2llc_ratio").desc("Ratio between the number of LLC cache lines and CoWs cache entries.").required();
      uint32_t cows_cache_assoc = param<uint32_t>("cows_cache_assoc").desc("Associativity of CoWs cache.").required();
      uint32_t dram_page_bytes = parse_capacity_str(param<std::string>("dram_page_bytes").desc("size of DRAM page.").required());
      uint32_t always_miss = param<bool>("cows_always_miss").desc("Always miss in CoWs cache.").required();
      std::string cows_replacement = param<std::string>("cows_replacement").desc("Replacement policy in CoWs cache.").required();
      CoWsCache::ReplPolicy *cows_policy = nullptr;
      if(cows_replacement == "LRU_nohit") {
        cows_policy = new CoWsCache::LRU_nohit();
      } else {
        throw ConfigurationError("Unknown CoWs replacement policy {}!", cows_replacement);
      }
      std::cerr<<"# cows_replacement: "<<cows_policy->name()<<std::endl;

      CoWsCache::CoWsStats cows_stats;
      cows_stats.stats_fname = param<std::string>("cows_stats_file").desc("Filename for to dump COWS stats.").default_val("");

      // Simulation parameters
      m_num_expected_insts = param<size_t>("num_expected_insts").desc("Number of instructions that the frontend should execute.").required();
      m_warmup_percent = param<size_t>("warmup_percent").desc("Percent of the run that is considered warmup.").required();

      // Create address translation module
      m_translation = create_child_ifce<ITranslation>();

      // Create CoWsCache
      m_cows_cache = nullptr;
      if(cows_enable) {
        std::cerr<<"# Creating CoWsCache"<<std::endl;

        uint32_t llc_nlines = llc_capacity_per_core * m_num_cores / llc_linesize_bytes;
        uint32_t cows_nlines = llc_nlines / llc2cows_ratio;
        // cows line number must be a multiple of its assoc
        //cows_nlines = cows_cache_assoc * ((cows_nlines + cows_cache_assoc - 1) / cows_cache_assoc);
        assert(cows_nlines % cows_cache_assoc == 0);
        assert(cows_nlines >= cows_cache_assoc);

        std::cerr<<"# LLC bytes="<<(llc_nlines*llc_linesize_bytes)<<" (or "<<llc_capacity_per_core*m_num_cores<<"), lines: "<<llc_nlines<<", COWS lines: "<<cows_nlines<<std::endl;

        m_cows_cache = new CoWsCache(cows_nlines,
                                    cows_cache_assoc,
                                    llc_linesize_bytes,
                                    dram_page_bytes,
                                    always_miss,
                                    cows_cache_access_latency,
                                    dram_latency_on_translation,
                                    cows_policy,
                                    cows_stats);

        std::cerr<<"# Finished creating CoWsCache"<<std::endl;
      }

      // Create the LLC
      m_llc = new SimpleO3LLC(llc_latency, llc_capacity_per_core * m_num_cores, llc_linesize_bytes, llc_associativity, llc_num_mshr_per_core * m_num_cores, m_cows_cache);
      // m_llc->deserialize(serialization_filename);
      // m_llc->serialize(serialization_filename);

      // Create the cores
      SimpleO3Core::Trace::dram_page_bytes = dram_page_bytes;
      for (int id = 0; id < m_num_cores; id++) {
        SimpleO3Core* core = new SimpleO3Core(id, ipc, depth, m_num_expected_insts, m_warmup_percent, trace_list[id], cows_accel_zero_page, m_translation, m_llc);
        core->m_callback = [this](Request& req){return this->receive(req);} ;
        m_cores.push_back(core);
      }

      m_logger = Logging::create_logger("SimpleO3");

      // Register the stats
      register_stat(m_num_expected_insts, false).name("num_expected_insts");// do not reset this counter
      register_stat(m_llc->s_llc_eviction).name("llc_eviction");
      register_stat(m_llc->s_llc_read_access).name("llc_read_access");
      register_stat(m_llc->s_llc_write_access).name("llc_write_access");
      register_stat(m_llc->s_llc_read_misses).name("llc_read_misses");
      register_stat(m_llc->s_llc_write_misses).name("llc_write_misses");
      register_stat(m_llc->s_llc_mshr_unavailable).name("llc_mshr_unavailable");

      static auto tot_llc_accesses = ADDi(m_llc->s_llc_read_access, m_llc->s_llc_write_access);
      static auto tot_llc_misses = ADDi(m_llc->s_llc_read_misses, m_llc->s_llc_write_misses);
      register_stat(tot_llc_accesses, false).name("llc_total_access"); // do not reset compount counters
      register_stat(tot_llc_misses, false).name("llc_total_misses");

      if(m_cows_cache != nullptr) {
        register_stat(m_cows_cache->s_hits).name("cows_cache_hits");
        register_stat(m_cows_cache->s_misses).name("cows_cache_misses");
        register_stat(m_cows_cache->s_access).name("cows_cache_access");
        register_stat(m_cows_cache->s_cows_cycles).name("cows_access_cycles");
        register_stat(m_cows_cache->s_cows_cycles_self).name("cows_access_cycles_self");
        register_stat(m_cows_cache->s_access).name("cows_accesses");
        register_stat(m_cows_cache->s_accesses_on_llc_hit).name("cows_accesses_on_llc_hit");
        register_stat(m_cows_cache->s_accesses_on_llc_miss).name("cows_accesses_on_llc_miss");
        register_stat(m_cows_cache->s_accesses_on_llc_evict).name("cows_accesses_on_llc_evict");
        register_stat(m_cows_cache->s_misses_on_llc_hit).name("cows_misses_on_llc_hit");
        register_stat(m_cows_cache->s_misses_on_llc_miss).name("cows_misses_on_llc_miss");
        register_stat(m_cows_cache->s_misses_on_llc_evict).name("cows_misses_on_llc_evict");
        register_stat(m_cows_cache->s_misses).name("cows_misses");
        register_stat(m_cows_cache->s_hits).name("cows_hits");
        register_stat(m_cows_cache->s_evicts).name("cows_evicts");
        static auto cows_missrate = DIV(m_cows_cache->s_misses, m_cows_cache->s_access);
        register_stat(cows_missrate, false).name("cows_missrate");

        static auto cows2llc_miss_ratio = DIV(m_cows_cache->s_misses, tot_llc_misses);
        register_stat(cows2llc_miss_ratio, false).name("cows2llc_miss_ratio");

        static auto avg_dram_lat = DIV(m_llc->s_avg_dram_read_lat_sum, m_llc->s_avg_dram_read_lat_cnt);
        register_stat(avg_dram_lat, false).name("avg_dram_read_latency");

        static auto avg_total_lat = DIV(m_llc->s_avg_total_read_lat_sum, m_llc->s_avg_total_read_lat_cnt);
        register_stat(avg_total_lat, false).name("avg_total_read_latency");

        static auto avg_total_miss_lat = DIV(m_llc->s_avg_total_read_miss_lat_sum, m_llc->s_avg_total_read_miss_lat_cnt);
        register_stat(avg_total_miss_lat, false).name("avg_total_read_miss_latency");
      }

      std::vector<size_t*> tot_insts_post_warmup;
      std::vector<size_t*> tot_cycles_post_warmup;
      for (int core_id = 0; core_id < m_cores.size(); core_id++) {
        register_stat(m_cores[core_id]->s_insts_retired, false).name("cycles_retired_core_{}", core_id);
        register_stat(m_cores[core_id]->s_cycles_recorded, false).name("cycles_recorded_core_{}", core_id);
        register_stat(m_cores[core_id]->s_cycles_recorded_post_warmup).name("cycles_recorded_post_warmup_core_{}", core_id);
        register_stat(m_cores[core_id]->s_insts_retired_post_warmup).name("insts_retired_post_warmup_core_{}", core_id);
        register_stat(m_cores[core_id]->s_mem_access_cycles).name("memory_access_cycles_recorded_core_{}", core_id);
        auto frac_cycles_post_warmup = new DIV(m_cores[core_id]->s_cycles_recorded_post_warmup, m_cores[core_id]->s_cycles_recorded);
        auto frac_insts_post_warmup = new DIV(m_cores[core_id]->s_insts_retired_post_warmup, m_num_expected_insts);
        register_stat(*frac_cycles_post_warmup, false).name("frac_cycles_post_warmup_{}", core_id);
        register_stat(*frac_insts_post_warmup, false).name("frac_insts_post_warmup_{}", core_id);

        auto core_ipc = new DIV(m_cores[core_id]->s_insts_retired_post_warmup, m_cores[core_id]->s_cycles_recorded_post_warmup);
        register_stat(*core_ipc, false).name("ipc_core_{}", core_id);


        tot_insts_post_warmup.push_back(&m_cores[core_id]->s_insts_retired_post_warmup);
        tot_cycles_post_warmup.push_back(&m_cores[core_id]->s_cycles_recorded_post_warmup);
      }
      auto tot_insts = new SUM(tot_insts_post_warmup);
      auto tot_cycles = new SUM(tot_cycles_post_warmup);
      auto tot_ipc = new DIV(*tot_insts, *tot_cycles);
      register_stat(*tot_ipc, false).name("ipc_total");

    }

    void tick() override {
      m_clk++;

      if(m_clk % 1000000 == 0) {
        m_logger->info("Processor Heartbeat {} cycles.", m_clk);
      }

      m_llc->tick();
      for (auto core : m_cores) {
        core->tick();
      }
    }

    void receive(Request& req) {
      req.done = m_clk;
      m_llc->receive(req);

      // TODO: LLC latency for the core to receive the request?
      for (auto r : m_llc->m_receive_requests[req.addr]) {
        r.done = m_clk;

        // collect total read latency samples
        auto total_read_latency = r.done - r.core2llc;
        m_llc->s_avg_total_read_lat_sum += total_read_latency;
        m_llc->s_avg_total_read_lat_cnt += 1;
        m_llc->s_stats_read_latencies.push_back(std::make_tuple(r.addr, total_read_latency));

        if(r.cache_status != Request::CacheStatus::Hit) {
          m_llc->s_avg_total_read_miss_lat_sum += total_read_latency;
          m_llc->s_avg_total_read_miss_lat_cnt += 1;
          m_llc->s_stats_read_miss_latencies.push_back(std::make_tuple(r.addr, total_read_latency));
        }

        m_cores[r.source_id]->receive(r);
      }
      m_llc->m_receive_requests[req.addr].clear();
    };

    bool is_finished() override {
      for (auto core : m_cores) {
        if (!(core->reached_expected_num_insts)){
          return false;
        }
      }
      if(m_cows_cache != nullptr) {
        m_cows_cache->fini();
      }
      m_llc->fini();

      return true;
    }

#if 1
    bool is_warmup_finished() override {
      for (auto core : m_cores) {
        if (!(core->finished_warmup)){
          return false;
        }
      }

      return true;
    }
#endif

    void connect_memory_system(IMemorySystem* memory_system) override {
      m_llc->connect_memory_system(memory_system);
    };

    int get_num_cores() override {
      return m_num_cores;
    };
};

}        // namespace Ramulator
