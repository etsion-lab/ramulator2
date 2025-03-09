#include <filesystem>
#include <iostream>
#include <fstream>
#include <regex>

#include <spdlog/spdlog.h>

#include "base/exception.h"
#include "base/utils.h"
#include "frontend/impl/processor/simpleO3/core.h"
#include "frontend/impl/processor/simpleO3/llc.h"

namespace Ramulator {

namespace fs = std::filesystem;

uint32_t SimpleO3Core::Trace::dram_page_bytes = 8192;


SimpleO3Core::Trace::Trace(std::string file_path_str) {
  m_file_path_str = file_path_str;

  // search for app ID
  std::vector<std::string> path_tokens;

  tokenize(path_tokens, file_path_str, ":");
  auto num_path_tokens = path_tokens.size();

  if(num_path_tokens != 1 && num_path_tokens != 2) {
    throw ConfigurationError("Badly formatted trace path {}!", file_path_str);
  }
  if(num_path_tokens == 2) {
    m_asid = std::stoll(path_tokens[1]);
    if(!is_valid_asid(m_asid))
      throw ConfigurationError("Bad ASID {} in trace_path {}!", m_asid, file_path_str);
  }

  printf("# Loading trace file %s with asid %ld (0x%lx)\n",
         path_tokens[0].c_str(), m_asid, m_asid);

  fs::path trace_path(path_tokens[0]);
  if (!fs::exists(trace_path)) {
    throw ConfigurationError("Trace {} does not exist!", file_path_str);
  }

  const std::string IN_UTIL = "zstdcat";
  std::string in_cmd = IN_UTIL + " " + path_tokens[0];
  m_trace_file = std::make_unique<PopenIstream>(in_cmd);
}

bool SimpleO3Core::Trace::read_next_inst()
{
  std::string line;
  const std::regex comment_regex(" *#.*$");

  std::getline(*m_trace_file, line);
  std::vector<std::string> tokens;

  // remove comments (try to avoid regexp since it seems to run slower)
//    line = std::regex_replace(line, comment_regex, "");
  size_t pos;
  if( ((pos = line.find("  #")) != std::string::npos) ||
      ((pos = line.find(" #")) != std::string::npos) ||
      ((pos = line.find("#")) != std::string::npos)) {
    line.resize(pos);
  }
  tokenize(tokens, line, " ");

  int num_tokens = tokens.size();

  if (num_tokens != 2 & num_tokens != 3) {
    throw ConfigurationError("Trace {} format invalid!", m_file_path_str);
  }
  int bubble_count = std::stoi(tokens[0]);

  Addr_t load_addr = std::stoll(tokens[1], nullptr, 0);
  load_addr += get_aslr_offset(m_asid);
  assert(addr_get_asid(load_addr) == 0); // make sure the ASID bits are 0.
  load_addr = addr_set_asid(load_addr, m_asid); // add ASID to the address

  bool has_store = num_tokens == 2 ? false : true;
  if (has_store) {
    Addr_t store_addr = std::stoll(tokens[2], nullptr, 0);
    store_addr += get_aslr_offset(m_asid);
    assert(addr_get_asid(store_addr) == 0);
    store_addr = addr_set_asid(store_addr, m_asid);  // add ASID to the address

    m_trace.push_back({bubble_count, load_addr, store_addr});
  } else {
    m_trace.push_back({bubble_count, load_addr, -1});
  }

  return !m_trace_file->fail();
}


const SimpleO3Core::Trace::Inst& SimpleO3Core::Trace::get_next_inst() {
  // do we need to read the next instruction from file?
  if(m_trace_file != nullptr) {
    if(!read_next_inst()) {
      // just read the last instruction
      m_trace_file.reset(); // frees trace file and makes unique_ptr==nullptr
    }
    m_trace_length = m_trace.size();
  }

  const Inst& inst = m_trace[m_curr_trace_idx];
  m_curr_trace_idx = (m_curr_trace_idx + 1) % m_trace_length;
  return inst;
}


SimpleO3Core::InstWindow::InstWindow(int ipc, int depth):
m_ipc(ipc), m_depth(depth),
m_ready_list(depth, false), m_addr_list(depth, -1) {};

bool SimpleO3Core::InstWindow::is_full() {
  return m_load == m_depth;
}

void SimpleO3Core::InstWindow::insert(bool ready, Addr_t addr) {
  m_ready_list.at(m_head_idx) = ready;
  m_addr_list.at(m_head_idx) = addr;

  m_head_idx = (m_head_idx + 1) % m_depth;
  m_load++;
}

int SimpleO3Core::InstWindow::retire() {
  if (m_load == 0) return 0;

  int num_retired = 0;
  while (m_load > 0 && num_retired < m_ipc) {
    if (!m_ready_list.at(m_tail_idx))
      break;

    m_tail_idx = (m_tail_idx + 1) % m_depth;
    m_load--;
    num_retired++;
  }
  return num_retired;
}

void SimpleO3Core::InstWindow::set_ready(Addr_t addr) {
  if (m_load == 0) return;

  int index = m_tail_idx;
  for (int i = 0; i < m_load; i++) {
    if (m_addr_list[index] == addr) {
      m_ready_list[index] = true;
    }
    index++;
    if (index == m_depth) {
      index = 0;
    }
  }
}

SimpleO3Core::SimpleO3Core(int id, int ipc, int depth, size_t num_expected_insts, std::string trace_path, ITranslation* translation, SimpleO3LLC* llc):
m_id(id), m_window(ipc, depth), m_trace(trace_path), m_num_expected_insts(num_expected_insts), m_translation(translation), m_llc(llc) {
  // Fetch the instructions and addresses for tick 0
  auto inst = m_trace.get_next_inst();
  m_num_bubbles = inst.bubble_count;
  m_load_addr = inst.load_addr;
  m_writeback_addr = inst.store_addr;
}

void SimpleO3Core::tick() {
  m_clk++;

  s_insts_retired += m_window.retire();

  if(m_id == 0) {
    const uint32_t PRINT_EVERY_INSTS = 1000000;
    static uint64_t last_printed_insts = 0;
    static uint64_t prev_insts_retired = 0;
    static uint64_t prev_clk = 0;
    if(s_insts_retired > last_printed_insts + PRINT_EVERY_INSTS) {
      auto insts = s_insts_retired - prev_insts_retired;
      auto cycle = m_clk - prev_clk;
      fprintf(stderr, "Core %u retired %lu insts in %lu cycles (curr ipc=%.3f).\n  ",
              m_id, s_insts_retired, m_clk, (1.0f*insts)/cycle);
      last_printed_insts += PRINT_EVERY_INSTS;
      prev_insts_retired = s_insts_retired;
      prev_clk = m_clk;
    }
  }

  if (!reached_expected_num_insts) {
    if (s_insts_retired >= m_num_expected_insts) {
      reached_expected_num_insts = true;
      s_cycles_recorded = m_clk;
    }
  }

  // First, issue the non-memory instructions
  int num_inserted_insts = 0;
  while (m_num_bubbles > 0) {
//      printf("m_clk=%ld, m_num_bubbles==%d\n", (unsigned long)m_clk, m_num_bubbles);
      if (num_inserted_insts == m_window.m_ipc) {
	  return;
      }
      if (m_window.is_full()) {
	  return;
      };
      m_window.insert(true, -1);
      num_inserted_insts++;
      m_num_bubbles--;
  }

  // Second, try to send the load to the LLC
  if (m_load_addr != -1) {
    if (num_inserted_insts == m_window.m_ipc) {
      return;
    }
    if (m_window.is_full()) {
      return;
    };

    Request load_request(m_load_addr, Request::Type::Read, m_id, m_callback);
    if (!m_translation->translate(load_request)) {
      return;
    };

    if (m_llc->send(load_request)) {
      m_window.insert(false, load_request.addr);
      m_load_addr = -1;
      if (m_writeback_addr != -1) {
        // If there is still writeback, return without getting the next trace line
        // The write back will be issued in the next cycle
        // TODO: Should we allow both load and writeback to issue at the same cycle?
        return;
      }
    } else {
      return;
    }
  }

  // Third, try to send the writeback to the LLC
  if (m_writeback_addr != -1) {
    Request writeback_request(m_writeback_addr, Request::Type::Write, m_id, m_callback);
    if (!m_translation->translate(writeback_request)) {
      return;
    };
    if (!m_llc->send(writeback_request)) {
      return;
    }
  }

  auto inst = m_trace.get_next_inst();
  m_num_bubbles = inst.bubble_count;
  m_load_addr = inst.load_addr;
  m_writeback_addr = inst.store_addr;
}

void SimpleO3Core::receive(Request& req) {
  m_window.set_ready(req.addr);

  if (req.arrive != -1 && req.depart > m_last_mem_cycle) {
    if (!reached_expected_num_insts) {
      s_mem_access_cycles += (req.depart - std::max(m_last_mem_cycle, req.arrive));
      m_last_mem_cycle = req.depart;
    }
  }
}

}        // namespace Ramulator
