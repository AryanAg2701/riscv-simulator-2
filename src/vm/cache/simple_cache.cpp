/**
 * @file simple_cache.cpp
 */
#include "vm/cache/simple_cache.h"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <iostream>

using namespace cache_compat;

Cache::Cache(int cache_size_, int block_size_, int associativity_,
             const std::string& write_back_policy_,
             const std::string& replacement_policy_)
  : cache_size(cache_size_),
    block_size(block_size_),
    associativity(associativity_),
    write_back_policy(write_back_policy_),
    replacement_policy(replacement_policy_) {
  int num_sets = NumSets();
  for (int i = 0; i < num_sets; ++i) {
    std::vector<CacheLine> set_lines;
    set_lines.reserve(static_cast<size_t>(associativity));
    for (int j = 0; j < associativity; ++j) {
      set_lines.emplace_back(block_size);
    }
    table[i] = std::move(set_lines);
  }
}

void Cache::Invalidate() {
  for (auto& kv : table) {
    for (auto& line : kv.second) {
      line.valid = false;
      line.dirty = false;
      line.toa = 0;
    }
  }
}

int Cache::NumSets() const {
  if (block_size == 0 || associativity == 0) return 0;
  return cache_size / (block_size * associativity);
}

int Cache::Index(uint64_t address) const {
  const uint64_t block_number = address / static_cast<uint64_t>(block_size);
  const int num_sets = NumSets();
  if (num_sets == 0) return 0;
  return static_cast<int>(block_number % static_cast<uint64_t>(num_sets));
}

uint64_t Cache::Tag(uint64_t address) const {
  const uint64_t block_number = address / static_cast<uint64_t>(block_size);
  const int num_sets = NumSets();
  if (num_sets <= 0) return 0;
  return static_cast<uint64_t>(block_number / static_cast<uint64_t>(num_sets));
}

uint64_t Cache::BlockBase(uint64_t address) const {
  return (address / static_cast<uint64_t>(block_size)) * static_cast<uint64_t>(block_size);
}

int Cache::SelectVictim(int set_index) {
  // Prefer invalid
  auto& set = table[set_index];
  for (int i = 0; i < static_cast<int>(set.size()); ++i) {
    if (!set[i].valid) return i;
  }
  if (replacement_policy == "RANDOM") {
    // Cheap deterministic psuedo-random based on time and set
    if (set.empty()) return 0;
    return static_cast<int>((timer_ + static_cast<uint64_t>(set_index)) % static_cast<uint64_t>(set.size()));
  }
  // LRU or FIFO: choose min toa
  if (set.empty()) return 0;
  int victim = 0;
  uint64_t best = set[0].toa;
  for (int i = 1; i < static_cast<int>(set.size()); ++i) {
    if (set[i].toa < best) {
      best = set[i].toa;
      victim = i;
    }
  }
  return victim;
}

uint8_t Cache::ReadByte(uint64_t address,
                        const std::function<uint8_t(uint64_t)>& backingRead,
                        const std::function<void(uint64_t, uint8_t)>& backingWrite) {
  // Fallback bypass if misconfigured
  if (block_size <= 0 || associativity <= 0 || NumSets() <= 0) {
    (void)backingWrite;
    return backingRead(address);
  }
  const int idx = Index(address);
  const uint64_t tag = Tag(address);
  const uint64_t base = BlockBase(address);
  const int offset = static_cast<int>(address - base);
  auto& set = table[idx]; // will exist from constructor if NumSets()>0
  if (set.empty()) {
    // Defensive: construct lines on demand to avoid OOB
    set.reserve(static_cast<size_t>(associativity));
    for (int j = 0; j < associativity; ++j) set.emplace_back(block_size);
  }

  for (auto& line : set) {
    if (line.valid && line.tag == tag) {
      // hit
      hits++;
      if (replacement_policy == "LRU") {
        timer_++;
        line.toa = timer_;
      }
      return line.data[static_cast<size_t>(offset)];
    }
  }
  // miss
  misses++;
  int victim = SelectVictim(idx);
  auto& line = set[victim];
  if (line.valid && line.dirty && write_back_policy == "WB") {
    const uint64_t victim_base = ((line.tag * static_cast<uint64_t>(NumSets())) + static_cast<uint64_t>(idx)) * static_cast<uint64_t>(block_size);
    for (int k = 0; k < block_size; ++k) {
      backingWrite(victim_base + static_cast<uint64_t>(k), line.data[static_cast<size_t>(k)]);
    }
  }
  // Fill line
  for (int k = 0; k < block_size; ++k) {
    line.data[static_cast<size_t>(k)] = backingRead(base + static_cast<uint64_t>(k));
  }
  line.tag = tag;
  line.valid = true;
  line.dirty = false;
  // FIFO and LRU both stamp on fill; LRU will be updated again on use
  timer_++;
  line.toa = timer_;
  return line.data[static_cast<size_t>(offset)];
}

void Cache::WriteByte(uint64_t address, uint8_t value,
                      const std::function<uint8_t(uint64_t)>& backingRead,
                      const std::function<void(uint64_t, uint8_t)>& backingWrite) {
  // Fallback bypass if misconfigured
  if (block_size <= 0 || associativity <= 0 || NumSets() <= 0) {
    backingWrite(address, value);
    return;
  }
  const int idx = Index(address);
  const uint64_t tag = Tag(address);
  const uint64_t base = BlockBase(address);
  const int offset = static_cast<int>(address - base);
  auto& set = table[idx];
  if (set.empty()) {
    set.reserve(static_cast<size_t>(associativity));
    for (int j = 0; j < associativity; ++j) set.emplace_back(block_size);
  }

  // Probe
  for (auto& line : set) {
    if (line.valid && line.tag == tag) {
      // hit
      hits++;
      if (replacement_policy == "LRU") {
        timer_++;
        line.toa = timer_;
      }
      line.data[static_cast<size_t>(offset)] = value;
      if (write_back_policy == "WB") {
        line.dirty = true;
      } else { // WT
        backingWrite(address, value);
      }
      return;
    }
  }
  // miss
  misses++;
  if (write_back_policy == "WT") {
    // no write-allocate
    backingWrite(address, value);
    return;
  }
  // WB: write-allocate
  int victim = SelectVictim(idx);
  auto& line = set[victim];
  if (line.valid && line.dirty) {
    const uint64_t victim_base = ((line.tag * static_cast<uint64_t>(NumSets())) + static_cast<uint64_t>(idx)) * static_cast<uint64_t>(block_size);
    for (int k = 0; k < block_size; ++k) {
      backingWrite(victim_base + static_cast<uint64_t>(k), line.data[static_cast<size_t>(k)]);
    }
  }
  // Fill then write
  for (int k = 0; k < block_size; ++k) {
    line.data[static_cast<size_t>(k)] = backingRead(base + static_cast<uint64_t>(k));
  }
  line.tag = tag;
  line.valid = true;
  // FIFO and LRU both stamp on fill
  timer_++;
  line.toa = timer_;

  line.data[static_cast<size_t>(offset)] = value;
  line.dirty = true;
}

// Utilities
std::unique_ptr<Cache> cache_compat::enableCache(const std::string& file_name) {
  std::ifstream file(file_name);
  if (!file.is_open()) {
    return nullptr;
  }
  std::string line;
  int i = 1;
  int cache_size = 0;
  int block_size = 0;
  int associativity = 0;
  std::string write_back_policy;
  std::string replacement_policy;
  while (std::getline(file, line)) {
    // trim spaces
    auto trim = [](std::string s) {
      size_t a = s.find_first_not_of(" \t\r\n");
      size_t b = s.find_last_not_of(" \t\r\n");
      if (a == std::string::npos) return std::string();
      return s.substr(a, b - a + 1);
    };
    line = trim(line);
    switch (i) {
      case 1: cache_size = std::stoi(line); break;
      case 2: block_size = std::stoi(line); break;
      case 3: associativity = std::stoi(line); break;
      case 4: replacement_policy = line; break;
      case 5: write_back_policy = line; break;
      default:
        std::cout << "Invalid file format" << std::endl;
        return nullptr;
    }
    i++;
  }
  file.close();
  if (associativity == 0 && block_size != 0) {
    associativity = cache_size / block_size;
  }
  // normalize policies
  auto upper = [](std::string s) {
    for (auto& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
  };
  auto norm_repl = upper(replacement_policy);
  if (norm_repl == "LRU" || norm_repl == "FIFO" || norm_repl == "RANDOM") {
    replacement_policy = norm_repl;
  }
  auto norm_wp = upper(write_back_policy);
  if (norm_wp == "WB" || norm_wp == "WRITEBACK" || norm_wp == "WRITE BACK") {
    write_back_policy = "WB";
  } else if (norm_wp == "WT" || norm_wp == "WRITETHROUGH" || norm_wp == "WRITE THROUGH") {
    write_back_policy = "WT";
  }
  auto newCache = std::make_unique<Cache>(cache_size, block_size, associativity, write_back_policy, replacement_policy);
  return newCache;
}

void cache_compat::printCacheStatus(const Cache* c) {
  if (!c) {
    std::cout << "Cache: null" << std::endl;
    return;
  }
  std::cout << "Cache Size: " << c->cache_size << std::endl;
  std::cout << "Block Size: " << c->block_size << std::endl;
  std::cout << "Associativity: " << c->associativity << std::endl;
  std::cout << "Replacement Policy: " << c->replacement_policy << std::endl;
  std::cout << "Write Back Policy: " << c->write_back_policy << std::endl;
}

void cache_compat::invalidateCache(Cache* c) {
  if (c) c->Invalidate();
}

void cache_compat::printCacheStats(const Cache* c) {
  if (!c) return;
  std::cout << "D-cache statistics:";
  std::cout << " Accesses=" << (c->hits + c->misses);
  std::cout << " ,Hit=" << c->hits;
  std::cout << " ,Miss=" << c->misses;
  double hit_rate = (c->hits + c->misses) ? (static_cast<double>(c->hits) / static_cast<double>(c->hits + c->misses)) : 0.0;
  std::cout << " ,Hit Rate=" << std::fixed << std::setprecision(2) << hit_rate << std::endl;
}

void cache_compat::dumpCache(const Cache* c, const std::string& file_name) {
  if (!c) return;
  std::ofstream file(file_name);
  for (const auto& it : c->table) {
    int set_index = it.first;
    const auto& lines = it.second;
    for (const auto& line : lines) {
      if (line.valid) {
        file << "Set: 0x" << std::hex << set_index;
        file << " ,Tag: 0x" << std::hex << line.tag;
        file << (line.dirty ? ", Dirty" : ", Clean");
        file << std::dec << "\n";
      }
    }
  }
  file.close();
}


