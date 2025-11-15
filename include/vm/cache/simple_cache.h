/**
 * @file simple_cache.h
 * @brief A simple set-associative cache modeled after cache_simulator.* for compatibility.
 */
#ifndef SIMPLE_CACHE_H
#define SIMPLE_CACHE_H

#include <cstdint>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <functional>

namespace cache_compat {

struct CacheLine {
  bool valid = false;
  bool dirty = false;
  std::vector<uint8_t> data;
  uint64_t tag = 0;
  uint64_t toa = 0; // time of access / load order

  explicit CacheLine(int size) : data(static_cast<size_t>(size), 0) {}
};

class Cache {
public:
  uint64_t hits = 0;
  uint64_t misses = 0;
  std::unordered_map<int, std::vector<CacheLine> > table;
  int cache_size = 0;
  int block_size = 0;
  int associativity = 0;
  std::string write_back_policy;    // "WB" or "WT"
  std::string replacement_policy;   // "LRU", "FIFO", "RANDOM"

  Cache(int cache_size, int block_size, int associativity,
        const std::string& write_back_policy,
        const std::string& replacement_policy);

  void Invalidate();

  // Backing memory accessors passed by caller (e.g., MemoryController)
  uint8_t ReadByte(uint64_t address,
                   const std::function<uint8_t(uint64_t)>& backingRead,
                   const std::function<void(uint64_t, uint8_t)>& backingWrite);

  void WriteByte(uint64_t address, uint8_t value,
                 const std::function<uint8_t(uint64_t)>& backingRead,
                 const std::function<void(uint64_t, uint8_t)>& backingWrite);

private:
  uint64_t timer_ = 0; // increases on access for LRU/FIFO

  int NumSets() const;
  int Index(uint64_t address) const;
  uint64_t Tag(uint64_t address) const;
  uint64_t BlockBase(uint64_t address) const;
  int SelectVictim(int set_index);
};

// Utility functions similar to cache_simulator.*
std::unique_ptr<Cache> enableCache(const std::string& file_name);
void printCacheStatus(const Cache* c);
void invalidateCache(Cache* c);
void printCacheStats(const Cache* c);
void dumpCache(const Cache* c, const std::string& file_name);

} // namespace cache_compat

#endif // SIMPLE_CACHE_H


