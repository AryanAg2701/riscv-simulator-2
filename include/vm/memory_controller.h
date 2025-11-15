/**
 * @file memory_controller.h
 * @brief Contains the declaration of the MemoryController class for managing memory in the VM.
 * @author Vishank Singh, https://github.com/VishankSingh
 */

#ifndef MEMORY_CONTROLLER_H
#define MEMORY_CONTROLLER_H

#include "../config.h"
#include "main_memory.h"
#include "cache/simple_cache.h"

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <functional>


/**
 * @brief The MemoryController class is responsible for managing memory in the VM.
 */
class MemoryController {
private:
    Memory memory_; ///< The main memory object.
    std::unique_ptr<cache_compat::Cache> compat_cache_; ///< Optional simple cache (compat with cache_simulator.* style)
    bool cache_enabled_ = false;
public:
    MemoryController() = default;

    void Reset() {
        memory_.Reset();
        if (compat_cache_) compat_cache_->Invalidate();
    }

    void EnableSimpleCacheFromFile(const std::string& file) {
        compat_cache_ = cache_compat::enableCache(file);
        cache_enabled_ = (compat_cache_ != nullptr);
    }

    void EnableSimpleCache(int cache_size, int block_size, int associativity,
                           const std::string& replacement_policy,
                           const std::string& write_back_policy) {
        compat_cache_ = std::make_unique<cache_compat::Cache>(cache_size, block_size, associativity, write_back_policy, replacement_policy);
        cache_enabled_ = true;
    }

    void DisableCache() {
        cache_enabled_ = false;
        compat_cache_.reset();
    }

    void InvalidateCache() {
        if (compat_cache_) compat_cache_->Invalidate();
    }

    void PrintCacheStatus() const {
        if (!cache_enabled_) { std::cout << "Cache: disabled" << std::endl; return; }
        cache_compat::printCacheStats(compat_cache_.get());
    }

    void WriteByte(uint64_t address, uint8_t value) {
        const uint64_t data_start = vm_config::config.getDataSectionStart();
        if (!cache_enabled_ || address < data_start) {
        memory_.WriteByte(address, value);
        return;
      }
      compat_cache_->WriteByte(address, value,
        [this](uint64_t a){ return memory_.ReadByte(a); },
        [this](uint64_t a, uint8_t v){ memory_.WriteByte(a, v); }
      );
    }

    void WriteHalfWord(uint64_t address, uint16_t value) {
        const uint64_t data_start = vm_config::config.getDataSectionStart();
        if (!cache_enabled_ || address < data_start) {
        memory_.WriteHalfWord(address, value);
        return;
      }
      compat_cache_->WriteByte(address, static_cast<uint8_t>(value & 0xFF),
        [this](uint64_t a){ return memory_.ReadByte(a); },
        [this](uint64_t a, uint8_t v){ memory_.WriteByte(a, v); }
      );
      compat_cache_->WriteByte(address + 1, static_cast<uint8_t>((value >> 8) & 0xFF),
        [this](uint64_t a){ return memory_.ReadByte(a); },
        [this](uint64_t a, uint8_t v){ memory_.WriteByte(a, v); }
      );
    }

    void WriteWord(uint64_t address, uint32_t value) {
        const uint64_t data_start = vm_config::config.getDataSectionStart();
        if (!cache_enabled_ || address < data_start) {
        memory_.WriteWord(address, value);
        return;
      }
      for (size_t i = 0; i < 4; ++i) {
        const uint8_t b = static_cast<uint8_t>((value >> (8*i)) & 0xFF);
        compat_cache_->WriteByte(address + i, b,
          [this](uint64_t a){ return memory_.ReadByte(a); },
          [this](uint64_t a, uint8_t v){ memory_.WriteByte(a, v); }
        );
      }
    }

    void WriteDoubleWord(uint64_t address, uint64_t value) {
        const uint64_t data_start = vm_config::config.getDataSectionStart();
        if (!cache_enabled_ || address < data_start) {
        memory_.WriteDoubleWord(address, value);
        return;
      }
      for (size_t i = 0; i < 8; ++i) {
        const uint8_t b = static_cast<uint8_t>((value >> (8*i)) & 0xFF);
        compat_cache_->WriteByte(address + i, b,
          [this](uint64_t a){ return memory_.ReadByte(a); },
          [this](uint64_t a, uint8_t v){ memory_.WriteByte(a, v); }
        );
      }
    }

    [[nodiscard]] uint8_t ReadByte(uint64_t address) {
        const uint64_t data_start = vm_config::config.getDataSectionStart();
        if (!cache_enabled_ || address < data_start) {
            return memory_.ReadByte(address);
        }
        return compat_cache_->ReadByte(address,
          [this](uint64_t a){ return memory_.ReadByte(a); },
          [this](uint64_t a, uint8_t v){ memory_.WriteByte(a, v); }
        );
    }

    [[nodiscard]] uint16_t ReadHalfWord(uint64_t address) {
        const uint64_t data_start = vm_config::config.getDataSectionStart();
        if (!cache_enabled_ || address < data_start) {
            return memory_.ReadHalfWord(address);
        }
        uint8_t b0 = compat_cache_->ReadByte(address,
          [this](uint64_t a){ return memory_.ReadByte(a); },
          [this](uint64_t a, uint8_t v){ memory_.WriteByte(a, v); });
        uint8_t b1 = compat_cache_->ReadByte(address + 1,
          [this](uint64_t a){ return memory_.ReadByte(a); },
          [this](uint64_t a, uint8_t v){ memory_.WriteByte(a, v); });
        return static_cast<uint16_t>(b0) | (static_cast<uint16_t>(b1) << 8);
    }

    [[nodiscard]] uint32_t ReadWord(uint64_t address) {
        const uint64_t data_start = vm_config::config.getDataSectionStart();
        if (!cache_enabled_ || address < data_start) {
            return memory_.ReadWord(address);
        }
        uint32_t value = 0;
        for (size_t i = 0; i < 4; ++i) {
          uint8_t bi = compat_cache_->ReadByte(address + i,
            [this](uint64_t a){ return memory_.ReadByte(a); },
            [this](uint64_t a, uint8_t v){ memory_.WriteByte(a, v); });
          value |= static_cast<uint32_t>(bi) << (8*i);
        }
        return value;
    }

    [[nodiscard]] uint64_t ReadDoubleWord(uint64_t address) {
        const uint64_t data_start = vm_config::config.getDataSectionStart();
        if (!cache_enabled_ || address < data_start) {
            return memory_.ReadDoubleWord(address);
        }
        uint64_t value = 0;
        for (size_t i = 0; i < 8; ++i) {
          uint8_t bi = compat_cache_->ReadByte(address + i,
            [this](uint64_t a){ return memory_.ReadByte(a); },
            [this](uint64_t a, uint8_t v){ memory_.WriteByte(a, v); });
          value |= static_cast<uint64_t>(bi) << (8*i);
        }
        return value;
    }

    // Functions to read memory directly with cache bypass

    [[nodiscard]] uint8_t ReadByte_d(uint64_t address) {
        return memory_.ReadByte(address);
    }

    [[nodiscard]] uint16_t ReadHalfWord_d(uint64_t address) {
        return memory_.ReadHalfWord(address);
    }

    [[nodiscard]] uint32_t ReadWord_d(uint64_t address) {
        return memory_.ReadWord(address);
    }

    [[nodiscard]] uint64_t ReadDoubleWord_d(uint64_t address) {
        return memory_.ReadDoubleWord(address);
    }

    void PrintMemory(const uint64_t address, unsigned int rows) {
      memory_.PrintMemory(address, rows);
    }

    void DumpMemory(std::vector<std::string> args) {
      memory_.DumpMemory(args);
    }

    void GetMemoryPoint(std::string address) {
      return memory_.GetMemoryPoint(address);
    }

};

#endif // MEMORY_CONTROLLER_H

