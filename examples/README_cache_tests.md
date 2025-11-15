Cache test programs

1) cache_read_loop_min.s
   - Minimal syntax accepted by this assembler. Repeatedly loads the same address after a first miss.
   - Expected: Misses ≈ 1; Hits ≫ Misses. Hit rate should be high.

   Run:
   - Start the VM binary in interactive mode, then:
     load /Users/aryanagarwal/Documents/riscv-simulator-2/examples/cache_read_loop_min.s
     run
   - Look for:
     MODE_TIME_MS=...
     D-cache statistics: Accesses=..., Hit=..., Miss=..., Hit Rate=...

2) cache_eviction_min.s
   - Minimal syntax. Accesses more distinct lines mapping to the same set than the cache associativity, forcing an eviction.
   - Expected: After filling associativity lines, the next distinct line causes eviction; re-accessing the first line should miss (depending on replacement policy).
   - STRIDE is 256 bytes; adjust if your cacheConfig.txt changes (num_sets/indexing).

   Run:
   - In interactive mode:
     load /Users/aryanagarwal/Documents/riscv-simulator-2/examples/cache_eviction_min.s
     run
   - Compare stats and try different replacement policies in:
     RISC-V-Simulator-main/backend/Simulator/cacheConfig.txt

Tips
- Toggle write policy (WB vs WT) and replacement (LRU/FIFO/RANDOM) in cacheConfig.txt and re-run to see changes in hit/miss patterns.
- For fully associative (associativity=0), fewer sets mean more contention; eviction should occur sooner for the same STRIDE.
 - Commands are lowercase (load, run, step, etc.).


