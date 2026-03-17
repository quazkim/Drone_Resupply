#ifndef PDP_CACHE_H
#define PDP_CACHE_H

#include "pdp_types.h"
#include <vector>
#include <unordered_map>
#include <functional>
#include <cstdint>
#include <limits>
#include <iostream>
#include <iomanip>

using namespace std;

/**
 * @brief Hash functor for std::vector<int> using hash_combine algorithm (similar to Boost).
 * 
 * Converts a vector to a hashable integer key by combining element hashes sequentially.
 * Uses the hash_combine pattern to minimize collisions:
 *   seed ^= hash(element) + 0x9e3779b9 + (seed << 6) + (seed >> 2)
 * 
 * This approach is optimized for permutation vectors used in GA chromosomes.
 */
struct SequenceHash {
    /**
     * @brief Combine two hash values using Boost-inspired algorithm.
     * @param seed Current hash accumulator
     * @param v Hash value to combine
     */
    static void hash_combine(size_t& seed, size_t v) {
        const size_t magic = 0x9e3779b9UL;  // Golden ratio constant
        seed ^= v + magic + (seed << 6) + (seed >> 2);
    }

    /**
     * @brief Compute hash for a sequence of integers.
     * @param vec Customer sequence (permutation of customer IDs)
     * @return Hash value suitable for unordered_map
     */
    size_t operator()(const vector<int>& vec) const {
        size_t seed = 0;
        hash<int> hasher;
        
        for (int i : vec) {
            hash_combine(seed, hasher(i));
        }
        
        return seed;
    }
};

/**
 * @brief Solution cache manager with automatic memory management.
 * 
 * Caches complete PDPSolution objects indexed by customer sequence.
 * Implements a simple cache clearing strategy when memory limit is exceeded.
 * 
 * Memory Model:
 * - Each PDPSolution ≈ 10KB for 100-customer problems
 * - MAX_CACHE_SIZE = 150000 entries ≈ 1.5GB RAM
 * - Clearing happens automatically when size threshold exceeded
 * 
 * Thread Safety: NOT thread-safe. Use only from single-threaded GA.
 */
class SolutionCache {
private:
    /// Cache storage: sequence -> solution
    unordered_map<vector<int>, PDPSolution, SequenceHash> cache;
    
    /// Maximum cache size before automatic clearing
    static constexpr size_t MAX_CACHE_SIZE = 150000;
    
    /// Counter for statistics
    size_t hits = 0;
    size_t misses = 0;
    size_t clears = 0;

public:
    /**
     * @brief Construct an empty cache.
     */
    SolutionCache() = default;

    /**
     * @brief Check if a sequence is in cache.
     * @param seq Customer sequence
     * @return true if sequence is cached
     */
    bool contains(const vector<int>& seq) const {
        return cache.find(seq) != cache.end();
    }

    /**
     * @brief Retrieve a cached solution.
     * Assumes the sequence is in cache. Use contains() to check first.
     * @param seq Customer sequence
     * @return Copy of the cached PDPSolution
     */
    PDPSolution get(const vector<int>& seq) const {
        auto it = cache.find(seq);
        if (it != cache.end()) {
            return it->second;
        }
        // Should not reach here if contains() was checked
        return PDPSolution();
    }

    /**
     * @brief Store a solution in cache.
     * Automatically clears cache if size exceeds MAX_CACHE_SIZE.
     * @param seq Customer sequence (key)
     * @param solution Complete PDPSolution (value)
     */
    void put(const vector<int>& seq, const PDPSolution& solution) {
        cache[seq] = solution;
        
        // Check memory management: if cache exceeds limit, clear all
        if (cache.size() >= MAX_CACHE_SIZE) {
            cache.clear();
            clears++;
        }
    }

    /**
     * @brief Manually clear all cached solutions.
     * Useful if memory needs to be freed or cache is corrupted.
     */
    void clear() {
        cache.clear();
        clears++;
    }

    /**
     * @brief Get current number of entries in cache.
     * @return Number of cached solutions
     */
    size_t size() const {
        return cache.size();
    }

    /**
     * @brief Get cache hit statistics.
     * @return Number of successful cache hits since initialization
     */
    size_t getHits() const {
        return hits;
    }

    /**
     * @brief Get cache miss statistics.
     * @return Number of cache misses since initialization
     */
    size_t getMisses() const {
        return misses;
    }

    /**
     * @brief Get number of times cache was cleared.
     * @return Number of automatic cache clears
     */
    size_t getClears() const {
        return clears;
    }

    /**
     * @brief Record a cache hit.
     * Called internally by evaluateWithCache.
     */
    void recordHit() {
        hits++;
    }

    /**
     * @brief Record a cache miss.
     * Called internally by evaluateWithCache.
     */
    void recordMiss() {
        misses++;
    }

    /**
     * @brief Print cache statistics to console.
     */
    void printStats() const {
        size_t total = hits + misses;
        double hitRate = (total > 0) ? (100.0 * hits / total) : 0.0;
        cout << "\n[CACHE STATS]" << endl;
        cout << "  Current size: " << cache.size() << " entries" << endl;
        cout << "  Hits: " << hits << endl;
        cout << "  Misses: " << misses << endl;
        cout << "  Total accesses: " << total << endl;
        cout << "  Hit rate: " << fixed << setprecision(2) << hitRate << "%" << endl;
        cout << "  Auto-clears: " << clears << endl;
    }
};

#endif // PDP_CACHE_H
