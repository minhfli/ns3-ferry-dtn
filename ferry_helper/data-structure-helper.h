#ifndef DS_HELPER_H
#define DS_HELPER_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "global.h"

namespace DataStructureHelper {
    std::vector<uint32_t> GetIndexVector(uint32_t n) {
        std::vector<uint32_t> index(n, 0);
        for (uint32_t i = 0; i < n; i++) {
            index[i] = i;
        }
        return index;
    }

    std::vector<uint32_t> GetShuffleIndexVector(uint32_t n) {
        std::vector<uint32_t> index = GetIndexVector(n);
        std::random_shuffle(index.begin(), index.end());
        return index;
    }

    std::set<uint32_t> GetReversedSet(const std::set<uint32_t>& s, uint32_t n) {
        std::set<uint32_t> result;
        for (uint32_t i = 0; i < n; i++) {
            if (s.find(i) == s.end()) {
                result.insert(i);
            }
        }
        return result;
    }
    std::set<uint32_t> GetReversedSet(const std::vector<uint32_t>& v, uint32_t n) {
        std::set<uint32_t> result;
        for (uint32_t i = 0; i < n; i++) {
            result.insert(i);
        }
        for (uint32_t i : v) {
            result.erase(i);
        }
        return result;
    }



};
#endif