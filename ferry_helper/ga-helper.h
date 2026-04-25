#ifndef GA_HELPER_H
#define GA_HELPER_H

#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>
#include <limits>
#include <set>

#include "datatypes.h"
#include "data-structure-helper.h"

uint32_t GA_Select_rank(uint32_t n) {
    uint32_t total_rank = n * (n + 1) / 2;
    std::uniform_int_distribution<uint32_t> dist(1, total_rank);

    uint32_t r = dist(globalRNG);
    for (uint32_t i = 0; i < n; ++i) {
        r -= n - i;
        if (r <= 0) {
            return i;
        }
    }
    return n - 1; // Fallback
}

std::pair<uint32_t, uint32_t> GA_RankRoullete(uint32_t n) {

    uint32_t p1 = GA_Select_rank(n);
    uint32_t p2 = p1;
    while (p1 == p2) {
        p2 = GA_Select_rank(n);
    }
    return { p1, p2 };
}

std::pair<uint32_t, uint32_t> GA_EliteRankRoullete(uint32_t n, uint32_t elite) {

    uint32_t p1 = GA_Select_rank(elite);
    uint32_t p2 = p1;
    while (p1 == p2) {
        p2 = GA_Select_rank(n);
    }
    return { p1, p2 };
}


#endif // GA_HELPER_H