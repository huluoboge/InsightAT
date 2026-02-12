#include "retrieval_types.h"
#include <map>
#include <algorithm>

namespace insight::algorithm::retrieval {

std::vector<ImagePair> deduplicateAndMerge(std::vector<ImagePair> pairs) {
    if (pairs.empty()) return pairs;
    
    // Use map to track unique pairs (canonical ordering: i < j)
    std::map<std::pair<int, int>, ImagePair> unique_pairs;
    
    for (auto& p : pairs) {
        if (!p.isValid()) continue;
        
        // Canonical ordering
        int i = std::min(p.image1_idx, p.image2_idx);
        int j = std::max(p.image1_idx, p.image2_idx);
        auto key = std::make_pair(i, j);
        
        auto it = unique_pairs.find(key);
        if (it == unique_pairs.end()) {
            // New pair
            p.image1_idx = i;
            p.image2_idx = j;
            unique_pairs[key] = p;
        } else {
            // Merge: sum scores, keep metadata with higher contribution
            auto& existing = it->second;
            existing.score += p.score;
            
            // Keep method that contributed more
            if (p.score > existing.score * 0.5) {
                existing.method = existing.method + "+" + p.method;
            }
            
            // Merge optional metadata
            if (!existing.spatial_distance && p.spatial_distance) {
                existing.spatial_distance = p.spatial_distance;
            }
            if (!existing.visual_similarity && p.visual_similarity) {
                existing.visual_similarity = p.visual_similarity;
            }
            if (!existing.angle_difference && p.angle_difference) {
                existing.angle_difference = p.angle_difference;
            }
        }
    }
    
    // Extract unique pairs
    std::vector<ImagePair> result;
    result.reserve(unique_pairs.size());
    for (auto& [key, pair] : unique_pairs) {
        result.push_back(pair);
    }
    
    return result;
}

std::vector<ImagePair> combinePairs(
    const std::vector<std::vector<ImagePair>>& all_pairs,
    bool deduplicate
) {
    std::vector<ImagePair> combined;
    
    // Estimate total size
    size_t total_size = 0;
    for (const auto& pairs : all_pairs) {
        total_size += pairs.size();
    }
    combined.reserve(total_size);
    
    // Concatenate all pairs
    for (const auto& pairs : all_pairs) {
        combined.insert(combined.end(), pairs.begin(), pairs.end());
    }
    
    if (deduplicate) {
        combined = deduplicateAndMerge(std::move(combined));
    }
    
    return combined;
}

}  // namespace insight::algorithm::retrieval
