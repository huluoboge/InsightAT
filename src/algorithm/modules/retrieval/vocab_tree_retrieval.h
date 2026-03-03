/**
 * @file  vocab_tree_retrieval.h
 * @brief DBoW3 词袋树检索封装：加载词表、建库、查询与 retrieve_by_vocab_tree 入口。
 */

#pragma once

#include "retrieval_types.h"

#include <memory>
#include <string>
#include <vector>

namespace DBoW3 {
class Vocabulary;
class Database;
}  // namespace DBoW3

namespace insight::algorithm::retrieval {

class VocabTreeRetriever {
public:
  VocabTreeRetriever();
  ~VocabTreeRetriever();

  bool load_vocabulary(const std::string& vocab_file);
  bool is_vocabulary_loaded() const;
  int add_image(int image_idx, const std::vector<float>& descriptors);
  std::vector<std::pair<int, float>> query(const std::vector<float>& descriptors,
                                           int max_results = 20);
  void clear_database();
  int get_database_size() const;

private:
  std::unique_ptr<DBoW3::Vocabulary> vocab_;
  std::unique_ptr<DBoW3::Database> database_;
  std::vector<int> image_indices_;
};

std::vector<ImagePair> retrieve_by_vocab_tree(const std::vector<ImageInfo>& images,
                                              const RetrievalOptions& options,
                                              const std::string& vocab_file,
                                              const std::string& cache_dir = "");

}  // namespace insight::algorithm::retrieval
