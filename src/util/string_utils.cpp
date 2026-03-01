/**
 * @file string_utils.cpp
 * @brief 字符串工具实现（从 Common 迁入，GetUUID 改为 C++11 实现，无 Boost）
 */

#include "string_utils.h"
#include <memory>
#include <cstdarg>
#include <cstdio>
#include <random>
#include <sstream>

#if defined(_WIN32) || defined(_WIN64)
#include <objbase.h>
#endif

namespace insight {

std::string GetUUID() {
#if defined(_WIN32) || defined(_WIN64)
  char buffer[64] = {0};
  GUID guid;
  if (CoCreateGuid(&guid) != S_OK) {
    return "";
  }
  snprintf(buffer, sizeof(buffer),
           "%08X%04X%04x%02X%02X%02X%02X%02X%02X%02X%02X",
           (unsigned)guid.Data1, (unsigned)guid.Data2, (unsigned)guid.Data3,
           (unsigned)guid.Data4[0], (unsigned)guid.Data4[1], (unsigned)guid.Data4[2],
           (unsigned)guid.Data4[3], (unsigned)guid.Data4[4], (unsigned)guid.Data4[5],
           (unsigned)guid.Data4[6], (unsigned)guid.Data4[7]);
  return std::string(buffer);
#else
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dis;
  uint64_t a = dis(gen), b = dis(gen);
  std::ostringstream oss;
  oss << std::hex << a << b;
  return oss.str();
#endif
}

void StringAppendV(std::string* dst, const char* format, va_list ap) {
  static const int kFixedBufferSize = 1024;
  char fixed_buffer[kFixedBufferSize];

  va_list backup_ap;
  va_copy(backup_ap, ap);
  int result = vsnprintf(fixed_buffer, kFixedBufferSize, format, backup_ap);
  va_end(backup_ap);

  if (result < kFixedBufferSize) {
    if (result >= 0) {
      dst->append(fixed_buffer, result);
      return;
    }
#if defined(_MSC_VER)
    va_copy(backup_ap, ap);
    result = vsnprintf(nullptr, 0, format, backup_ap);
    va_end(backup_ap);
#endif
    if (result < 0)
      return;
  }

  const int variable_buffer_size = result + 1;
  std::unique_ptr<char[]> variable_buffer(new char[variable_buffer_size]);
  va_copy(backup_ap, ap);
  result = vsnprintf(variable_buffer.get(), variable_buffer_size, format, backup_ap);
  va_end(backup_ap);

  if (result >= 0 && result < variable_buffer_size) {
    dst->append(variable_buffer.get(), result);
  }
}

std::string StringPrintf(const char* format, ...) {
  va_list ap;
  va_start(ap, format);
  std::string result;
  StringAppendV(&result, format, ap);
  va_end(ap);
  return result;
}

}  // namespace insight
