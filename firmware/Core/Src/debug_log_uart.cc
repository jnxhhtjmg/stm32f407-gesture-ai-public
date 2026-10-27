// TFLM debug output is intentionally disabled on this bare-metal target.
// The stock implementation calls vfprintf(stderr, ...), which pulls in the
// C library's stdio/semihosting startup. The firmware uses its own small
// formatter in freertos.c instead.

#include <cstdarg>
#include <cstddef>

#include "tensorflow/lite/micro/debug_log.h"

extern "C" void DebugLog(const char* format, va_list args)
{
  (void)format;
  (void)args;
}

extern "C" int DebugVsnprintf(char* buffer, size_t buf_size,
                              const char* format, va_list vlist)
{
  (void)buffer;
  (void)buf_size;
  (void)format;
  (void)vlist;
  return 0;
}
