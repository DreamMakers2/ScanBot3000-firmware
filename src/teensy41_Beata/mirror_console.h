#pragma once

#include <Arduino.h>

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

class MirrorConsole : public Print {
public:
  MirrorConsole(Print& a, Print& b) : a_(a), b_(b) {}

  size_t write(uint8_t c) override {
    a_.write(c);
    b_.write(c);
    return 1;
  }

  size_t write(const uint8_t* buffer, size_t size) override {
    a_.write(buffer, size);
    b_.write(buffer, size);
    return size;
  }

  using Print::write;

  void vprintf(const char* fmt, va_list args) {
    char local[256];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(local, sizeof(local), fmt, copy);
    va_end(copy);
    if (n < 0) return;
    if ((size_t)n < sizeof(local)) {
      write(reinterpret_cast<const uint8_t*>(local), (size_t)n);
      return;
    }
    char* buf = static_cast<char*>(malloc((size_t)n + 1));
    if (!buf) return;
    va_list copy2;
    va_copy(copy2, args);
    vsnprintf(buf, (size_t)n + 1, fmt, copy2);
    va_end(copy2);
    write(reinterpret_cast<const uint8_t*>(buf), (size_t)n);
    free(buf);
  }

  void printf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
  }

private:
  Print& a_;
  Print& b_;
};

