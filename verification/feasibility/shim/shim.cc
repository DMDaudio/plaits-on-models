// libc/libm pieces Plaits needs on the ColdFire, freestanding.
#include <stddef.h>
#include <stdint.h>

extern "C" {

void* memcpy(void* d, const void* s, size_t n) {
  uint8_t* dd = (uint8_t*)d; const uint8_t* ss = (const uint8_t*)s;
  if ((((uintptr_t)dd | (uintptr_t)ss | n) & 3) == 0) {
    uint32_t* dw = (uint32_t*)dd; const uint32_t* sw = (const uint32_t*)ss;
    for (n >>= 2; n; --n) *dw++ = *sw++;
    return d;
  }
  while (n--) *dd++ = *ss++;
  return d;
}
void* memmove(void* d, const void* s, size_t n) {
  uint8_t* dd = (uint8_t*)d; const uint8_t* ss = (const uint8_t*)s;
  if (dd < ss) { while (n--) *dd++ = *ss++; }
  else { dd += n; ss += n; while (n--) *--dd = *--ss; }
  return d;
}
void* memset(void* d, int c, size_t n) {
  uint8_t* dd = (uint8_t*)d;
  while (n--) *dd++ = (uint8_t)c;
  return d;
}
int memcmp(const void* a, const void* b, size_t n) {
  const uint8_t* x = (const uint8_t*)a; const uint8_t* y = (const uint8_t*)b;
  for (; n; --n, ++x, ++y) if (*x != *y) return *x - *y;
  return 0;
}
int printf(const char*, ...) { return 0; }

static const double kPi = 3.14159265358979323846;

double sin(double x) {
  // range-reduce to [-pi, pi], then a 9th-order Taylor in [-pi/2, pi/2]
  double k = (double)(long long)(x / (2 * kPi));
  x -= k * 2 * kPi;
  if (x > kPi) x -= 2 * kPi;
  if (x < -kPi) x += 2 * kPi;
  if (x > kPi / 2) x = kPi - x;
  if (x < -kPi / 2) x = -kPi - x;
  double x2 = x * x;
  return x * (1 - x2 / 6 * (1 - x2 / 20 * (1 - x2 / 42 * (1 - x2 / 72 * (1 - x2 / 110)))));
}
double cos(double x) { return sin(x + kPi / 2); }
double tan(double x) { return sin(x) / cos(x); }
double sqrt(double x) {
  if (x <= 0) return 0;
  double r = x > 1 ? x : 1;
  for (int i = 0; i < 40; ++i) r = 0.5 * (r + x / r);
  return r;
}
double exp(double x) {
  // exp(x) = 2^k * exp(r), |r| <= ln2/2
  const double ln2 = 0.69314718055994530942;
  long k = (long)(x / ln2 + (x < 0 ? -0.5 : 0.5));
  double r = x - k * ln2, t = 1, s = 1;
  for (int i = 1; i < 16; ++i) { t *= r / i; s += t; }
  while (k > 0) { s *= 2; --k; }
  while (k < 0) { s *= 0.5; ++k; }
  return s;
}
double log(double x) {
  if (x <= 0) return -1e300;
  const double ln2 = 0.69314718055994530942;
  int k = 0;
  while (x > 1.5) { x *= 0.5; ++k; }
  while (x < 0.75) { x *= 2; --k; }
  double y = (x - 1) / (x + 1), y2 = y * y, t = y, s = 0;
  for (int i = 1; i < 40; i += 2) { s += t / i; t *= y2; }
  return 2 * s + k * ln2;
}
double log2(double x) { return log(x) / 0.69314718055994530942; }
double pow(double x, double y) { return exp(y * log(x)); }

float sinf(float x) { return (float)sin(x); }
float cosf(float x) { return (float)cos(x); }
float tanf(float x) { return (float)tan(x); }
float sqrtf(float x) { return (float)sqrt(x); }
float expf(float x) { return (float)exp(x); }
float logf(float x) { return (float)log(x); }
float powf(float x, float y) { return (float)pow(x, y); }
float floorf(float x) { float t = (float)(long)x; return t > x ? t - 1 : t; }

}  // extern "C"
