#ifndef PRIMEUTIL_H
#define PRIMEUTIL_H

#include <vector>
#include <algorithm>

namespace prime_util {
class PrimeUtil {

public:
  PrimeUtil();
  ~PrimeUtil();

  auto is_prime(size_t target) -> bool;

}; 
} //prime_util

#endif // PRIMEUTIL_H  
