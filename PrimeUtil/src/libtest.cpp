#include <iostream>
#include "../include/primeutil.h"
#include "gtest/gtest.h"


TEST(PUTest, ModularExponentEvenTest) {
  auto base = 7;
  auto exponent = 256;
  auto mod = 13;

  prime_util::PrimeUtil p;

  auto result = p.mod_exponentiate(base,exponent,mod);
  EXPECT_EQ(result, 9);
}

TEST(PUTest, ModularExponentOdd1Test) {
  auto base = 2;
  auto exponent = 9;
  auto mod = 3;

  prime_util::PrimeUtil p;

  auto result = p.mod_exponentiate(base,exponent,mod);
  EXPECT_EQ(result, 2);
}

TEST(PUTest, ModularExponentOdd2Test) {
  auto base = 4;
  auto exponent = 13;
  auto mod = 497;

  prime_util::PrimeUtil p;

  auto result = p.mod_exponentiate(base,exponent,mod);
  EXPECT_EQ(result, 445);
}

TEST(PUTest, NaivePrimeTest) {

  auto n = 221;
  prime_util::PrimeUtil p;
 
  auto result = p.naive_is_prime(n);
  EXPECT_EQ(result, false);
}

TEST(PUTest, NaivePrimeTestTrue) {

  auto n = 37;
  prime_util::PrimeUtil p;
 
  auto result = p.naive_is_prime(n);
  EXPECT_EQ(result, true);
}

TEST(PUTest, MillerRabinTwoTest) {

  auto n = 2;
  auto k = 50;
  prime_util::PrimeUtil p;
 
  auto result = p.miller_rabin_is_prime(n, k);
  EXPECT_EQ(result, true);
}

TEST(PUTest, MillerRabinOddTest) {

  auto n = 221;
  auto k = 50;
  prime_util::PrimeUtil p;
 
  auto result = p.miller_rabin_is_prime(n, k);
  EXPECT_EQ(result, false);
}

TEST(PUTest, MillerRabinOddBigTest) {

  auto n = 104513;
  auto k = 50;
  prime_util::PrimeUtil p;
 
  auto result = p.miller_rabin_is_prime(n, k);
  EXPECT_EQ(result, true);
}
