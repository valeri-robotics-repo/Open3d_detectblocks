cmake_minimum_required(VERSION 2.8.9)
project (PrimeUtil)
set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(PrimeUtil SHARED src/primeutil.cpp)
install(TARGETS PrimeUtil DESTINATION /usr/lib)
install(FILES include/primeutil.h DESTINATION include)
