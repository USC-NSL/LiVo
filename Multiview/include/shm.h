#pragma once
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

using namespace std;
using namespace boost::interprocess;

void destroy_shm(const char* name);

void create_shm(const char* name, size_t size, mapped_region& region);

uint32_t get_shm_size(const std::string &path, std::string _type);