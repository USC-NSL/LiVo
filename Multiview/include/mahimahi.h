#pragma once
#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <map>

using namespace std;

class Mahimahi {
public:
    Mahimahi() {}
    ~Mahimahi() {}
    bool load_trace(uint64_t _start_time, std::string _fname);
    int get_throughput();

private:
    uint64_t start_time;
    vector<int> throughput;
    std::string fname;
    uint64_t max_time;
};

bool load_trace(Mahimahi *up, Mahimahi *down);