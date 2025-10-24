#include <fstream>
#include "mahimahi.h"
#include "consts.h"

using namespace std;

bool load_trace(Mahimahi *up, Mahimahi *down) 
{
	LOG(INFO) << "load mahimahi trace";
    ifstream mahi_file("mm_time");
    if (!mahi_file.is_open()) {
        LOG(FATAL) << "[WebRTC] Mahimahi metadata not found";
        return false;
    }

    // Load mahimahi trace metadata
    string tmp;
    getline(mahi_file, tmp);
    uint64_t start_time = std::stoull(tmp);
    start_time /= 1000000;

    string up_fname, down_fname;
    getline(mahi_file, up_fname);
    getline(mahi_file, down_fname);

    up->load_trace(start_time, up_fname);
    down->load_trace(start_time, down_fname);

	LOG(INFO) << "Mahimahi trace " << up_fname << " and " << down_fname << " loaded";

    mahi_file.close();

    return true;
}

bool Mahimahi::load_trace(uint64_t _start_time, string _fname) {
    start_time = _start_time;
    fname = _fname;
    max_time = 0;

    // load up file
    ifstream file(fname);
    CHECK(file.is_open()) << "Failed to open trace file: " << fname;

    string tmp_s;
    int tmp = 0;
    map<int, int> trace;
    while(!getline(file, tmp_s).eof()) {
        tmp = stoi(tmp_s);
        if (trace.find(tmp) == trace.end()) trace[tmp] = 1;
        else trace[tmp] ++;
    }

    auto it = trace.end();
    max_time =  (--it) -> first;

    if (max_time > 1000) {
        for (auto it = trace.begin(); it != trace.end(); ++it) {
            if (it->first > 1000) break;
            trace[it->first + max_time] = it->second;
        }

        auto s = trace.begin();
        auto t = trace.begin();

        int last_window_throughput = 0;

        while(t->first < 1000) {
            last_window_throughput += t->second;
            t++;
        }

        for (int i = 0; i < max_time; i++) {
            while(s->first < i && s->second <= max_time) {
                if (s == trace.end()) break;
                last_window_throughput -= s->second;
                s++;
            
            }
            while(t->first < i + 1000 && t->second <= max_time) {
                if (t == trace.end()) break;
                last_window_throughput += t->second;
                t++;
            }
            throughput.push_back(last_window_throughput);
        }
    }

    else {
        int sum = 0;
        for (auto it = trace.begin(); it != trace.end(); ++it) sum += it->second;
        auto start = trace.begin()->first;
        throughput.push_back(int (sum * 1000 / (max_time - start + 1)));
    }

    file.close();
    LOG(INFO) << "Mahimahi trace file " << _fname << " loaded";
    return true;
}

/**
 * @brief Get estimated throughput in Kbits per second from the trace
 * 
 * @return int Estimates throughput in Kbits per second
 */
int Mahimahi::get_throughput() 
{
    uint64_t ms = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
    uint64_t dur = ms - start_time + 1000;
    return int(throughput[dur % max_time] * 12000 / 1000); // Kbits;
}