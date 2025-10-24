#include <iostream>
#include "timer.h"
#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"
typedef boost::mt19937 RNGType;

using namespace std;
int main() 
{
	RNGType rng;
	boost::uniform_int<> rangei(1, 15);
	boost::variate_generator<RNGType, boost::uniform_int<>> rand_gen(rng, rangei);

	FPSCounter2 fps_limiter(30);
	FPSCounter2 fps_counter(30);
	StopWatch sw;
	int count = 0;
	while (true) 
	{
		float fps_returned = fps_limiter.rate_limit();
		uint64_t delay = 25 + rand_gen();
		sleep_ms(delay);
		count++;
		cout << "ID: " << count << ", FPS: " << fps_counter.fps_counter() << endl;
	}

	return 0;
}