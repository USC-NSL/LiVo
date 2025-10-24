#include <iostream>
#include <atomic>
#include <cstring>  // for memcpy
#include <thread>   // for std::thread
#include "timer.h"

#define BUF_ELEM 50     // Size of each buffer element (100 bytes)
#define MAX_BUF_SIZE 30  // Maximum number of buffer elements

class CircularBuffer {
public:
    CircularBuffer(const int max_elem, const int elem_size) : head(0), tail(0), max_elem(max_elem), elem_size(elem_size) 
	{
		max_buf_size = max_elem * elem_size;

        // Preallocate memory for the buffer
        buffer = new uint8_t[max_buf_size]();
    }

    ~CircularBuffer() {
        delete[] buffer;
    }

    // Insert data into the buffer
    bool insert(const uint8_t* data) {
        // Atomically check if the buffer has space (non-blocking)
        if (size.load(std::memory_order_acquire) == max_elem) {
            return false; // Buffer is full
        }

        // Copy the data to the buffer
        memcpy(&buffer[tail * elem_size], data, elem_size);

        // Update the tail position (circular behavior)
        tail = (tail + 1) % max_elem;

        std::cout << "Produced data, capacity: " << size.load() + 1 << std::endl;

        // Atomically increase the size
        size.fetch_add(1, std::memory_order_release);

        return true;
    }

    // Remove data from the buffer
    bool remove(uint8_t* data) {
        // Atomically check if the buffer is empty (non-blocking)
        if (size.load(std::memory_order_acquire) == 0) {
            return false; // Buffer is empty
        }

        // Copy the data from the buffer
        memcpy(data, &buffer[head * elem_size], elem_size);

        // Update the head position (circular behavior)
        head = (head + 1) % max_elem;

        std::cout << "Consumed data, capacity: " << size.load() - 1 << std::endl;

        // Atomically decrease the size
        size.fetch_sub(1, std::memory_order_release);

        return true;
    }

private:
    uint8_t* buffer;                   // Pointer to the buffer memory
    int head, tail;                    // Head and tail indices (non-atomic)
    std::atomic<int> size{0};         // Atomic size (number of valid elements)

	int max_elem;
	int elem_size;
	int max_buf_size;
};

// Producer thread function
void producer(CircularBuffer& cb) {
    uint8_t data[BUF_ELEM] = {0}; // Data to insert

    uint8_t counter = 0;
    FPSCounter2 fps(30.0); // Limit the producer to 30 FPS
    while (true) {
        fps.rate_limit();

        // Fill the buffer with example data
        for (int i = 0; i < BUF_ELEM; ++i) {
            data[i] = ++counter;
        }

        if (cb.insert(data)) {
            std::cout << "Producer: Inserted data" << std::endl;
        } else {
            std::cout << "Producer: Buffer full, waiting..." << std::endl;
        }
    }
}

// Consumer thread function
void consumer(CircularBuffer& cb) {
    uint8_t data[BUF_ELEM]; // Buffer to store removed data

    FPSCounter2 fps;
    while (true) {
        if (cb.remove(data)) {
            std::cout << "FPS: " << fps.fps_counter() << std::endl;
            std::cout << "Consumer: Removed data: ";
            for (int i = 0; i < BUF_ELEM; ++i) {
                std::cout << (int)data[i] << " ";
            }
            std::cout << std::endl;
        } else {
            std::cout << "Consumer: Buffer empty, waiting..." << std::endl;
        }

        sleep_ms(10);
    }
}

int main() {
    CircularBuffer cb(MAX_BUF_SIZE, BUF_ELEM);

    // Create producer and consumer threads
    std::thread producer_thread(producer, std::ref(cb));
    std::thread consumer_thread(consumer, std::ref(cb));

    // Join threads to main (this will keep the main function alive while threads run)
    producer_thread.join();
    consumer_thread.join();

    return 0;
}