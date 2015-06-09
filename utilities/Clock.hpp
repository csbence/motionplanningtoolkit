#ifndef MOTIONPLANNING_CLOCK_HPP
#define MOTIONPLANNING_CLOCK_HPP

#include <chrono>
#include <mutex>

class Clock {

    typedef std::chrono::duration<unsigned int, std::milli> Millisecond;

public:

    Clock(std::string name = "") : running(false),
                              totalDuration(0),
                              lock(),
                              name(name) {
    }

    ~Clock() { }

    void start() {
        std::lock_guard<std::mutex> guard(lock);
        if (!running) {
            startTime = std::chrono::steady_clock::now();
            running = true;
        } else {
            fprintf(stderr, "Clock[%s] is already running, cannot be started.", name.c_str());
        }
    }

    unsigned int stop() {
        std::lock_guard<std::mutex> guard(lock);

        if (running) {
            std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();
            Millisecond duration(std::chrono::duration_cast<Millisecond>(endTime - startTime));

            totalDuration += duration;
            running = false;
        } else {
            fprintf(stderr, "Clock[%s] is not running, cannot be stopped.", name.c_str());
        }

        return totalDuration.count();
    }

    unsigned int getDurationInMillis() {
        std::lock_guard<std::mutex> guard(lock);

        return running ?: totalDuration.count();
    }

private:
    std::chrono::steady_clock::time_point startTime;
    bool running;
    Millisecond totalDuration;
    std::mutex lock;
    const std::string name;
};

#endif //MOTIONPLANNING_CLOCK_HPP
