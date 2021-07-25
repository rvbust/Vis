#include <Vis/Vis.h>
#include <chrono>
#include <thread>

static inline void Sleep(uint64_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char *argv[]) {
    Vis::View v("Basics");
    v.Ground(20, 0.1, {1, 0, 0});
    v.Box({0, 0, 1}, {1, 1, 1}, {0, 1, 0});
    Sleep(10000);
    return 0;
}
