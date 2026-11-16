#include "UsoMemoria.hpp"

#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#endif

namespace geodrone {

double peakMemoryMB() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS counters;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters))) {
        return static_cast<double>(counters.PeakWorkingSetSize) / (1024.0 * 1024.0);
    }
#endif
    return 0.0;
}

} // namespace geodrone
