#include "UsoMemoria.hpp"

#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#endif

namespace geodrone {

double memoriaPicoMB() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS contadores;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &contadores, sizeof(contadores))) {
        return static_cast<double>(contadores.PeakWorkingSetSize) / (1024.0 * 1024.0);
    }
#endif
    return 0.0;
}

} // namespace geodrone
