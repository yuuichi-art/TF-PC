#ifndef GEODRONE_RANDOM_HPP
#define GEODRONE_RANDOM_HPP

#include <random>

namespace geodrone {

/**
 * @brief Entero uniforme en [lo, hi] con std::mt19937_64. Lo usan el generador de datasets y
 * la búsqueda local iterada.
 *
 * Se usa el módulo en lugar de std::uniform_int_distribution porque la salida de mt19937_64
 * está fijada por el estándar y la de la distribución no: así una semilla da el mismo resultado
 * con cualquier compilador. El sesgo del módulo es despreciable con rangos de miles frente a 2^64.
 */
inline long long randomInt(std::mt19937_64& rng, long long lo, long long hi) {
    unsigned long long range = static_cast<unsigned long long>(hi - lo) + 1ULL;
    return lo + static_cast<long long>(rng() % range);
}

} // namespace geodrone

#endif // GEODRONE_RANDOM_HPP
