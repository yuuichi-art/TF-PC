#ifndef GEODRONE_GENERADORDATASETS_HPP
#define GEODRONE_GENERADORDATASETS_HPP

#include "Escenario.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Rangos de tamaño de una categoría (confirmados con mediciones, ver docs/decisiones.md §7).
 * El generador elige cada cantidad al azar dentro de su rango.
 */
struct RangosCategoria {
    int minClientes;
    int maxClientes;
    int minEstaciones;
    int maxEstaciones;
    int minObstaculos;
    int maxObstaculos;
    int maxPuntosPorObstaculo;   // Puntos aleatorios por obstáculo; su envolvente puede tener menos vértices
};

/**
 * @brief Generador determinista y reproducible de escenarios con obstáculos, estaciones y clientes.
 *
 * - Usa std::mt19937_64 con la semilla recibida: la misma semilla da el mismo escenario.
 * - Coordenadas enteras dentro del área (docs/decisiones.md §2).
 * - Obstáculos convexos sin solape (§2): el área se divide en celdas y cada
 *   obstáculo es la envolvente convexa de puntos aleatorios dentro de una celda distinta.
 * - Base, estaciones y clientes en puntos libres: fuera de todo obstáculo y sin repetir.
 *
 * Complejidad: O(M·k log k) para M obstáculos de k puntos, más O((N + S)·V) para colocar
 * N clientes y S estaciones comprobando los V vértices de obstáculos.
 */
class GeneradorDatasets {
public:
    // Margen sobre la autonomía mínima: con 1,2 todos los clientes se pueden atender y aun
    // así hacen falta recargas, que es lo que se quiere poner a prueba.
    static constexpr double MARGEN_BATERIA = 1.2;

    static RangosCategoria rangosDe(Categoria categoria);

    // Los mismos rangos con cada cantidad fija en su máximo: el peor caso de la categoría.
    static RangosCategoria rangosPeorCaso(Categoria categoria);

    /**
     * @brief Genera un escenario completo de la categoría dada (SMALL, MEDIUM, LARGE o EXTREME):
     * la geometría de generarGeometria más la batería de bateriaPara.
     * El nombre es "<categoria>_<semilla>", por ejemplo "small_3".
     * Lanza std::invalid_argument con ADVERSARIAL: esos casos se escriben a mano.
     */
    static Escenario generar(Categoria categoria, uint64_t semilla,
                             double ancho = 1000.0, double alto = 1000.0);

    /**
     * @brief Solo la geometría: obstáculos, base, estaciones y clientes, con batería 0 (sin
     * calcular). Es rápida porque no construye el grafo de misión; la usan las pruebas y la
     * medición del peor caso (con todos los rangos en su máximo).
     */
    static Escenario generarGeometria(const RangosCategoria& rangos, Categoria categoria, uint64_t semilla,
                                   double ancho = 1000.0, double alto = 1000.0);

    /**
     * @brief Batería del escenario: la autonomía mínima (búsqueda binaria, RF10) por
     * MARGEN_BATERIA, redondeada hacia arriba a un entero. Al menos 1.
     * Construye el grafo de misión, así que cuesta lo mismo que prepararlo para resolver.
     */
    static double bateriaPara(const Escenario& geometria);

    /**
     * @brief Genera las semillas 1..semillasPorCategoria de cada categoría y las guarda en carpeta
     * como "<categoria>_<semilla>.txt". Con incluirPeorCaso agrega "extreme_max.txt": un Extreme
     * con todos los rangos en su máximo (semilla 1), para comprobar que el peor caso cabe en el
     * límite. Devuelve las rutas creadas, de menor a mayor categoría.
     */
    static std::vector<std::string> generarConjuntoBenchmark(const std::string& carpeta, int semillasPorCategoria = 5,
                                                         bool incluirPeorCaso = true);
};

} // namespace geodrone

#endif // GEODRONE_GENERADORDATASETS_HPP
