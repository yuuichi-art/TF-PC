#ifndef GEODRONE_JUEZ_HPP
#define GEODRONE_JUEZ_HPP

#include "../escenario/Escenario.hpp"
#include "../solvers/SolverMision.hpp"
#include "../solvers/Ruta.hpp"

#include <ostream>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Resultado de verificar una ruta: válida, o el primer problema encontrado.
 */
struct ResultadoVerificacion {
    bool valida = true;
    std::string problema;
};

/**
 * @brief Una fila del CSV de resultados: un escenario resuelto con un algoritmo.
 */
struct FilaJuez {
    std::string escenario;
    Categoria categoria = Categoria::SMALL;
    int clientes = 0;                // Clientes del escenario, incluidos los excluidos
    std::string algoritmo;
    std::string veredicto;            // OK, WA, TLE o MLE
    std::string problema;            // Motivo del WA (se muestra en consola, no va al CSV)
    double costo = 0.0;
    int recargas = 0;
    int excluidos = 0;
    double tiempoMs = 0.0;            // Preparación + resolución
    double memoriaMB = 0.0;          // Memoria pico del proceso al terminar el caso
    double gapMejor = 0.0;           // % sobre la mejor ruta OK del caso; solo con veredicto OK
    double gapCota = 0.0;          // % sobre la cota del árbol de expansión mínima; solo con OK
};

/**
 * @brief Una fila del ranking de una categoría.
 */
struct FilaRanking {
    Categoria categoria = Categoria::SMALL;
    int posicion = 0;
    std::string algoritmo;
    int casosOk = 0;
    int casos = 0;
    double gapMedio = 0.0;           // Promedio de gapMejor en los casos OK
    double tiempoMedioMs = 0.0;        // Promedio de tiempo en los casos OK
};

/**
 * @brief Juez competitivo y benchmark (RF14), según docs/decisiones.md §5.
 */
namespace Juez {

    /**
     * @brief Verifica una ruta con código independiente de los solvers: recorre la trayectoria
     * exportada con el escenario y la geometría. Del solver solo usa el grafo de misión y la red
     * de recarga, para confirmar las exclusiones. Comprueba que
     * - empieza y termina en la base, y cada punto etiquetado está donde dice el escenario;
     * - ningún tramo cruza un obstáculo;
     * - la batería simulada tramo a tramo nunca baja de 0 (se llena en la base y las estaciones);
     * - cada cliente se visita una vez o está excluido, y cada exclusión es real según el grafo
     *   de misión (separado por obstáculos o con 2·dRecarga > B);
     * - el costo, las recargas y el orden informados coinciden con la trayectoria.
     * Complejidad: O(P · M) con P puntos de la trayectoria y M aristas de obstáculos.
     */
    ResultadoVerificacion verificarRuta(const Escenario& escenario, const SolverMision& solver, const Ruta& ruta);

    /**
     * @brief Veredicto: WA si la ruta no es válida; si es válida, TLE si excede el tiempo,
     * MLE si excede la memoria y OK en otro caso.
     */
    std::string veredictoPara(bool valida, double tiempoMs, double memoriaMB, const LimitesCategoria& limites);

    // Archivos .txt de una carpeta, ordenados por nombre. Vacío si la carpeta no existe.
    std::vector<std::string> buscarEscenarios(const std::string& carpeta);

    /**
     * @brief Ejecuta todos los algoritmos sobre todos los escenarios, de menor a mayor tamaño,
     * para que la memoria pico de cada caso sea la de ese caso. El exacto solo corre en Small y
     * en los adversariales. Guarda cada ruta en carpetaRutas y escribe el avance en registro.
     */
    std::vector<FilaJuez> ejecutar(const std::vector<std::string>& archivosEscenario, const std::string& carpetaRutas,
                              std::ostream& registro);

    /**
     * @brief Ranking por categoría: más casos OK, luego menor gap medio, luego menor tiempo medio.
     */
    std::vector<FilaRanking> armarRanking(const std::vector<FilaJuez>& filas);

} // namespace Juez

} // namespace geodrone

#endif // GEODRONE_JUEZ_HPP
