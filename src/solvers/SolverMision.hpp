#ifndef GEODRONE_SOLVERMISION_HPP
#define GEODRONE_SOLVERMISION_HPP

#include "Ruta.hpp"
#include "../grafo/RedRecarga.hpp"
#include "../grafo/GrafoMision.hpp"
#include "../escenario/Escenario.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Algoritmos que deciden el orden de visita. El nombre es el que aparece en los
 * archivos de ruta y en el CSV del juez.
 */
enum class Algoritmo {
    BRANCH_AND_BOUND,           // Exacto, solo Small (hasta SolverExacto::MAX_CLIENTES clientes)
    VECINO_MAS_CERCANO,
    DOS_APROXIMACION,
    VECINO_MAS_CERCANO_2OPT,
    DOS_APROXIMACION_2OPT,
    BUSQUEDA_LOCAL_ITERADA       // Parte del vecino más cercano
};

std::string nombreAlgoritmo(Algoritmo algoritmo);
const std::vector<Algoritmo>& todosLosAlgoritmos();

/**
 * @brief Resuelve un escenario completo:
 * 1. Prepara una sola vez el grafo de misión y la red de recarga.
 * 2. Excluye los clientes que los obstáculos separan de la base (DFS) y los que la batería no
 *    permite atender (2·dRecarga > B).
 * 3. Para cada algoritmo pedido: orden de visita, inserción de recargas y trayectoria real.
 *
 * Preparar una vez y resolver varias permite que el juez compare algoritmos sin reconstruir
 * el grafo. El tiempo de una ruta es preparación + resolución, como si se resolviera sola.
 *
 * Guarda el grafo y la red como miembros, que se referencian entre sí: no se puede copiar.
 */
class SolverMision {
public:
    explicit SolverMision(const Escenario& escenario);
    SolverMision(const SolverMision&) = delete;
    SolverMision& operator=(const SolverMision&) = delete;

    // El exacto solo se puede usar con pocos clientes atendibles.
    bool admiteExacto() const;

    /**
     * @brief Resuelve con el algoritmo dado. La búsqueda local iterada se detiene antes de llegar
     * al 90 % del tiempo límite de la categoría, descontando la preparación.
     * Lanza std::invalid_argument si se pide el exacto y admiteExacto() es false.
     */
    Ruta resolver(Algoritmo algoritmo) const;

    const GrafoMision& mision() const { return mision_; }
    const RedRecarga& red() const { return red_; }
    const std::vector<ClienteExcluido>& excluidos() const { return excluidos_; }
    const std::vector<int>& clientesAtendibles() const { return atendibles_; }   // Claves
    double preparacionMs() const { return preparacionMs_; }

private:
    std::string nombre_;
    uint64_t semilla_;
    double limiteTiempoSegundos_;
    double preparacionMs_ = 0.0;
    // Los miembros se construyen en el orden en que se declaran: este va antes que el grafo
    // para que la medición de la preparación lo incluya.
    std::chrono::steady_clock::time_point inicioConstruccion_;
    GrafoMision mision_;
    RedRecarga red_;              // Referencia a mision_: se declara después
    std::vector<int> atendibles_;
    std::vector<ClienteExcluido> excluidos_;

    std::vector<int> elegirOrden(Algoritmo algoritmo) const;
    std::vector<PuntoRuta> expandirTrayectoria(const std::vector<int>& secuenciaClaves) const;
};

} // namespace geodrone

#endif // GEODRONE_SOLVERMISION_HPP
