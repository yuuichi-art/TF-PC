#ifndef GEODRONE_PLANIFICADORRECARGAS_HPP
#define GEODRONE_PLANIFICADORRECARGAS_HPP

#include "../grafo/RedRecarga.hpp"
#include "../grafo/GrafoMision.hpp"

#include <vector>

namespace geodrone {

/**
 * @brief Ruta de nodos clave con las recargas ya insertadas.
 */
struct RechargePlan {
    bool feasible = false;
    std::vector<int> keySequence;   // Empieza y termina en la base (clave 0)
    int recharges = 0;              // Paradas en estaciones o en la base a mitad de ruta
    double cost = 0.0;              // Distancia total volada, incluidos los desvíos
};

/**
 * @brief Inserción de recargas sobre un orden de clientes ya decidido (docs/decisiones.md §3:
 * los solvers deciden el orden y este paso común agrega las recargas).
 *
 * Regla greedy segura (docs/decisiones.md §4): antes de cada tramo se comprueba que, al
 * llegar al destino, quede batería para alcanzar su recarga más cercana (dRecarga).
 * - Si alcanza, se vuela directo.
 * - Si no, se busca el desvío más corto por la red de recarga: un Dijkstra sobre los puntos
 *   de recarga, partiendo de los que se alcanzan con la batería actual, hasta uno desde el
 *   cual el destino sea seguro con la batería llena. Un desvío a una sola estación es el
 *   caso de un salto; si hace falta, el camino encadena varias estaciones.
 * - Si no existe ese camino, la ruta es infactible.
 *
 * Si todos los clientes cumplen 2·dRecarga ≤ B, la ruta siempre es factible: siempre se
 * alcanza la recarga más cercana, desde ahí la red llega a la recarga más cercana del
 * destino, y desde esa el destino es seguro.
 *
 * Complejidad: O(N · R^2) en tiempo y O(R) en espacio extra, con N clientes y R puntos de recarga.
 * La red y el grafo de misión deben seguir existiendo mientras se use el planificador.
 */
class RechargePlanner {
public:
    RechargePlanner(const MissionGraph& mission, const ChargingNetwork& network);
    // Prohibido con temporales: las referencias quedarían apuntando a objetos destruidos.
    RechargePlanner(MissionGraph&& mission, const ChargingNetwork& network) = delete;
    RechargePlanner(const MissionGraph& mission, ChargingNetwork&& network) = delete;

    // clientOrder: claves de los clientes en el orden de visita (sin la base).
    RechargePlan plan(const std::vector<int>& clientOrder) const;

private:
    const MissionGraph& mission_;
    const ChargingNetwork& network_;

    // Recargas a visitar desde `from` con batería `bateria` para llegar seguro a `target`.
    // Devuelve false si no hay forma. El camino no incluye a `from` ni a `target`.
    bool findRechargePath(int from, double bateria, int target, std::vector<int>& path) const;

    // Paso 1: Dijkstra O(R^2) sobre la red de recarga. Orígenes: los puntos que se alcanzan
    // desde `from` con la batería actual. best[i] es la distancia hasta reachablePoints()[i]
    // y previous[i] el punto anterior en ese camino (-1 si es un origen o no se alcanza).
    std::vector<double> networkDistances(int from, double bateria, std::vector<int>& previous) const;

    // Paso 2: el último punto de recarga, desde el cual `target` es seguro con batería llena,
    // que minimiza el total. Devuelve su posición en reachablePoints() o -1.
    int chooseLastRecharge(const std::vector<double>& best, int target) const;
};

} // namespace geodrone

#endif // GEODRONE_PLANIFICADORRECARGAS_HPP
