#ifndef GEODRONE_RECHARGEPLANNER_HPP
#define GEODRONE_RECHARGEPLANNER_HPP

#include "../graph/ChargingNetwork.hpp"
#include "../graph/MissionGraph.hpp"

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
 * @brief Inserción de recargas sobre un orden de clientes ya decidido (decisión 5:
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

    // clientOrder: claves de los clientes en el orden de visita (sin la base).
    RechargePlan plan(const std::vector<int>& clientOrder) const;

private:
    const MissionGraph& mission_;
    const ChargingNetwork& network_;

    // Recargas a visitar desde `from` con batería `battery` para llegar seguro a `target`.
    // Vacío si no hay forma. No incluye a `from` ni a `target`.
    bool findRechargePath(int from, double battery, int target, std::vector<int>& path) const;
};

} // namespace geodrone

#endif // GEODRONE_RECHARGEPLANNER_HPP
