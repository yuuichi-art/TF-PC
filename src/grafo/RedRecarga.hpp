#ifndef GEODRONE_REDRECARGA_HPP
#define GEODRONE_REDRECARGA_HPP

#include "GrafoMision.hpp"
#include "ArbolMinimo.hpp"

#include <vector>

namespace geodrone {

/**
 * @brief Red de recarga de un escenario para una autonomía B (docs/decisiones.md §4).
 *
 * Nodos: los puntos de recarga, es decir la base (clave 0) y las estaciones (claves 1..S).
 * Dos puntos se conectan si la distancia de vuelo entre ellos no supera B: el dron puede
 * ir de uno a otro con la batería llena.
 *
 * - Un DFS desde la base (Tarjan) marca los puntos alcanzables y los puntos de articulación:
 *   estaciones críticas, cuya pérdida deja estaciones sin conexión con la base.
 * - dRecarga(k) es la distancia del nodo clave k al punto de recarga alcanzable más cercano.
 * - Un cliente se puede atender si 2·dRecarga(c) ≤ B: hay que llegar desde una recarga y
 *   volver a una recarga, y lo mejor en ambos tramos es la más cercana.
 *
 * Construcción: O(R^2 + K·R) en tiempo, con R = S + 1 puntos de recarga y K nodos clave.
 */
class ChargingNetwork {
public:
    // Tolerancia para simular la batería tramo a tramo (planificador y juez), donde se acumulan
    // errores de redondeo. La red en sí compara de forma exacta: si la tolerancia formara parte
    // de "factible", la búsqueda binaria convergería justo al borde de esa tolerancia y la
    // simulación fallaría por diferencias de 1e-13. Así la autonomía mínima siempre deja a la
    // simulación este margen a favor.
    static constexpr double BATTERY_TOLERANCE = 1e-6;

    // Guarda una referencia al grafo de misión, que debe seguir existiendo mientras se use la red.
    ChargingNetwork(const MissionGraph& mission, double bateria);
    // Prohibido con un grafo temporal: la referencia quedaría apuntando a un objeto destruido.
    ChargingNetwork(MissionGraph&& mission, double bateria) = delete;

    double bateria() const { return battery_; }

    // Arista de la red: distancia de vuelo entre dos puntos de recarga ≤ B.
    bool linked(int a, int b) const;

    bool isReachable(int rechargeKey) const { return reachable_[rechargeKey] != 0; }
    const std::vector<int>& reachablePoints() const { return reachablePoints_; }

    // dRecarga: distancia al punto de recarga alcanzable más cercano (infinito si no hay).
    double distanceToRecharge(int key) const { return distanceToRecharge_[key]; }

    bool canServe(int clientKey) const;

    // Estaciones (claves) que son puntos de articulación de la red alcanzable desde la base.
    std::vector<int> criticalStations() const;

    /**
     * @brief Factibilidad de una autonomía B: todos los clientes que los obstáculos no separan
     * de la base se pueden atender. Es monótona: si vale para B, vale para cualquier B mayor,
     * porque con más batería la red tiene más aristas, alcanza más puntos de recarga y dRecarga
     * de cada cliente solo puede bajar. Esa monotonía es la que permite la búsqueda binaria.
     */
    static bool isFeasible(const MissionGraph& mission, double bateria);

    /**
     * @brief Menor autonomía factible, por búsqueda binaria sobre reales (RF10).
     * Busca entre 0 y el doble de la mayor distancia finita de la matriz, con 60 iteraciones
     * fijas: el intervalo final mide menos de 1e-14 veces el inicial.
     * Complejidad: O(60 · (R^2 + K·R)).
     */
    static double minimumAutonomy(const MissionGraph& mission);

    /**
     * @brief Red mínima de soporte (RF12): árbol de expansión mínima entre la base y las
     * estaciones que los obstáculos no separan de la base. No depende de B. Los nodos del
     * resultado son esas claves, con la base en la posición 0.
     */
    static MstResult supportTree(const MissionGraph& mission, std::vector<int>& nodes);

private:
    const MissionGraph& mission_;
    double battery_;
    int rechargeCount_;                     // R = S + 1
    std::vector<char> reachable_;           // Por punto de recarga
    std::vector<char> articulation_;        // Por punto de recarga
    std::vector<int> reachablePoints_;      // Claves alcanzables, base primero
    std::vector<double> distanceToRecharge_; // Por nodo clave

    void tarjan(int u, int parent, int& timer, std::vector<int>& discovery, std::vector<int>& low);
};

} // namespace geodrone

#endif // GEODRONE_REDRECARGA_HPP
