#include "ChargingNetwork.hpp"

#include <algorithm>

namespace geodrone {

ChargingNetwork::ChargingNetwork(const MissionGraph& mission, double battery)
    : mission_(mission), battery_(battery), rechargeCount_(mission.numStations() + 1) {
    // DFS de Tarjan desde la base: marca los alcanzables (discovery > 0) y las articulaciones.
    std::vector<int> discovery(rechargeCount_, 0);
    std::vector<int> low(rechargeCount_, 0);
    articulation_.assign(rechargeCount_, 0);
    int timer = 0;
    tarjan(mission.depotKey(), -1, timer, discovery, low);

    reachable_.assign(rechargeCount_, 0);
    for (int r = 0; r < rechargeCount_; ++r) {
        if (discovery[r] > 0) {
            reachable_[r] = 1;
            reachablePoints_.push_back(r);  // La base (0) queda primera
        }
    }

    // dRecarga de cada nodo clave: mínimo sobre los puntos de recarga alcanzables.
    distanceToRecharge_.assign(mission.numKeyNodes(), INF);
    for (int key = 0; key < mission.numKeyNodes(); ++key) {
        for (int r : reachablePoints_) {
            distanceToRecharge_[key] = std::min(distanceToRecharge_[key], mission.distance(key, r));
        }
    }
}

// Comparación exacta, sin tolerancia: ver BATTERY_TOLERANCE en el encabezado.
bool ChargingNetwork::linked(int a, int b) const {
    return a != b && mission_.distance(a, b) <= battery_;
}

// Tarjan para puntos de articulación. discovery[u] es el orden de visita y low[u] el menor
// orden alcanzable desde el subárbol de u usando una arista de retroceso. Un nodo que no es
// raíz es articulación si algún hijo v cumple low[v] >= discovery[u]: el subárbol de v no
// tiene otra forma de llegar más arriba que pasando por u. La raíz lo es si tiene 2 o más hijos.
// La red es densa y pequeña, así que los vecinos se recorren por la matriz: O(R^2).
void ChargingNetwork::tarjan(int u, int parent, int& timer, std::vector<int>& discovery, std::vector<int>& low) {
    discovery[u] = low[u] = ++timer;
    int children = 0;
    for (int v = 0; v < rechargeCount_; ++v) {
        if (!linked(u, v)) continue;
        if (discovery[v] == 0) {
            ++children;
            tarjan(v, u, timer, discovery, low);
            low[u] = std::min(low[u], low[v]);
            if (parent != -1 && low[v] >= discovery[u]) articulation_[u] = 1;
        } else if (v != parent) {
            low[u] = std::min(low[u], discovery[v]);
        }
    }
    if (parent == -1 && children > 1) articulation_[u] = 1;
}

bool ChargingNetwork::canServe(int clientKey) const {
    return 2.0 * distanceToRecharge_[clientKey] <= battery_;
}

std::vector<int> ChargingNetwork::criticalStations() const {
    std::vector<int> stations;
    for (int r = 1; r < rechargeCount_; ++r) {     // La base (0) no se considera: no puede fallar
        if (articulation_[r]) stations.push_back(r);
    }
    return stations;
}

bool ChargingNetwork::isFeasible(const MissionGraph& mission, double battery) {
    ChargingNetwork network(mission, battery);
    for (int c = 0; c < mission.numClients(); ++c) {
        int key = mission.clientKey(c);
        if (mission.isConnectedToDepot(key) && !network.canServe(key)) return false;
    }
    return true;
}

double ChargingNetwork::minimumAutonomy(const MissionGraph& mission) {
    // Con B = 2 · (mayor distancia finita) todo es factible: cualquier cliente conectado
    // puede ir y volver directo a la base.
    double maxDistance = 0.0;
    for (const auto& row : mission.distances()) {
        for (double d : row) {
            if (d != INF) maxDistance = std::max(maxDistance, d);
        }
    }
    double lo = 0.0;
    double hi = 2.0 * maxDistance;
    for (int iteration = 0; iteration < 60; ++iteration) {
        double mid = (lo + hi) / 2.0;
        if (isFeasible(mission, mid)) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    return hi;
}

MstResult ChargingNetwork::supportTree(const MissionGraph& mission, std::vector<int>& nodes) {
    nodes.clear();
    for (int r = 0; r <= mission.numStations(); ++r) {
        if (mission.isConnectedToDepot(r)) nodes.push_back(r);
    }
    return primMst(mission.distances(), nodes);
}

} // namespace geodrone
