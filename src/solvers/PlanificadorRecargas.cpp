#include "PlanificadorRecargas.hpp"

#include <algorithm>

namespace geodrone {

namespace {
const double TOL = ChargingNetwork::BATTERY_TOLERANCE;
}

RechargePlanner::RechargePlanner(const MissionGraph& mission, const ChargingNetwork& network)
    : mission_(mission), network_(network) {}

std::vector<double> RechargePlanner::networkDistances(int from, double battery, std::vector<int>& previous) const {
    const std::vector<int>& points = network_.reachablePoints();
    int r = static_cast<int>(points.size());
    std::vector<double> best(r, INF);
    std::vector<char> done(r, 0);
    previous.assign(r, -1);

    // Orígenes: los puntos alcanzables con la batería actual, con su distancia como costo inicial.
    for (int i = 0; i < r; ++i) {
        double d = mission_.distance(from, points[i]);
        if (d <= battery + TOL) best[i] = d;
    }
    // Dijkstra sin cola de prioridad: con pocos nodos (R ≤ 26) basta buscar el mínimo en cada paso.
    for (int step = 0; step < r; ++step) {
        int u = -1;
        for (int i = 0; i < r; ++i) {
            if (!done[i] && best[i] < INF && (u == -1 || best[i] < best[u])) u = i;
        }
        if (u == -1) break;
        done[u] = 1;
        for (int v = 0; v < r; ++v) {
            if (done[v] || !network_.linked(points[u], points[v])) continue;
            double candidate = best[u] + mission_.distance(points[u], points[v]);
            if (candidate < best[v]) {
                best[v] = candidate;
                previous[v] = u;
            }
        }
    }
    return best;
}

int RechargePlanner::chooseLastRecharge(const std::vector<double>& best, int target) const {
    const std::vector<int>& points = network_.reachablePoints();
    int chosen = -1;
    double bestTotal = INF;
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        if (best[i] == INF) continue;
        double leg = mission_.distance(points[i], target);
        bool safe = network_.battery() - leg + TOL >= network_.distanceToRecharge(target);
        if (safe && best[i] + leg < bestTotal) {
            bestTotal = best[i] + leg;
            chosen = i;
        }
    }
    return chosen;
}

bool RechargePlanner::findRechargePath(int from, double battery, int target, std::vector<int>& path) const {
    std::vector<int> previous;
    std::vector<double> best = networkDistances(from, battery, previous);
    int last = chooseLastRecharge(best, target);
    if (last == -1) return false;

    // Paso 3: reconstruir el camino de recargas siguiendo los predecesores.
    const std::vector<int>& points = network_.reachablePoints();
    path.clear();
    for (int i = last; i != -1; i = previous[i]) path.push_back(points[i]);
    std::reverse(path.begin(), path.end());
    // `from` o `target` pueden ser puntos de recarga: no se repiten como paradas.
    if (!path.empty() && path.front() == from) path.erase(path.begin());
    if (!path.empty() && path.back() == target) path.pop_back();
    return true;
}

RechargePlan RechargePlanner::plan(const std::vector<int>& clientOrder) const {
    RechargePlan result;
    const double full = network_.battery();
    double battery = full;
    int position = mission_.depotKey();
    result.keySequence.push_back(position);

    // Sin clientes el dron no despega: la ruta es solo la base.
    if (clientOrder.empty()) {
        result.feasible = true;
        return result;
    }

    auto flyTo = [&](int key) {
        double d = mission_.distance(position, key);
        result.cost += d;
        battery -= d;
        position = key;
        result.keySequence.push_back(key);
    };

    std::vector<int> targets = clientOrder;
    targets.push_back(mission_.depotKey());   // Al final se vuelve a la base

    for (int target : targets) {
        // Directo es seguro si, al llegar, queda batería para alcanzar la recarga más cercana del
        // destino: así nunca se llega a un punto del que no se pueda salir.
        bool safeDirect = battery - mission_.distance(position, target) + TOL >= network_.distanceToRecharge(target);
        if (!safeDirect) {
            std::vector<int> recharges;
            if (!findRechargePath(position, battery, target, recharges)) {
                return result;   // feasible = false; la secuencia queda hasta donde se llegó
            }
            for (int stop : recharges) {
                flyTo(stop);
                battery = full;
                ++result.recharges;
            }
        }
        flyTo(target);
    }
    result.feasible = true;
    return result;
}

} // namespace geodrone
