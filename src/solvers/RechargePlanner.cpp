#include "RechargePlanner.hpp"

#include <algorithm>
#include <limits>

namespace geodrone {

namespace {
const double INF = std::numeric_limits<double>::infinity();
const double TOL = ChargingNetwork::BATTERY_TOLERANCE;
}

RechargePlanner::RechargePlanner(const MissionGraph& mission, const ChargingNetwork& network)
    : mission_(mission), network_(network) {}

bool RechargePlanner::findRechargePath(int from, double battery, int target, std::vector<int>& path) const {
    const std::vector<int>& points = network_.reachablePoints();
    int r = static_cast<int>(points.size());
    double full = network_.battery();

    // Dijkstra O(R^2) sobre la red de recarga. Orígenes: los puntos que se alcanzan desde
    // `from` con la batería actual, con su distancia como costo inicial.
    std::vector<double> best(r, INF);
    std::vector<int> previous(r, -1);
    std::vector<char> done(r, 0);
    for (int i = 0; i < r; ++i) {
        double d = mission_.distance(from, points[i]);
        if (d <= battery + TOL) best[i] = d;
    }
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

    // Último punto de recarga: desde él, con batería llena, el destino debe ser seguro.
    int end = -1;
    double bestTotal = INF;
    for (int i = 0; i < r; ++i) {
        if (best[i] == INF) continue;
        double leg = mission_.distance(points[i], target);
        if (full - leg + TOL < network_.distanceToRecharge(target)) continue;
        if (best[i] + leg < bestTotal) {
            bestTotal = best[i] + leg;
            end = i;
        }
    }
    if (end == -1) return false;

    path.clear();
    for (int i = end; i != -1; i = previous[i]) path.push_back(points[i]);
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
