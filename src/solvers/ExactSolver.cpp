#include "ExactSolver.hpp"
#include "../graph/Mst.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace geodrone {

namespace {

// Estado de la búsqueda: lo que comparten todas las llamadas recursivas.
struct Search {
    const std::vector<std::vector<double>>& dist;
    int depot;
    const std::vector<int>& clients;

    std::vector<char> visited;      // visited[i]: clients[i] ya está en la ruta parcial
    std::vector<int> current;       // Ruta parcial (claves de clientes)
    ExactResult best;

    Search(const std::vector<std::vector<double>>& d, int base, const std::vector<int>& c)
        : dist(d), depot(base), clients(c), visited(c.size(), 0) {
        best.cost = INF;
    }

    // Árbol de expansión mínima de {posición, clientes pendientes, base}.
    double lowerBound(int position) const {
        std::vector<int> nodes = {position};
        for (size_t i = 0; i < clients.size(); ++i) {
            if (!visited[i]) nodes.push_back(clients[i]);
        }
        if (position != depot) nodes.push_back(depot);
        return primMst(dist, nodes).cost;
    }

    // Clientes pendientes, del más cercano al más lejano desde `position`.
    std::vector<int> candidatesFrom(int position) const {
        std::vector<int> candidates;
        for (size_t i = 0; i < clients.size(); ++i) {
            if (!visited[i]) candidates.push_back(static_cast<int>(i));
        }
        std::sort(candidates.begin(), candidates.end(), [&](int a, int b) {
            return dist[position][clients[a]] < dist[position][clients[b]];
        });
        return candidates;
    }

    void branch(int position, double cost) {
        ++best.nodesExplored;

        // Hoja: todos los clientes visitados, solo falta volver a la base.
        if (current.size() == clients.size()) {
            double total = cost + dist[position][depot];
            if (total < best.cost) {
                best.cost = total;
                best.order = current;
            }
            return;
        }

        // Poda: ni en el mejor caso esta rama mejora la ruta conocida.
        if (cost + lowerBound(position) >= best.cost) {
            ++best.nodesPruned;
            return;
        }

        for (int i : candidatesFrom(position)) {
            double nextCost = cost + dist[position][clients[i]];
            if (nextCost >= best.cost) {
                ++best.nodesPruned;
                continue;
            }
            visited[i] = 1;
            current.push_back(clients[i]);
            branch(clients[i], nextCost);
            current.pop_back();
            visited[i] = 0;
        }
    }
};

} // namespace

ExactResult ExactSolver::solve(const std::vector<std::vector<double>>& dist, int depot,
                               const std::vector<int>& clients) {
    if (static_cast<int>(clients.size()) > MAX_CLIENTS) {
        throw std::invalid_argument("el solver exacto admite hasta " + std::to_string(MAX_CLIENTS) +
                                    " clientes y se pidieron " + std::to_string(clients.size()));
    }
    Search search(dist, depot, clients);
    search.branch(depot, 0.0);
    return search.best;
}

} // namespace geodrone
