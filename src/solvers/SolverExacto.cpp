#include "SolverExacto.hpp"
#include "../grafo/ArbolMinimo.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace geodrone {

namespace {

// Estado de la búsqueda: lo que comparten todas las llamadas recursivas.
struct Search {
    const std::vector<std::vector<double>>& dist;
    int base;
    const std::vector<int>& clientes;

    std::vector<char> visited;      // visited[i]: clientes[i] ya está en la ruta parcial
    std::vector<int> current;       // Ruta parcial (claves de clientes)
    ExactResult best;

    Search(const std::vector<std::vector<double>>& d, int b, const std::vector<int>& c)
        : dist(d), base(b), clientes(c), visited(c.size(), 0) {
        best.costo = INF;
    }

    // Árbol de expansión mínima de {posición, clientes pendientes, base}.
    double lowerBound(int position) const {
        std::vector<int> nodes = {position};
        for (size_t i = 0; i < clientes.size(); ++i) {
            if (!visited[i]) nodes.push_back(clientes[i]);
        }
        if (position != base) nodes.push_back(base);
        return arbolMinimoPrim(dist, nodes).costo;
    }

    // Clientes pendientes, del más cercano al más lejano desde `position`.
    std::vector<int> candidatesFrom(int position) const {
        std::vector<int> candidates;
        for (size_t i = 0; i < clientes.size(); ++i) {
            if (!visited[i]) candidates.push_back(static_cast<int>(i));
        }
        std::sort(candidates.begin(), candidates.end(), [&](int a, int b) {
            return dist[position][clientes[a]] < dist[position][clientes[b]];
        });
        return candidates;
    }

    void branch(int position, double costo) {
        ++best.nodesExplored;

        // Hoja: todos los clientes visitados, solo falta volver a la base.
        if (current.size() == clientes.size()) {
            double total = costo + dist[position][base];
            if (total < best.costo) {
                best.costo = total;
                best.order = current;
            }
            return;
        }

        // Poda: ni en el mejor caso esta rama mejora la ruta conocida.
        if (costo + lowerBound(position) >= best.costo) {
            ++best.nodesPruned;
            return;
        }

        for (int i : candidatesFrom(position)) {
            double nextCost = costo + dist[position][clientes[i]];
            if (nextCost >= best.costo) {    // Solo llegar a ese cliente ya iguala la mejor ruta
                ++best.nodesPruned;
                continue;
            }
            visited[i] = 1;
            current.push_back(clientes[i]);
            branch(clientes[i], nextCost);
            current.pop_back();
            visited[i] = 0;
        }
    }
};

} // namespace

ExactResult ExactSolver::solve(const std::vector<std::vector<double>>& dist, int base,
                               const std::vector<int>& clientes) {
    if (static_cast<int>(clientes.size()) > MAX_CLIENTS) {
        throw std::invalid_argument("el solver exacto admite hasta " + std::to_string(MAX_CLIENTS) +
                                    " clientes y se pidieron " + std::to_string(clientes.size()));
    }
    Search search(dist, base, clientes);
    search.branch(base, 0.0);
    return search.best;
}

} // namespace geodrone
