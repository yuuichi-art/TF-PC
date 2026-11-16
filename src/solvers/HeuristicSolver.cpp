#include "HeuristicSolver.hpp"
#include "../graph/Mst.hpp"
#include "../scenario/Random.hpp"

#include <algorithm>
#include <random>

namespace geodrone {
namespace HeuristicSolver {

namespace {

// Mejora mínima para aplicar un intercambio: evita ciclos infinitos por errores de redondeo.
const double MIN_IMPROVEMENT = 1e-7;

// Preorden del árbol: se visita un nodo y luego sus hijos, del más cercano al más lejano.
void preorder(int node, const std::vector<std::vector<int>>& children, const std::vector<int>& nodes,
              std::vector<int>& order) {
    order.push_back(nodes[node]);
    for (int child : children[node]) {
        preorder(child, children, nodes, order);
    }
}

// Double-bridge: corta en tres puntos al azar (A | B | C | D) y arma A C B D.
std::vector<int> doubleBridge(const std::vector<int>& order, std::mt19937_64& rng) {
    int n = static_cast<int>(order.size());
    int p1 = static_cast<int>(randomInt(rng, 1, n - 3));
    int p2 = static_cast<int>(randomInt(rng, p1 + 1, n - 2));
    int p3 = static_cast<int>(randomInt(rng, p2 + 1, n - 1));
    std::vector<int> result(order.begin(), order.begin() + p1);                  // A
    result.insert(result.end(), order.begin() + p2, order.begin() + p3);         // C
    result.insert(result.end(), order.begin() + p1, order.begin() + p2);         // B
    result.insert(result.end(), order.begin() + p3, order.end());                // D
    return result;
}

} // namespace

double tourCost(const std::vector<std::vector<double>>& dist, int depot, const std::vector<int>& order) {
    double cost = 0.0;
    int position = depot;
    for (int client : order) {
        cost += dist[position][client];
        position = client;
    }
    return cost + dist[position][depot];
}

std::vector<int> nearestNeighbor(const std::vector<std::vector<double>>& dist, int depot,
                                 const std::vector<int>& clients) {
    std::vector<int> order;
    std::vector<char> visited(clients.size(), 0);
    int position = depot;
    for (size_t step = 0; step < clients.size(); ++step) {
        int next = -1;
        for (size_t i = 0; i < clients.size(); ++i) {
            if (visited[i]) continue;
            if (next == -1 || dist[position][clients[i]] < dist[position][clients[next]]) {
                next = static_cast<int>(i);
            }
        }
        visited[next] = 1;
        position = clients[next];
        order.push_back(position);
    }
    return order;
}

std::vector<int> mstApproximation(const std::vector<std::vector<double>>& dist, int depot,
                                  const std::vector<int>& clients) {
    std::vector<int> nodes = {depot};
    nodes.insert(nodes.end(), clients.begin(), clients.end());
    MstResult tree = primMst(dist, nodes);

    // Hijos de cada nodo del árbol (posiciones dentro de nodes), del más cercano al más lejano.
    // Un nodo sin padre solo aparece si no tiene camino (no debería pasar: los clientes deben ser
    // alcanzables); se cuelga de la raíz para no acceder fuera del vector.
    std::vector<std::vector<int>> children(nodes.size());
    for (size_t i = 1; i < nodes.size(); ++i) {
        int parent = tree.parent[i] >= 0 ? tree.parent[i] : 0;
        children[parent].push_back(static_cast<int>(i));
    }
    for (size_t p = 0; p < nodes.size(); ++p) {
        std::sort(children[p].begin(), children[p].end(), [&](int a, int b) {
            return dist[nodes[p]][nodes[a]] < dist[nodes[p]][nodes[b]];
        });
    }

    std::vector<int> order;
    preorder(0, children, nodes, order);
    order.erase(order.begin());   // El preorden empieza en la base; el orden lleva solo clientes
    return order;
}

bool twoOpt(const std::vector<std::vector<double>>& dist, int depot, std::vector<int>& order,
            Clock::time_point deadline) {
    // Ruta completa con la base en ambos extremos: tour[0] = tour[n + 1] = base.
    std::vector<int> tour = {depot};
    tour.insert(tour.end(), order.begin(), order.end());
    tour.push_back(depot);
    int n = static_cast<int>(order.size());

    // Se repiten pasadas sobre todos los pares (i, j) hasta que una pasada no mejore nada.
    // Cada mejora se aplica en el momento y la pasada sigue con la ruta ya cambiada.
    // finished pasa a false si se acaba el tiempo: la ruta queda como esté en ese momento.
    bool improved = true;
    bool finished = true;
    while (improved && finished) {
        improved = false;
        for (int i = 1; i < n && finished; ++i) {
            if (Clock::now() > deadline) {
                finished = false;
                break;
            }
            for (int j = i + 1; j <= n; ++j) {
                // Quitar (a,b) y (c,d); poner (a,c) y (b,d) invirtiendo el tramo b..c.
                int a = tour[i - 1], b = tour[i], c = tour[j], d = tour[j + 1];
                double delta = dist[a][c] + dist[b][d] - dist[a][b] - dist[c][d];
                if (delta < -MIN_IMPROVEMENT) {
                    std::reverse(tour.begin() + i, tour.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }

    order.assign(tour.begin() + 1, tour.end() - 1);
    return finished;
}

IlsResult iteratedLocalSearch(const std::vector<std::vector<double>>& dist, int depot,
                              const std::vector<int>& initialOrder, uint64_t seed,
                              int maxIterations, Clock::time_point deadline) {
    IlsResult result;
    result.order = initialOrder;
    result.stoppedByTime = !twoOpt(dist, depot, result.order, deadline);
    double bestCost = tourCost(dist, depot, result.order);

    // Double-bridge necesita al menos 4 clientes para cortar la ruta en cuatro tramos.
    if (result.order.size() < 4) return result;

    std::mt19937_64 rng(seed);
    while (result.iterations < maxIterations && !result.stoppedByTime) {
        if (Clock::now() > deadline) {
            result.stoppedByTime = true;
            break;
        }
        ++result.iterations;
        std::vector<int> candidate = doubleBridge(result.order, rng);
        if (!twoOpt(dist, depot, candidate, deadline)) {
            result.stoppedByTime = true;   // La candidata quedó a medias: no se considera
            break;
        }
        double cost = tourCost(dist, depot, candidate);
        if (cost < bestCost - MIN_IMPROVEMENT) {
            result.order = std::move(candidate);
            bestCost = cost;
        }
    }
    return result;
}

} // namespace HeuristicSolver
} // namespace geodrone
