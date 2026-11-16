#include "SolverExacto.hpp"
#include "../grafo/ArbolMinimo.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace geodrone {

namespace {

// Estado de la búsqueda: lo que comparten todas las llamadas recursivas.
struct Busqueda {
    const std::vector<std::vector<double>>& distancias;
    int base;
    const std::vector<int>& clientes;

    std::vector<char> visitado;    // visitado[i]: clientes[i] ya está en la ruta parcial
    std::vector<int> actual;       // Ruta parcial (claves de clientes)
    ResultadoExacto mejor;

    Busqueda(const std::vector<std::vector<double>>& d, int b, const std::vector<int>& c)
        : distancias(d), base(b), clientes(c), visitado(c.size(), 0) {
        mejor.costo = INF;
    }

    // Árbol de expansión mínima de {posición, clientes pendientes, base}.
    double cotaInferior(int posicion) const {
        std::vector<int> nodos = {posicion};
        for (size_t i = 0; i < clientes.size(); ++i) {
            if (!visitado[i]) nodos.push_back(clientes[i]);
        }
        if (posicion != base) nodos.push_back(base);
        return arbolMinimoPrim(distancias, nodos).costo;
    }

    // Clientes pendientes, del más cercano al más lejano desde `posicion`.
    std::vector<int> candidatosDesde(int posicion) const {
        std::vector<int> candidatos;
        for (size_t i = 0; i < clientes.size(); ++i) {
            if (!visitado[i]) candidatos.push_back(static_cast<int>(i));
        }
        std::sort(candidatos.begin(), candidatos.end(), [&](int a, int b) {
            return distancias[posicion][clientes[a]] < distancias[posicion][clientes[b]];
        });
        return candidatos;
    }

    void ramificar(int posicion, double costo) {
        ++mejor.nodosExplorados;

        // Hoja: todos los clientes visitados, solo falta volver a la base.
        if (actual.size() == clientes.size()) {
            double total = costo + distancias[posicion][base];
            if (total < mejor.costo) {
                mejor.costo = total;
                mejor.orden = actual;
            }
            return;
        }

        // Poda: ni en el mejor caso esta rama mejora la ruta conocida.
        if (costo + cotaInferior(posicion) >= mejor.costo) {
            ++mejor.nodosPodados;
            return;
        }

        for (int i : candidatosDesde(posicion)) {
            double costoSiguiente = costo + distancias[posicion][clientes[i]];
            if (costoSiguiente >= mejor.costo) {    // Solo llegar a ese cliente ya iguala la mejor ruta
                ++mejor.nodosPodados;
                continue;
            }
            visitado[i] = 1;
            actual.push_back(clientes[i]);
            ramificar(clientes[i], costoSiguiente);
            actual.pop_back();
            visitado[i] = 0;
        }
    }
};

} // namespace

ResultadoExacto SolverExacto::resolver(const std::vector<std::vector<double>>& distancias, int base,
                                       const std::vector<int>& clientes) {
    if (static_cast<int>(clientes.size()) > MAX_CLIENTES) {
        throw std::invalid_argument("el solver exacto admite hasta " + std::to_string(MAX_CLIENTES) +
                                    " clientes y se pidieron " + std::to_string(clientes.size()));
    }
    Busqueda busqueda(distancias, base, clientes);
    busqueda.ramificar(base, 0.0);
    return busqueda.mejor;
}

} // namespace geodrone
