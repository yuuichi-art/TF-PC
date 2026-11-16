#include "SolverHeuristico.hpp"
#include "../grafo/ArbolMinimo.hpp"
#include "../escenario/Aleatorio.hpp"

#include <algorithm>
#include <random>

namespace geodrone {
namespace SolverHeuristico {

namespace {

// Mejora mínima para aplicar un intercambio: evita ciclos infinitos por errores de redondeo.
const double MEJORA_MINIMA = 1e-7;

// Preorden del árbol: se visita un nodo y luego sus hijos, del más cercano al más lejano.
void preorden(int nodo, const std::vector<std::vector<int>>& hijos, const std::vector<int>& nodos,
              std::vector<int>& orden) {
    orden.push_back(nodos[nodo]);
    for (int hijo : hijos[nodo]) {
        preorden(hijo, hijos, nodos, orden);
    }
}

// Double-bridge: corta en tres puntos al azar (A | B | C | D) y arma A C B D.
std::vector<int> doblePuente(const std::vector<int>& orden, std::mt19937_64& rng) {
    int n = static_cast<int>(orden.size());
    int p1 = static_cast<int>(enteroAleatorio(rng, 1, n - 3));
    int p2 = static_cast<int>(enteroAleatorio(rng, p1 + 1, n - 2));
    int p3 = static_cast<int>(enteroAleatorio(rng, p2 + 1, n - 1));
    std::vector<int> resultado(orden.begin(), orden.begin() + p1);                  // A
    resultado.insert(resultado.end(), orden.begin() + p2, orden.begin() + p3);         // C
    resultado.insert(resultado.end(), orden.begin() + p1, orden.begin() + p2);         // B
    resultado.insert(resultado.end(), orden.begin() + p3, orden.end());                // D
    return resultado;
}

} // namespace

double costoCiclo(const std::vector<std::vector<double>>& distancias, int base, const std::vector<int>& orden) {
    double costo = 0.0;
    int posicion = base;
    for (int cliente : orden) {
        costo += distancias[posicion][cliente];
        posicion = cliente;
    }
    return costo + distancias[posicion][base];
}

std::vector<int> vecinoMasCercano(const std::vector<std::vector<double>>& distancias, int base,
                                 const std::vector<int>& clientes) {
    std::vector<int> orden;
    std::vector<char> visitado(clientes.size(), 0);
    int posicion = base;
    for (size_t paso = 0; paso < clientes.size(); ++paso) {
        int siguiente = -1;
        for (size_t i = 0; i < clientes.size(); ++i) {
            if (visitado[i]) continue;
            if (siguiente == -1 || distancias[posicion][clientes[i]] < distancias[posicion][clientes[siguiente]]) {
                siguiente = static_cast<int>(i);
            }
        }
        visitado[siguiente] = 1;
        posicion = clientes[siguiente];
        orden.push_back(posicion);
    }
    return orden;
}

std::vector<int> dosAproximacion(const std::vector<std::vector<double>>& distancias, int base,
                                  const std::vector<int>& clientes) {
    std::vector<int> nodos = {base};
    nodos.insert(nodos.end(), clientes.begin(), clientes.end());
    ArbolMinimo arbol = arbolMinimoPrim(distancias, nodos);

    // Hijos de cada nodo del árbol (posiciones dentro de nodos), del más cercano al más lejano.
    // Un nodo sin padre solo aparece si no tiene camino (no debería pasar: los clientes deben ser
    // alcanzables); se cuelga de la raíz para no acceder fuera del vector.
    std::vector<std::vector<int>> hijos(nodos.size());
    for (size_t i = 1; i < nodos.size(); ++i) {
        int padre = arbol.padre[i] >= 0 ? arbol.padre[i] : 0;
        hijos[padre].push_back(static_cast<int>(i));
    }
    for (size_t p = 0; p < nodos.size(); ++p) {
        std::sort(hijos[p].begin(), hijos[p].end(), [&](int a, int b) {
            return distancias[nodos[p]][nodos[a]] < distancias[nodos[p]][nodos[b]];
        });
    }

    std::vector<int> orden;
    preorden(0, hijos, nodos, orden);
    orden.erase(orden.begin());   // El preorden empieza en la base; el orden lleva solo clientes
    return orden;
}

bool aplicar2Opt(const std::vector<std::vector<double>>& distancias, int base, std::vector<int>& orden,
            Reloj::time_point limite) {
    // Ruta completa con la base en ambos extremos: ciclo[0] = ciclo[n + 1] = base.
    std::vector<int> ciclo = {base};
    ciclo.insert(ciclo.end(), orden.begin(), orden.end());
    ciclo.push_back(base);
    int n = static_cast<int>(orden.size());

    // Se repiten pasadas sobre todos los pares (i, j) hasta que una pasada no mejore nada.
    // Cada mejora se aplica en el momento y la pasada sigue con la ruta ya cambiada.
    // terminado pasa a false si se acaba el tiempo: la ruta queda como esté en ese momento.
    bool mejoro = true;
    bool terminado = true;
    while (mejoro && terminado) {
        mejoro = false;
        for (int i = 1; i < n && terminado; ++i) {
            if (Reloj::now() > limite) {
                terminado = false;
                break;
            }
            for (int j = i + 1; j <= n; ++j) {
                // Quitar (a,b) y (c,d); poner (a,c) y (b,d) invirtiendo el tramo b..c.
                int a = ciclo[i - 1], b = ciclo[i], c = ciclo[j], d = ciclo[j + 1];
                double delta = distancias[a][c] + distancias[b][d] - distancias[a][b] - distancias[c][d];
                if (delta < -MEJORA_MINIMA) {
                    std::reverse(ciclo.begin() + i, ciclo.begin() + j + 1);
                    mejoro = true;
                }
            }
        }
    }

    orden.assign(ciclo.begin() + 1, ciclo.end() - 1);
    return terminado;
}

ResultadoBusquedaIterada busquedaLocalIterada(const std::vector<std::vector<double>>& distancias, int base,
                              const std::vector<int>& ordenInicial, uint64_t semilla,
                              int maxIteraciones, Reloj::time_point limite) {
    ResultadoBusquedaIterada resultado;
    resultado.orden = ordenInicial;
    resultado.cortadaPorTiempo = !aplicar2Opt(distancias, base, resultado.orden, limite);
    double mejorCosto = costoCiclo(distancias, base, resultado.orden);

    // Double-bridge necesita al menos 4 clientes para cortar la ruta en cuatro tramos.
    if (resultado.orden.size() < 4) return resultado;

    std::mt19937_64 rng(semilla);
    while (resultado.iteraciones < maxIteraciones && !resultado.cortadaPorTiempo) {
        if (Reloj::now() > limite) {
            resultado.cortadaPorTiempo = true;
            break;
        }
        ++resultado.iteraciones;
        std::vector<int> candidato = doblePuente(resultado.orden, rng);
        if (!aplicar2Opt(distancias, base, candidato, limite)) {
            resultado.cortadaPorTiempo = true;   // La candidata quedó a medias: no se considera
            break;
        }
        double costo = costoCiclo(distancias, base, candidato);
        if (costo < mejorCosto - MEJORA_MINIMA) {
            resultado.orden = std::move(candidato);
            mejorCosto = costo;
        }
    }
    return resultado;
}

} // namespace SolverHeuristico
} // namespace geodrone
