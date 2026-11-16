#ifndef GEODRONE_ENVOLVENTECONVEXA_HPP
#define GEODRONE_ENVOLVENTECONVEXA_HPP

#include "Punto.hpp"

#include <algorithm>
#include <vector>

namespace geodrone {

/**
 * @brief Envolvente convexa con el algoritmo monotone chain (Andrew).
 * Ordena los puntos por (x, y) y arma la cadena inferior y la superior, descartando
 * todo giro que no sea a la izquierda.
 *
 * Uso: el generador crea cada obstáculo como la envolvente de puntos aleatorios,
 * lo que garantiza un polígono convexo y simple.
 * Complejidad: O(n log n) en tiempo por el ordenamiento, O(n) en espacio.
 *
 * @return Vértices de la envolvente en sentido antihorario, sin repetir el primero y sin
 *         puntos colineales. Si hay menos de 3 puntos distintos no colineales, devuelve
 *         menos de 3 vértices.
 */
inline std::vector<Punto> envolventeConvexa(std::vector<Punto> puntos) {
    std::sort(puntos.begin(), puntos.end());
    puntos.erase(std::unique(puntos.begin(), puntos.end()), puntos.end());
    int n = static_cast<int>(puntos.size());
    if (n < 3) return puntos;

    std::vector<Punto> envolvente(2 * n);
    int k = 0;

    // Cadena inferior: de izquierda a derecha.
    for (int i = 0; i < n; ++i) {
        while (k >= 2 &&
               (envolvente[k - 1] - envolvente[k - 2]).cruz(puntos[i] - envolvente[k - 2]) <= EPS) {
            --k;
        }
        envolvente[k++] = puntos[i];
    }

    // Cadena superior: de derecha a izquierda. tamanoInferior evita borrar la cadena inferior.
    int tamanoInferior = k + 1;
    for (int i = n - 2; i >= 0; --i) {
        while (k >= tamanoInferior &&
               (envolvente[k - 1] - envolvente[k - 2]).cruz(puntos[i] - envolvente[k - 2]) <= EPS) {
            --k;
        }
        envolvente[k++] = puntos[i];
    }

    envolvente.resize(k - 1); // El último punto repite el primero
    return envolvente;
}

} // namespace geodrone

#endif // GEODRONE_ENVOLVENTECONVEXA_HPP
