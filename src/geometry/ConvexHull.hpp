#ifndef GEODRONE_CONVEXHULL_HPP
#define GEODRONE_CONVEXHULL_HPP

#include "Point.hpp"

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
inline std::vector<Point> convexHull(std::vector<Point> points) {
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());
    int n = static_cast<int>(points.size());
    if (n < 3) return points;

    std::vector<Point> hull(2 * n);
    int k = 0;

    // Cadena inferior: de izquierda a derecha.
    for (int i = 0; i < n; ++i) {
        while (k >= 2 && (hull[k - 1] - hull[k - 2]).cross(points[i] - hull[k - 2]) <= EPS) {
            --k;
        }
        hull[k++] = points[i];
    }

    // Cadena superior: de derecha a izquierda. lowerSize evita borrar la cadena inferior.
    int lowerSize = k + 1;
    for (int i = n - 2; i >= 0; --i) {
        while (k >= lowerSize && (hull[k - 1] - hull[k - 2]).cross(points[i] - hull[k - 2]) <= EPS) {
            --k;
        }
        hull[k++] = points[i];
    }

    hull.resize(k - 1); // El último punto repite el primero
    return hull;
}

} // namespace geodrone

#endif // GEODRONE_CONVEXHULL_HPP
