#ifndef GEODRONE_GEOMETRIA_HPP
#define GEODRONE_GEOMETRIA_HPP

#include "Punto.hpp"
#include "Segmento.hpp"
#include "Poligono.hpp"
#include <vector>

namespace geodrone {

/**
 * @brief Ubicación de un punto respecto a un polígono.
 */
enum class UbicacionPunto {
    FUERA = 0,   // Estrictamente exterior
    BORDE = 1,   // Sobre el borde o vértice
    DENTRO = 2   // Estrictamente interior
};

namespace Geometria {

    /**
     * @brief Calcula la orientación relativa 2D (CCW) de la terna (A, B, C).
     * Retorna +1 (antihorario / izquierda), -1 (horario / derecha) o 0 (colineal).
     */
    int orientacion(const Punto& a, const Punto& b, const Punto& c);

    /**
     * @brief Verifica si el punto P está sobre el segmento cerrado AB: colineal con A y B, y dentro
     * del rectángulo que forman (con tolerancia EPS).
     */
    bool estaEnSegmento(const Punto& p, const Punto& a, const Punto& b);
    bool estaEnSegmento(const Punto& p, const Segmento& s);

    /**
     * @brief Determina si dos segmentos s1 y s2 se intersecan en el plano.
     * @param soloPropia Si es true, solo reporta cruce estricto en el interior de ambos segmentos (sin tocar extremos).
     */
    bool segmentosSeIntersecan(const Segmento& s1, const Segmento& s2, bool soloPropia = false);

    /**
     * @brief Determina si dos segmentos tienen una intersección propia (en cruz, sin extremos).
     */
    bool interseccionPropia(const Segmento& s1, const Segmento& s2);

    /**
     * @brief Determina si un punto está dentro, en el borde o fuera de un polígono mediante Ray Casting
     * (teorema de la curva de Jordan).
     * Complejidad: O(V) donde V es el número de vértices del polígono.
     */
    UbicacionPunto ubicacionEnPoligono(const Punto& punto, const Poligono& poligono);

    /**
     * @brief true si el punto está estrictamente en el interior del polígono (no en el borde).
     */
    bool estaEstrictamenteDentro(const Punto& punto, const Poligono& poligono);

    /**
     * @brief Valida si una trayectoria entre u y v interseca o penetra el interior de un polígono de exclusión.
     * Requisito RF04: segmento(u, v) ∩ interior(poligono) = ∅.
     * Complejidad, con n vértices del polígono: O(n) en el caso común (el tramo no pasa por
     * vértices); O(n^2) si pasa por vértices, porque se revisa un punto medio por cada tramo
     * entre ellos y cada revisión es O(n). Espacio O(1) en el caso común, O(n) en el otro.
     */
    bool segmentoCruzaInterior(const Segmento& segmento, const Poligono& poligono);

    /**
     * @brief Valida si una trayectoria entre u y v es factible frente a una colección de obstáculos poligonales.
     * Complejidad: O(M) en total, con M el número de aristas de todos los obstáculos (caso común).
     */
    bool tramoLibre(const Segmento& segmento, const std::vector<Poligono>& obstaculos);

} // namespace Geometria

} // namespace geodrone

#endif // GEODRONE_GEOMETRIA_HPP
