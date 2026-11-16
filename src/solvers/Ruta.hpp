#ifndef GEODRONE_RUTA_HPP
#define GEODRONE_RUTA_HPP

#include "../geometria/Punto.hpp"

#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Punto de la trayectoria volada. La etiqueta indica qué es:
 * "D0" base, "S2" estación 2, "C5" cliente 5, "V" vértice de obstáculo.
 */
struct RoutePoint {
    Punto point;
    std::string etiqueta;
};

/**
 * @brief Cliente que quedó fuera de la misión (docs/decisiones.md §1) y por qué.
 */
struct ExcludedClient {
    std::string etiqueta;      // "C3"
    std::string reason;     // "OBSTACLES" (los obstáculos lo separan de la base) o "BATTERY"
};

/**
 * @brief Ruta final de una misión: lo que se exporta (docs/decisiones.md §6) y lo que verifica
 * el juez. Es autosuficiente: no depende de las estructuras internas del solver.
 */
struct Route {
    std::string escenario;
    std::string algorithm;
    bool feasible = false;
    double costo = 0.0;                      // Distancia total volada, con los desvíos a recargar
    int recharges = 0;
    std::vector<ExcludedClient> excluded;
    std::vector<std::string> order;         // Nodos clave visitados: D0 C2 S1 C1 ... D0
    std::vector<RoutePoint> path;           // Trayectoria completa, con los vértices de obstáculos
    double preparationMs = 0.0;             // Grafo de misión y red de recarga
    double solveMs = 0.0;                   // Orden de visita, recargas y trayectoria

    double totalMs() const { return preparationMs + solveMs; }
};

} // namespace geodrone

#endif // GEODRONE_RUTA_HPP
