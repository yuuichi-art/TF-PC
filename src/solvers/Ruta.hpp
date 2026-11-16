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
struct PuntoRuta {
    Punto punto;
    std::string etiqueta;
};

/**
 * @brief Cliente que quedó fuera de la misión (docs/decisiones.md §1) y por qué.
 */
struct ClienteExcluido {
    std::string etiqueta;      // "C3"
    std::string motivo;     // "OBSTACLES" (los obstáculos lo separan de la base) o "BATTERY"
};

/**
 * @brief Ruta final de una misión: lo que se exporta (docs/decisiones.md §6) y lo que verifica
 * el juez. Es autosuficiente: no depende de las estructuras internas del solver.
 */
struct Ruta {
    std::string escenario;
    std::string algoritmo;
    bool factible = false;
    double costo = 0.0;                      // Distancia total volada, con los desvíos a recargar
    int recargas = 0;
    std::vector<ClienteExcluido> excluidos;
    std::vector<std::string> orden;         // Nodos clave visitados: D0 C2 S1 C1 ... D0
    std::vector<PuntoRuta> trayectoria;           // Trayectoria completa, con los vértices de obstáculos
    double preparacionMs = 0.0;             // Grafo de misión y red de recarga
    double resolucionMs = 0.0;                   // Orden de visita, recargas y trayectoria

    double totalMs() const { return preparacionMs + resolucionMs; }
};

} // namespace geodrone

#endif // GEODRONE_RUTA_HPP
