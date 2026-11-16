#ifndef GEODRONE_PLANIFICADORRECARGAS_HPP
#define GEODRONE_PLANIFICADORRECARGAS_HPP

#include "../grafo/RedRecarga.hpp"
#include "../grafo/GrafoMision.hpp"

#include <vector>

namespace geodrone {

/**
 * @brief Ruta de nodos clave con las recargas ya insertadas.
 */
struct PlanRecargas {
    bool factible = false;
    std::vector<int> secuenciaClaves;   // Empieza y termina en la base (clave 0)
    int recargas = 0;              // Paradas en estaciones o en la base a mitad de ruta
    double costo = 0.0;              // Distancia total volada, incluidos los desvíos
};

/**
 * @brief Inserción de recargas sobre un orden de clientes ya decidido (docs/decisiones.md §3:
 * los solvers deciden el orden y este paso común agrega las recargas).
 *
 * Regla greedy segura (docs/decisiones.md §4): antes de cada tramo se comprueba que, al
 * llegar al destino, quede batería para alcanzar su recarga más cercana (dRecarga).
 * - Si alcanza, se vuela directo.
 * - Si no, se busca el desvío más corto por la red de recarga: un Dijkstra sobre los puntos
 *   de recarga, partiendo de los que se alcanzan con la batería actual, hasta uno desde el
 *   cual el destino sea seguro con la batería llena. Un desvío a una sola estación es el
 *   caso de un salto; si hace falta, el camino encadena varias estaciones.
 * - Si no existe ese camino, la ruta es infactible.
 *
 * Si todos los clientes cumplen 2·dRecarga ≤ B, la ruta siempre es factible: siempre se
 * alcanza la recarga más cercana, desde ahí la red llega a la recarga más cercana del
 * destino, y desde esa el destino es seguro.
 *
 * Complejidad: O(N · R^2) en tiempo y O(R) en espacio extra, con N clientes y R puntos de recarga.
 * La red y el grafo de misión deben seguir existiendo mientras se use el planificador.
 */
class PlanificadorRecargas {
public:
    PlanificadorRecargas(const GrafoMision& mision, const RedRecarga& red);
    // Prohibido con temporales: las referencias quedarían apuntando a objetos destruidos.
    PlanificadorRecargas(GrafoMision&& mision, const RedRecarga& red) = delete;
    PlanificadorRecargas(const GrafoMision& mision, RedRecarga&& red) = delete;

    // ordenClientes: claves de los clientes en el orden de visita (sin la base).
    PlanRecargas planificar(const std::vector<int>& ordenClientes) const;

private:
    const GrafoMision& mision_;
    const RedRecarga& red_;

    // Recargas a visitar desde `inicio` con batería `bateria` para llegar seguro a `destino`.
    // Devuelve false si no hay forma. El camino no incluye a `inicio` ni a `destino`.
    bool buscarCaminoRecarga(int inicio, double bateria, int destino, std::vector<int>& camino) const;

    // Paso 1: Dijkstra O(R^2) sobre la red de recarga. Orígenes: los puntos que se alcanzan
    // desde `inicio` con la batería actual. mejor[i] es la distancia hasta puntosAlcanzables()[i]
    // y anterior[i] el punto anterior en ese camino (-1 si es un origen o no se alcanza).
    std::vector<double> distanciasEnRed(int inicio, double bateria, std::vector<int>& anterior) const;

    // Paso 2: el último punto de recarga, desde el cual `destino` es seguro con batería llena,
    // que minimiza el total. Devuelve su posición en puntosAlcanzables() o -1.
    int elegirUltimaRecarga(const std::vector<double>& mejor, int destino) const;
};

} // namespace geodrone

#endif // GEODRONE_PLANIFICADORRECARGAS_HPP
