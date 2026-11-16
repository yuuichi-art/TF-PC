#include "PlanificadorRecargas.hpp"

#include <algorithm>

namespace geodrone {

namespace {
const double TOL = RedRecarga::TOLERANCIA_BATERIA;
}

PlanificadorRecargas::PlanificadorRecargas(const GrafoMision& mision, const RedRecarga& red)
    : mision_(mision), red_(red) {}

std::vector<double> PlanificadorRecargas::distanciasEnRed(int inicio, double bateria, std::vector<int>& anterior) const {
    const std::vector<int>& puntos = red_.puntosAlcanzables();
    int r = static_cast<int>(puntos.size());
    std::vector<double> mejor(r, INF);
    std::vector<char> listo(r, 0);
    anterior.assign(r, -1);

    // Orígenes: los puntos alcanzables con la batería actual, con su distancia como costo inicial.
    for (int i = 0; i < r; ++i) {
        double d = mision_.distancia(inicio, puntos[i]);
        if (d <= bateria + TOL) mejor[i] = d;
    }
    // Dijkstra sin cola de prioridad: con pocos nodos (R ≤ 26) basta buscar el mínimo en cada paso.
    for (int paso = 0; paso < r; ++paso) {
        int u = -1;
        for (int i = 0; i < r; ++i) {
            if (!listo[i] && mejor[i] < INF && (u == -1 || mejor[i] < mejor[u])) u = i;
        }
        if (u == -1) break;
        listo[u] = 1;
        for (int v = 0; v < r; ++v) {
            if (listo[v] || !red_.conectados(puntos[u], puntos[v])) continue;
            double candidato = mejor[u] + mision_.distancia(puntos[u], puntos[v]);
            if (candidato < mejor[v]) {
                mejor[v] = candidato;
                anterior[v] = u;
            }
        }
    }
    return mejor;
}

int PlanificadorRecargas::elegirUltimaRecarga(const std::vector<double>& mejor, int destino) const {
    const std::vector<int>& puntos = red_.puntosAlcanzables();
    int elegido = -1;
    double mejorTotal = INF;
    for (int i = 0; i < static_cast<int>(puntos.size()); ++i) {
        if (mejor[i] == INF) continue;
        double tramo = mision_.distancia(puntos[i], destino);
        bool seguro = red_.bateria() - tramo + TOL >= red_.distanciaARecarga(destino);
        if (seguro && mejor[i] + tramo < mejorTotal) {
            mejorTotal = mejor[i] + tramo;
            elegido = i;
        }
    }
    return elegido;
}

bool PlanificadorRecargas::buscarCaminoRecarga(int inicio, double bateria, int destino, std::vector<int>& camino) const {
    std::vector<int> anterior;
    std::vector<double> mejor = distanciasEnRed(inicio, bateria, anterior);
    int ultimo = elegirUltimaRecarga(mejor, destino);
    if (ultimo == -1) return false;

    // Paso 3: reconstruir el camino de recargas siguiendo los predecesores.
    const std::vector<int>& puntos = red_.puntosAlcanzables();
    camino.clear();
    for (int i = ultimo; i != -1; i = anterior[i]) camino.push_back(puntos[i]);
    std::reverse(camino.begin(), camino.end());
    // `inicio` o `destino` pueden ser puntos de recarga: no se repiten como paradas.
    if (!camino.empty() && camino.front() == inicio) camino.erase(camino.begin());
    if (!camino.empty() && camino.back() == destino) camino.pop_back();
    return true;
}

PlanRecargas PlanificadorRecargas::planificar(const std::vector<int>& ordenClientes) const {
    PlanRecargas resultado;
    const double llena = red_.bateria();
    double bateria = llena;
    int posicion = mision_.claveBase();
    resultado.secuenciaClaves.push_back(posicion);

    // Sin clientes el dron no despega: la ruta es solo la base.
    if (ordenClientes.empty()) {
        resultado.factible = true;
        return resultado;
    }

    auto volarA = [&](int clave) {
        double d = mision_.distancia(posicion, clave);
        resultado.costo += d;
        bateria -= d;
        posicion = clave;
        resultado.secuenciaClaves.push_back(clave);
    };

    std::vector<int> destinos = ordenClientes;
    destinos.push_back(mision_.claveBase());   // Al final se vuelve a la base

    for (int destino : destinos) {
        // Directo es seguro si, al llegar, queda batería para alcanzar la recarga más cercana del
        // destino: así nunca se llega a un punto del que no se pueda salir.
        bool directoSeguro = bateria - mision_.distancia(posicion, destino) + TOL >= red_.distanciaARecarga(destino);
        if (!directoSeguro) {
            std::vector<int> recargas;
            if (!buscarCaminoRecarga(posicion, bateria, destino, recargas)) {
                return resultado;   // factible = false; la secuencia queda hasta donde se llegó
            }
            for (int parada : recargas) {
                volarA(parada);
                bateria = llena;
                ++resultado.recargas;
            }
        }
        volarA(destino);
    }
    resultado.factible = true;
    return resultado;
}

} // namespace geodrone
