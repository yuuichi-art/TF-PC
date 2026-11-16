#include "RedRecarga.hpp"

#include <algorithm>

namespace geodrone {

RedRecarga::RedRecarga(const GrafoMision& mision, double bateria)
    : mision_(mision), bateria_(bateria), numPuntosRecarga_(mision.numEstaciones() + 1) {
    // DFS de Tarjan desde la base: marca los alcanzables (descubrimiento > 0) y las articulaciones.
    std::vector<int> descubrimiento(numPuntosRecarga_, 0);
    std::vector<int> bajo(numPuntosRecarga_, 0);
    articulacion_.assign(numPuntosRecarga_, 0);
    int contador = 0;
    tarjan(mision.claveBase(), -1, contador, descubrimiento, bajo);

    alcanzable_.assign(numPuntosRecarga_, 0);
    for (int r = 0; r < numPuntosRecarga_; ++r) {
        if (descubrimiento[r] > 0) {
            alcanzable_[r] = 1;
            puntosAlcanzables_.push_back(r);  // La base (0) queda primera
        }
    }

    // dRecarga de cada nodo clave: mínimo sobre los puntos de recarga alcanzables.
    distanciaARecarga_.assign(mision.numNodosClave(), INF);
    for (int clave = 0; clave < mision.numNodosClave(); ++clave) {
        for (int r : puntosAlcanzables_) {
            distanciaARecarga_[clave] = std::min(distanciaARecarga_[clave], mision.distancia(clave, r));
        }
    }
}

// Comparación exacta, sin tolerancia: ver TOLERANCIA_BATERIA en el encabezado.
bool RedRecarga::enlazados(int a, int b) const {
    return a != b && mision_.distancia(a, b) <= bateria_;
}

// Tarjan para puntos de articulación. descubrimiento[u] es el orden de visita y bajo[u] el menor
// orden alcanzable desde el subárbol de u usando una arista de retroceso. Un nodo que no es
// raíz es articulación si algún hijo v cumple bajo[v] >= descubrimiento[u]: el subárbol de v no
// tiene otra forma de llegar más arriba que pasando por u. La raíz lo es si tiene 2 o más hijos.
// La red es densa y pequeña, así que los vecinos se recorren por la matriz: O(R^2).
void RedRecarga::tarjan(int u, int padre, int& contador, std::vector<int>& descubrimiento, std::vector<int>& bajo) {
    descubrimiento[u] = bajo[u] = ++contador;
    int hijos = 0;
    for (int v = 0; v < numPuntosRecarga_; ++v) {
        if (!enlazados(u, v)) continue;
        if (descubrimiento[v] == 0) {
            ++hijos;
            tarjan(v, u, contador, descubrimiento, bajo);
            bajo[u] = std::min(bajo[u], bajo[v]);
            if (padre != -1 && bajo[v] >= descubrimiento[u]) articulacion_[u] = 1;
        } else if (v != padre) {
            bajo[u] = std::min(bajo[u], descubrimiento[v]);
        }
    }
    if (padre == -1 && hijos > 1) articulacion_[u] = 1;
}

bool RedRecarga::puedeAtender(int claveCliente) const {
    return 2.0 * distanciaARecarga_[claveCliente] <= bateria_;
}

std::vector<int> RedRecarga::estacionesCriticas() const {
    std::vector<int> estaciones;
    for (int r = 1; r < numPuntosRecarga_; ++r) {     // La base (0) no se considera: no puede fallar
        if (articulacion_[r]) estaciones.push_back(r);
    }
    return estaciones;
}

bool RedRecarga::esFactible(const GrafoMision& mision, double bateria) {
    RedRecarga red(mision, bateria);
    for (int c = 0; c < mision.numClientes(); ++c) {
        int clave = mision.claveCliente(c);
        if (mision.conectadoABase(clave) && !red.puedeAtender(clave)) return false;
    }
    return true;
}

double RedRecarga::autonomiaMinima(const GrafoMision& mision) {
    // Con B = 2 · (mayor distancia finita) todo es factible: cualquier cliente conectado
    // puede ir y volver directo a la base.
    double distanciaMaxima = 0.0;
    for (const auto& fila : mision.distancias()) {
        for (double d : fila) {
            if (d != INF) distanciaMaxima = std::max(distanciaMaxima, d);
        }
    }
    // Invariante: hi siempre es factible y lo no lo es (o es 0). Cada iteración parte el
    // intervalo a la mitad y conserva la mitad donde está el cambio de no factible a factible.
    double lo = 0.0;
    double hi = 2.0 * distanciaMaxima;
    for (int iteracion = 0; iteracion < 60; ++iteracion) {
        double medio = (lo + hi) / 2.0;
        if (esFactible(mision, medio)) {
            hi = medio;
        } else {
            lo = medio;
        }
    }
    return hi;
}

ArbolMinimo RedRecarga::arbolSoporte(const GrafoMision& mision, std::vector<int>& nodos) {
    nodos.clear();
    for (int r = 0; r <= mision.numEstaciones(); ++r) {
        if (mision.conectadoABase(r)) nodos.push_back(r);
    }
    return arbolMinimoPrim(mision.distancias(), nodos);
}

} // namespace geodrone
