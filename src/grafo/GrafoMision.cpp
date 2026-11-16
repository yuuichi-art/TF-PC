#include "GrafoMision.hpp"
#include "Dijkstra.hpp"

#include <algorithm>
#include <string>

namespace geodrone {

GrafoMision::GrafoMision(const Escenario& escenario)
    : numEstaciones_(escenario.numEstaciones()), numClientes_(escenario.numClientes()) {
    // Primero los nodos clave, en el orden base, estaciones, clientes; después los obstáculos.
    grafo_.agregarNodo(escenario.base, TipoNodo::BASE, "D0");
    for (int i = 0; i < numEstaciones_; ++i) {
        grafo_.agregarNodo(escenario.estaciones[i], TipoNodo::ESTACION, "S" + std::to_string(i + 1));
    }
    for (int i = 0; i < numClientes_; ++i) {
        grafo_.agregarNodo(escenario.clientes[i], TipoNodo::CLIENTE, "C" + std::to_string(i + 1));
    }
    for (const Poligono& poligono : escenario.obstaculos) {
        grafo_.agregarObstaculo(poligono);
    }
    grafo_.construirAristas();

    // Un Dijkstra por nodo clave: da una fila de la matriz y los predecesores para
    // reconstruir los caminos reales desde ese nodo.
    int numClaves = numNodosClave();
    distancias_.assign(numClaves, std::vector<double>(numClaves, INF));
    padres_.resize(numClaves);
    for (int a = 0; a < numClaves; ++a) {
        ResultadoDijkstra resultado = Dijkstra::ejecutar(grafo_, a, true);
        for (int b = 0; b < numClaves; ++b) {
            distancias_[a][b] = resultado.distancia[b];
        }
        padres_[a] = std::move(resultado.padre);
    }

    conectados_.assign(grafo_.numNodos(), 0);
    marcarConectados(claveBase());
}

// DFS recursivo: marca todo lo alcanzable desde nodo. Cada nodo se visita una vez y cada
// arista se revisa dos veces: O(V + E). La profundidad máxima es V (unos pocos miles).
void GrafoMision::marcarConectados(int nodo) {
    conectados_[nodo] = 1;
    for (const Arista& arista : grafo_.vecinos(nodo)) {
        if (!conectados_[arista.destino]) {
            marcarConectados(arista.destino);
        }
    }
}

std::vector<int> GrafoMision::clientesInalcanzables() const {
    std::vector<int> clientes;
    for (int i = 0; i < numClientes_; ++i) {
        if (!conectadoABase(claveCliente(i))) {
            clientes.push_back(i);
        }
    }
    return clientes;
}

std::vector<int> GrafoMision::nodosCamino(int claveOrigen, int claveDestino) const {
    std::vector<int> camino;
    if (distancias_[claveOrigen][claveDestino] == INF) return camino;

    for (int nodo = claveDestino; nodo != -1; nodo = padres_[claveOrigen][nodo]) {
        camino.push_back(nodo);
        if (nodo == claveOrigen) break;
    }
    std::reverse(camino.begin(), camino.end());
    return camino;
}

} // namespace geodrone
