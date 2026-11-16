#ifndef GEODRONE_GRAFOVISIBILIDAD_HPP
#define GEODRONE_GRAFOVISIBILIDAD_HPP

#include "../geometria/Punto.hpp"
#include "../geometria/Segmento.hpp"
#include "../geometria/Poligono.hpp"
#include "../geometria/Geometria.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Tipo de nodo del grafo. Base, clientes y estaciones son los nodos clave.
 */
enum class TipoNodo {
    BASE,              // Base logística (D)
    CLIENTE,             // Cliente con entrega obligatoria (C)
    ESTACION,   // Estación de recarga (S)
    VERTICE_OBSTACULO     // Vértice de un obstáculo, para rodearlo
};

/**
 * @brief Nodo del grafo de visibilidad G = (V, E).
 */
struct Nodo {
    Punto punto;
    TipoNodo tipo;
    std::string nombre;   // "D0", "S1", "C3"... o "Obs2_V0" para un vértice de obstáculo

    bool esNodoClave() const {
        return tipo != TipoNodo::VERTICE_OBSTACULO;
    }
};

/**
 * @brief Arista ponderada en el grafo de visibilidad: vecino y distancia.
 */
struct Arista {
    int destino;
    double peso;
};

/**
 * @brief Caja envolvente alineada a los ejes: el rectángulo mínimo que contiene una figura.
 */
struct CajaEnvolvente {
    double minX, minY, maxX, maxY;

    // Dos cajas son disjuntas si una queda completamente a un lado de la otra.
    bool esDisjuntaDe(const CajaEnvolvente& o) const {
        return maxX < o.minX - EPS || o.maxX < minX - EPS ||
               maxY < o.minY - EPS || o.maxY < minY - EPS;
    }
};

/**
 * @brief Grafo de visibilidad euclidiano G = (V, E) libre de colisiones.
 * Transforma el espacio continuo 2D en un grafo ponderado discreto conectando nodos
 * visibles cuyos trayectos no intersecan el interior de ningún polígono obstáculo.
 */
class GrafoVisibilidad {
private:
    std::vector<Nodo> nodos_;
    std::vector<Poligono> obstaculos_;
    std::vector<CajaEnvolvente> cajasObstaculos_;   // Caja envolvente de cada obstáculo
    std::vector<std::vector<Arista>> adyacencia_;
    int numAristas_ = 0;

    // Un tramo es válido si no atraviesa el interior de ningún obstáculo. Con el filtro activo,
    // los obstáculos cuya caja no toca la caja del tramo se descartan sin revisar sus aristas:
    // si las cajas no se tocan, el tramo no puede entrar al polígono.
    bool tramoVisible(const Segmento& segmento, bool usarFiltroCajas) const {
        CajaEnvolvente cajaTramo{std::min(segmento.a.x, segmento.b.x), std::min(segmento.a.y, segmento.b.y),
                           std::max(segmento.a.x, segmento.b.x), std::max(segmento.a.y, segmento.b.y)};
        for (size_t k = 0; k < obstaculos_.size(); ++k) {
            if (usarFiltroCajas && cajaTramo.esDisjuntaDe(cajasObstaculos_[k])) continue;
            if (Geometria::segmentoCruzaInterior(segmento, obstaculos_[k])) return false;
        }
        return true;
    }

    static CajaEnvolvente cajaDe(const Poligono& poligono) {
        CajaEnvolvente caja{poligono[0].x, poligono[0].y, poligono[0].x, poligono[0].y};
        for (const Punto& v : poligono.vertices) {
            caja.minX = std::min(caja.minX, v.x);
            caja.minY = std::min(caja.minY, v.y);
            caja.maxX = std::max(caja.maxX, v.x);
            caja.maxY = std::max(caja.maxY, v.y);
        }
        return caja;
    }

public:
    /**
     * @brief Agrega un nodo (base, cliente o estación) y devuelve su índice en el grafo.
     */
    int agregarNodo(const Punto& punto, TipoNodo tipo, const std::string& nombre) {
        nodos_.push_back({punto, tipo, nombre});
        adyacencia_.emplace_back();
        return static_cast<int>(nodos_.size()) - 1;
    }

    /**
     * @brief Agrega un polígono obstáculo e incorpora sus vértices al conjunto de nodos V.
     */
    void agregarObstaculo(const Poligono& poligono) {
        obstaculos_.push_back(poligono);
        cajasObstaculos_.push_back(cajaDe(poligono));
        for (int i = 0; i < poligono.numVertices(); ++i) {
            agregarNodo(poligono[i], TipoNodo::VERTICE_OBSTACULO, "Obs" + std::to_string(poligono.id) + "_V" + std::to_string(i));
        }
    }

    /**
     * @brief Construye las aristas de visibilidad evaluando todos los pares de nodos (u, v).
     * Complejidad: O(|V|^2 * M) en el peor caso, donde M es el número total de aristas de
     * obstáculos. El filtro por caja envolvente no cambia el peor caso, pero en la práctica
     * descarta casi todos los obstáculos lejanos con 4 comparaciones cada uno.
     * Espacio: O(|V| + |E|).
     * @param usarFiltroCajas Se desactiva solo en las pruebas, para comprobar que el
     *        filtro produce exactamente las mismas aristas.
     */
    void construirAristas(bool usarFiltroCajas = true) {
        int n = static_cast<int>(nodos_.size());
        numAristas_ = 0;
        for (int i = 0; i < n; ++i) {
            adyacencia_[i].clear();
        }

        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                Segmento segmento(nodos_[i].punto, nodos_[j].punto);
                if (tramoVisible(segmento, usarFiltroCajas)) {
                    double distancia = nodos_[i].punto.distancia(nodos_[j].punto);
                    adyacencia_[i].push_back({j, distancia});
                    adyacencia_[j].push_back({i, distancia});
                    numAristas_++;
                }
            }
        }
    }

    int numNodos() const { return static_cast<int>(nodos_.size()); }
    int numAristas() const { return numAristas_; }
    const Nodo& nodo(int indice) const { return nodos_[indice]; }
    const std::vector<std::vector<Arista>>& adyacencia() const { return adyacencia_; }
    const std::vector<Arista>& vecinos(int u) const { return adyacencia_[u]; }
};

} // namespace geodrone

#endif // GEODRONE_GRAFOVISIBILIDAD_HPP
