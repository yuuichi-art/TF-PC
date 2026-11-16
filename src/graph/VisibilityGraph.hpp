#ifndef GEODRONE_VISIBILITYGRAPH_HPP
#define GEODRONE_VISIBILITYGRAPH_HPP

#include "../geometry/Point.hpp"
#include "../geometry/Segment.hpp"
#include "../geometry/Polygon.hpp"
#include "../geometry/GeometryUtils.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace geodrone {

/**
 * @brief Tipo de nodo del grafo. Base, clientes y estaciones son los nodos clave.
 */
enum class NodeType {
    DEPOT,              // Base logística (D)
    CLIENT,             // Cliente con entrega obligatoria (C)
    CHARGING_STATION,   // Estación de recarga (S)
    OBSTACLE_VERTEX     // Vértice de un obstáculo, para rodearlo
};

/**
 * @brief Nodo del grafo de visibilidad G = (V, E).
 */
struct Node {
    Point pt;
    NodeType type;
    std::string name;   // "D0", "S1", "C3"... o "Obs2_V0" para un vértice de obstáculo

    bool isKeyNode() const {
        return type != NodeType::OBSTACLE_VERTEX;
    }
};

/**
 * @brief Arista ponderada en el grafo de visibilidad: vecino y distancia.
 */
struct GraphEdge {
    int to;
    double weight;
};

/**
 * @brief Caja envolvente alineada a los ejes: el rectángulo mínimo que contiene una figura.
 */
struct BoundingBox {
    double minX, minY, maxX, maxY;

    // Dos cajas son disjuntas si una queda completamente a un lado de la otra.
    bool isDisjointFrom(const BoundingBox& o) const {
        return maxX < o.minX - EPS || o.maxX < minX - EPS ||
               maxY < o.minY - EPS || o.maxY < minY - EPS;
    }
};

/**
 * @brief Grafo de visibilidad euclidiano G = (V, E) libre de colisiones.
 * Transforma el espacio continuo 2D en un grafo ponderado discreto conectando nodos
 * visibles cuyos trayectos no intersecan el interior de ningún polígono obstáculo.
 */
class VisibilityGraph {
private:
    std::vector<Node> nodes_;
    std::vector<Polygon> obstacles_;
    std::vector<BoundingBox> obstacleBoxes_;   // Caja envolvente de cada obstáculo
    std::vector<std::vector<GraphEdge>> adj_;
    int edgeCount_ = 0;

    // Un tramo es válido si no atraviesa el interior de ningún obstáculo. Con el filtro activo,
    // los obstáculos cuya caja no toca la caja del tramo se descartan sin revisar sus aristas:
    // si las cajas no se tocan, el tramo no puede entrar al polígono.
    bool isSegmentClear(const Segment& seg, bool useBoundingBoxFilter) const {
        BoundingBox segBox{std::min(seg.a.x, seg.b.x), std::min(seg.a.y, seg.b.y),
                           std::max(seg.a.x, seg.b.x), std::max(seg.a.y, seg.b.y)};
        for (size_t k = 0; k < obstacles_.size(); ++k) {
            if (useBoundingBoxFilter && segBox.isDisjointFrom(obstacleBoxes_[k])) continue;
            if (GeometryUtils::segmentIntersectsPolygonInterior(seg, obstacles_[k])) return false;
        }
        return true;
    }

    static BoundingBox boxOf(const Polygon& poly) {
        BoundingBox box{poly[0].x, poly[0].y, poly[0].x, poly[0].y};
        for (const Point& v : poly.vertices) {
            box.minX = std::min(box.minX, v.x);
            box.minY = std::min(box.minY, v.y);
            box.maxX = std::max(box.maxX, v.x);
            box.maxY = std::max(box.maxY, v.y);
        }
        return box;
    }

public:
    /**
     * @brief Agrega un nodo (base, cliente o estación) y devuelve su índice en el grafo.
     */
    int addNode(const Point& pt, NodeType type, const std::string& name) {
        nodes_.push_back({pt, type, name});
        adj_.emplace_back();
        return static_cast<int>(nodes_.size()) - 1;
    }

    /**
     * @brief Agrega un polígono obstáculo e incorpora sus vértices al conjunto de nodos V.
     */
    void addObstacle(const Polygon& poly) {
        obstacles_.push_back(poly);
        obstacleBoxes_.push_back(boxOf(poly));
        for (int i = 0; i < poly.size(); ++i) {
            addNode(poly[i], NodeType::OBSTACLE_VERTEX, "Obs" + std::to_string(poly.id) + "_V" + std::to_string(i));
        }
    }

    /**
     * @brief Construye las aristas de visibilidad evaluando todos los pares de nodos (u, v).
     * Complejidad: O(|V|^2 * M) en el peor caso, donde M es el número total de aristas de
     * obstáculos. El filtro por caja envolvente no cambia el peor caso, pero en la práctica
     * descarta casi todos los obstáculos lejanos con 4 comparaciones cada uno.
     * Espacio: O(|V| + |E|).
     * @param useBoundingBoxFilter Se desactiva solo en las pruebas, para comprobar que el
     *        filtro produce exactamente las mismas aristas.
     */
    void buildVisibilityEdges(bool useBoundingBoxFilter = true) {
        int n = static_cast<int>(nodes_.size());
        edgeCount_ = 0;
        for (int i = 0; i < n; ++i) {
            adj_[i].clear();
        }

        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                Segment seg(nodes_[i].pt, nodes_[j].pt);
                if (isSegmentClear(seg, useBoundingBoxFilter)) {
                    double dist = nodes_[i].pt.dist(nodes_[j].pt);
                    adj_[i].push_back({j, dist});
                    adj_[j].push_back({i, dist});
                    edgeCount_++;
                }
            }
        }
    }

    int numVertices() const { return static_cast<int>(nodes_.size()); }
    int numEdges() const { return edgeCount_; }
    const Node& getNode(int index) const { return nodes_[index]; }
    const std::vector<std::vector<GraphEdge>>& getAdj() const { return adj_; }
    const std::vector<GraphEdge>& getNeighbors(int u) const { return adj_[u]; }
};

} // namespace geodrone

#endif // GEODRONE_VISIBILITYGRAPH_HPP
