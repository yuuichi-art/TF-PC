#ifndef GEODRONE_VISIBILITYGRAPH_HPP
#define GEODRONE_VISIBILITYGRAPH_HPP

#include "../geometry/Point.hpp"
#include "../geometry/Segment.hpp"
#include "../geometry/Polygon.hpp"
#include "../geometry/GeometryUtils.hpp"

#include <vector>
#include <string>
#include <iostream>

namespace geodrone {

/**
 * @brief Clasificación de nodos operacionales según la especificación del GeoDrone Challenge.
 */
enum class NodeType {
    DEPOT,              // Base logística o centro de distribución (D)
    CLIENT,             // Clientes con demanda obligatoria de entrega (C)
    CHARGING_STATION,   // Estaciones de recarga rápida de batería (S)
    OBSTACLE_VERTEX     // Vértices de polígonos de exclusión para sortear obstáculos
};

/**
 * @brief Representación de un nodo en el Grafo de Navegación G = (V, E).
 */
struct Node {
    Point pt;
    NodeType type;
    int index;          // Índice global dentro del grafo |V|
    int entityId;       // Identificador original de la entidad (ej. cliente #3)
    double demand;      // Demanda del cliente o capacidad
    std::string name;

    Node() : pt(), type(NodeType::OBSTACLE_VERTEX), index(-1), entityId(-1), demand(0.0), name("") {}
    Node(const Point& pt_, NodeType type_, int index_, int entityId_ = -1, double demand_ = 0.0, const std::string& name_ = "")
        : pt(pt_), type(type_), index(index_), entityId(entityId_), demand(demand_), name(name_) {}

    bool isKeyNode() const {
        return type == NodeType::DEPOT || type == NodeType::CLIENT || type == NodeType::CHARGING_STATION;
    }
};

/**
 * @brief Arista ponderada en el Grafo de Visibilidad.
 */
struct GraphEdge {
    int to;
    double weight;

    GraphEdge() : to(-1), weight(0.0) {}
    GraphEdge(int to_, double weight_) : to(to_), weight(weight_) {}
};

/**
 * @brief Grafo de Visibilidad Euclidiano G = (V, E) libre de colisiones.
 * Transforma el espacio continuo 2D en un grafo ponderado discreto conectando nodos
 * visibles cuyos trayectos no intersecan el interior de ningún polígono obstáculo.
 */
class VisibilityGraph {
private:
    std::vector<Node> nodes_;
    std::vector<Polygon> obstacles_;
    std::vector<std::vector<GraphEdge>> adj_;
    int edgeCount_ = 0;

public:
    VisibilityGraph() = default;

    /**
     * @brief Registra el depósito inicial.
     */
    int setDepot(const Point& depotPt, const std::string& name = "Base Logistica D") {
        int idx = static_cast<int>(nodes_.size());
        nodes_.emplace_back(depotPt, NodeType::DEPOT, idx, 0, 0.0, name);
        adj_.emplace_back();
        return idx;
    }

    /**
     * @brief Registra un cliente con coordenadas y demanda.
     */
    int addClient(const Point& pt, double demand = 1.0, const std::string& name = "") {
        int idx = static_cast<int>(nodes_.size());
        int entityId = 1;
        for (const auto& n : nodes_) {
            if (n.type == NodeType::CLIENT) entityId++;
        }
        nodes_.emplace_back(pt, NodeType::CLIENT, idx, entityId, demand, name);
        adj_.emplace_back();
        return idx;
    }

    /**
     * @brief Registra una estación de recarga rápida de batería.
     */
    int addChargingStation(const Point& pt, const std::string& name = "") {
        int idx = static_cast<int>(nodes_.size());
        int entityId = 1;
        for (const auto& n : nodes_) {
            if (n.type == NodeType::CHARGING_STATION) entityId++;
        }
        nodes_.emplace_back(pt, NodeType::CHARGING_STATION, idx, entityId, 0.0, name);
        adj_.emplace_back();
        return idx;
    }

    /**
     * @brief Agrega un polígono obstáculo e incorpora sus vértices al conjunto de nodos V.
     */
    void addObstacle(const Polygon& poly) {
        obstacles_.push_back(poly);
        int polyIdx = static_cast<int>(obstacles_.size()) - 1;
        for (int i = 0; i < poly.size(); ++i) {
            int idx = static_cast<int>(nodes_.size());
            nodes_.emplace_back(poly[i], NodeType::OBSTACLE_VERTEX, idx, polyIdx, 0.0,
                                "Obs" + std::to_string(poly.id) + "_V" + std::to_string(i));
            adj_.emplace_back();
        }
    }

    /**
     * @brief Construye las aristas de visibilidad evaluando todos los pares de nodos (u, v).
     * Complejidad: O(|V|^2 * M), donde M es el número total de aristas de obstáculos.
     */
    void buildVisibilityEdges() {
        int n = static_cast<int>(nodes_.size());
        edgeCount_ = 0;
        for (int i = 0; i < n; ++i) {
            adj_[i].clear();
        }

        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                Segment seg(nodes_[i].pt, nodes_[j].pt);
                if (GeometryUtils::isFlightPathClear(seg, obstacles_)) {
                    double dist = nodes_[i].pt.dist(nodes_[j].pt);
                    adj_[i].emplace_back(j, dist);
                    adj_[j].emplace_back(i, dist);
                    edgeCount_++;
                }
            }
        }
    }

    // Getters
    int numVertices() const { return static_cast<int>(nodes_.size()); }
    int numEdges() const { return edgeCount_; }
    const std::vector<Node>& getNodes() const { return nodes_; }
    const Node& getNode(int index) const { return nodes_[index]; }
    const std::vector<std::vector<GraphEdge>>& getAdj() const { return adj_; }
    const std::vector<GraphEdge>& getNeighbors(int u) const { return adj_[u]; }
    const std::vector<Polygon>& getObstacles() const { return obstacles_; }

    /**
     * @brief Retorna los índices de todos los nodos clave K = {D} ∪ C ∪ S.
     */
    std::vector<int> getKeyNodeIndices() const {
        std::vector<int> keyIndices;
        for (const auto& n : nodes_) {
            if (n.isKeyNode()) {
                keyIndices.push_back(n.index);
            }
        }
        return keyIndices;
    }
};

} // namespace geodrone

#endif // GEODRONE_VISIBILITYGRAPH_HPP
