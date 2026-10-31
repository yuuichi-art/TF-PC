#include "geometry/Point.hpp"
#include "geometry/Segment.hpp"
#include "geometry/Polygon.hpp"
#include "geometry/GeometryUtils.hpp"
#include "graph/VisibilityGraph.hpp"
#include "graph/Dijkstra.hpp"
#include "solvers/ExactSolver.hpp"
#include "solvers/HeuristicSolver.hpp"
#include "judge/DatasetGenerator.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

using namespace geodrone;

void printBanner() {
    std::cout << "========================================================================\n";
    std::cout << "       UNIVERSIDAD SAN IGNACIO DE LOYOLA - FACULTAD DE INGENIERIA       \n";
    std::cout << "             PROGRAMACION COMPETITIVA (2026) - GRUPO 4                 \n";
    std::cout << "                       GEODRONE CHALLENGE                               \n";
    std::cout << "   Optimizacion de Rutas de Entrega con Drones en Entornos Urbanos      \n";
    std::cout << "========================================================================\n";
    std::cout << " Integrantes:\n";
    std::cout << "   - Caso Egaovil, Pedro Yuichi        (2411176)\n";
    std::cout << "   - Fernandez Totocayo, Henry Daniel  (2412247)\n";
    std::cout << "   - Patino Quispe, Renato Enrique     (2410049)\n";
    std::cout << "   - Chanca Melgar, Yarel Joshua       (2411997)\n";
    std::cout << " Docente: Jomark Pablo Noriega Zapata\n";
    std::cout << "========================================================================\n\n";
}

void printSystemStatus() {
    std::cout << "[INFO] Estado de Inicializacion del Sistema:\n";
    std::cout << "  [OK] Estandar C++: C++17/C++20 compilado con optimizacion (-O3)\n";
    std::cout << "  [OK] Fast I/O: std::ios_base::sync_with_stdio(false); cin.tie(NULL)\n";
    std::cout << "  [OK] Tolerancia Geometrica: EPS = " << EPS << "\n";
    std::cout << "  [OK] Modulo Geometrico: Point, Segment, Polygon, CCW, Ray Casting\n";
    std::cout << "  [OK] Modulo de Grafos: VisibilityGraph G=(V,E), Dijkstra SSSP\n";
    std::cout << "  [OK] Modulo de Solvers: Exact (Branch & Bound), Heuristic (NN + 2-Opt)\n";
    std::cout << "  [OK] Juez y Benchmarking: Generador de datasets reproducible (mt19937_64)\n\n";
}

void runDemonstration() {
    std::cout << "--- DEMOSTRACION EN VIVO: GRAFO DE VISIBILIDAD CON OBSTACULO ---\n";
    VisibilityGraph g;

    // 1. Base logística (Depósito)
    int depotIdx = g.setDepot(Point(0.0, 5.0), "Base Central D");
    std::cout << "Base registrada: " << g.getNode(depotIdx).name << " en (0.0, 5.0)\n";

    // 2. Clientes
    g.addClient(Point(10.0, 5.0), 1.0, "Cliente C1");
    g.addClient(Point(10.0, 10.0), 1.0, "Cliente C2");
    std::cout << "Clientes registrados: C1 en (10.0, 5.0), C2 en (10.0, 10.0)\n";

    // 3. Estación de recarga
    g.addChargingStation(Point(5.0, 8.0), "Estacion Solar S1");
    std::cout << "Estacion registrada: S1 en (5.0, 8.0)\n";

    // 4. Obstáculo poligonal intermedio (bloquea la visión directa entre Base y C1)
    // Edificio cuadrado de [3.0, 3.0] a [7.0, 7.0]
    Polygon building(1, "Edificio_Central");
    building.addVertex(Point(3.0, 3.0));
    building.addVertex(Point(7.0, 3.0));
    building.addVertex(Point(7.0, 7.0));
    building.addVertex(Point(3.0, 7.0));
    g.addObstacle(building);
    std::cout << "Obstaculo registrado: Cuadrado [3,7]x[3,7] (bloquea linea recta Base -> C1)\n";

    // 5. Construcción del Grafo de Visibilidad
    g.buildVisibilityEdges();
    std::cout << "\nGrafo de Visibilidad construido:\n";
    std::cout << "  Total de Vertices |V|: " << g.numVertices() << "\n";
    std::cout << "  Total de Aristas  |E|: " << g.numEdges() << "\n";

    // 6. Ejecución de Dijkstra desde el Depósito
    DijkstraResult sssp = Dijkstra::run(g, depotIdx);
    std::cout << "\nCaminos minimos calculados con Dijkstra desde " << g.getNode(depotIdx).name << ":\n";

    std::vector<int> keyNodes = g.getKeyNodeIndices();
    for (int keyIdx : keyNodes) {
        if (keyIdx == depotIdx) continue;
        const auto& node = g.getNode(keyIdx);
        std::cout << "  Destino " << std::setw(18) << std::left << node.name
                  << " | Distancia: " << std::fixed << std::setprecision(3) << sssp.dist[keyIdx]
                  << " | Ruta: ";
        auto path = sssp.getPathTo(keyIdx);
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << g.getNode(path[i]).name;
            if (i + 1 < path.size()) std::cout << " -> ";
        }
        std::cout << "\n";
    }

    // 7. Matriz reducida de distancias
    std::cout << "\nMatriz de Costos Reducida |K|x|K| (Grafo de Mision):\n";
    auto matrix = Dijkstra::buildReducedCostMatrix(g, keyNodes);
    for (size_t i = 0; i < keyNodes.size(); ++i) {
        for (size_t j = 0; j < keyNodes.size(); ++j) {
            std::cout << std::setw(8) << std::fixed << std::setprecision(2) << matrix[i][j] << " ";
        }
        std::cout << "\n";
    }
    std::cout << "----------------------------------------------------------------\n\n";
}

void printDatasetsTable() {
    std::cout << "--- CATEGORIAS DE DATASETS Y LIMITES COMPETITIVOS (TABLA 1) ---\n";
    std::cout << "| Categoria | Clientes (N) | Obstaculos (M) | Vertices Obs | TL (s) | ML (MB) |\n";
    std::cout << "|-----------|--------------|----------------|--------------|--------|---------|\n";
    std::cout << "| Small     |  1 <= N <= 10|    1 <= M <= 5 |    <= 30     | 1.0 s  |  256 MB |\n";
    std::cout << "| Medium    | 11 <= N <= 40|    5 <= M <= 15|    <= 100    | 2.0 s  |  256 MB |\n";
    std::cout << "| Large     | 41 <= N <=200|   15 <= M <= 50|    <= 400    | 3.0 s  |  512 MB |\n";
    std::cout << "| Extreme   |201 <= N<=1000|   50 <= M<= 150|    <= 1500   | 5.0 s  |  512 MB |\n";
    std::cout << "-------------------------------------------------------------------------\n\n";
}

int main() {
    // Configuración de I/O Rápido para entornos de Programación Competitiva
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    printBanner();
    printSystemStatus();
    printDatasetsTable();
    runDemonstration();

    std::cout << "Ejecucion completada con exito. Sistema listo para Hitos 2 y 3.\n";
    return 0;
}
