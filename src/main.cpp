#include "graph/ChargingNetwork.hpp"
#include "graph/MissionGraph.hpp"
#include "judge/Judge.hpp"
#include "judge/MemoryUsage.hpp"
#include "report/Exporter.hpp"
#include "scenario/DatasetGenerator.hpp"
#include "scenario/ScenarioIO.hpp"
#include "solvers/MissionSolver.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace geodrone;

// ---------------------------------------------------------------------------------------------
// Entrada por teclado (decisión del menú, docs/decisiones.md §5)
// ---------------------------------------------------------------------------------------------

// Si la entrada se cierra (por ejemplo, al pasar las respuestas por tubería), se usan los
// valores por defecto y el menú termina en lugar de quedar en un bucle.
bool inputClosed = false;

// La memoria pico del proceso nunca baja: si antes del juez se hizo otro trabajo en esta
// sesión, la memoria de cada caso queda tapada por ese pico anterior.
bool sessionUsedMemory = false;

std::string trim(const std::string& text) {
    size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

// Lee una línea completa (las rutas pueden tener espacios). Enter acepta el valor por defecto.
// Con cin.tie(NULL) la pregunta no se mostraría antes de leer: por eso termina en flush.
std::string askText(const std::string& question, const std::string& defaultValue) {
    std::cout << question << " [" << defaultValue << "]: " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) {
        inputClosed = true;
        std::cout << '\n';
        return defaultValue;
    }
    line = trim(line);
    return line.empty() ? defaultValue : line;
}

// Pide un entero en [min, max] y repite la pregunta hasta recibir uno válido.
int askNumber(const std::string& question, int defaultValue, int min, int max) {
    while (true) {
        std::string text = askText(question, std::to_string(defaultValue));
        try {
            size_t used = 0;
            int value = std::stoi(text, &used);
            if (used == text.size() && value >= min && value <= max) return value;
        } catch (const std::exception&) {
            // Texto que no es un número: se vuelve a preguntar
        }
        if (inputClosed) return defaultValue;
        std::cout << "  Valor invalido: escriba un numero entre " << min << " y " << max << ".\n";
    }
}

// ---------------------------------------------------------------------------------------------
// Presentación
// ---------------------------------------------------------------------------------------------

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
    std::cout << "========================================================================\n";
}

void printScenarioSummary(const Scenario& s) {
    // Formato normal de números (otras pantallas dejan activo std::fixed): la batería es entera.
    std::cout << std::defaultfloat << std::setprecision(6);
    std::cout << "  Escenario " << s.name << " (" << categoryName(s.category) << ", semilla " << s.seed << "): "
              << "clientes " << s.numClients() << ", estaciones " << s.numStations() << ", obstaculos "
              << s.numObstacles() << " (" << s.totalObstacleVertices() << " vertices), bateria " << s.battery << "\n";
}

// Lista de etiquetas; si es muy larga muestra el inicio y el final.
void printLabels(const std::vector<std::string>& labels) {
    const size_t head = 15, tail = 5;
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels.size() > head + tail && i == head) {
            std::cout << " ... (" << labels.size() - head - tail << " mas) ...";
            i = labels.size() - tail;
        }
        std::cout << ' ' << labels[i];
    }
    std::cout << '\n';
}

std::string labelOf(const MissionGraph& mission, int key) {
    return mission.graph().getNode(key).name;
}

// ---------------------------------------------------------------------------------------------
// Opción 1: generar datasets
// ---------------------------------------------------------------------------------------------

void generateDatasets() {
    std::cout << "\n--- GENERAR DATASETS ---\n";
    std::cout << "  Categoria  Clientes   Estaciones  Obstaculos  Tiempo  Memoria\n";
    for (DatasetCategory c : {DatasetCategory::SMALL, DatasetCategory::MEDIUM, DatasetCategory::LARGE,
                              DatasetCategory::EXTREME}) {
        CategorySpec spec = DatasetGenerator::specFor(c);
        CategoryLimits limits = limitsFor(c);
        std::cout << "  " << std::left << std::setw(10) << categoryName(c) << std::right
                  << std::setw(5) << spec.minClients << "-" << std::left << std::setw(6) << spec.maxClients
                  << std::right << std::setw(5) << spec.minStations << "-" << std::left << std::setw(6) << spec.maxStations
                  << std::right << std::setw(5) << spec.minObstacles << "-" << std::left << std::setw(6) << spec.maxObstacles
                  << std::right << std::setw(4) << limits.timeLimitSeconds << " s" << std::setw(6)
                  << limits.memoryLimitMB << " MB\n";
    }

    std::string folder = askText("Carpeta de destino", "data/generated");
    int seeds = askNumber("Semillas por categoria", 5, 1, 50);
    std::cout << "  Generando (cada escenario calcula su bateria con la busqueda binaria)...\n" << std::flush;

    std::vector<std::string> paths = DatasetGenerator::generateBenchmarkSet(folder, seeds);
    for (const std::string& path : paths) {
        printScenarioSummary(ScenarioIO::load(path));
    }
    std::cout << "  " << paths.size() << " escenarios guardados en " << folder << "\n";
}

// ---------------------------------------------------------------------------------------------
// Opción 2: resolver un escenario
// ---------------------------------------------------------------------------------------------

void solveScenario() {
    std::cout << "\n--- RESOLVER UN ESCENARIO ---\n";
    std::string path = askText("Archivo de escenario", "data/generated/small_1.txt");
    Scenario scenario = ScenarioIO::load(path);
    printScenarioSummary(scenario);

    MissionSolver solver(scenario);
    std::cout << std::fixed << std::setprecision(2)
              << "  Grafo de mision y red de recarga listos en " << solver.preparationMs() << " ms\n";

    const std::vector<Algorithm>& algorithms = allAlgorithms();
    for (size_t i = 0; i < algorithms.size(); ++i) {
        bool available = algorithms[i] != Algorithm::BRANCH_AND_BOUND || solver.supportsExact();
        std::cout << "  " << i + 1 << ". " << algorithmName(algorithms[i])
                  << (available ? "" : "  (no disponible: demasiados clientes para el exacto)") << '\n';
    }
    int defaultChoice = solver.supportsExact() ? 1 : static_cast<int>(algorithms.size());
    int choice = askNumber("Algoritmo", defaultChoice, 1, static_cast<int>(algorithms.size()));
    while (algorithms[choice - 1] == Algorithm::BRANCH_AND_BOUND && !solver.supportsExact() && !inputClosed) {
        std::cout << "  El exacto no esta disponible para este escenario.\n";
        choice = askNumber("Algoritmo", defaultChoice, 1, static_cast<int>(algorithms.size()));
    }
    if (algorithms[choice - 1] == Algorithm::BRANCH_AND_BOUND && !solver.supportsExact()) choice = defaultChoice;

    Route route = solver.solve(algorithms[choice - 1]);
    CategoryLimits limits = limitsFor(scenario.category);

    std::cout << "\n  Algoritmo:  " << route.algorithm << "\n";
    std::cout << "  Estado:     " << (route.feasible ? "OK" : "INFACTIBLE") << "\n";
    std::cout << std::setprecision(2) << "  Costo:      " << route.cost << " (distancia volada, con recargas)\n";
    std::cout << "  Recargas:   " << route.recharges << "\n";
    std::cout << "  Excluidos:  " << route.excluded.size();
    for (const ExcludedClient& client : route.excluded) std::cout << "  " << client.label << " (" << client.reason << ")";
    std::cout << '\n';
    std::cout << "  Tiempo:     " << route.totalMs() << " ms = " << route.preparationMs << " de preparacion + "
              << route.solveMs << " de resolucion (limite " << static_cast<int>(limits.timeLimitSeconds * 1000)
              << " ms)\n";
    std::cout << "  Memoria:    " << peakMemoryMB() << " MB pico del proceso (limite "
              << static_cast<int>(limits.memoryLimitMB) << " MB)\n";
    std::cout << "  Orden:     ";
    printLabels(route.order);

    std::string output = "output/" + route.scenario + "_" + route.algorithm + ".txt";
    Exporter::saveRoute(route, output);
    std::cout << "  Ruta completa (" << route.path.size() << " puntos) exportada en " << output << "\n";
}

// ---------------------------------------------------------------------------------------------
// Opción 3: juez y benchmark (RF14, RF15)
// ---------------------------------------------------------------------------------------------

void printRanking(const std::vector<RankingRow>& ranking) {
    int category = -1;
    for (const RankingRow& row : ranking) {
        if (static_cast<int>(row.category) != category) {
            category = static_cast<int>(row.category);
            std::cout << "\n  " << categoryName(row.category) << "\n"
                      << "    #  algoritmo                 OK/casos   gap medio   tiempo medio\n";
        }
        std::cout << "    " << row.position << "  " << std::left << std::setw(26) << row.algorithm << std::right
                  << std::setw(3) << row.okCases << "/" << std::left << std::setw(6) << row.cases << std::right
                  << std::fixed << std::setprecision(2) << std::setw(8) << row.meanGap << " %"
                  << std::setw(12) << row.meanTimeMs << " ms\n";
    }
}

void runJudge() {
    std::cout << "\n--- JUEZ Y BENCHMARK ---\n";
    std::string folder = askText("Carpeta de escenarios", "data/generated");
    std::string adversarial = askText("Incluir los casos de data/adversarial (s/n)", "s");

    std::vector<std::string> files = Judge::findScenarioFiles(folder);
    if (!adversarial.empty() && (adversarial[0] == 's' || adversarial[0] == 'S')) {
        std::vector<std::string> extra = Judge::findScenarioFiles("data/adversarial");
        files.insert(files.end(), extra.begin(), extra.end());
    }
    if (files.empty()) {
        std::cout << "  No hay escenarios en " << folder << ". Genere datasets con la opcion 1.\n";
        return;
    }

    const std::string output = "output/judge";
    if (sessionUsedMemory) {
        std::cout << "  Aviso: en esta sesion ya se uso memoria (pico actual " << std::fixed << std::setprecision(1)
                  << peakMemoryMB() << " MB). La columna de memoria no sera por caso: para medirla,\n"
                  << "  ejecute el juez como primera opcion al abrir el programa.\n";
    }
    std::cout << "  Ejecutando " << files.size() << " escenarios de menor a mayor...\n" << std::flush;
    std::vector<JudgeRow> rows = Judge::run(files, output + "/routes", std::cout);
    std::vector<RankingRow> ranking = Judge::rank(rows);
    Exporter::saveResultsCsv(rows, output + "/results.csv");
    Exporter::saveRankingCsv(ranking, output + "/ranking.csv");

    int ok = 0, wa = 0, tle = 0, mle = 0;
    for (const JudgeRow& row : rows) {
        ok += row.verdict == "OK";
        wa += row.verdict == "WA";
        tle += row.verdict == "TLE";
        mle += row.verdict == "MLE";
    }
    std::cout << "\n  Veredictos: " << ok << " OK, " << wa << " WA, " << tle << " TLE, " << mle << " MLE (de "
              << rows.size() << " ejecuciones)\n";
    std::cout << "  Ranking por categoria (mas casos OK, luego menor gap medio, luego menor tiempo):";
    printRanking(ranking);
    std::cout << "\n  Resultados en " << output << "/results.csv, ranking en " << output
              << "/ranking.csv y rutas en " << output << "/routes/\n";
}

// ---------------------------------------------------------------------------------------------
// Opción 4: analizar la red de recarga (RF10, RF11, RF12)
// ---------------------------------------------------------------------------------------------

void analyzeNetwork() {
    std::cout << "\n--- ANALIZAR RED DE RECARGA ---\n";
    std::string path = askText("Archivo de escenario", "data/generated/medium_1.txt");
    Scenario scenario = ScenarioIO::load(path);
    printScenarioSummary(scenario);

    MissionGraph mission(scenario);
    ChargingNetwork network(mission, scenario.battery);
    std::cout << std::fixed << std::setprecision(2);

    // Alcance de la red con la batería del escenario (DFS desde la base).
    std::vector<std::string> unreachable;
    for (int s = 0; s < mission.numStations(); ++s) {
        if (!network.isReachable(mission.stationKey(s))) unreachable.push_back(labelOf(mission, mission.stationKey(s)));
    }
    std::cout << "  Estaciones alcanzables desde la base: " << mission.numStations() - unreachable.size()
              << " de " << mission.numStations() << '\n';
    if (!unreachable.empty()) {
        std::cout << "  Sin conexion con la base:";
        printLabels(unreachable);
    }

    // Estaciones críticas: puntos de articulación (Tarjan).
    std::vector<std::string> critical;
    for (int key : network.criticalStations()) critical.push_back(labelOf(mission, key));
    std::cout << "  Estaciones criticas (puntos de articulacion): ";
    if (critical.empty()) {
        std::cout << "ninguna\n";
    } else {
        printLabels(critical);
    }

    // Red mínima de soporte: árbol de expansión mínima entre la base y las estaciones (Prim).
    std::vector<int> nodes;
    MstResult tree = ChargingNetwork::supportTree(mission, nodes);
    std::cout << "  Red minima de soporte: costo " << tree.cost << ", tramo mas largo " << tree.maxEdge << '\n';
    for (size_t i = 1; i < nodes.size(); ++i) {
        if (tree.parent[i] < 0) continue;
        int from = nodes[tree.parent[i]];
        std::cout << "    " << labelOf(mission, from) << " - " << labelOf(mission, nodes[i])
                  << "  " << mission.distance(from, nodes[i]) << '\n';
    }

    // Autonomía mínima (búsqueda binaria) frente a la del escenario.
    double minimum = ChargingNetwork::minimumAutonomy(mission);
    std::cout << "  Autonomia minima para atender a todos los clientes: " << minimum
              << " (el escenario tiene " << scenario.battery << ")\n";

    std::vector<std::string> byObstacles, byBattery;
    for (int c = 0; c < mission.numClients(); ++c) {
        int key = mission.clientKey(c);
        if (!mission.isConnectedToDepot(key)) {
            byObstacles.push_back(labelOf(mission, key));
        } else if (!network.canServe(key)) {
            byBattery.push_back(labelOf(mission, key));
        }
    }
    std::cout << "  Clientes separados por obstaculos: " << byObstacles.size();
    if (!byObstacles.empty()) printLabels(byObstacles); else std::cout << '\n';
    std::cout << "  Clientes fuera del alcance de la bateria: " << byBattery.size();
    if (!byBattery.empty()) printLabels(byBattery); else std::cout << '\n';
}

// ---------------------------------------------------------------------------------------------

int main() {
    // I/O rápido de programación competitiva; las preguntas del menú hacen flush explícito.
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(NULL);

    printBanner();
    while (true) {
        std::cout << "\n=== MENU ===\n"
                  << "  1. Generar datasets\n"
                  << "  2. Resolver un escenario\n"
                  << "  3. Ejecutar juez y benchmark\n"
                  << "  4. Analizar red de recarga\n"
                  << "  0. Salir\n";
        int option = askNumber("Opcion", 0, 0, 4);
        if (option == 0) break;
        try {
            if (option == 1) generateDatasets();
            if (option == 2) solveScenario();
            if (option == 3) runJudge();
            if (option == 4) analyzeNetwork();
        } catch (const std::exception& error) {
            std::cout << "  [ERROR] " << error.what() << '\n';
        }
        sessionUsedMemory = true;
        if (inputClosed) break;
    }
    std::cout << "Hasta luego.\n";
    return 0;
}
