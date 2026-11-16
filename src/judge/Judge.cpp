#include "Judge.hpp"
#include "MemoryUsage.hpp"
#include "../geometry/GeometryUtils.hpp"
#include "../graph/Mst.hpp"
#include "../report/Exporter.hpp"
#include "../scenario/ScenarioIO.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <map>

namespace geodrone {
namespace Judge {

namespace {

const double TOL = ChargingNetwork::BATTERY_TOLERANCE;
const double NOT_AVAILABLE = std::numeric_limits<double>::quiet_NaN();   // Gap de una fila que no es OK

VerificationResult fail(const std::string& problem) {
    return {false, problem};
}

// Interpreta una etiqueta "D0", "S2" o "C5": tipo y número (1..S o 1..N; 0 para la base).
// Devuelve false si no corresponde a ningún punto del escenario.
bool parseLabel(const Scenario& s, const std::string& label, char& type, int& number) {
    if (label.size() < 2) return false;
    type = label[0];
    try {
        size_t used = 0;
        number = std::stoi(label.substr(1), &used);
        if (used != label.size() - 1) return false;
    } catch (const std::exception&) {
        return false;
    }
    if (type == 'D') return number == 0;
    if (type == 'S') return number >= 1 && number <= s.numStations();
    if (type == 'C') return number >= 1 && number <= s.numClients();
    return false;
}

// Coordenadas que el escenario asigna a la etiqueta (ya validada con parseLabel).
Point pointOf(const Scenario& s, char type, int number) {
    if (type == 'D') return s.depot;
    if (type == 'S') return s.stations[number - 1];
    return s.clients[number - 1];
}

double percentOver(double value, double reference) {
    return reference > 0 ? 100.0 * (value / reference - 1.0) : 0.0;
}

// Una exclusión es real si el grafo de misión la confirma.
bool exclusionIsGenuine(const MissionSolver& solver, int clientIndex, const std::string& reason) {
    int key = solver.mission().clientKey(clientIndex);
    bool connected = solver.mission().isConnectedToDepot(key);
    if (reason == "OBSTACLES") return !connected;
    if (reason == "BATTERY") return connected && !solver.network().canServe(key);
    return false;
}

// Tamaño de un escenario para ordenarlos de menor a mayor.
int sizeOf(const Scenario& s) {
    return s.numClients() + s.numStations() + s.totalObstacleVertices();
}

} // namespace

VerificationResult verifyRoute(const Scenario& scenario, const MissionSolver& solver, const Route& route) {
    if (!route.feasible) return fail("el solver no encontro una ruta factible");
    if (route.path.empty()) return fail("la trayectoria esta vacia");
    if (route.path.front().label != "D0" || route.path.back().label != "D0") {
        return fail("la ruta no empieza y termina en la base");
    }

    double battery = scenario.battery;
    double length = 0.0;
    int recharges = 0;
    std::vector<int> visits(scenario.numClients(), 0);
    std::vector<std::string> keyLabels;     // Nodos clave de la trayectoria, para compararlos con ORDER

    for (size_t i = 0; i < route.path.size(); ++i) {
        const RoutePoint& p = route.path[i];
        char type = 'V';
        int number = 0;
        if (p.label != "V") {
            if (!parseLabel(scenario, p.label, type, number)) return fail("etiqueta desconocida '" + p.label + "'");
            if (p.point != pointOf(scenario, type, number)) {
                return fail(p.label + " no esta en las coordenadas del escenario");
            }
            keyLabels.push_back(p.label);
        }
        if (i == 0) continue;

        Segment leg(route.path[i - 1].point, p.point);
        if (!GeometryUtils::isFlightPathClear(leg, scenario.obstacles)) {
            return fail("el tramo " + std::to_string(i) + " (hacia " + p.label + ") cruza un obstaculo");
        }
        length += leg.length();
        battery -= leg.length();
        if (battery < -TOL) {
            return fail("la bateria se agota en el tramo " + std::to_string(i) + " (hacia " + p.label + ")");
        }

        // Llegar a la base o a una estación llena la batería. La llegada final a la base no
        // cuenta como recarga, porque ahí termina la misión.
        if (type == 'D' || type == 'S') {
            battery = scenario.battery;
            if (i + 1 < route.path.size()) ++recharges;
        } else if (type == 'C' && ++visits[number - 1] > 1) {
            return fail(p.label + " se visita mas de una vez");
        }
    }

    std::vector<char> excluded(scenario.numClients(), 0);
    for (const ExcludedClient& client : route.excluded) {
        char type = 'V';
        int number = 0;
        if (!parseLabel(scenario, client.label, type, number) || type != 'C') {
            return fail("excluido desconocido '" + client.label + "'");
        }
        if (visits[number - 1] > 0) return fail(client.label + " esta excluido y tambien se visita");
        if (!exclusionIsGenuine(solver, number - 1, client.reason)) {
            return fail(client.label + " se excluyo por " + client.reason + " pero se puede atender");
        }
        excluded[number - 1] = 1;
    }
    for (int c = 0; c < scenario.numClients(); ++c) {
        if (visits[c] == 0 && !excluded[c]) return fail("C" + std::to_string(c + 1) + " no se visita ni esta excluido");
    }

    if (std::abs(length - route.cost) > TOL * std::max(1.0, length)) {
        return fail("el costo informado no coincide con la trayectoria");
    }
    if (recharges != route.recharges) return fail("las recargas informadas no coinciden con la trayectoria");
    if (keyLabels != route.order) return fail("el orden informado no coincide con la trayectoria");
    return {};
}

std::string verdictFor(bool valid, double timeMs, double memoryMB, const CategoryLimits& limits) {
    if (!valid) return "WA";
    if (timeMs > limits.timeLimitSeconds * 1000.0) return "TLE";
    if (memoryMB > limits.memoryLimitMB) return "MLE";
    return "OK";
}

std::vector<std::string> findScenarioFiles(const std::string& folder) {
    std::vector<std::string> files;
    if (!std::filesystem::is_directory(folder)) return files;
    for (const auto& entry : std::filesystem::directory_iterator(folder)) {
        if (entry.is_regular_file() && entry.path().extension() == ".txt") {
            files.push_back(entry.path().generic_string());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

namespace {

// Resuelve un escenario con cada algoritmo, guarda y verifica cada ruta, y le da veredicto y gap.
std::vector<JudgeRow> judgeScenario(const Scenario& s, const std::string& routesFolder) {
    MissionSolver solver(s);
    bool smallCase = s.category == DatasetCategory::SMALL || s.category == DatasetCategory::ADVERSARIAL;

    std::vector<JudgeRow> rows;
    std::vector<bool> valid;
    for (Algorithm algorithm : allAlgorithms()) {
        if (algorithm == Algorithm::BRANCH_AND_BOUND && !(smallCase && solver.supportsExact())) continue;
        Route route = solver.solve(algorithm);
        Exporter::saveRoute(route, routesFolder + "/" + route.scenario + "_" + route.algorithm + ".txt");
        VerificationResult check = verifyRoute(s, solver, route);

        JudgeRow row;
        row.scenario = s.name;
        row.category = s.category;
        row.clients = s.numClients();
        row.algorithm = route.algorithm;
        row.problem = check.problem;
        row.cost = route.cost;
        row.recharges = route.recharges;
        row.excluded = static_cast<int>(route.excluded.size());
        row.timeMs = route.totalMs();
        rows.push_back(row);
        valid.push_back(check.valid);
    }

    // Memoria pico al terminar el caso; los veredictos se dan después de medirla.
    double memory = peakMemoryMB();
    CategoryLimits limits = limitsFor(s.category);
    double best = INF;
    for (size_t i = 0; i < rows.size(); ++i) {
        rows[i].memoryMB = memory;
        rows[i].verdict = verdictFor(valid[i], rows[i].timeMs, memory, limits);
        if (rows[i].verdict == "OK") best = std::min(best, rows[i].cost);
    }

    // Gap contra la mejor ruta OK y contra la cota inferior: el árbol de expansión mínima de la
    // base y los clientes atendibles.
    std::vector<int> nodes = {solver.mission().depotKey()};
    nodes.insert(nodes.end(), solver.servableClients().begin(), solver.servableClients().end());
    double bound = primMst(solver.mission().distances(), nodes).cost;
    for (JudgeRow& row : rows) {
        bool isOk = row.verdict == "OK";
        row.gapBest = isOk ? percentOver(row.cost, best) : NOT_AVAILABLE;
        row.gapBound = isOk ? percentOver(row.cost, bound) : NOT_AVAILABLE;
    }
    return rows;
}

// Avance en consola: una línea por caso y el motivo de cada ruta que no dio OK.
void printCase(std::ostream& log, size_t number, size_t total, const Scenario& s, const std::vector<JudgeRow>& rows) {
    int ok = 0;
    double best = INF;
    for (const JudgeRow& row : rows) {
        if (row.verdict == "OK") {
            ++ok;
            best = std::min(best, row.cost);
        }
    }
    log << "  [" << std::setw(2) << number << "/" << total << "] " << std::left << std::setw(26) << s.name
        << std::right << std::setw(5) << s.numClients() << " clientes  " << ok << "/" << rows.size()
        << " OK  mejor " << std::fixed << std::setprecision(2) << best << "\n" << std::flush;
    for (const JudgeRow& row : rows) {
        if (row.verdict != "OK") {
            log << "         " << row.algorithm << ": " << row.verdict
                << (row.problem.empty() ? "" : " (" + row.problem + ")") << "\n";
        }
    }
}

} // namespace

std::vector<JudgeRow> run(const std::vector<std::string>& scenarioFiles, const std::string& routesFolder,
                          std::ostream& log) {
    std::vector<Scenario> scenarios;
    for (const std::string& file : scenarioFiles) scenarios.push_back(ScenarioIO::load(file));
    std::stable_sort(scenarios.begin(), scenarios.end(),
                     [](const Scenario& a, const Scenario& b) { return sizeOf(a) < sizeOf(b); });

    std::vector<JudgeRow> rows;
    for (size_t i = 0; i < scenarios.size(); ++i) {
        std::vector<JudgeRow> caseRows = judgeScenario(scenarios[i], routesFolder);
        printCase(log, i + 1, scenarios.size(), scenarios[i], caseRows);
        rows.insert(rows.end(), caseRows.begin(), caseRows.end());
    }
    return rows;
}

std::vector<RankingRow> rank(const std::vector<JudgeRow>& rows) {
    // Acumulados por categoría y algoritmo.
    std::map<std::pair<int, std::string>, RankingRow> totals;
    for (const JudgeRow& row : rows) {
        RankingRow& total = totals[{static_cast<int>(row.category), row.algorithm}];
        total.category = row.category;
        total.algorithm = row.algorithm;
        ++total.cases;
        if (row.verdict == "OK") {
            ++total.okCases;
            total.meanGap += row.gapBest;
            total.meanTimeMs += row.timeMs;
        }
    }

    // Promedios sobre los casos OK, agrupados por categoría para ordenarlos dentro de cada una.
    std::map<int, std::vector<RankingRow>> byCategory;
    for (auto& entry : totals) {
        RankingRow& total = entry.second;
        if (total.okCases > 0) {
            total.meanGap /= total.okCases;
            total.meanTimeMs /= total.okCases;
        }
        byCategory[entry.first.first].push_back(total);
    }

    std::vector<RankingRow> ranking;
    for (auto& entry : byCategory) {
        std::vector<RankingRow>& list = entry.second;
        std::sort(list.begin(), list.end(), [](const RankingRow& a, const RankingRow& b) {
            if (a.okCases != b.okCases) return a.okCases > b.okCases;
            if (a.meanGap != b.meanGap) return a.meanGap < b.meanGap;
            return a.meanTimeMs < b.meanTimeMs;
        });
        for (size_t i = 0; i < list.size(); ++i) {
            list[i].position = static_cast<int>(i) + 1;
            ranking.push_back(list[i]);
        }
    }
    return ranking;
}

} // namespace Judge
} // namespace geodrone
