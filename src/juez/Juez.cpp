#include "Juez.hpp"
#include "UsoMemoria.hpp"
#include "../geometria/Geometria.hpp"
#include "../grafo/ArbolMinimo.hpp"
#include "../exportacion/Exportador.hpp"
#include "../escenario/ArchivoEscenario.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <map>

namespace geodrone {
namespace Judge {

namespace {

const double TOL = RedRecarga::TOLERANCIA_BATERIA;
const double NOT_AVAILABLE = std::numeric_limits<double>::quiet_NaN();   // Gap de una fila que no es OK

VerificationResult fail(const std::string& problem) {
    return {false, problem};
}

// Interpreta una etiqueta "D0", "S2" o "C5": tipo y número (1..S o 1..N; 0 para la base).
// Devuelve false si no corresponde a ningún punto del escenario.
bool parseLabel(const Escenario& s, const std::string& etiqueta, char& tipo, int& number) {
    if (etiqueta.size() < 2) return false;
    tipo = etiqueta[0];
    try {
        size_t used = 0;
        number = std::stoi(etiqueta.substr(1), &used);
        if (used != etiqueta.size() - 1) return false;
    } catch (const std::exception&) {
        return false;
    }
    if (tipo == 'D') return number == 0;
    if (tipo == 'S') return number >= 1 && number <= s.numEstaciones();
    if (tipo == 'C') return number >= 1 && number <= s.numClientes();
    return false;
}

// Coordenadas que el escenario asigna a la etiqueta (ya validada con parseLabel).
Punto pointOf(const Escenario& s, char tipo, int number) {
    if (tipo == 'D') return s.base;
    if (tipo == 'S') return s.estaciones[number - 1];
    return s.clientes[number - 1];
}

double percentOver(double value, double reference) {
    return reference > 0 ? 100.0 * (value / reference - 1.0) : 0.0;
}

// Una exclusión es real si el grafo de misión la confirma.
bool exclusionIsGenuine(const MissionSolver& solver, int clientIndex, const std::string& reason) {
    int key = solver.mision().claveCliente(clientIndex);
    bool connected = solver.mision().conectadoABase(key);
    if (reason == "OBSTACLES") return !connected;
    if (reason == "BATTERY") return connected && !solver.red().puedeAtender(key);
    return false;
}

// Tamaño de un escenario para ordenarlos de menor a mayor.
int sizeOf(const Escenario& s) {
    return s.numClientes() + s.numEstaciones() + s.totalVerticesObstaculos();
}

} // namespace

VerificationResult verifyRoute(const Escenario& escenario, const MissionSolver& solver, const Route& route) {
    if (!route.feasible) return fail("el solver no encontro una ruta factible");
    if (route.path.empty()) return fail("la trayectoria esta vacia");
    if (route.path.front().etiqueta != "D0" || route.path.back().etiqueta != "D0") {
        return fail("la ruta no empieza y termina en la base");
    }

    double bateria = escenario.bateria;
    double length = 0.0;
    int recharges = 0;
    std::vector<int> visits(escenario.numClientes(), 0);
    std::vector<std::string> keyLabels;     // Nodos clave de la trayectoria, para compararlos con ORDER

    for (size_t i = 0; i < route.path.size(); ++i) {
        const RoutePoint& p = route.path[i];
        char tipo = 'V';
        int number = 0;
        if (p.etiqueta != "V") {
            if (!parseLabel(escenario, p.etiqueta, tipo, number)) return fail("etiqueta desconocida '" + p.etiqueta + "'");
            if (p.point != pointOf(escenario, tipo, number)) {
                return fail(p.etiqueta + " no esta en las coordenadas del escenario");
            }
            keyLabels.push_back(p.etiqueta);
        }
        if (i == 0) continue;

        Segmento leg(route.path[i - 1].point, p.point);
        if (!Geometria::tramoLibre(leg, escenario.obstaculos)) {
            return fail("el tramo " + std::to_string(i) + " (hacia " + p.etiqueta + ") cruza un obstaculo");
        }
        length += leg.longitud();
        bateria -= leg.longitud();
        if (bateria < -TOL) {
            return fail("la bateria se agota en el tramo " + std::to_string(i) + " (hacia " + p.etiqueta + ")");
        }

        // Llegar a la base o a una estación llena la batería. La llegada final a la base no
        // cuenta como recarga, porque ahí termina la misión.
        if (tipo == 'D' || tipo == 'S') {
            bateria = escenario.bateria;
            if (i + 1 < route.path.size()) ++recharges;
        } else if (tipo == 'C' && ++visits[number - 1] > 1) {
            return fail(p.etiqueta + " se visita mas de una vez");
        }
    }

    std::vector<char> excluded(escenario.numClientes(), 0);
    for (const ExcludedClient& client : route.excluded) {
        char tipo = 'V';
        int number = 0;
        if (!parseLabel(escenario, client.etiqueta, tipo, number) || tipo != 'C') {
            return fail("excluido desconocido '" + client.etiqueta + "'");
        }
        if (visits[number - 1] > 0) return fail(client.etiqueta + " esta excluido y tambien se visita");
        if (!exclusionIsGenuine(solver, number - 1, client.reason)) {
            return fail(client.etiqueta + " se excluyo por " + client.reason + " pero se puede atender");
        }
        excluded[number - 1] = 1;
    }
    for (int c = 0; c < escenario.numClientes(); ++c) {
        if (visits[c] == 0 && !excluded[c]) return fail("C" + std::to_string(c + 1) + " no se visita ni esta excluido");
    }

    if (std::abs(length - route.costo) > TOL * std::max(1.0, length)) {
        return fail("el costo informado no coincide con la trayectoria");
    }
    if (recharges != route.recharges) return fail("las recargas informadas no coinciden con la trayectoria");
    if (keyLabels != route.order) return fail("el orden informado no coincide con la trayectoria");
    return {};
}

std::string verdictFor(bool valid, double timeMs, double memoryMB, const LimitesCategoria& limits) {
    if (!valid) return "WA";
    if (timeMs > limits.limiteTiempoSegundos * 1000.0) return "TLE";
    if (memoryMB > limits.limiteMemoriaMB) return "MLE";
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
std::vector<JudgeRow> judgeScenario(const Escenario& s, const std::string& routesFolder) {
    MissionSolver solver(s);
    bool smallCase = s.categoria == Categoria::SMALL || s.categoria == Categoria::ADVERSARIAL;

    std::vector<JudgeRow> rows;
    std::vector<bool> valid;
    for (Algorithm algorithm : allAlgorithms()) {
        if (algorithm == Algorithm::BRANCH_AND_BOUND && !(smallCase && solver.supportsExact())) continue;
        Route route = solver.solve(algorithm);
        Exporter::saveRoute(route, routesFolder + "/" + route.escenario + "_" + route.algorithm + ".txt");
        VerificationResult check = verifyRoute(s, solver, route);

        JudgeRow row;
        row.escenario = s.nombre;
        row.categoria = s.categoria;
        row.clientes = s.numClientes();
        row.algorithm = route.algorithm;
        row.problem = check.problem;
        row.costo = route.costo;
        row.recharges = route.recharges;
        row.excluded = static_cast<int>(route.excluded.size());
        row.timeMs = route.totalMs();
        rows.push_back(row);
        valid.push_back(check.valid);
    }

    // Memoria pico al terminar el caso; los veredictos se dan después de medirla.
    double memory = peakMemoryMB();
    LimitesCategoria limits = limitesDe(s.categoria);
    double best = INF;
    for (size_t i = 0; i < rows.size(); ++i) {
        rows[i].memoryMB = memory;
        rows[i].verdict = verdictFor(valid[i], rows[i].timeMs, memory, limits);
        if (rows[i].verdict == "OK") best = std::min(best, rows[i].costo);
    }

    // Gap contra la mejor ruta OK y contra la cota inferior: el árbol de expansión mínima de la
    // base y los clientes atendibles.
    std::vector<int> nodes = {solver.mision().claveBase()};
    nodes.insert(nodes.end(), solver.servableClients().begin(), solver.servableClients().end());
    double bound = arbolMinimoPrim(solver.mision().distancias(), nodes).costo;
    for (JudgeRow& row : rows) {
        bool isOk = row.verdict == "OK";
        row.gapBest = isOk ? percentOver(row.costo, best) : NOT_AVAILABLE;
        row.gapBound = isOk ? percentOver(row.costo, bound) : NOT_AVAILABLE;
    }
    return rows;
}

// Avance en consola: una línea por caso y el motivo de cada ruta que no dio OK.
void printCase(std::ostream& log, size_t number, size_t total, const Escenario& s, const std::vector<JudgeRow>& rows) {
    int ok = 0;
    double best = INF;
    for (const JudgeRow& row : rows) {
        if (row.verdict == "OK") {
            ++ok;
            best = std::min(best, row.costo);
        }
    }
    log << "  [" << std::setw(2) << number << "/" << total << "] " << std::left << std::setw(26) << s.nombre
        << std::right << std::setw(5) << s.numClientes() << " clientes  " << ok << "/" << rows.size()
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
    std::vector<Escenario> scenarios;
    for (const std::string& file : scenarioFiles) scenarios.push_back(ArchivoEscenario::cargar(file));
    std::stable_sort(scenarios.begin(), scenarios.end(),
                     [](const Escenario& a, const Escenario& b) { return sizeOf(a) < sizeOf(b); });

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
        RankingRow& total = totals[{static_cast<int>(row.categoria), row.algorithm}];
        total.categoria = row.categoria;
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
