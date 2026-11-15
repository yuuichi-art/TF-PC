#include "Exporter.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace geodrone {
namespace Exporter {

namespace {

// Crea el archivo y sus carpetas; lanza std::runtime_error si no se puede.
std::ofstream openForWriting(const std::string& path) {
    std::filesystem::path filePath(path);
    if (filePath.has_parent_path()) {
        std::filesystem::create_directories(filePath.parent_path());
    }
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("no se pudo crear el archivo " + path);
    }
    return file;
}

} // namespace

void writeRoute(const Route& route, std::ostream& out) {
    out << "SCENARIO " << route.scenario << '\n';
    out << "ALGORITHM " << route.algorithm << '\n';
    out << "STATUS " << (route.feasible ? "OK" : "INFEASIBLE") << '\n';
    // Seis decimales: suficientes para que el juez compare el costo con el que recalcula.
    out << "COST " << std::fixed << std::setprecision(6) << route.cost << '\n';
    out << "RECHARGES " << route.recharges << '\n';

    out << "EXCLUDED " << route.excluded.size() << '\n';
    for (const ExcludedClient& client : route.excluded) {
        out << client.label << ' ' << client.reason << '\n';
    }

    out << "ORDER " << route.order.size() << '\n';
    for (size_t i = 0; i < route.order.size(); ++i) {
        out << (i > 0 ? " " : "") << route.order[i];
    }
    out << '\n';

    // Coordenadas como en los escenarios: los enteros sin decimales.
    out << std::defaultfloat << std::setprecision(15);
    out << "PATH " << route.path.size() << '\n';
    for (const RoutePoint& p : route.path) {
        out << p.point.x << ' ' << p.point.y << ' ' << p.label << '\n';
    }
}

void saveRoute(const Route& route, const std::string& path) {
    std::ofstream file = openForWriting(path);
    writeRoute(route, file);
    if (!file) {
        throw std::runtime_error("error al escribir el archivo " + path);
    }
}

void writeResultsCsv(const std::vector<JudgeRow>& rows, std::ostream& out) {
    out << "escenario,categoria,clientes,algoritmo,veredicto,costo,recargas,excluidos,tiempo_ms,memoria_mb,"
           "gap_mejor,gap_cota\n";
    out << std::fixed << std::setprecision(3);
    for (const JudgeRow& row : rows) {
        out << row.scenario << ',' << categoryName(row.category) << ',' << row.clients << ',' << row.algorithm << ','
            << row.verdict << ',' << row.cost << ',' << row.recharges << ',' << row.excluded << ',' << row.timeMs
            << ',' << row.memoryMB << ',';
        if (!std::isnan(row.gapBest)) out << row.gapBest;
        out << ',';
        if (!std::isnan(row.gapBound)) out << row.gapBound;
        out << '\n';
    }
}

void saveResultsCsv(const std::vector<JudgeRow>& rows, const std::string& path) {
    std::ofstream file = openForWriting(path);
    writeResultsCsv(rows, file);
    if (!file) throw std::runtime_error("error al escribir el archivo " + path);
}

void writeRankingCsv(const std::vector<RankingRow>& ranking, std::ostream& out) {
    out << "categoria,posicion,algoritmo,casos_ok,casos,gap_medio,tiempo_medio_ms\n";
    out << std::fixed << std::setprecision(3);
    for (const RankingRow& row : ranking) {
        out << categoryName(row.category) << ',' << row.position << ',' << row.algorithm << ',' << row.okCases
            << ',' << row.cases << ',' << row.meanGap << ',' << row.meanTimeMs << '\n';
    }
}

void saveRankingCsv(const std::vector<RankingRow>& ranking, const std::string& path) {
    std::ofstream file = openForWriting(path);
    writeRankingCsv(ranking, file);
    if (!file) throw std::runtime_error("error al escribir el archivo " + path);
}

} // namespace Exporter
} // namespace geodrone
