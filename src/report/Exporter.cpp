#include "Exporter.hpp"

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

} // namespace Exporter
} // namespace geodrone
