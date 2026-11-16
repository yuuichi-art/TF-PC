#include "Exportador.hpp"
#include "../escenario/ArchivoEscenario.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace geodrone {
namespace Exporter {

void writeRoute(const Route& route, std::ostream& out) {
    out << "SCENARIO " << route.escenario << '\n';
    out << "ALGORITHM " << route.algorithm << '\n';
    out << "STATUS " << (route.feasible ? "OK" : "INFEASIBLE") << '\n';
    // Seis decimales: suficientes para que el juez compare el costo con el que recalcula.
    out << "COST " << std::fixed << std::setprecision(6) << route.costo << '\n';
    out << "RECHARGES " << route.recharges << '\n';

    out << "EXCLUDED " << route.excluded.size() << '\n';
    for (const ExcludedClient& client : route.excluded) {
        out << client.etiqueta << ' ' << client.reason << '\n';
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
        out << p.point.x << ' ' << p.point.y << ' ' << p.etiqueta << '\n';
    }
}

void saveRoute(const Route& route, const std::string& path) {
    std::ofstream file = ArchivoEscenario::abrirParaEscribir(path);
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
        out << row.escenario << ',' << nombreCategoria(row.categoria) << ',' << row.clientes << ',' << row.algorithm << ','
            << row.verdict << ',' << row.costo << ',' << row.recharges << ',' << row.excluded << ',' << row.timeMs
            << ',' << row.memoryMB << ',';
        if (!std::isnan(row.gapBest)) out << row.gapBest;
        out << ',';
        if (!std::isnan(row.gapBound)) out << row.gapBound;
        out << '\n';
    }
}

void saveResultsCsv(const std::vector<JudgeRow>& rows, const std::string& path) {
    std::ofstream file = ArchivoEscenario::abrirParaEscribir(path);
    writeResultsCsv(rows, file);
    if (!file) throw std::runtime_error("error al escribir el archivo " + path);
}

void writeRankingCsv(const std::vector<RankingRow>& ranking, std::ostream& out) {
    out << "categoria,posicion,algoritmo,casos_ok,casos,gap_medio,tiempo_medio_ms\n";
    out << std::fixed << std::setprecision(3);
    for (const RankingRow& row : ranking) {
        out << nombreCategoria(row.categoria) << ',' << row.position << ',' << row.algorithm << ',' << row.okCases
            << ',' << row.cases << ',' << row.meanGap << ',' << row.meanTimeMs << '\n';
    }
}

void saveRankingCsv(const std::vector<RankingRow>& ranking, const std::string& path) {
    std::ofstream file = ArchivoEscenario::abrirParaEscribir(path);
    writeRankingCsv(ranking, file);
    if (!file) throw std::runtime_error("error al escribir el archivo " + path);
}

namespace {

// Conversión de coordenadas del escenario a píxeles del dibujo.
struct Canvas {
    double scale;
    double left;
    double top;
    double altoArea;

    double x(double value) const { return left + value * scale; }
    double y(double value) const { return top + (altoArea - value) * scale; }   // y crece hacia abajo en SVG
};

std::string escapeXml(const std::string& text) {
    std::string result;
    for (char c : text) {
        if (c == '&') result += "&amp;";
        else if (c == '<') result += "&lt;";
        else if (c == '>') result += "&gt;";
        else if (c == '"') result += "&quot;";
        else result += c;
    }
    return result;
}

void writePoints(std::ostream& out, const Canvas& canvas, const std::vector<Punto>& points) {
    for (size_t i = 0; i < points.size(); ++i) {
        out << (i > 0 ? " " : "") << canvas.x(points[i].x) << ',' << canvas.y(points[i].y);
    }
}

void writeStation(std::ostream& out, double x, double y, double size, const std::string& attributes) {
    out << "<polygon " << attributes << " points=\"" << x << ',' << y - size * 1.2 << ' ' << x - size << ','
        << y + size * 0.7 << ' ' << x + size << ',' << y + size * 0.7 << "\" fill=\"#16a34a\"/>\n";
}

void writeCross(std::ostream& out, double x, double y, double size, const std::string& attributes) {
    out << "<path " << attributes << " d=\"M" << x - size << ',' << y - size << " L" << x + size << ',' << y + size
        << " M" << x - size << ',' << y + size << " L" << x + size << ',' << y - size
        << "\" stroke=\"#dc2626\" stroke-width=\"2\"/>\n";
}

void writeLabel(std::ostream& out, double x, double y, const std::string& text) {
    out << "<text x=\"" << x + 6 << "\" y=\"" << y - 6 << "\" font-family=\"sans-serif\" font-size=\"10\" "
        << "fill=\"#374151\">" << escapeXml(text) << "</text>\n";
}

} // namespace

void writeSvg(const Escenario& escenario, const Route& route, std::ostream& out) {
    const double mapSize = 800.0, margin = 20.0, header = 40.0, footer = 34.0;
    double scale = mapSize / std::max(escenario.anchoArea, escenario.altoArea);
    Canvas canvas{scale, margin, header, escenario.altoArea};
    double width = escenario.anchoArea * scale + 2 * margin;
    double height = header + escenario.altoArea * scale + margin + footer;

    // Con muchos clientes los puntos se achican y se omiten las etiquetas para que se lea.
    bool withLabels = escenario.numClientes() <= 40;
    double radius = escenario.numClientes() > 200 ? 2.5 : 4.0;

    out << std::fixed << std::setprecision(2);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\"" << height
        << "\" viewBox=\"0 0 " << width << ' ' << height << "\">\n";
    out << "<rect width=\"" << width << "\" height=\"" << height << "\" fill=\"#ffffff\"/>\n";
    out << "<text x=\"" << margin << "\" y=\"26\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">"
        << "<tspan font-weight=\"bold\">" << escapeXml(route.escenario) << "</tspan>  " << escapeXml(route.algorithm)
        << "  |  costo " << route.costo << "  |  recargas " << route.recharges << "  |  excluidos "
        << route.excluded.size() << (route.feasible ? "" : "  |  INFACTIBLE") << "</text>\n";

    // Área del escenario y obstáculos.
    out << "<rect class=\"area\" x=\"" << canvas.x(0) << "\" y=\"" << canvas.y(escenario.altoArea) << "\" width=\""
        << escenario.anchoArea * scale << "\" height=\"" << escenario.altoArea * scale
        << "\" fill=\"#f9fafb\" stroke=\"#d1d5db\"/>\n";
    for (const Poligono& poly : escenario.obstaculos) {
        out << "<polygon class=\"obstacle\" points=\"";
        writePoints(out, canvas, poly.vertices);
        out << "\" fill=\"#9ca3af\" fill-opacity=\"0.6\" stroke=\"#4b5563\" stroke-width=\"1\"/>\n";
    }

    // Trayectoria volada.
    std::vector<Punto> path;
    for (const RoutePoint& p : route.path) path.push_back(p.point);
    out << "<polyline class=\"route\" points=\"";
    writePoints(out, canvas, path);
    out << "\" fill=\"none\" stroke=\"#2563eb\" stroke-width=\"1.5\" stroke-linejoin=\"round\" opacity=\"0.9\"/>\n";

    // Clientes: círculo si se atiende, cruz roja si se excluyó.
    std::vector<std::string> excluded;
    for (const ExcludedClient& client : route.excluded) excluded.push_back(client.etiqueta);
    for (int c = 0; c < escenario.numClientes(); ++c) {
        std::string etiqueta = "C" + std::to_string(c + 1);
        double x = canvas.x(escenario.clientes[c].x), y = canvas.y(escenario.clientes[c].y);
        if (std::find(excluded.begin(), excluded.end(), etiqueta) != excluded.end()) {
            writeCross(out, x, y, radius + 1, "class=\"excluded\"");
        } else {
            out << "<circle class=\"client\" cx=\"" << x << "\" cy=\"" << y << "\" r=\"" << radius
                << "\" fill=\"#111827\"/>\n";
        }
        if (withLabels) writeLabel(out, x, y, etiqueta);
    }

    // Estaciones (triángulos) y base (cuadrado).
    for (int s = 0; s < escenario.numEstaciones(); ++s) {
        double x = canvas.x(escenario.estaciones[s].x), y = canvas.y(escenario.estaciones[s].y);
        writeStation(out, x, y, radius + 3, "class=\"station\"");
        if (withLabels) writeLabel(out, x, y, "S" + std::to_string(s + 1));
    }
    double depotX = canvas.x(escenario.base.x), depotY = canvas.y(escenario.base.y);
    out << "<rect class=\"depot\" x=\"" << depotX - 6 << "\" y=\"" << depotY - 6
        << "\" width=\"12\" height=\"12\" fill=\"#dc2626\"/>\n";
    if (withLabels) writeLabel(out, depotX, depotY, "D0");

    // Leyenda (sin clases, para no confundirse con los elementos del mapa). Es una fila de pares
    // símbolo + texto: (lx, ly) es la posición del símbolo actual, y legendText escribe el texto a
    // su derecha y corre lx hacia el siguiente par (unos 8 px por letra, más un espacio).
    double ly = height - 14;
    double lx = margin;
    auto legendText = [&](const std::string& text) {
        out << "<text x=\"" << lx + 14 << "\" y=\"" << ly + 4 << "\" font-family=\"sans-serif\" font-size=\"12\" "
            << "fill=\"#374151\">" << text << "</text>\n";
        lx += 14 + 8.0 * static_cast<double>(text.size()) + 18;
    };
    out << "<rect x=\"" << lx - 5 << "\" y=\"" << ly - 5 << "\" width=\"10\" height=\"10\" fill=\"#dc2626\"/>\n";
    legendText("Base");
    writeStation(out, lx, ly, 6, "");
    legendText("Estacion");
    out << "<circle cx=\"" << lx << "\" cy=\"" << ly << "\" r=\"4\" fill=\"#111827\"/>\n";
    legendText("Cliente");
    writeCross(out, lx, ly, 5, "");
    legendText("Excluido");
    out << "<line x1=\"" << lx - 7 << "\" y1=\"" << ly << "\" x2=\"" << lx + 7 << "\" y2=\"" << ly
        << "\" stroke=\"#2563eb\" stroke-width=\"2\"/>\n";
    legendText("Ruta");
    out << "<rect x=\"" << lx - 6 << "\" y=\"" << ly - 6 << "\" width=\"12\" height=\"12\" fill=\"#9ca3af\" "
        << "stroke=\"#4b5563\"/>\n";
    legendText("Obstaculo");

    out << "</svg>\n";
}

void saveSvg(const Escenario& escenario, const Route& route, const std::string& path) {
    std::ofstream file = ArchivoEscenario::abrirParaEscribir(path);
    writeSvg(escenario, route, file);
    if (!file) throw std::runtime_error("error al escribir el archivo " + path);
}

} // namespace Exporter
} // namespace geodrone
