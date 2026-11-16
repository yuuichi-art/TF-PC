#include "Exportador.hpp"
#include "../escenario/ArchivoEscenario.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace geodrone {
namespace Exportador {

void escribirRuta(const Ruta& ruta, std::ostream& salida) {
    salida << "SCENARIO " << ruta.escenario << '\n';
    salida << "ALGORITHM " << ruta.algoritmo << '\n';
    salida << "STATUS " << (ruta.factible ? "OK" : "INFEASIBLE") << '\n';
    // Seis decimales: suficientes para que el juez compare el costo con el que recalcula.
    salida << "COST " << std::fixed << std::setprecision(6) << ruta.costo << '\n';
    salida << "RECHARGES " << ruta.recargas << '\n';

    salida << "EXCLUDED " << ruta.excluidos.size() << '\n';
    for (const ClienteExcluido& cliente : ruta.excluidos) {
        salida << cliente.etiqueta << ' ' << cliente.motivo << '\n';
    }

    salida << "ORDER " << ruta.orden.size() << '\n';
    for (size_t i = 0; i < ruta.orden.size(); ++i) {
        salida << (i > 0 ? " " : "") << ruta.orden[i];
    }
    salida << '\n';

    // Coordenadas como en los escenarios: los enteros sin decimales.
    salida << std::defaultfloat << std::setprecision(15);
    salida << "PATH " << ruta.trayectoria.size() << '\n';
    for (const PuntoRuta& p : ruta.trayectoria) {
        salida << p.punto.x << ' ' << p.punto.y << ' ' << p.etiqueta << '\n';
    }
}

void guardarRuta(const Ruta& ruta, const std::string& nombreArchivo) {
    std::ofstream archivo = ArchivoEscenario::abrirParaEscribir(nombreArchivo);
    escribirRuta(ruta, archivo);
    if (!archivo) {
        throw std::runtime_error("error al escribir el archivo " + nombreArchivo);
    }
}

void escribirResultadosCsv(const std::vector<FilaJuez>& filas, std::ostream& salida) {
    salida << "escenario,categoria,clientes,algoritmo,veredicto,costo,recargas,excluidos,tiempo_ms,memoria_mb,"
           "gap_mejor,gap_cota\n";
    salida << std::fixed << std::setprecision(3);
    for (const FilaJuez& fila : filas) {
        salida << fila.escenario << ',' << nombreCategoria(fila.categoria) << ',' << fila.clientes << ',' << fila.algoritmo << ','
            << fila.veredicto << ',' << fila.costo << ',' << fila.recargas << ',' << fila.excluidos << ',' << fila.tiempoMs
            << ',' << fila.memoriaMB << ',';
        if (!std::isnan(fila.gapMejor)) salida << fila.gapMejor;
        salida << ',';
        if (!std::isnan(fila.gapCota)) salida << fila.gapCota;
        salida << '\n';
    }
}

void guardarResultadosCsv(const std::vector<FilaJuez>& filas, const std::string& nombreArchivo) {
    std::ofstream archivo = ArchivoEscenario::abrirParaEscribir(nombreArchivo);
    escribirResultadosCsv(filas, archivo);
    if (!archivo) throw std::runtime_error("error al escribir el archivo " + nombreArchivo);
}

void escribirRankingCsv(const std::vector<FilaRanking>& ranking, std::ostream& salida) {
    salida << "categoria,posicion,algoritmo,casos_ok,casos,gap_medio,tiempo_medio_ms\n";
    salida << std::fixed << std::setprecision(3);
    for (const FilaRanking& fila : ranking) {
        salida << nombreCategoria(fila.categoria) << ',' << fila.posicion << ',' << fila.algoritmo << ',' << fila.casosOk
            << ',' << fila.casos << ',' << fila.gapMedio << ',' << fila.tiempoMedioMs << '\n';
    }
}

void guardarRankingCsv(const std::vector<FilaRanking>& ranking, const std::string& nombreArchivo) {
    std::ofstream archivo = ArchivoEscenario::abrirParaEscribir(nombreArchivo);
    escribirRankingCsv(ranking, archivo);
    if (!archivo) throw std::runtime_error("error al escribir el archivo " + nombreArchivo);
}

namespace {

// Conversión de coordenadas del escenario a píxeles del dibujo.
struct Lienzo {
    double escala;
    double izquierda;
    double arriba;
    double altoArea;

    double x(double valor) const { return izquierda + valor * escala; }
    double y(double valor) const { return arriba + (altoArea - valor) * escala; }   // y crece hacia abajo en SVG
};

std::string escaparXml(const std::string& texto) {
    std::string resultado;
    for (char c : texto) {
        if (c == '&') resultado += "&amp;";
        else if (c == '<') resultado += "&lt;";
        else if (c == '>') resultado += "&gt;";
        else if (c == '"') resultado += "&quot;";
        else resultado += c;
    }
    return resultado;
}

void escribirPuntos(std::ostream& salida, const Lienzo& lienzo, const std::vector<Punto>& puntos) {
    for (size_t i = 0; i < puntos.size(); ++i) {
        salida << (i > 0 ? " " : "") << lienzo.x(puntos[i].x) << ',' << lienzo.y(puntos[i].y);
    }
}

void dibujarEstacion(std::ostream& salida, double x, double y, double tamano, const std::string& atributos) {
    salida << "<polygon " << atributos << " points=\"" << x << ',' << y - tamano * 1.2 << ' ' << x - tamano << ','
        << y + tamano * 0.7 << ' ' << x + tamano << ',' << y + tamano * 0.7 << "\" fill=\"#16a34a\"/>\n";
}

void dibujarCruz(std::ostream& salida, double x, double y, double tamano, const std::string& atributos) {
    salida << "<path " << atributos << " d=\"M" << x - tamano << ',' << y - tamano << " L" << x + tamano << ',' << y + tamano
        << " M" << x - tamano << ',' << y + tamano << " L" << x + tamano << ',' << y - tamano
        << "\" stroke=\"#dc2626\" stroke-width=\"2\"/>\n";
}

void escribirEtiqueta(std::ostream& salida, double x, double y, const std::string& texto) {
    salida << "<text x=\"" << x + 6 << "\" y=\"" << y - 6 << "\" font-family=\"sans-serif\" font-size=\"10\" "
        << "fill=\"#374151\">" << escaparXml(texto) << "</text>\n";
}

} // namespace

void escribirSvg(const Escenario& escenario, const Ruta& ruta, std::ostream& salida) {
    const double tamanoMapa = 800.0, margen = 20.0, encabezado = 40.0, pie = 34.0;
    double escala = tamanoMapa / std::max(escenario.anchoArea, escenario.altoArea);
    Lienzo lienzo{escala, margen, encabezado, escenario.altoArea};
    double ancho = escenario.anchoArea * escala + 2 * margen;
    double alto = encabezado + escenario.altoArea * escala + margen + pie;

    // Con muchos clientes los puntos se achican y se omiten las etiquetas para que se lea.
    bool conEtiquetas = escenario.numClientes() <= 40;
    double radio = escenario.numClientes() > 200 ? 2.5 : 4.0;

    salida << std::fixed << std::setprecision(2);
    salida << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << ancho << "\" height=\"" << alto
        << "\" viewBox=\"0 0 " << ancho << ' ' << alto << "\">\n";
    salida << "<rect width=\"" << ancho << "\" height=\"" << alto << "\" fill=\"#ffffff\"/>\n";
    salida << "<text x=\"" << margen << "\" y=\"26\" font-family=\"sans-serif\" font-size=\"15\" fill=\"#111827\">"
        << "<tspan font-weight=\"bold\">" << escaparXml(ruta.escenario) << "</tspan>  " << escaparXml(ruta.algoritmo)
        << "  |  costo " << ruta.costo << "  |  recargas " << ruta.recargas << "  |  excluidos "
        << ruta.excluidos.size() << (ruta.factible ? "" : "  |  INFACTIBLE") << "</text>\n";

    // Área del escenario y obstáculos.
    salida << "<rect class=\"area\" x=\"" << lienzo.x(0) << "\" y=\"" << lienzo.y(escenario.altoArea) << "\" width=\""
        << escenario.anchoArea * escala << "\" height=\"" << escenario.altoArea * escala
        << "\" fill=\"#f9fafb\" stroke=\"#d1d5db\"/>\n";
    for (const Poligono& poligono : escenario.obstaculos) {
        salida << "<polygon class=\"obstacle\" points=\"";
        escribirPuntos(salida, lienzo, poligono.vertices);
        salida << "\" fill=\"#9ca3af\" fill-opacity=\"0.6\" stroke=\"#4b5563\" stroke-width=\"1\"/>\n";
    }

    // Trayectoria volada.
    std::vector<Punto> puntosRuta;
    for (const PuntoRuta& p : ruta.trayectoria) puntosRuta.push_back(p.punto);
    salida << "<polyline class=\"route\" points=\"";
    escribirPuntos(salida, lienzo, puntosRuta);
    salida << "\" fill=\"none\" stroke=\"#2563eb\" stroke-width=\"1.5\" stroke-linejoin=\"round\" opacity=\"0.9\"/>\n";

    // Clientes: círculo si se atiende, cruz roja si se excluyó.
    std::vector<std::string> excluidos;
    for (const ClienteExcluido& cliente : ruta.excluidos) excluidos.push_back(cliente.etiqueta);
    for (int c = 0; c < escenario.numClientes(); ++c) {
        std::string etiqueta = "C" + std::to_string(c + 1);
        double x = lienzo.x(escenario.clientes[c].x), y = lienzo.y(escenario.clientes[c].y);
        if (std::find(excluidos.begin(), excluidos.end(), etiqueta) != excluidos.end()) {
            dibujarCruz(salida, x, y, radio + 1, "class=\"excluded\"");
        } else {
            salida << "<circle class=\"client\" cx=\"" << x << "\" cy=\"" << y << "\" r=\"" << radio
                << "\" fill=\"#111827\"/>\n";
        }
        if (conEtiquetas) escribirEtiqueta(salida, x, y, etiqueta);
    }

    // Estaciones (triángulos) y base (cuadrado).
    for (int s = 0; s < escenario.numEstaciones(); ++s) {
        double x = lienzo.x(escenario.estaciones[s].x), y = lienzo.y(escenario.estaciones[s].y);
        dibujarEstacion(salida, x, y, radio + 3, "class=\"station\"");
        if (conEtiquetas) escribirEtiqueta(salida, x, y, "S" + std::to_string(s + 1));
    }
    double baseX = lienzo.x(escenario.base.x), baseY = lienzo.y(escenario.base.y);
    salida << "<rect class=\"depot\" x=\"" << baseX - 6 << "\" y=\"" << baseY - 6
        << "\" width=\"12\" height=\"12\" fill=\"#dc2626\"/>\n";
    if (conEtiquetas) escribirEtiqueta(salida, baseX, baseY, "D0");

    // Leyenda (sin clases, para no confundirse con los elementos del mapa). Es una fila de pares
    // símbolo + texto: (lx, ly) es la posición del símbolo actual, y textoLeyenda escribe el texto a
    // su derecha y corre lx hacia el siguiente par (unos 8 px por letra, más un espacio).
    double ly = alto - 14;
    double lx = margen;
    auto textoLeyenda = [&](const std::string& texto) {
        salida << "<text x=\"" << lx + 14 << "\" y=\"" << ly + 4 << "\" font-family=\"sans-serif\" font-size=\"12\" "
            << "fill=\"#374151\">" << texto << "</text>\n";
        lx += 14 + 8.0 * static_cast<double>(texto.size()) + 18;
    };
    salida << "<rect x=\"" << lx - 5 << "\" y=\"" << ly - 5 << "\" width=\"10\" height=\"10\" fill=\"#dc2626\"/>\n";
    textoLeyenda("Base");
    dibujarEstacion(salida, lx, ly, 6, "");
    textoLeyenda("Estacion");
    salida << "<circle cx=\"" << lx << "\" cy=\"" << ly << "\" r=\"4\" fill=\"#111827\"/>\n";
    textoLeyenda("Cliente");
    dibujarCruz(salida, lx, ly, 5, "");
    textoLeyenda("Excluido");
    salida << "<line x1=\"" << lx - 7 << "\" y1=\"" << ly << "\" x2=\"" << lx + 7 << "\" y2=\"" << ly
        << "\" stroke=\"#2563eb\" stroke-width=\"2\"/>\n";
    textoLeyenda("Ruta");
    salida << "<rect x=\"" << lx - 6 << "\" y=\"" << ly - 6 << "\" width=\"12\" height=\"12\" fill=\"#9ca3af\" "
        << "stroke=\"#4b5563\"/>\n";
    textoLeyenda("Obstaculo");

    salida << "</svg>\n";
}

void guardarSvg(const Escenario& escenario, const Ruta& ruta, const std::string& nombreArchivo) {
    std::ofstream archivo = ArchivoEscenario::abrirParaEscribir(nombreArchivo);
    escribirSvg(escenario, ruta, archivo);
    if (!archivo) throw std::runtime_error("error al escribir el archivo " + nombreArchivo);
}

} // namespace Exportador
} // namespace geodrone
