#include "ScenarioIO.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace geodrone {
namespace ScenarioIO {

namespace {

// Valor leído del archivo junto con la línea donde aparece, para dar errores útiles.
struct Token {
    std::string text;
    int line;
};

// Lector secuencial de valores. El formato es una lista fija de etiquetas y números,
// así que basta con consumirlos en orden y validar cada uno al leerlo.
class TokenReader {
public:
    explicit TokenReader(std::istream& in) {
        std::string line;
        int lineNumber = 0;
        while (std::getline(in, line)) {
            ++lineNumber;
            // El Bloc de notas puede guardar "UTF-8 con BOM": tres bytes invisibles al inicio
            // que, si no se quitan, se pegan a la primera etiqueta.
            if (lineNumber == 1 && line.compare(0, 3, "\xEF\xBB\xBF") == 0) {
                line.erase(0, 3);
            }
            std::istringstream words(line);
            std::string word;
            while (words >> word) {
                tokens_.push_back({word, lineNumber});
            }
        }
    }

    void expectLabel(const std::string& label) {
        const Token& token = next("la etiqueta " + label);
        if (token.text != label) {
            failAt(token.line, "se esperaba la etiqueta " + label + " y se encontro '" + token.text + "'");
        }
    }

    std::string readWord(const std::string& what) {
        return next(what).text;
    }

    double readNumber(const std::string& what) {
        const Token& token = next(what);
        double value = 0.0;
        if (!parseDouble(token.text, value)) {
            failAt(token.line, "se esperaba " + what + " (un numero) y se encontro '" + token.text + "'");
        }
        return value;
    }

    // Cantidad de elementos de un bloque: entero no negativo.
    int readCount(const std::string& what) {
        double value = readNumber(what);
        if (value < 0 || value != std::floor(value) || value > 1e7) {
            failAtLast(what + " debe ser un entero no negativo");
        }
        return static_cast<int>(value);
    }

    uint64_t readSeed() {
        const Token& token = next("la semilla");
        bool valid = !token.text.empty() && token.text.find_first_not_of("0123456789") == std::string::npos;
        if (!valid) {
            failAt(token.line, "la semilla debe ser un entero no negativo y se encontro '" + token.text + "'");
        }
        try {
            return std::stoull(token.text);
        } catch (const std::exception&) {
            failAt(token.line, "la semilla '" + token.text + "' es demasiado grande");
        }
    }

    // ofWhat completa el mensaje de error: "de la base", "del cliente 2", ...
    Point readPoint(const std::string& ofWhat) {
        double x = readNumber("la coordenada x " + ofWhat);
        double y = readNumber("la coordenada y " + ofWhat);
        return Point(x, y);
    }

    // Tras los obstáculos no debe quedar nada: así se detectan cantidades mal escritas.
    void expectEnd() {
        if (pos_ < tokens_.size()) {
            failAt(tokens_[pos_].line, "contenido inesperado '" + tokens_[pos_].text + "' despues de los obstaculos");
        }
    }

    [[noreturn]] void failAtLast(const std::string& message) const {
        failAt(pos_ == 0 ? 0 : tokens_[pos_ - 1].line, message);
    }

private:
    std::vector<Token> tokens_;
    size_t pos_ = 0;

    const Token& next(const std::string& what) {
        if (pos_ >= tokens_.size()) {
            int lastLine = tokens_.empty() ? 0 : tokens_.back().line;
            failAt(lastLine, "el archivo termina antes de " + what);
        }
        return tokens_[pos_++];
    }

    static bool parseDouble(const std::string& text, double& value) {
        try {
            size_t used = 0;
            value = std::stod(text, &used);
            return used == text.size() && std::isfinite(value);
        } catch (const std::exception&) {
            return false;
        }
    }

    [[noreturn]] static void failAt(int line, const std::string& message) {
        throw std::runtime_error("linea " + std::to_string(line) + ": " + message);
    }
};

// Bloque "ETIQUETA cantidad" seguido de un punto por elemento (estaciones o clientes).
// pluralName y ofItem solo sirven para los mensajes de error ("clientes", "del cliente").
std::vector<Point> readPointBlock(TokenReader& reader, const std::string& label,
                                  const std::string& pluralName, const std::string& ofItem) {
    reader.expectLabel(label);
    int count = reader.readCount("la cantidad de " + pluralName);
    std::vector<Point> points;
    points.reserve(count);
    for (int i = 0; i < count; ++i) {
        points.push_back(reader.readPoint(ofItem + " " + std::to_string(i + 1)));
    }
    return points;
}

// El nombre se lee como una sola palabra: con espacios el archivo no se podría volver a leer.
void checkName(const std::string& name) {
    bool valid = !name.empty() &&
                 std::none_of(name.begin(), name.end(), [](unsigned char c) { return std::isspace(c) != 0; });
    if (!valid) {
        throw std::invalid_argument("el nombre del escenario no puede estar vacio ni tener espacios: '" + name + "'");
    }
}

std::vector<Polygon> readObstacles(TokenReader& reader) {
    reader.expectLabel("OBSTACLES");
    int count = reader.readCount("la cantidad de obstaculos");
    std::vector<Polygon> obstacles;
    obstacles.reserve(count);
    for (int i = 0; i < count; ++i) {
        std::string number = std::to_string(i + 1);
        int vertexCount = reader.readCount("la cantidad de vertices del obstaculo " + number);
        if (vertexCount < 3) {
            reader.failAtLast("el obstaculo " + number + " necesita al menos 3 vertices");
        }
        std::vector<Point> vertices;
        vertices.reserve(vertexCount);
        for (int v = 0; v < vertexCount; ++v) {
            vertices.push_back(reader.readPoint("del vertice " + std::to_string(v + 1) + " del obstaculo " + number));
        }
        obstacles.emplace_back(i + 1, vertices, "Obs" + std::to_string(i + 1));
    }
    return obstacles;
}

} // namespace

Scenario read(std::istream& in) {
    TokenReader reader(in);
    Scenario scenario;

    reader.expectLabel("NAME");
    scenario.name = reader.readWord("el nombre del escenario");

    reader.expectLabel("CATEGORY");
    std::string categoryText = reader.readWord("la categoria");
    if (!parseCategory(categoryText, scenario.category)) {
        reader.failAtLast("categoria desconocida '" + categoryText +
                          "' (se espera SMALL, MEDIUM, LARGE, EXTREME o ADVERSARIAL)");
    }

    reader.expectLabel("SEED");
    scenario.seed = reader.readSeed();

    reader.expectLabel("AREA");
    scenario.areaWidth = reader.readNumber("el ancho del area");
    scenario.areaHeight = reader.readNumber("el alto del area");
    if (scenario.areaWidth <= 0 || scenario.areaHeight <= 0) {
        reader.failAtLast("las dimensiones del area deben ser positivas");
    }

    reader.expectLabel("BATTERY");
    scenario.battery = reader.readNumber("la autonomia de la bateria");
    if (scenario.battery <= 0) {
        reader.failAtLast("la autonomia de la bateria debe ser positiva");
    }

    reader.expectLabel("DEPOT");
    scenario.depot = reader.readPoint("de la base");

    scenario.stations = readPointBlock(reader, "STATIONS", "estaciones", "de la estacion");
    scenario.clients = readPointBlock(reader, "CLIENTS", "clientes", "del cliente");
    scenario.obstacles = readObstacles(reader);
    reader.expectEnd();

    return scenario;
}

Scenario load(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("no se pudo abrir el archivo " + path);
    }
    try {
        return read(file);
    } catch (const std::runtime_error& error) {
        throw std::runtime_error(path + ", " + error.what());
    }
}

void write(const Scenario& scenario, std::ostream& out) {
    // 15 cifras significativas: los enteros se escriben sin decimales y los reales sin pérdida visible.
    checkName(scenario.name);

    out << std::setprecision(15);
    out << "NAME " << scenario.name << '\n';
    out << "CATEGORY " << categoryName(scenario.category) << '\n';
    out << "SEED " << scenario.seed << '\n';
    out << "AREA " << scenario.areaWidth << ' ' << scenario.areaHeight << '\n';
    out << "BATTERY " << scenario.battery << '\n';
    out << "DEPOT " << scenario.depot.x << ' ' << scenario.depot.y << '\n';

    out << "STATIONS " << scenario.stations.size() << '\n';
    for (const Point& p : scenario.stations) {
        out << p.x << ' ' << p.y << '\n';
    }

    out << "CLIENTS " << scenario.clients.size() << '\n';
    for (const Point& p : scenario.clients) {
        out << p.x << ' ' << p.y << '\n';
    }

    out << "OBSTACLES " << scenario.obstacles.size() << '\n';
    for (const Polygon& poly : scenario.obstacles) {
        out << poly.size();
        for (const Point& v : poly.vertices) {
            out << "  " << v.x << ' ' << v.y;
        }
        out << '\n';
    }
}

void save(const Scenario& scenario, const std::string& path) {
    checkName(scenario.name);   // Antes de crear el archivo, para no dejarlo vacío
    std::filesystem::path filePath(path);
    if (filePath.has_parent_path()) {
        std::filesystem::create_directories(filePath.parent_path());
    }
    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("no se pudo crear el archivo " + path);
    }
    write(scenario, file);
    if (!file) {
        throw std::runtime_error("error al escribir el archivo " + path);
    }
}

} // namespace ScenarioIO
} // namespace geodrone
