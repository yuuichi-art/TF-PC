# GeoDrone Challenge: Optimización Competitiva de Rutas de Entrega

**Universidad San Ignacio de Loyola (USIL)**  
**Facultad de Ingeniería e Inteligencia Artificial**  
*Curso:* Programación Competitiva (2026)  
*Docente:* Jomark Pablo Noriega Zapata  
*Grupo 4:*
- Caso Egaovil, Pedro Yuichi (2411176)
- Fernández Totocayo, Henry Daniel (2412247)
- Patiño Quispe, Renato Enrique (2410049)
- Chanca Melgar, Yarel Joshua (2411997)

---

## 1. Descripción del Proyecto

El **GeoDrone Challenge** modela y optimiza misiones logísticas de última milla operadas por vehículos aéreos no tripulados (drones autónomos) dentro de un contexto urbano restringido en el plano bidimensional $\mathbb{R}^2$.

El sistema resuelve el ruteo sin colisiones sorteando polígonos de exclusión $P = \{P_1, P_2, \dots, P_m\}$, gestionando autonomía finita de batería ($B_{\max}$) con estaciones de recarga rápida $S$, y garantizando visita única a clientes $C$ partiendo y retornando al depósito central $D$.

---

## 2. Arquitectura del Repositorio

```text
GeoDroneChallenge/
├── CMakeLists.txt              # Configuración modular de compilación C++17 (-O3, CTest)
├── .gitignore                  # Exclusiones de build, binarios y editores
├── README.md                   # Documentación técnica del sistema
├── src/
│   ├── geometry/               # Módulo de Geometría Computacional Propia
│   │   ├── Point.hpp           # Vector 2D, aritmética, producto cruz, dot, dist, EPS
│   │   ├── Segment.hpp         # Segmentos, longitud, midpoint, test de pertenencia
│   │   ├── Polygon.hpp         # Polígonos de exclusión, aristas, área Shoelace
│   │   ├── GeometryUtils.hpp   # Declaraciones de CCW, intersección y Ray Casting
│   │   └── GeometryUtils.cpp   # Implementaciones optimizadas sin dependencias externas
│   ├── graph/                  # Módulo de Grafos y Navegación
│   │   ├── VisibilityGraph.hpp # Construcción del Grafo de Visibilidad G = (V, E)
│   │   └── Dijkstra.hpp        # SSSP con std::priority_queue y Matriz Reducida
│   ├── solvers/                # Motores de Optimización (Preparación Avances 2 y 3)
│   │   ├── ExactSolver.hpp     # Branch & Bound con poda MST / DP Bitmask (Small)
│   │   └── HeuristicSolver.hpp # Nearest Neighbor + 2-Opt local search (Medium-Extreme)
│   ├── judge/                  # Entorno de Evaluación y Benchmarking
│   │   └── DatasetGenerator.hpp# Generador pseudoaleatorio reproducible (mt19937_64)
│   └── main.cpp                # CLI interactivo, validación y demostración en vivo
└── tests/
    └── test_geometry.cpp       # Suite exhaustiva de pruebas unitarias con assert()
```

---

## 3. Restricciones y Prácticas Competitivas Implementadas

1. **Cero Dependencias Externas:** Exclusivamente la STL estándar de C++ (`<vector>`, `<queue>`, `<cmath>`, `<algorithm>`, `<chrono>`, `<random>`).
2. **Precisión Numérica y Robustez:** Aritmética de punto flotante de doble precisión (`double`) gobernada por una tolerancia épsilon `EPS = 1e-9`.
3. **I/O Rápido:**
   ```cpp
   std::ios_base::sync_with_stdio(false);
   std::cin.tie(NULL);
   ```
4. **Reproducibilidad:** Generador determinista basado en `std::mt19937_64` con semillas fijadas por parámetro.
5. **Categorías de Datasets (Límites de Evaluación):**
   - **Small:** $1 \le N \le 10$, $1 \le M \le 5$, $\sum |P_k| \le 30$ | TL: 1.0s, ML: 256 MB.
   - **Medium:** $11 \le N \le 40$, $5 \le M \le 15$, $\sum |P_k| \le 100$ | TL: 2.0s, ML: 256 MB.
   - **Large:** $41 \le N \le 200$, $15 \le M \le 50$, $\sum |P_k| \le 400$ | TL: 3.0s, ML: 512 MB.
   - **Extreme:** $201 \le N \le 1000$, $50 \le M \le 150$, $\sum |P_k| \le 1500$ | TL: 5.0s, ML: 512 MB.

---

## 4. Compilación y Ejecución

### Prerrequisitos
- Compilador con soporte C++17 o superior (GCC, Clang o MSVC).
- CMake $\ge 3.16$.

### Instrucciones de Compilación (Release -O3)
```bash
# 1. Configuración de CMake
cmake -B build -DCMAKE_BUILD_TYPE=Release

# 2. Compilar todos los ejecutables
cmake --build build --config Release

# 3. Ejecutar las pruebas unitarias geométricas
./build/test_geometry

# O mediante CTest:
ctest --test-dir build --output-on-failure

# 4. Ejecutar el programa principal
./build/geodrone
```
