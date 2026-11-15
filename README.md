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

## 2. Estado actual

| Estado | Módulos |
| --- | --- |
| Implementado y probado | Geometría (orientación, intersección de segmentos, punto en polígono, validación de trayectorias), grafo de visibilidad, Dijkstra y matriz de distancias entre nodos clave, lectura y escritura de escenarios en TXT, casos adversariales hechos a mano, envolvente convexa y generador de datasets por semilla |
| Solo declarado, sin implementación | Solver exacto, solver heurístico |
| Pendiente | Gestión de recargas, búsqueda binaria de autonomía, DFS y puntos de articulación, árbol de expansión mínima, juez y benchmark, exportación de rutas y resultados |

Las decisiones de diseño y el alcance están en [`docs/decisiones.md`](docs/decisiones.md).

---

## 3. Arquitectura del Repositorio

```text
GeoDroneChallenge/
├── CMakeLists.txt              # Configuración modular de compilación C++17 (-O3, CTest)
├── .gitignore                  # Exclusiones de build, binarios y editores
├── README.md                   # Documentación técnica del sistema
├── docs/
│   └── decisiones.md           # Decisiones de diseño, alcance y pendientes
├── data/
│   └── adversarial/            # Casos adversariales hechos a mano (ver su README)
├── src/
│   ├── geometry/               # Módulo de Geometría Computacional Propia
│   │   ├── Point.hpp           # Vector 2D, aritmética, producto cruz, dot, dist, EPS
│   │   ├── Segment.hpp         # Segmentos, longitud, midpoint, test de pertenencia
│   │   ├── Polygon.hpp         # Polígonos de exclusión, aristas, área Shoelace
│   │   ├── GeometryUtils.hpp   # Declaraciones de CCW, intersección y Ray Casting
│   │   ├── GeometryUtils.cpp   # Implementaciones optimizadas sin dependencias externas
│   │   └── ConvexHull.hpp      # Envolvente convexa (monotone chain)
│   ├── graph/                  # Módulo de Grafos y Navegación
│   │   ├── VisibilityGraph.hpp # Construcción del Grafo de Visibilidad G = (V, E)
│   │   └── Dijkstra.hpp        # SSSP con std::priority_queue y Matriz Reducida
│   ├── scenario/               # Escenarios de misión
│   │   ├── Scenario.hpp        # Escenario, categorías y límites competitivos
│   │   ├── ScenarioIO.hpp      # Lectura y escritura del TXT con etiquetas en orden fijo
│   │   ├── ScenarioIO.cpp
│   │   ├── DatasetGenerator.hpp# Generador reproducible por semilla (mt19937_64)
│   │   └── DatasetGenerator.cpp
│   ├── solvers/                # Motores de optimización (solo declaraciones por ahora)
│   │   ├── ExactSolver.hpp     # Branch & Bound con cota MST (Small)
│   │   └── HeuristicSolver.hpp # Nearest Neighbor + 2-Opt (Medium-Extreme)
│   └── main.cpp                # Demostración del grafo de visibilidad y Dijkstra
└── tests/
    ├── test_geometry.cpp       # Pruebas de geometría, trayectorias y envolvente convexa
    ├── test_scenario.cpp       # Pruebas de lectura, escritura y errores de escenarios
    └── test_generator.cpp      # Pruebas de reproducibilidad y validez del generador
```

---

## 4. Restricciones y Prácticas Competitivas Implementadas

1. **Cero Dependencias Externas:** Exclusivamente la STL estándar de C++ (`<vector>`, `<queue>`, `<cmath>`, `<algorithm>`, `<chrono>`, `<random>`).
2. **Precisión Numérica y Robustez:** Aritmética de punto flotante de doble precisión (`double`) gobernada por una tolerancia épsilon `EPS = 1e-9`. Los datasets usan coordenadas enteras en un rango acotado, de modo que los productos cruz se calculan sin error de redondeo.
3. **I/O Rápido:**
   ```cpp
   std::ios_base::sync_with_stdio(false);
   std::cin.tie(NULL);
   ```
4. **Reproducibilidad:** Generador determinista basado en `std::mt19937_64` con semillas fijadas por parámetro.
5. **Categorías de Datasets (límites tentativos, se ajustarán tras medir tiempos reales):**
   - **Small:** $1 \le N \le 10$, $1 \le S \le 3$, $1 \le M \le 5$, $\sum |P_k| \le 30$ | TL: 1.0s, ML: 256 MB.
   - **Medium:** $11 \le N \le 40$, $3 \le S \le 6$, $5 \le M \le 15$, $\sum |P_k| \le 100$ | TL: 2.0s, ML: 256 MB.
   - **Large:** $41 \le N \le 200$, $6 \le S \le 12$, $15 \le M \le 50$, $\sum |P_k| \le 400$ | TL: 3.0s, ML: 512 MB.
   - **Extreme:** $201 \le N \le 1000$, $12 \le S \le 25$, $50 \le M \le 150$, $\sum |P_k| \le 1500$ | TL: 5.0s, ML: 512 MB.

   $N$ = clientes, $S$ = estaciones, $M$ = obstáculos, $\sum |P_k|$ = total de vértices de obstáculos.

---

## 5. Compilación y Ejecución

### Prerrequisitos
- Compilador con soporte C++17 o superior (GCC, Clang o MSVC).
- CMake $\ge 3.16$.

### Instrucciones de Compilación
```bash
# 1. Configurar y compilar (Release, -O3)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

# 2. Ejecutar las pruebas unitarias (los assert siguen activos en Release)
ctest --test-dir build --output-on-failure

# 3. Ejecutar el programa principal
./build/geodrone
```
