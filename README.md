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
| Implementado y probado | Geometría (orientación, intersección de segmentos, punto en polígono, validación de trayectorias), envolvente convexa, lectura y escritura de escenarios en TXT, casos adversariales hechos a mano, generador de datasets por semilla, grafo de misión (grafo de visibilidad con filtro por caja envolvente, matriz de distancias con Dijkstra, caminos reales y DFS de conectividad), red de recarga (Prim, puntos de articulación con Tarjan, búsqueda binaria de la autonomía mínima), inserción de recargas, solver exacto (branch and bound con cota de árbol de expansión mínima) y heurísticas (vecino más cercano, 2-aproximación, 2-opt y búsqueda local iterada) |
| Pendiente | Juez y benchmark, exportación de rutas y resultados, menú interactivo |

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
│   │   ├── Dijkstra.hpp        # SSSP con std::priority_queue y Matriz Reducida
│   │   ├── MissionGraph.hpp    # Grafo de misión: matriz, caminos reales y DFS desde la base
│   │   ├── MissionGraph.cpp
│   │   ├── Mst.hpp             # Árbol de expansión mínima (Prim sobre matriz)
│   │   ├── ChargingNetwork.hpp # Red de recarga: Tarjan, red mínima y autonomía mínima
│   │   └── ChargingNetwork.cpp
│   ├── scenario/               # Escenarios de misión
│   │   ├── Scenario.hpp        # Escenario, categorías y límites competitivos
│   │   ├── ScenarioIO.hpp      # Lectura y escritura del TXT con etiquetas en orden fijo
│   │   ├── ScenarioIO.cpp
│   │   ├── DatasetGenerator.hpp# Generador reproducible por semilla (mt19937_64)
│   │   └── DatasetGenerator.cpp
│   ├── solvers/                # Motores de optimización
│   │   ├── ExactSolver.hpp     # Branch & Bound con cota MST (Small, hasta 12 clientes)
│   │   ├── ExactSolver.cpp
│   │   ├── HeuristicSolver.hpp # Vecino más cercano, 2-aproximación, 2-opt y búsqueda local iterada
│   │   ├── HeuristicSolver.cpp
│   │   ├── RechargePlanner.hpp # Inserción de recargas sobre un orden de clientes
│   │   └── RechargePlanner.cpp
│   └── main.cpp                # Demostración del grafo de visibilidad y Dijkstra
└── tests/
    ├── test_geometry.cpp       # Pruebas de geometría, trayectorias y envolvente convexa
    ├── test_scenario.cpp       # Pruebas de lectura, escritura y errores de escenarios
    ├── test_generator.cpp      # Pruebas de reproducibilidad y validez del generador
    ├── test_graph.cpp          # Pruebas del grafo, Dijkstra, caminos y DFS
    ├── test_network.cpp        # Pruebas de Prim, Tarjan, búsqueda binaria y recargas
    └── test_solvers.cpp        # Pruebas del exacto contra fuerza bruta y de las heurísticas
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
5. **Categorías de Datasets (límites confirmados con mediciones, ver `docs/decisiones.md` §7):**
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

Para depurar (por ejemplo en CLion o VS Code), compilar en Debug, que no optimiza:

```bash
cmake -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug
```

Si no se indica `CMAKE_BUILD_TYPE`, se usa Release.
