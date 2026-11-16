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
| Implementado y probado | Geometría (orientación, intersección de segmentos, punto en polígono, validación de trayectorias), envolvente convexa, lectura y escritura de escenarios en TXT, casos adversariales hechos a mano, generador de datasets por semilla, grafo de misión (grafo de visibilidad con filtro por caja envolvente, matriz de distancias con Dijkstra, caminos reales y DFS de conectividad), red de recarga (Prim, puntos de articulación con Tarjan, búsqueda binaria de la autonomía mínima), inserción de recargas, solver exacto (branch and bound con cota de árbol de expansión mínima) y heurísticas (vecino más cercano, 2-aproximación, 2-opt y búsqueda local iterada), resolución completa con exclusiones y trayectoria real, juez y benchmark (verificación independiente, veredictos OK/WA/TLE/MLE, gap, ranking), exportación de rutas en TXT y de resultados y ranking en CSV, dibujo de rutas en SVG y menú interactivo |
| Pendiente | Nada: los 15 requerimientos funcionales están implementados (ver la tabla de `docs/decisiones.md` §3) |

Las decisiones de diseño y el alcance están en [`docs/decisiones.md`](docs/decisiones.md).

**Resultados del benchmark final** ([`docs/resultados/`](docs/resultados/README.md)): 26 escenarios y 140 ejecuciones, todas OK. Incluye el peor caso Extreme (1000 clientes), que se resuelve en 4,50 s de 5 con 33 MB de memoria. La búsqueda local iterada queda primera en todas las categorías, con un gap medio menor a 1 %.

---

## 3. Arquitectura del Repositorio

```text
TF-PC/
├── CMakeLists.txt                    # Configuración modular de compilación C++17 (-O3, CTest)
├── .gitignore                        # Exclusiones de build, binarios y editores
├── README.md                         # Documentación técnica del sistema
├── docs/
│   ├── decisiones.md                 # Decisiones de diseño, alcance y pendientes
│   ├── glosario.md                   # Nombres del código: inglés anterior y español actual
│   └── resultados/                   # Benchmark final: CSV, ranking, rutas de ejemplo y hallazgos
├── data/
│   └── adversarial/                  # Casos adversariales hechos a mano (ver su README)
├── src/
│   ├── geometria/                    # Geometría computacional propia
│   │   ├── Punto.hpp                 # Punto 2D: aritmética, producto cruz, distancia, EPS e INF
│   │   ├── Segmento.hpp              # Segmentos, longitud y punto medio
│   │   ├── Poligono.hpp              # Polígonos de exclusión, aristas, área Shoelace
│   │   ├── Geometria.hpp             # Declaraciones de CCW, intersección y Ray Casting
│   │   ├── Geometria.cpp             # Implementación, incluida la validación de tramos (RF04)
│   │   └── EnvolventeConvexa.hpp     # Envolvente convexa (monotone chain)
│   ├── grafo/                        # Grafos y navegación
│   │   ├── GrafoVisibilidad.hpp      # Construcción del grafo de visibilidad G = (V, E)
│   │   ├── Dijkstra.hpp              # Dijkstra desde un origen con cola de prioridad
│   │   ├── GrafoMision.hpp           # Grafo de misión: matriz, caminos reales y DFS desde la base
│   │   ├── GrafoMision.cpp
│   │   ├── ArbolMinimo.hpp           # Árbol de expansión mínima (Prim sobre matriz)
│   │   ├── RedRecarga.hpp            # Red de recarga: Tarjan, red mínima y autonomía mínima
│   │   └── RedRecarga.cpp
│   ├── escenario/                    # Escenarios de misión
│   │   ├── Escenario.hpp             # Escenario, categorías y límites competitivos
│   │   ├── ArchivoEscenario.hpp      # Lectura y escritura del TXT con etiquetas en orden fijo
│   │   ├── ArchivoEscenario.cpp
│   │   ├── Aleatorio.hpp             # Entero aleatorio reproducible con cualquier compilador
│   │   ├── GeneradorDatasets.hpp     # Generador reproducible por semilla (mt19937_64)
│   │   └── GeneradorDatasets.cpp
│   ├── solvers/                      # Motores de optimización
│   │   ├── SolverExacto.hpp          # Branch and bound con cota MST (Small, hasta 12 clientes)
│   │   ├── SolverExacto.cpp
│   │   ├── SolverHeuristico.hpp      # Vecino más cercano, 2-aproximación, 2-opt y búsqueda local iterada
│   │   ├── SolverHeuristico.cpp
│   │   ├── PlanificadorRecargas.hpp  # Inserción de recargas sobre un orden de clientes
│   │   ├── PlanificadorRecargas.cpp
│   │   ├── Ruta.hpp                  # Ruta final: orden, trayectoria, excluidos y tiempos
│   │   ├── SolverMision.hpp          # Resolver un escenario completo con el algoritmo elegido
│   │   └── SolverMision.cpp
│   ├── juez/                         # Juez y benchmark
│   │   ├── UsoMemoria.hpp            # Memoria pico del proceso (GetProcessMemoryInfo)
│   │   ├── UsoMemoria.cpp
│   │   ├── Juez.hpp                  # Verificación, veredictos, benchmark y ranking
│   │   └── Juez.cpp
│   ├── exportacion/                  # Exportación de resultados
│   │   ├── Exportador.hpp            # Ruta en TXT y SVG, resultados y ranking en CSV
│   │   └── Exportador.cpp
│   └── main.cpp                      # Menú interactivo
└── tests/
    ├── prueba_geometria.cpp          # Pruebas de geometría, trayectorias y envolvente convexa
    ├── prueba_escenario.cpp          # Pruebas de lectura, escritura y errores de escenarios
    ├── prueba_generador.cpp          # Pruebas de reproducibilidad y validez del generador
    ├── prueba_grafo.cpp              # Pruebas del grafo, Dijkstra, caminos y DFS
    ├── prueba_red.cpp                # Pruebas de Prim, Tarjan, búsqueda binaria y recargas
    ├── prueba_solvers.cpp            # Pruebas del exacto contra fuerza bruta y de las heurísticas
    ├── prueba_mision.cpp             # Pruebas de la misión completa y de la ruta exportada
    └── prueba_juez.cpp               # Pruebas del verificador, veredictos, ranking y benchmark
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

# 3. Ejecutar el programa principal desde la raíz del proyecto
./build/geodrone
```

### Uso del menú

El programa se ejecuta desde la raíz del proyecto, porque las rutas por defecto son relativas a ella. Cada pregunta muestra entre corchetes un valor por defecto que se acepta con Enter.

| Opción | Qué hace |
| --- | --- |
| 1. Generar datasets | Crea las semillas 1..N de cada categoría y el peor caso `extreme_max` en `data/generated/`, con la batería calculada por búsqueda binaria. |
| 2. Resolver un escenario | Lee un escenario, deja elegir el algoritmo, muestra costo, recargas, excluidos, tiempo y memoria, y exporta la ruta en `output/` como TXT y como dibujo SVG, que se abre con cualquier navegador. |
| 3. Ejecutar juez y benchmark | Corre todos los algoritmos sobre los escenarios de una carpeta (y los adversariales), verifica cada ruta, da veredictos OK/WA/TLE/MLE, calcula el gap y muestra el ranking por categoría. Guarda `output/judge/results.csv`, `output/judge/ranking.csv` y las rutas. Para que la memoria sea por caso, debe ser lo primero que se ejecuta al abrir el programa. |
| 4. Analizar red de recarga | Muestra estaciones alcanzables, estaciones críticas (puntos de articulación), red mínima de soporte y autonomía mínima. |

Con MinGW el ejecutable se enlaza estático, así que funciona en otra PC con Windows sin instalar nada.

Para depurar (por ejemplo en CLion o VS Code), compilar en Debug, que no optimiza:

```bash
cmake -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug
```

Si no se indica `CMAKE_BUILD_TYPE`, se usa Release.
