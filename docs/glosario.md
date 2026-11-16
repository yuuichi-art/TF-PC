# Glosario de nombres del código

Propuesta de nombres en español para el código, para que el grupo la revise y la apruebe antes del renombrado. Cuando se apruebe, este archivo queda como referencia de los nombres viejos y los nuevos.

## Reglas

**Estilo:**
- Sin tildes ni ñ (`tamano`, `anio`).
- Mismo estilo de ahora:
  - `PascalCase` para tipos;
  - `camelCase` para funciones y variables;
  - `MAYUSCULAS` para constantes y valores de `enum`;
  - sufijo `_` en los miembros privados.

**Términos técnicos:** se mantienen los que la guía del curso usa en inglés: *solver*, *dataset*, *benchmark*, *ranking*, *gap*, *branch and bound* y *2-opt*. También se mantienen los nombres propios: Dijkstra, Prim y Tarjan.

**No se traduce:**
- La biblioteca estándar (`std::`), `main`, los operadores y la API de Windows.
- El namespace `geodrone`, las constantes `EPS` e `INF`, las coordenadas `x` e `y`, y los campos `id`, `minX`, `maxX`, `minY` y `maxY`.
- Las categorías `SMALL`, `MEDIUM`, `LARGE`, `EXTREME` y `ADVERSARIAL`, que son términos de la guía.
- Las carpetas raíz `src/`, `tests/`, `docs/`, `data/` y `output/`.
- **Los formatos de archivo** (`docs/decisiones.md` §6), porque son datos y no código:
  - etiquetas del TXT;
  - `OBSTACLES` y `BATTERY`;
  - veredictos `OK`, `WA`, `TLE` y `MLE`;
  - nombres de algoritmo en archivos y CSV (`iterated_local_search`);
  - etiquetas `D0`, `S1`, `C1` y `V`;
  - encabezados del CSV;
  - nombres de los casos adversariales.

  El código solo cambia el nombre interno; el texto que se escribe en los archivos queda igual.

**Variables locales y parámetros:** no están en este glosario. Se traducen con estas mismas reglas (`battery` → `bateria`, `position` → `posicion`) y se revisan en el diff de cada fase. Las variables de una letra (`i`, `j`, `u`, `v`, `a`, `b`) y `lo`/`hi` de la búsqueda binaria se quedan.

## Carpetas y archivos

| Actual | Nuevo |
| --- | --- |
| `src/geometry/` | `src/geometria/` |
| `Point.hpp`, `Segment.hpp`, `Polygon.hpp` | `Punto.hpp`, `Segmento.hpp`, `Poligono.hpp` |
| `GeometryUtils.hpp/.cpp` | `Geometria.hpp/.cpp` |
| `ConvexHull.hpp` | `EnvolventeConvexa.hpp` |
| `src/graph/` | `src/grafo/` |
| `VisibilityGraph.hpp` | `GrafoVisibilidad.hpp` |
| `Dijkstra.hpp` | `Dijkstra.hpp` (sin cambio) |
| `MissionGraph.hpp/.cpp` | `GrafoMision.hpp/.cpp` |
| `Mst.hpp` | `ArbolMinimo.hpp` |
| `ChargingNetwork.hpp/.cpp` | `RedRecarga.hpp/.cpp` |
| `src/scenario/` | `src/escenario/` |
| `Scenario.hpp` | `Escenario.hpp` |
| `ScenarioIO.hpp/.cpp` | `ArchivoEscenario.hpp/.cpp` |
| `DatasetGenerator.hpp/.cpp` | `GeneradorDatasets.hpp/.cpp` |
| `Random.hpp` | `Aleatorio.hpp` |
| `src/solvers/` | `src/solvers/` (término de la guía) |
| `ExactSolver.hpp/.cpp` | `SolverExacto.hpp/.cpp` |
| `HeuristicSolver.hpp/.cpp` | `SolverHeuristico.hpp/.cpp` |
| `RechargePlanner.hpp/.cpp` | `PlanificadorRecargas.hpp/.cpp` |
| `Route.hpp` | `Ruta.hpp` |
| `MissionSolver.hpp/.cpp` | `SolverMision.hpp/.cpp` |
| `src/judge/` | `src/juez/` |
| `MemoryUsage.hpp/.cpp` | `UsoMemoria.hpp/.cpp` |
| `Judge.hpp/.cpp` | `Juez.hpp/.cpp` |
| `src/report/` | `src/exportacion/` |
| `Exporter.hpp/.cpp` | `Exportador.hpp/.cpp` |
| `src/main.cpp` | `src/main.cpp` (sin cambio) |
| `tests/test_<modulo>.cpp` | `tests/prueba_<modulo>.cpp`: `geometria`, `escenario`, `generador`, `grafo`, `red`, `solvers`, `mision`, `juez` |
| Pruebas de CTest (`GeometryUnitTests`, `ScenarioTests`...) | `PruebasGeometria`, `PruebasEscenario`, `PruebasGenerador`, `PruebasGrafo`, `PruebasRed`, `PruebasSolvers`, `PruebasMision`, `PruebasJuez` |
| Biblioteca de CMake `geodrone_core` | `geodrone_nucleo` (el ejecutable `geodrone` no cambia) |

Las guardas de inclusión siguen el nombre del archivo (`GEODRONE_GRAFOMISION_HPP`).

## Geometría

| Actual | Nuevo |
| --- | --- |
| `Point` | `Punto` |
| `cross`, `dist` | `cruz`, `distancia` |
| `Segment` | `Segmento` |
| `length`, `midpoint` | `longitud`, `puntoMedio` |
| `Polygon` | `Poligono` |
| `name`, `vertices` | `nombre`, `vertices` |
| `size`, `addVertex`, `getEdges` | `numVertices`, `agregarVertice`, `aristas` |
| `perimeter`, `signedArea`, `area` | `perimetro`, `areaConSigno`, `area` |
| `PointLocation`: `OUTSIDE`, `BOUNDARY`, `INSIDE` | `UbicacionPunto`: `FUERA`, `BORDE`, `DENTRO` |
| namespace `GeometryUtils` | namespace `Geometria` |
| `ccw` | `orientacion` |
| `onSegment` | `estaEnSegmento` |
| `segmentsIntersect`, `properIntersection` | `segmentosSeIntersecan`, `interseccionPropia` |
| `pointInPolygon` | `ubicacionEnPoligono` |
| `isPointStrictlyInside` | `estaEstrictamenteDentro` |
| `segmentIntersectsPolygonInterior` | `segmentoCruzaInterior` |
| `isFlightPathClear` | `tramoLibre` |
| `convexHull` | `envolventeConvexa` |

## Escenario

| Actual | Nuevo |
| --- | --- |
| `DatasetCategory` (valores sin cambio) | `Categoria` |
| `CategoryLimits` | `LimitesCategoria` |
| `timeLimitSeconds`, `memoryLimitMB` | `limiteTiempoSegundos`, `limiteMemoriaMB` |
| `limitsFor`, `categoryName`, `parseCategory` | `limitesDe`, `nombreCategoria`, `interpretarCategoria` |
| `Scenario` | `Escenario` |
| `name`, `category`, `seed` | `nombre`, `categoria`, `semilla` |
| `areaWidth`, `areaHeight`, `battery` | `anchoArea`, `altoArea`, `bateria` |
| `depot`, `stations`, `clients`, `obstacles` | `base`, `estaciones`, `clientes`, `obstaculos` |
| `numStations`, `numClients`, `numObstacles` | `numEstaciones`, `numClientes`, `numObstaculos` |
| `totalObstacleVertices` | `totalVerticesObstaculos` |
| namespace `ScenarioIO` | namespace `ArchivoEscenario` |
| `read`, `load`, `write`, `save` | `leer`, `cargar`, `escribir`, `guardar` |
| `openForWriting` | `abrirParaEscribir` |
| `CategorySpec` | `RangosCategoria` |
| `minClients`, `maxClients` | `minClientes`, `maxClientes` |
| `minStations`, `maxStations` | `minEstaciones`, `maxEstaciones` |
| `minObstacles`, `maxObstacles` | `minObstaculos`, `maxObstaculos` |
| `maxPointsPerObstacle` | `maxPuntosPorObstaculo` |
| `DatasetGenerator` | `GeneradorDatasets` |
| `BATTERY_MARGIN` | `MARGEN_BATERIA` |
| `specFor`, `worstCaseSpecFor` | `rangosDe`, `rangosPeorCaso` |
| `generate`, `generateLayout` | `generar`, `generarGeometria` |
| `batteryFor`, `generateBenchmarkSet` | `bateriaPara`, `generarConjuntoBenchmark` |
| `randomInt` | `enteroAleatorio` |

## Grafo y red de recarga

| Actual | Nuevo |
| --- | --- |
| `NodeType`: `DEPOT`, `CLIENT`, `CHARGING_STATION`, `OBSTACLE_VERTEX` | `TipoNodo`: `BASE`, `CLIENTE`, `ESTACION`, `VERTICE_OBSTACULO` |
| `Node` | `Nodo` |
| `pt`, `type`, `name`, `isKeyNode` | `punto`, `tipo`, `nombre`, `esNodoClave` |
| `GraphEdge` | `Arista` |
| `to`, `weight` | `destino`, `peso` |
| `BoundingBox` | `CajaEnvolvente` |
| `isDisjointFrom` | `esDisjuntaDe` |
| `VisibilityGraph` | `GrafoVisibilidad` |
| `addNode`, `addObstacle`, `buildVisibilityEdges` | `agregarNodo`, `agregarObstaculo`, `construirAristas` |
| `numVertices`, `numEdges` | `numNodos`, `numAristas` |
| `getNode`, `getAdj`, `getNeighbors` | `nodo`, `adyacencia`, `vecinos` |
| `DijkstraResult` | `ResultadoDijkstra` |
| `source`, `dist`, `parent` | `origen`, `distancia`, `padre` |
| `Dijkstra` (sin cambio) | `run` → `ejecutar` |
| parámetro `keyNodesAsEndpoints` | `nodosClaveComoExtremos` |
| `MissionGraph` | `GrafoMision` |
| `numKeyNodes`, `numStations`, `numClients` | `numNodosClave`, `numEstaciones`, `numClientes` |
| `depotKey`, `stationKey`, `clientKey` | `claveBase`, `claveEstacion`, `claveCliente` |
| `keyPoint`, `label` | `puntoClave`, `etiqueta` |
| `distance`, `distances` | `distancia`, `distancias` |
| `isConnectedToDepot`, `unreachableClients` | `conectadoABase`, `clientesInalcanzables` |
| `pathNodes`, `graph` | `nodosCamino`, `grafo` |
| `MstResult` | `ArbolMinimo` |
| `cost`, `maxEdge`, `parent`, `spansAll` | `costo`, `aristaMaxima`, `padre`, `cubreTodos` |
| `primMst` | `arbolMinimoPrim` |
| `ChargingNetwork` | `RedRecarga` |
| `BATTERY_TOLERANCE` | `TOLERANCIA_BATERIA` |
| `battery`, `linked` | `bateria`, `conectados` |
| `isReachable`, `reachablePoints` | `esAlcanzable`, `puntosAlcanzables` |
| `distanceToRecharge`, `canServe` | `distanciaARecarga`, `puedeAtender` |
| `criticalStations` | `estacionesCriticas` |
| `isFeasible`, `minimumAutonomy`, `supportTree` | `esFactible`, `autonomiaMinima`, `arbolSoporte` |
| `tarjan` | `tarjan` (sin cambio) |

## Solvers

| Actual | Nuevo |
| --- | --- |
| `ExactResult` | `ResultadoExacto` |
| `order`, `cost`, `nodesExplored`, `nodesPruned` | `orden`, `costo`, `nodosExplorados`, `nodosPodados` |
| `ExactSolver` | `SolverExacto` |
| `MAX_CLIENTS`, `solve` | `MAX_CLIENTES`, `resolver` |
| namespace `HeuristicSolver` | namespace `SolverHeuristico` |
| `Clock`, `noDeadline`, `DEFAULT_ILS_ITERATIONS` | `Reloj`, `sinLimite`, `ITERACIONES_POR_DEFECTO` |
| `tourCost`, `nearestNeighbor` | `costoCiclo`, `vecinoMasCercano` |
| `mstApproximation`, `twoOpt` | `dosAproximacion`, `aplicar2Opt` |
| `IlsResult` | `ResultadoBusquedaIterada` |
| `order`, `iterations`, `stoppedByTime` | `orden`, `iteraciones`, `cortadaPorTiempo` |
| `iteratedLocalSearch` | `busquedaLocalIterada` |
| `RechargePlan` | `PlanRecargas` |
| `feasible`, `keySequence`, `recharges`, `cost` | `factible`, `secuenciaClaves`, `recargas`, `costo` |
| `RechargePlanner` | `PlanificadorRecargas` |
| `plan` | `planificar` |
| `RoutePoint` | `PuntoRuta` |
| `point`, `label` | `punto`, `etiqueta` |
| `ExcludedClient` | `ClienteExcluido` |
| `label`, `reason` | `etiqueta`, `motivo` |
| `Route` | `Ruta` |
| `scenario`, `algorithm`, `feasible`, `cost` | `escenario`, `algoritmo`, `factible`, `costo` |
| `recharges`, `excluded`, `order`, `path` | `recargas`, `excluidos`, `orden`, `trayectoria` |
| `preparationMs`, `solveMs`, `totalMs` | `preparacionMs`, `resolucionMs`, `totalMs` |
| `Algorithm` | `Algoritmo` |
| `BRANCH_AND_BOUND` | `BRANCH_AND_BOUND` (término de la guía) |
| `NEAREST_NEIGHBOR`, `MST_APPROXIMATION` | `VECINO_MAS_CERCANO`, `DOS_APROXIMACION` |
| `NEAREST_NEIGHBOR_2OPT`, `MST_APPROXIMATION_2OPT` | `VECINO_MAS_CERCANO_2OPT`, `DOS_APROXIMACION_2OPT` |
| `ITERATED_LOCAL_SEARCH` | `BUSQUEDA_LOCAL_ITERADA` |
| `algorithmName`, `allAlgorithms` | `nombreAlgoritmo`, `todosLosAlgoritmos` |
| `MissionSolver` | `SolverMision` |
| `supportsExact`, `solve` | `admiteExacto`, `resolver` |
| `mission`, `network`, `excluded` | `mision`, `red`, `excluidos` |
| `servableClients`, `preparationMs` | `clientesAtendibles`, `preparacionMs` |

## Juez y exportación

| Actual | Nuevo |
| --- | --- |
| `peakMemoryMB` | `memoriaPicoMB` |
| `VerificationResult` | `ResultadoVerificacion` |
| `valid`, `problem` | `valida`, `problema` |
| `JudgeRow` | `FilaJuez` |
| `scenario`, `category`, `clients`, `algorithm` | `escenario`, `categoria`, `clientes`, `algoritmo` |
| `verdict`, `problem`, `cost`, `recharges` | `veredicto`, `problema`, `costo`, `recargas` |
| `excluded`, `timeMs`, `memoryMB` | `excluidos`, `tiempoMs`, `memoriaMB` |
| `gapBest`, `gapBound` | `gapMejor`, `gapCota` |
| `RankingRow` | `FilaRanking` |
| `category`, `position`, `algorithm` | `categoria`, `posicion`, `algoritmo` |
| `okCases`, `cases`, `meanGap`, `meanTimeMs` | `casosOk`, `casos`, `gapMedio`, `tiempoMedioMs` |
| namespace `Judge` | namespace `Juez` |
| `verifyRoute`, `verdictFor` | `verificarRuta`, `veredictoPara` |
| `findScenarioFiles`, `run`, `rank` | `buscarEscenarios`, `ejecutar`, `armarRanking` |
| namespace `Exporter` | namespace `Exportador` |
| `writeRoute`, `saveRoute` | `escribirRuta`, `guardarRuta` |
| `writeResultsCsv`, `saveResultsCsv` | `escribirResultadosCsv`, `guardarResultadosCsv` |
| `writeRankingCsv`, `saveRankingCsv` | `escribirRankingCsv`, `guardarRankingCsv` |
| `writeSvg`, `saveSvg` | `escribirSvg`, `guardarSvg` |

## Funciones internas (revisión opcional)

Son nombres que solo se usan dentro de un archivo. Se listan para que se vean coherentes con el resto.

| Archivo | Actual → Nuevo |
| --- | --- |
| `ArchivoEscenario.cpp` | `Token` → `Palabra` (`text` → `texto`, `line` → `linea`), `TokenReader` → `LectorPalabras`, `expectLabel` → `esperarEtiqueta`, `readWord` → `leerPalabra`, `readNumber` → `leerNumero`, `readCount` → `leerCantidad`, `readSeed` → `leerSemilla`, `readPoint` → `leerPunto`, `expectEnd` → `esperarFin`, `next` → `siguiente`, `failAt` → `fallarEn`, `failAtLast` → `fallarEnUltima`, `parseDouble` → `interpretarReal`, `readPointBlock` → `leerBloquePuntos`, `checkName` → `validarNombre`, `readObstacles` → `leerObstaculos` |
| `GeneradorDatasets.cpp` | `generateObstacles` → `generarObstaculos`, `randomFreePoint` → `puntoLibreAleatorio`, `lowercase` → `minusculas` |
| `GrafoVisibilidad.hpp` | `isSegmentClear` → `tramoVisible`, `boxOf` → `cajaDe`, parámetro `useBoundingBoxFilter` → `usarFiltroCajas` |
| `GrafoMision.cpp` | `markConnected` → `marcarConectados` |
| `SolverExacto.cpp` | `Search` → `Busqueda`, `lowerBound` → `cotaInferior`, `candidatesFrom` → `candidatosDesde`, `branch` → `ramificar` |
| `SolverHeuristico.cpp` | `MIN_IMPROVEMENT` → `MEJORA_MINIMA`, `preorder` → `preorden`, `doubleBridge` → `doblePuente` |
| `PlanificadorRecargas` | `findRechargePath` → `buscarCaminoRecarga`, `networkDistances` → `distanciasEnRed`, `chooseLastRecharge` → `elegirUltimaRecarga` |
| `SolverMision.cpp` | `TIME_BUDGET_FRACTION` → `FRACCION_PRESUPUESTO`, `millisecondsSince` → `milisegundosDesde`, `chooseOrder` → `elegirOrden`, `expandPath` → `expandirTrayectoria` |
| `Juez.cpp` | `NOT_AVAILABLE` → `NO_DISPONIBLE`, `fail` → `falla`, `parseLabel` → `interpretarEtiqueta`, `pointOf` → `puntoDe`, `percentOver` → `porcentajeSobre`, `exclusionIsGenuine` → `exclusionEsReal`, `sizeOf` → `tamanoDe`, `judgeScenario` → `juzgarEscenario`, `printCase` → `imprimirCaso` |
| `Exportador.cpp` | `Canvas` → `Lienzo`, `escapeXml` → `escaparXml`, `writePoints` → `escribirPuntos`, `writeStation` → `dibujarEstacion`, `writeCross` → `dibujarCruz`, `writeLabel` → `escribirEtiqueta`, `legendText` → `textoLeyenda` |
| `main.cpp` | `trim` → `recortar`, `askText` → `preguntarTexto`, `askNumber` → `preguntarNumero`, `printBanner` → `imprimirPortada`, `printScenarioSummary` → `imprimirResumenEscenario`, `printLabels` → `imprimirEtiquetas`, `generateDatasets` → `generarDatasets`, `solveScenario` → `resolverEscenario`, `printRanking` → `imprimirRanking`, `runJudge` → `ejecutarJuez`, `analyzeNetwork` → `analizarRed`, `inputClosed` → `entradaCerrada`, `sessionUsedMemory` → `memoriaUsadaEnSesion` |
