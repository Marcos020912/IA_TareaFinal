# Implementación del algoritmos de Búsqueda en Java

## Búsqueda Voraz (Greedy Best-First Search)

Para ilustrar cómo utilizar el algoritmo de Búsqueda Voraz (Greedy Best-First Search) implementado en el repositorio aima-java, vamos a plantear un problema sencillo pero interesante: encontrar el camino más corto desde un punto de inicio a un punto de destino en un mapa de ciudades, utilizando las distancias heurísticas directas entre las ciudades como guía.

### Paso 1: Definir el Problema

Supongamos que tenemos un conjunto de ciudades conectadas por carreteras, y queremos encontrar la ruta más corta desde una ciudad de inicio hasta una ciudad de destino. Las distancias directas (heurísticas) entre las ciudades son conocidas y se utilizan para guiar la búsqueda.

### Paso 2: Configurar el Entorno de Desarrollo

1. Clonar el repositorio: Primero, necesitas clonar el repositorio aima-java desde GitHub.

    ```bash
    git clone https://github.com/aimacode/aima-java.git
    ```

2. Configurar el proyecto en un IDE: Importa el proyecto en un IDE que soporte Java, como IntelliJ IDEA o Eclipse.

3. Asegurarse de que el proyecto se construye correctamente: Verifica que el proyecto se compila sin errores.

### Paso 3: Implementar el Mapa de Ciudades

1. Crear la clase Ciudad: Esta clase representará una ciudad en el mapa.

    ```java

    public class Ciudad {
        private String nombre;
        private Map<Ciudad, Double> vecinos;

        public Ciudad(String nombre) {
            this.nombre = nombre;
            this.vecinos = new HashMap<>();
        }

        public void agregarVecino(Ciudad vecino, double distancia) {
            vecinos.put(vecino, distancia);
        }

        public Map<Ciudad, Double> getVecinos() {
            return vecinos;
        }

        public String getNombre() {
            return nombre;
        }
    }
    ```

2. Definir el estado y el problema: Necesitas una clase que represente un estado en el mapa (una ciudad específica) y otra que implemente la interfaz Problem de AIMA para definir el problema de búsqueda.

    ```java
    import aima.core.search.framework.problem.GeneralProblem;
    import aima.core.search.framework.problem.Problem;

    public class MapaProblem extends GeneralProblem<Ciudad, MoveToAction> {
        public MapaProblem(Ciudad initialState, Ciudad goalState) {
            super(initialState, MapFunctions.createActionsFunction(), MapFunctions.createResultFunction(), GoalTest.isEqual(goalState));
        }
    }
    ```

### Paso 4: Implementar la Búsqueda Voraz

Utilizaremos la búsqueda voraz, que selecciona el nodo más prometedor basado en una función heurística que estima la distancia al objetivo.

    ```java
    import aima.core.search.framework.qsearch.GraphSearch;
    import aima.core.search.informed.GreedyBestFirstSearch;
    import aima.core.search.informed.HeuristicFunction;

    public class MapaGreedySearch {
        public static void main(String[] args) {
            Ciudad inicio = new Ciudad("Inicio");
            Ciudad objetivo = new Ciudad("Objetivo");
            // Suponer que se han agregado ciudades y conexiones aquí

            Problem<Ciudad, MoveToAction> problem = new MapaProblem(inicio, objetivo);
            HeuristicFunction<Ciudad> heuristic = new StraightLineDistanceHeuristic(objetivo);
            GreedyBestFirstSearch<Ciudad, MoveToAction> search = new GreedyBestFirstSearch<>(new GraphSearch<>(), heuristic);

            Optional<List<MoveToAction>> actions = search.findActions(problem);
            actions.ifPresent(System.out::println);
        }
    }
    ```

### Paso 5: Ejecutar la Búsqueda

Configura y ejecuta la búsqueda voraz para encontrar la ruta más corta en el mapa de ciudades.

## Búsqueda de Costo Uniforme (Uniform Cost Search)

Para ilustrar cómo utilizar el algoritmo de Búsqueda de Costo Uniforme implementado en el repositorio aima-java, vamos a plantear un problema sencillo pero práctico: encontrar la ruta más económica para viajar entre ciudades en un mapa, considerando el costo de viaje entre ellas.

### Paso 1: Definir el Problema

Supongamos que tenemos un conjunto de ciudades conectadas por rutas con diferentes costos asociados. El objetivo es encontrar la ruta de menor costo desde una ciudad de inicio hasta una ciudad de destino.

### Paso 2: Configurar el Entorno de Desarrollo

1. Clonar el repositorio: Primero, necesitas clonar el repositorio aima-java desde GitHub.

    ```bash
    git clone https://github.com/aimacode/aima-java.git
    ```

2. Configurar el proyecto en un IDE: Importa el proyecto en un IDE que soporte Java, como IntelliJ IDEA o Eclipse.

3. Asegurarse de que el proyecto se construye correctamente: Verifica que el proyecto se compila sin errores.

### Paso 3: Implementar el Mapa de Ciudades

1. Crear la clase Ruta: Esta clase representará una conexión entre dos ciudades y su costo asociado.

    ```java
    public class Ruta {
        private Ciudad origen;
        private Ciudad destino;
        private double costo;

        public Ruta(Ciudad origen, Ciudad destino, double costo) {
            this.origen = origen;
            this.destino = destino;
            this.costo = costo;
        }

        public Ciudad getOrigen() {
            return origen;
        }

        public Ciudad getDestino() {
            return destino;
        }

        public double getCosto() {
            return costo;
        }
    }
    ```

2. Definir el estado y el problema: Necesitas una clase que represente un estado en el mapa (una ciudad específica) y otra que implemente la interfaz Problem de AIMA para definir el problema de búsqueda.

    ```java
    import aima.core.search.framework.problem.GeneralProblem;
    import aima.core.search.framework.problem.Problem;

    public class MapaProblem extends GeneralProblem<Ciudad, Ruta> {
        public MapaProblem(Ciudad initialState, Ciudad goalState) {
            super(initialState, MapFunctions.createActionsFunction(), MapFunctions.createResultFunction(), GoalTest.isEqual(goalState));
        }
    }
    ```

### Paso 4: Implementar la Búsqueda de Costo Uniforme

Utilizaremos la búsqueda de costo uniforme, que selecciona el nodo a expandir basado en el costo acumulado desde el nodo inicial.

```java
import aima.core.search.framework.qsearch.GraphSearch;
import aima.core.search.uninformed.UniformCostSearch;

public class MapaUniformCostSearch {
    public static void main(String[] args) {
        Ciudad inicio = new Ciudad("Inicio");
        Ciudad objetivo = new Ciudad("Objetivo");
        // Suponer que se han agregado ciudades y conexiones aquí

        Problem<Ciudad, Ruta> problem = new MapaProblem(inicio, objetivo);
        UniformCostSearch<Ciudad, Ruta> search = new UniformCostSearch<>(new GraphSearch<>());

        Optional<List<Ruta>> actions = search.findActions(problem);
        actions.ifPresent(System.out::println);
    }
}
```

### Paso 5: Ejecutar la Búsqueda

Configura y ejecuta la búsqueda de costo uniforme para encontrar la ruta de menor costo en el mapa de ciudades.

## Ascenso de Colinas (Hill Climbing)

Para ilustrar cómo utilizar el algoritmo de Ascenso de Colinas implementado en el repositorio aima-java, vamos a plantear un problema sencillo pero representativo: encontrar una configuración óptima de números en un arreglo para maximizar o minimizar una función objetivo dada.

### Paso 1: Definir el Problema

Supongamos que tenemos un arreglo de números enteros y queremos reorganizar sus elementos para maximizar la suma de los productos de cada par de elementos adyacentes. Este es un problema de optimización que puede ser abordado eficazmente con el algoritmo de Ascenso de Colinas.

### Paso 2: Configurar el Entorno de Desarrollo

1. Clonar el repositorio: Primero, necesitas clonar el repositorio aima-java desde GitHub.

    ```bash
    git clone https://github.com/aimacode/aima-java.git
    ```

2. Configurar el proyecto en un IDE: Importa el proyecto en un IDE que soporte Java, como IntelliJ IDEA o Eclipse.

3. Asegurarse de que el proyecto se construye correctamente: Verifica que el proyecto se compila sin errores.

### Paso 3: Implementar la Función Objetivo y el Estado

1. Crear la clase Estado: Esta clase representará un estado posible del problema, es decir, una configuración particular del arreglo.

    ```java
    public class Estado {
        private int[] arreglo;

        public Estado(int[] arreglo) {
            this.arreglo = arreglo.clone();
        }

        public int[] getArreglo() {
            return arreglo;
        }

        // Función para calcular el valor objetivo del estado
        public int calcularValor() {
            int suma = 0;
            for (int i = 0; i < arreglo.length - 1; i++) {
                suma += arreglo[i] * arreglo[i + 1];
            }
            return suma;
        }
    }
    ```

2. Generar vecinos: Implementa una función para generar estados vecinos alterando el arreglo actual.

    ```java
    public List<Estado> generarVecinos() {
        List<Estado> vecinos = new ArrayList<>();
        for (int i = 0; i < arreglo.length - 1; i++) {
            int[] nuevoArreglo = arreglo.clone();
            // Intercambiar elementos
            int temp = nuevoArreglo[i];
            nuevoArreglo[i] = nuevoArreglo[i + 1];
            nuevoArreglo[i + 1] = temp;
            vecinos.add(new Estado(nuevoArreglo));
        }
        return vecinos;
    }
    ```

### Paso 4: Implementar el Algoritmo de Ascenso de Colinas

Utilizaremos el algoritmo de Ascenso de Colinas, que selecciona el mejor vecino hasta que no se encuentren mejoras.

```java
import aima.core.search.local.HillClimbingSearch;

public class OptimizacionArreglo {
    public static void main(String[] args) {
        int[] inicial = {1, 2, 3, 4, 5};
        Estado estadoInicial = new Estado(inicial);
        HillClimbingSearch<Estado, Integer> hc = new HillClimbingSearch<>(
            Estado::calcularValor
        );

        Estado resultado = hc.findBestState(estadoInicial);
        System.out.println("Configuración óptima encontrada con valor: " + resultado.calcularValor());
    }
}
```

### Paso 5: Ejecutar la Búsqueda

Configura y ejecuta el algoritmo de Ascenso de Colinas para encontrar la configuración óptima del arreglo.

## Búsqueda de Haz Local (Beam Search)

Para ilustrar cómo utilizar el algoritmo de Búsqueda de Haz Local implementado en el repositorio aima-java, vamos a plantear un problema sencillo pero representativo: encontrar la mejor ruta en un grafo desde un nodo de inicio hasta un nodo objetivo, utilizando una función de evaluación basada en pesos de aristas y un ancho de haz limitado.

### Paso 1: Definir el Problema

Supongamos que tenemos un grafo donde los nodos representan ciudades y las aristas representan carreteras con ciertos costos asociados. El objetivo es encontrar una ruta óptima desde una ciudad de inicio hasta una ciudad de destino, utilizando el algoritmo de Búsqueda de Haz Local para explorar eficientemente el espacio de búsqueda.

### Paso 2: Configurar el Entorno de Desarrollo

1. Clonar el repositorio: Primero, necesitas clonar el repositorio aima-java desde GitHub.

    ```bash
    git clone https://github.com/aimacode/aima-java.git
    ```

2. Configurar el proyecto en un IDE: Importa el proyecto en un IDE que soporte Java, como IntelliJ IDEA o Eclipse.

3. Asegurarse de que el proyecto se construye correctamente: Verifica que el proyecto se compila sin errores.

### Paso 3: Implementar el Grafo

1. Crear la clase Grafo: Esta clase representará el grafo y sus conexiones.

    ```java
    public class Grafo {
        private Map<String, List<Conexion>> conexiones = new HashMap<>();

        public void agregarConexion(String origen, String destino, double costo) {
            conexiones.putIfAbsent(origen, new ArrayList<>());
            conexiones.get(origen).add(new Conexion(destino, costo));
        }

        public List<Conexion> getConexiones(String nodo) {
            return conexiones.getOrDefault(nodo, Collections.emptyList());
        }
    }

    public class Conexion {
        String destino;
        double costo;

        public Conexion(String destino, double costo) {
            this.destino = destino;
            this.costo = costo;
        }
    }
    ```

### Paso 4: Implementar la Búsqueda de Haz Local

Utilizaremos la búsqueda de haz local, que explora el espacio de búsqueda manteniendo solo un número limitado de los mejores nodos en cada nivel del árbol de búsqueda.

```java
import aima.core.search.framework.problem.Problem;
import aima.core.search.local.BeamSearch;

public class BusquedaGrafo {
    public static void main(String[] args) {
        Grafo grafo = new Grafo();
        // Suponer que se han agregado conexiones aquí

        Problem<String, Conexion> problem = new ProblemGrafo("Inicio", "Objetivo", grafo);
        BeamSearch<String, Conexion> beamSearch = new BeamSearch<>(2); // Ancho de haz

        List<Conexion> resultado = beamSearch.search(problem);
        System.out.println("Ruta encontrada: " + resultado);
    }
}
```

### Paso 5: Ejecutar la Búsqueda

Configura y ejecuta la búsqueda de haz local para encontrar la mejor ruta en el grafo.

## Algoritmo Genético

Para ilustrar cómo utilizar el algoritmo genético implementado en el repositorio aima-java, vamos a plantear un problema sencillo pero representativo: optimizar una cadena de caracteres para que se acerque lo más posible a una cadena objetivo dada, utilizando una función de aptitud basada en la coincidencia de caracteres.

### Paso 1: Definir el Problema

Supongamos que tenemos una cadena objetivo, por ejemplo, "HELLO WORLD", y queremos generar una cadena que se acerque lo más posible a esta utilizando un algoritmo genético. Cada individuo en la población será una cadena de la misma longitud que la cadena objetivo.

### Paso 2: Configurar el Entorno de Desarrollo

1. Clonar el repositorio: Primero, necesitas clonar el repositorio aima-java desde GitHub.

    ```bash
    git clone https://github.com/aimacode/aima-java.git
    ```

2. Configurar el proyecto en un IDE: Importa el proyecto en un IDE que soporte Java, como IntelliJ IDEA o Eclipse.

3. Asegurarse de que el proyecto se construye correctamente: Verifica que el proyecto se compila sin errores.

### Paso 3: Implementar la Función de Aptitud

1. Crear la función de aptitud: Esta función evaluará qué tan cerca está una cadena de la cadena objetivo.

    ```java
    import aima.core.search.local.FitnessFunction;

    public class StringFitnessFunction implements FitnessFunction<String> {
        private String target;

        public StringFitnessFunction(String target) {
            this.target = target;
        }

        @Override
        public double apply(String individual) {
            int fitness = 0;
            for (int i = 0; i < individual.length(); i++) {
                if (individual.charAt(i) == target.charAt(i)) {
                    fitness++;
                }
            }
            return fitness;
        }
    }
    ```

### Paso 4: Configurar y Ejecutar el Algoritmo Genético

1. Configurar el algoritmo genético: Define los parámetros del algoritmo, como la longitud del individuo, el alfabeto finito, la probabilidad de mutación y el tamaño de la población.

    ```java
    import aima.core.search.local.GeneticAlgorithm;
    import aima.core.search.local.Individual;
    import java.util.ArrayList;
    import java.util.List;
    import java.util.Random;

    public class StringOptimization {
        public static void main(String[] args) {
            int individualLength = "HELLO WORLD".length();
            List<Character> finiteAlphabet = new ArrayList<>();
            for (char c = ' '; c <= 'Z'; c++) {
                finiteAlphabet.add(c);
            }
            double mutationProbability = 0.1;
            GeneticAlgorithm<Character> ga = new GeneticAlgorithm<>(
                individualLength,
                finiteAlphabet,
                mutationProbability,
                new Random()
            );

            // Inicializar población
            List<Individual<Character>> population = new ArrayList<>();
            for (int i = 0; i < 100; i++) {
                population.add(ga.randomIndividual());
            }

            // Ejecutar algoritmo genético
            Individual<Character> bestIndividual = ga.geneticAlgorithm(
                population,
                new StringFitnessFunction("HELLO WORLD"),
                1000
            );

            System.out.println("Mejor individuo encontrado: " + bestIndividual);
        }
    }
    ```

### Paso 5: Observar los Resultados

El algoritmo genético iterará, seleccionando, cruzando y mutando individuos en la población, y deberías observar cómo la aptitud del mejor individuo se acerca a la máxima posible (la longitud de la cadena objetivo).

## Recocido Simulado (Simulated Annealing)

Para ilustrar cómo utilizar el algoritmo de Recocido Simulado implementado en el repositorio aima-java, vamos a plantear un problema sencillo pero representativo: optimizar la disposición de ciudades en un problema del viajante de comercio (TSP, por sus siglas en inglés) para minimizar la distancia total del recorrido.

### Paso 1: Definir el Problema

Supongamos que tenemos un conjunto de ciudades y las distancias entre cada par de ellas. El objetivo es encontrar el recorrido que minimice la distancia total recorrida, visitando cada ciudad una sola vez y regresando al punto de partida.

### Paso 2: Configurar el Entorno de Desarrollo

1. Clonar el repositorio: Primero, necesitas clonar el repositorio aima-java desde GitHub.

    ```bash
    git clone https://github.com/aimacode/aima-java.git
    ```

2. Configurar el proyecto en un IDE: Importa el proyecto en un IDE que soporte Java, como IntelliJ IDEA o Eclipse.

3. Asegurarse de que el proyecto se construye correctamente: Verifica que el proyecto se compila sin errores.

### Paso 3: Implementar el Problema del TSP

1. Crear la clase TSP: Esta clase representará el problema del viajante de comercio, incluyendo las ciudades y las distancias entre ellas.

    ```java
    public class TSP {
        private int[][] distancias;
        private int numCiudades;

        public TSP(int[][] distancias) {
            this.distancias = distancias;
            this.numCiudades = distancias.length;
        }

        public int getDistancia(int ciudad1, int ciudad2) {
            return distancias[ciudad1][ciudad2];
        }

        public int getNumCiudades() {
            return numCiudades;
        }
    }
    ```

### Paso 4: Implementar el Algoritmo de Recocido Simulado

Utilizaremos el algoritmo de Recocido Simulado, que permite escapar de óptimos locales aceptando soluciones peores con una probabilidad que disminuye con el tiempo.

```java
import aima.core.search.local.SimulatedAnnealingSearch;
import aima.core.search.local.Scheduler;

public class TSPSimulatedAnnealing {
    public static void main(String[] args) {
        int[][] distancias = {{0, 2, 3}, {2, 0, 1}, {3, 1, 0}};
        TSP tsp = new TSP(distancias);

        Scheduler scheduler = new Scheduler(1000, 0.95, 100);
        SimulatedAnnealingSearch<int[], Double> sa = new SimulatedAnnealingSearch<>(
            new TSPHeuristicFunction(),
            scheduler
        );

        int[] initialState = new int[tsp.getNumCiudades()];
        // Inicializar el estado, por ejemplo, con las ciudades en orden
        for (int i = 0; i < initialState.length; i++) {
            initialState[i] = i;
        }

        int[] result = sa.findState(new TSPProblem(initialState, tsp));
        System.out.println("Ruta encontrada: " + Arrays.toString(result));
    }
}
```

### Paso 5: Ejecutar la Búsqueda

Configura y ejecuta el algoritmo de Recocido Simulado para encontrar la ruta óptima en el problema del TSP.

## Búsqueda en Amplitud (Breadth-First Search, BFS)

Para ilustrar cómo utilizar el algoritmo de Búsqueda en Amplitud (Breadth-First Search, BFS) implementado en el repositorio aima-java, vamos a plantear un problema sencillo pero representativo: encontrar el camino más corto en un grafo no ponderado desde un nodo de inicio hasta un nodo objetivo.

### Paso 1: Definir el Problema

Supongamos que tenemos un grafo no ponderado donde los nodos representan estados y las aristas representan transiciones posibles entre estos estados. El objetivo es encontrar el camino más corto desde un nodo de inicio hasta un nodo objetivo.

### Paso 2: Configurar el Entorno de Desarrollo

1. Clonar el repositorio: Primero, necesitas clonar el repositorio aima-java desde GitHub.

    ```bash
    git clone https://github.com/aimacode/aima-java.git
    ```

2. Configurar el proyecto en un IDE: Importa el proyecto en un IDE que soporte Java, como IntelliJ IDEA o Eclipse.

3. Asegurarse de que el proyecto se construye correctamente: Verifica que el proyecto se compila sin errores.

### Paso 3: Implementar el Grafo

1. Crear la clase Grafo: Esta clase representará el grafo y sus conexiones.

    ```java
    public class Grafo {
        private Map<String, List<String>> adjList;

        public Grafo() {
            this.adjList = new HashMap<>();
        }

        public void agregarArista(String origen, String destino) {
            adjList.putIfAbsent(origen, new ArrayList<>());
            adjList.get(origen).add(destino);
        }

        public List<String> getVecinos(String nodo) {
            return adjList.getOrDefault(nodo, new ArrayList<>());
        }
    }
    ```

### Paso 4: Implementar la Búsqueda en Amplitud

Utilizaremos la búsqueda en amplitud, que explora el grafo nivel por nivel, asegurando que se encuentre el camino más corto en términos de número de aristas.

```java
import aima.core.search.framework.problem.Problem;
import aima.core.search.uninformed.BreadthFirstSearch;

public class BusquedaGrafo {
    public static void main(String[] args) {
        Grafo grafo = new Grafo();
        // Suponer que se han agregado conexiones aquí

        Problem<String, String> problem = new ProblemGrafo("Inicio", "Objetivo", grafo);
        BreadthFirstSearch<String, String> bfs = new BreadthFirstSearch<>(new GraphSearch<>());

        List<String> resultado = bfs.findActions(problem);
        System.out.println("Camino encontrado: " + resultado);
    }
}
```

### Paso 5: Ejecutar la Búsqueda

Configura y ejecuta la búsqueda en amplitud para encontrar el camino más corto en el grafo.

## Búsqueda en Profundidad Limitada (Depth-Limited Search)

Para ilustrar cómo utilizar el algoritmo de Búsqueda en Profundidad Limitada implementado en el repositorio aima-java, vamos a plantear un problema sencillo pero representativo: encontrar un camino en un árbol de decisión para un juego de rompecabezas simple, limitando la profundidad de búsqueda para evitar la exploración excesiva en árboles muy profundos.

### Paso 1: Definir el Problema

Supongamos que tenemos un juego de rompecabezas, como el rompecabezas de deslizamiento (sliding puzzle), donde el objetivo es alcanzar un estado objetivo específico a partir de un estado inicial. El problema se puede complicar rápidamente a medida que aumenta el tamaño del rompecabezas, por lo que utilizaremos una búsqueda en profundidad limitada para controlar la complejidad.

### Paso 2: Configurar el Entorno de Desarrollo

1. Clonar el repositorio: Primero, necesitas clonar el repositorio aima-java desde GitHub.

    ```bash
    git clone https://github.com/aimacode/aima-java.git
    ```

2. Configurar el proyecto en un IDE: Importa el proyecto en un IDE que soporte Java, como IntelliJ IDEA o Eclipse.

3. Asegurarse de que el proyecto se construye correctamente: Verifica que el proyecto se compila sin errores.

### Paso 3: Implementar el Estado y el Problema

1. Crear la clase Estado: Esta clase representará un estado del rompecabezas.

    ```java
    public class PuzzleState {
        private int[] board;

        public PuzzleState(int[] board) {
            this.board = board.clone();
        }

        public int[] getBoard() {
            return board;
        }
    }
    ```

2. Definir el problema: Implementa el problema utilizando las clases de aima-java.

    ```java
    import aima.core.search.framework.problem.Problem;
    import aima.core.search.framework.problem.GeneralProblem;

    public class PuzzleProblem extends GeneralProblem<PuzzleState, Action> {
        public PuzzleProblem(PuzzleState initialState, GoalTest<PuzzleState> goalTest) {
            super(initialState, PuzzleFunctions::getActions, PuzzleFunctions::getResult, goalTest);
        }
    }
    ```

### Paso 4: Implementar la Búsqueda en Profundidad Limitada

Utilizaremos la búsqueda en profundidad limitada, especificando un límite para la profundidad de búsqueda para evitar la exploración de caminos demasiado largos o ciclos infinitos.

```java
import aima.core.search.uninformed.DepthLimitedSearch;

public class PuzzleDepthLimitedSearch {
    public static void main(String[] args) {
        PuzzleState initialState = new PuzzleState(new int[]{...}); // Estado inicial del rompecabezas
        PuzzleProblem problem = new PuzzleProblem(initialState, state -> Arrays.equals(state.getBoard(), new int[]{...})); // Estado objetivo

        DepthLimitedSearch<PuzzleState, Action> dls = new DepthLimitedSearch<>(10); // Límite de profundidad establecido en 10
        List<Action> actions = dls.findActions(problem);

        if (actions.isEmpty()) {
            System.out.println("No se encontró solución o se alcanzó el límite de profundidad.");
        } else {
            System.out.println("Solución encontrada: " + actions);
        }
    }
}
```

### Paso 5: Ejecutar la Búsqueda

Configura y ejecuta la búsqueda en profundidad limitada para encontrar un camino en el rompecabezas dentro del límite de profundidad especificado.

## Búsqueda Iterativa en Profundidad (Iterative Deepening Search, IDS)

Para ilustrar cómo utilizar el algoritmo de Búsqueda Iterativa en Profundidad (Iterative Deepening Search, IDS) implementado en el repositorio aima-java, vamos a plantear un problema sencillo pero representativo: encontrar un camino en un laberinto desde un punto de inicio hasta un punto de destino.

### Paso 1: Definir el Problema

Supongamos que tenemos un laberinto representado como una cuadrícula donde cada celda puede ser un espacio libre o un obstáculo. El objetivo es encontrar un camino desde la celda de inicio hasta la celda de destino, explorando el espacio de manera eficiente.

### Paso 2: Configurar el Entorno de Desarrollo

1. Clonar el repositorio: Primero, necesitas clonar el repositorio aima-java desde GitHub.

    ```bash
    git clone https://github.com/aimacode/aima-java.git
    ```

2. Configurar el proyecto en un IDE: Importa el proyecto en un IDE que soporte Java, como IntelliJ IDEA o Eclipse.

3. Asegurarse de que el proyecto se construye correctamente: Verifica que el proyecto se compila sin errores.

### Paso 3: Implementar el Laberinto

1. Crear la clase Laberinto: Esta clase representará el laberinto como una matriz de celdas.

    ```java
    public class Laberinto {
        private int[][] grid;
        private int startRow, startCol;
        private int goalRow, goalCol;

        public Laberinto(int[][] grid, int startRow, int startCol, int goalRow, int goalCol) {
            this.grid = grid;
            this.startRow = startRow;
            this.startCol = startCol;
            this.goalRow = goalRow;
            this.goalCol = goalCol;
        }

        public boolean isFree(int row, int col) {
            return grid[row][col] == 0; // 0 representa un espacio libre
        }

        // Getters y setters
    }
    ```

### Paso 4: Implementar la Búsqueda Iterativa en Profundidad

Utilizaremos la búsqueda iterativa en profundidad, que aplica repetidamente una búsqueda en profundidad limitada con límites de profundidad crecientes hasta encontrar una solución o explorar completamente el espacio de búsqueda.

```java
import aima.core.search.framework.problem.Problem;
import aima.core.search.uninformed.IterativeDeepeningSearch;

public class LaberintoSearch {
    public static void main(String[] args) {
        Laberinto laberinto = new Laberinto(...); // Inicializar con datos específicos
        Problem<String, MoveToAction> problem = new LaberintoProblem(laberinto, "start", "goal");
        IterativeDeepeningSearch<String, MoveToAction> ids = new IterativeDeepeningSearch<>();

        List<MoveToAction> actions = ids.findActions(problem);
        System.out.println("Camino encontrado: " + actions);
    }
}
```

### Paso 5: Ejecutar la Búsqueda

Configura y ejecuta la búsqueda iterativa en profundidad para encontrar el camino en el laberinto.

## Algoritmo Generalizado para Resolver Problemas de Búsqueda con aima-java

Para generalizar el proceso de implementación y resolución de problemas utilizando métodos de búsqueda en la biblioteca aima-java, podemos diseñar un algoritmo paso a paso que sirva como guía para desarrolladores que están aprendiendo a usar esta librería. Este algoritmo abarcará desde la definición del problema hasta la ejecución de la búsqueda, adaptable a cualquier tipo de problema y método de búsqueda soportado por la biblioteca.

### Paso 1: Definir el Problema

1. Identificar el problema: Determina claramente qué problema necesitas resolver y cómo puede modelarse como un problema de búsqueda.
2. Establecer estados: Define qué constituye un 'estado' en el contexto de tu problema. Un estado debe representar una configuración completa del problema en un momento dado.
3. Definir acciones: Especifica las acciones posibles que se pueden realizar en cada estado. Cada acción debe llevar de un estado a otro.

### Paso 2: Modelar el Problema en Java

1. Crear clases de estado: Implementa una clase para representar los estados del problema.
2. Definir transiciones: Implementa una función que dado un estado y una acción, retorne el nuevo estado resultante de aplicar esa acción.
3. Establecer el estado objetivo: Define cuál es el estado final deseado o las condiciones que debe cumplir un estado para considerarse solución.

### Paso 3: Implementar el Problema Usando aima-java

1. Importar clases necesarias: Asegúrate de importar las clases y paquetes adecuados de aima-java.
2. Implementar la interfaz `Problem`: Utiliza GeneralProblem o cualquier otra implementación relevante para definir tu problema.
3. Configurar el problema: Crea una instancia de tu problema, especificando el estado inicial, la función de acciones, la función de resultado, y la función de prueba de objetivo.

### Paso 4: Elegir y Configurar el Método de Búsqueda

1. Seleccionar el método de búsqueda: Elige entre búsqueda no informada o informada (heurística), dependiendo de la naturaleza del problema.
2. Configurar la búsqueda: Si es una búsqueda informada, define y proporciona una función heurística adecuada.
3. Crear una instancia de búsqueda: Utiliza la clase correspondiente de aima-java para instanciar el objeto de búsqueda.

### Paso 5: Ejecutar la Búsqueda y Manejar Resultados

1. Ejecutar la búsqueda: Utiliza el método de búsqueda para encontrar una solución al problema.
2. Procesar la solución: Extrae y procesa la solución del método de búsqueda, que puede ser una secuencia de acciones, un estado final, o una indicación de que no hay solución.
3. Manejo de errores: Asegúrate de manejar posibles errores o casos donde no se encuentre una solución.

### Paso 6: Validación y Pruebas

1. Validar la solución: Verifica que la solución obtenida resuelve efectivamente el problema según las reglas y condiciones definidas.
2. Realizar pruebas adicionales: Prueba el algoritmo con diferentes estados iniciales o variaciones del problema para asegurar su robustez y fiabilidad.

### Ejemplo de Implementación

Aquí un esquema básico de cómo se podría implementar y utilizar este algoritmo en código:

```java
// Paso 2: Modelar el problema
public class MiEstado {
    // detalles del estado
}

public class MiProblema extends GeneralProblem<MiEstado, Action> {
    public MiProblema(MiEstado initialState) {
        super(initialState, MiEstado::accionesPosibles, MiEstado::resultadoDeAccion, MiEstado::esObjetivo);
    }
}

// Paso 4: Configurar método de búsqueda
Problem<MiEstado, Action> problema = new MiProblema(estadoInicial);
SearchForActions<MiEstado, Action> search = new BreadthFirstSearch<>(new GraphSearch<>());

// Paso 5: Ejecutar búsqueda
Optional<List<Action>> actions = search.findActions(problema);
actions.ifPresent(System.out::println);
```

Este algoritmo generalizado y el ejemplo de implementación proporcionan una base sólida para entender y utilizar la biblioteca aima-java para resolver problemas de búsqueda en una variedad de contextos.
