## Metro Route Finder Project

This project involves implementing various search algorithms to find the shortest path in the Paris RATP metro system. The algorithms implemented include basic, heuristic, and optimal search strategies. The goal is to simulate each algorithm on the metro system to find the fastest route between two stations and compare their performance.

### Project Structure

- **Data Loading**: Loads metro data from a JSON file. There are 14 metro lines and 383 stations.
- **Algorithms**:
  - **BFS (Breadth-First Search)**: A basic search algorithm that explores all possible paths level by level.
  - **Random Search**: A basic search algorithm that randomly selects paths.
  - **Greedy Search**: A heuristic search that selects the most promising path based on an estimated distance to the goal.
  - **Beam Search**: A heuristic search that limits the number of paths explored at each level.
  - **A* Search**: An optimal search algorithm combining uniform cost search, branch and bound, and heuristic estimation to find the shortest path.

### Setup

1. Ensure you have Python installed on your system.
2. Place the `metro_data.json` file in the `data` directory.
3. Install required Python libraries (if any).

### Running the Project

To run the project, execute the `main` function in the `main.py` script:

```bash
python main.py
```

### Algorithms

#### BFS (Breadth-First Search)

```python
def bfs_search2(metro_data, depart, arrivee):
    ...
```

- **Description**: Explores all possible paths level by level, guaranteeing the shortest path in an unweighted graph.
- **Usage**: Finds the shortest path without considering travel time or line changes.
- **Performance**: May consume significant memory and time for large graphs.

#### Random Search

```python
def random_search2(metro_data, start, goal):
    ...
```

- **Description**: Randomly selects paths until the goal is found. Launched it 5 times to get average results with memory usage and execution time of the algorithm.
- **Usage**: Used for comparison purposes due to its non-deterministic nature.
- **Performance**: Inefficient and unreliable, used primarily as a baseline for comparison.

#### Greedy Search

```python
def greedy_search2(metro_data, start, goal):
    ...
```

- **Description**: Uses a heuristic to guide the search towards the goal by selecting the most promising path.
- **Usage**: Faster than BFS but may not always find the optimal path.
- **Performance**: Efficient in terms of time but can get stuck in local minima.

#### Beam Search

```python
def beam_search2(metro_data, start, goal, width=2):
    ...
```

- **Description**: Limits the number of paths explored at each level to the most promising ones based on a heuristic.
- **Usage**: Balances between breadth-first search and greedy search by limiting the breadth.
- **Performance**: More efficient than BFS in terms of memory and time, but the quality of the path depends on the beam width.

#### A* Search

```python
def a_star_search2(metro_data, start, goal):
    ...
```

- **Description**: Combines uniform cost search, branch and bound, and heuristic estimation to find the optimal path.
- **Usage**: Finds the shortest path by considering both the cost to reach the current node and the estimated cost to the goal.
- **Performance**: Guarantees the shortest path and is more efficient than BFS, but may still be memory-intensive.

### Example Application

The example application uses the RATP metro system to test the routes between "Alexandre Dumas" and "Balard" stations using all five algorithms. Each algorithm is simulated, and the results, including path quality, number of iterations, execution speed, memory usage, and path length, are logged and compared.

### Results Comparison

The results are compared based on:

- **Path Quality**: The length and complexity of the path.
- **Number of Iterations**: The number of iterations taken to find the path.
- **Speed**: The execution time of the algorithm.
- **Memory Usage**: The memory consumed during execution.
- **Implementation Complexity**: The complexity of implementing the algorithm.

### Logging and Output

The results are logged in the `stat.txt` file, including the details of the paths found, iterations, execution time, memory usage, and more.

### Conclusion

This project demonstrates the implementation and comparison of various search algorithms for finding the shortest path in a metro system. Each algorithm has its strengths and weaknesses, and their performance varies based on the specific requirements and constraints of the problem. The choice of algorithm depends on the balance between optimality, efficiency, and resource usage.
