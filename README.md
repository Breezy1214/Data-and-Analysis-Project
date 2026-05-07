# Shortest Path Project

Single-file Python program that implements **Dijkstra's** and
**Bellman-Ford** shortest-path algorithms over a Maryland road
network, with side-by-side comparison and a `networkx` +
`matplotlib` visualization.

## Requirements

Python 3.9 or later. Everything else (`networkx`, `matplotlib`)
is installed automatically by the launcher scripts on first run.

If Python isn't installed:

- **Windows:** download from <https://www.python.org/downloads/> and
  check "Add Python to PATH" during installation.
- **macOS:** `brew install python` (Homebrew) or download from python.org.
- **Linux:** use your distro's package manager, e.g. `apt install python3 python3-pip`.

## How to Run

### Windows

Double-click `run.bat` — the launcher installs `networkx` and
`matplotlib` on first run, then starts the program. A console window
stays open until you press a key.

Or, from a terminal:

```
run.bat
```

### macOS / Linux

From a terminal in the project directory:

```
bash run.sh
```

(or `chmod +x run.sh` once, then `./run.sh` after that.)

### Run directly with Python (no launcher)

If you already have the dependencies installed:

```
python shortest_path.py        # Windows
python3 shortest_path.py       # macOS / Linux
```

You will be prompted for an input file (default `graph.txt`), then
shown a menu:

```
1. Show graph data (nodes and edges)
2. Run Dijkstra's algorithm
3. Run Bellman-Ford algorithm
4. Compare both algorithms
5. Visualize the graph
6. Visualize a shortest path
7. Load a different file
8. Quit
```

## Sample Input Files

| File                  | What it shows                                                                                       |
|-----------------------|-----------------------------------------------------------------------------------------------------|
| `graph.txt`           | 9 central-Maryland cities, 14 roads, undirected -- default sample, hand-verifiable.                 |
| `maryland.txt`        | 15 Maryland cities with realistic road distances -- demo set.                                       |
| `negative.txt`        | Directed graph with one negative edge -- Dijkstra returns the wrong answer, Bellman-Ford the right. |
| `negative_cycle.txt`  | Directed graph with a negative cycle -- Bellman-Ford detects and reports it.                        |

## Example Session

```
$ python shortest_path.py
==================================================
   Shortest Path Project
   Dijkstra & Bellman-Ford on Maryland Roads
==================================================
Input file [graph.txt]:
Loaded 9 nodes and 14 edges from graph.txt.

==================================================
   Shortest Path Project
==================================================
1. Show graph data (nodes and edges)
2. Run Dijkstra's algorithm
3. Run Bellman-Ford algorithm
4. Compare both algorithms
5. Visualize the graph
6. Visualize a shortest path
7. Load a different file
8. Quit
Choose an option [1-8]: 4
Source node: Baltimore
Destination node: Hagerstown

Source:      Baltimore
Destination: Hagerstown

Algorithm      Distance   Path                                               Time (ms)
---------------------------------------------------------------------------------------
Dijkstra       75         Baltimore -> Frederick -> Hagerstown                    0.123
Bellman-Ford   75         Baltimore -> Frederick -> Hagerstown                    2.456

Both algorithms agree.
```

## Demonstrating Bellman-Ford's Strengths

Load `negative.txt` (option 7), then run option 4:

```
Source node: A
Destination node: D

Note: graph has negative-weight edges. Dijkstra's result may be incorrect; running it anyway for comparison.

Algorithm      Distance   Path                                               Time (ms)
---------------------------------------------------------------------------------------
Dijkstra       2          A -> D                                                  0.020
Bellman-Ford   -4         A -> B -> C -> D                                        0.030

Algorithms differ -- see notes above (likely negative weights).
```

Dijkstra terminates at the direct A -> D edge (distance 2) without
exploring the longer A -> B -> C -> D route, which has a -10 edge that
makes its total -4. Bellman-Ford finds the true shortest path.

Then load `negative_cycle.txt` and run option 3:

```
Source node: A
Destination node: D

Source:      A
Destination: D
Negative cycle detected -- no shortest path defined.
Cycle: A -> B -> C -> A
```

## Input File Format

Plain text. Lines starting with `#` are comments; blank lines are
ignored. The first non-comment line is a header:

```
<node_count> <edge_count> <directed|undirected>
```

Each subsequent line is one edge:

```
<from> <to> <weight>
```

Node names are strings without spaces. Weights are integers or
decimals (negatives allowed for Bellman-Ford demos). For
`undirected` graphs the loader automatically adds the reverse edge.

## Notes

- Dijkstra uses a binary min-heap (`heapq`). Complexity:
  *O((V + E) log V)*.
- Bellman-Ford uses up to `V - 1` relaxation passes plus one
  detection pass. Complexity: *O(V * E)*.
- When you visualize a shortest path (option 6), the figure is also
  saved to `path.png` for inclusion in the report.
