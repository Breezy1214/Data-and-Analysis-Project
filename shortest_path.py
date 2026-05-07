"""
shortest_path.py
================

Shortest Path Project -- Implementation and Analysis of Dijkstra's
and Bellman-Ford Algorithms.

This program loads a weighted graph (a Maryland road network) from a
text file, then lets the user compute and visualize shortest paths
between any two cities using Dijkstra's algorithm or the Bellman-Ford
algorithm. It also compares the two algorithms side-by-side and
demonstrates Bellman-Ford's handling of negative weights and
negative-cycle detection.

Usage:
    python shortest_path.py
"""


# =====================================================================
# Section 1: Imports
# =====================================================================
import heapq
import time
import sys

import networkx
import matplotlib.pyplot as pyplot


# =====================================================================
# Section 2: Graph loading from file
# =====================================================================
def load_graph(file_path):
    """
    Load a weighted graph from a text file.

    Returns a tuple (graph, is_directed) where graph is a dict mapping
    node name -> list of (neighbor, weight) tuples. Raises ValueError
    on malformed input, FileNotFoundError if path is missing.
    """
    graph = {}
    is_directed = False
    header_seen = False
    declared_node_count = 0
    declared_edge_count = 0
    edges_loaded = 0

    try:
        input_file = open(file_path, "r", encoding="utf-8")
    except FileNotFoundError:
        raise FileNotFoundError(f"File not found: {file_path}") from None

    with input_file:
        for line_number, raw_line in enumerate(input_file, start=1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue

            tokens = line.split()
            if not header_seen:
                if len(tokens) != 3:
                    raise ValueError(
                        f"Line {line_number}: header must be "
                        f"'<node_count> <edge_count> <directed|undirected>'"
                    )
                try:
                    declared_node_count = int(tokens[0])
                    declared_edge_count = int(tokens[1])
                except ValueError:
                    raise ValueError(
                        f"Line {line_number}: node and edge counts must be integers"
                    )
                direction = tokens[2].lower()
                if direction not in ("directed", "undirected"):
                    raise ValueError(
                        f"Line {line_number}: third header token must be "
                        f"'directed' or 'undirected'"
                    )
                is_directed = (direction == "directed")
                header_seen = True
                continue

            if len(tokens) != 3:
                raise ValueError(
                    f"Line {line_number}: edge must be '<from> <to> <weight>'"
                )
            from_node, to_node, weight_text = tokens
            try:
                weight = float(weight_text)
            except ValueError:
                raise ValueError(
                    f"Line {line_number}: weight '{weight_text}' is not a number"
                )

            # Insert the edge into the adjacency dict.
            # ensure to_node exists as a key even if it has no outgoing edges of its own
            graph.setdefault(to_node, [])
            if not is_directed:
                graph.setdefault(to_node, []).append((from_node, weight))
            edges_loaded += 1

    if not header_seen:
        raise ValueError("File contains no header line")
    if edges_loaded != declared_edge_count:
        print(
            f"Warning: header declared {declared_edge_count} edges "
            f"but {edges_loaded} were read."
        )
    if len(graph) != declared_node_count:
        print(
            f"Warning: header declared {declared_node_count} nodes "
            f"but {len(graph)} unique nodes were read."
        )

    return graph, is_directed


def has_negative_weight(graph):
    """Return True if any edge in the graph has a negative weight."""
    for neighbors in graph.values():
        for _, weight in neighbors:
            if weight < 0:
                return True
    return False


def sorted_node_names(graph):
    """Return the graph's node names sorted alphabetically."""
    return sorted(graph.keys())


# =====================================================================
# Section 3: Dijkstra's algorithm
# =====================================================================
def dijkstra(graph, source, destination):
    """
    Compute the shortest path from source to destination using
    Dijkstra's algorithm with a binary min-heap.

    Returns (distance, path, runtime_ms). If no path exists, distance
    is float('inf') and path is the empty list.

    Assumes all edge weights are non-negative. Caller is responsible
    for warning the user if the graph has any negative edges.
    """
    start_time = time.perf_counter()

    distances = {node: float("inf") for node in graph}
    predecessors = {node: None for node in graph}
    distances[source] = 0.0

    priority_queue = [(0.0, source)]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        # heapq has no "decrease-key" operation, so each
        # time we find a shorter path to a node we PUSH a new entry instead
        # of updating the old one.
        if current_distance > distances[current_node]:
            continue

        # Early exit: because we always pop the smallest distance first, the
        # moment the destination comes out of the heap its distance is final.
        # No later relaxation can improve it, so we can stop the whole search.
        if current_node == destination:
            break

        # Relaxation step: try every outgoing edge from current_node and see
        # if going through current_node gives a shorter path to the neighbor.
        for neighbor, edge_weight in graph[current_node]:
            tentative_distance = current_distance + edge_weight
            if tentative_distance < distances[neighbor]:
                distances[neighbor] = tentative_distance
                predecessors[neighbor] = current_node
                heapq.heappush(priority_queue, (tentative_distance, neighbor))

    runtime_ms = (time.perf_counter() - start_time) * 1000.0
    path = reconstruct_path(predecessors, source, destination)
    return distances[destination], path, runtime_ms


# =====================================================================
# Section 4: Bellman-Ford algorithm
# =====================================================================
def bellman_ford(graph, source, destination):
    """
    Compute the shortest path from source to destination using the
    Bellman-Ford algorithm.

    Returns (distance, path, runtime_ms, negative_cycle).
    If a negative cycle is reachable from source, distance is
    float('-inf'), path is [], and negative_cycle is a list of node
    names forming a cycle (for reporting). Otherwise, negative_cycle
    is None and distance/path behave like dijkstra().
    """
    start_time = time.perf_counter()

    distances = {node: float("inf") for node in graph}
    predecessors = {node: None for node in graph}
    distances[source] = 0.0
    
    # Each pass relaxes information by one more edge along every path, so
    # after V-1 passes every shortest path has been fully discovered.
    for _ in range(len(graph) - 1):
        relaxed_any = False
        # Inner loop: try to relax EVERY edge in the graph this pass.
        for from_node, neighbors in graph.items():
            for to_node, weight in neighbors:
                if distances[from_node] + weight < distances[to_node]:
                    distances[to_node] = distances[from_node] + weight
                    predecessors[to_node] = from_node
                    relaxed_any = True
        if not relaxed_any:
            break

    negative_cycle = None
    for from_node, neighbors in graph.items():
        for to_node, weight in neighbors:
            if distances[from_node] + weight < distances[to_node]:
                negative_cycle = _trace_negative_cycle(predecessors, to_node)
                break
        if negative_cycle is not None:
            break

    runtime_ms = (time.perf_counter() - start_time) * 1000.0

    if negative_cycle is not None:
        return float("-inf"), [], runtime_ms, negative_cycle

    path = reconstruct_path(predecessors, source, destination)
    return distances[destination], path, runtime_ms, None


def _trace_negative_cycle(predecessors, start_node):
    """
    Given that start_node was relaxed in the V-th pass, walk back
    through predecessors V times to guarantee we land inside the
    cycle, then walk the cycle once to collect its nodes in order.
    """
    node = start_node
    for _ in range(len(predecessors)):
        node = predecessors[node]

    cycle = [node]
    cursor = predecessors[node]
    while cursor != node:
        cycle.append(cursor)
        cursor = predecessors[cursor]
    cycle.append(node)        # close the loop visually: A -> B -> C -> A
    cycle.reverse()           # we built it backwards; flip to forward order
    return cycle


# =====================================================================
# Section 5: Path reconstruction helper
# =====================================================================
def reconstruct_path(predecessors, source, destination):
    """
    Walk backwards through the predecessors map to build the path
    from source to destination. Returns [] if destination is
    unreachable from source.
    """

    if destination not in predecessors:
        return []
    
    # this means destination is unreachable from source.
    if predecessors[destination] is None and destination != source:
        return []

    # list ends up in REVERSE order (destination first, source last).
    path = []
    cursor = destination
    while cursor is not None:
        path.append(cursor)
        if cursor == source:
            break  # stop before dereferencing predecessors[source] (None)
        cursor = predecessors[cursor]

    # Sanity check: if we never hit source, the chain dead-ended somewhere
    # else -- treat that as "unreachable" rather than returning a partial
    # path that doesn't actually start at the source.
    if not path or path[-1] != source:
        return []

    path.reverse()  # flip so the path runs source -> ... -> destination
    return path


# =====================================================================
# Section 6: Visualization (networkx + matplotlib)
# =====================================================================
def _build_networkx_graph(graph, is_directed):
    """Convert our adjacency dict into a networkx Graph or DiGraph."""
    if is_directed:
        nx_graph = networkx.DiGraph()
    else:
        nx_graph = networkx.Graph()
    for node in graph:
        nx_graph.add_node(node)
    for from_node, neighbors in graph.items():
        for to_node, weight in neighbors:
            nx_graph.add_edge(from_node, to_node, weight=weight)
    return nx_graph


def visualize_graph(graph, is_directed):
    """
    Draw the entire graph with weight labels on every edge using a
    spring layout. Opens a matplotlib window.
    """
    nx_graph = _build_networkx_graph(graph, is_directed)
    layout = networkx.spring_layout(nx_graph, seed=42)

    figure = pyplot.figure(figsize=(10, 8))
    networkx.draw_networkx_nodes(nx_graph, layout, node_color="#a8d0e6", node_size=900)
    networkx.draw_networkx_labels(nx_graph, layout, font_size=9)
    networkx.draw_networkx_edges(nx_graph, layout, edge_color="#888888", width=1.4)

    edge_labels = {
        (u, v): f"{data['weight']:g}"
        for u, v, data in nx_graph.edges(data=True)
    }
    networkx.draw_networkx_edge_labels(nx_graph, layout, edge_labels=edge_labels, font_size=8)

    pyplot.title("Graph (all nodes and edges)")
    pyplot.axis("off")
    pyplot.tight_layout()
    pyplot.show()
    pyplot.close(figure)


def visualize_path(graph, is_directed, path, output_file="path.png"):
    """
    Draw the graph with the given shortest path highlighted in red.
    Non-path edges are drawn in light grey. Saves the figure to
    `output_file` (default 'path.png') and displays it.
    """
    nx_graph = _build_networkx_graph(graph, is_directed)
    layout = networkx.spring_layout(nx_graph, seed=42)

    # Build a set of edges that belong to the shortest path. Using a SET
    # graph into "highlight" or "non-path" buckets.
    path_edges = set()
    for index in range(len(path) - 1):
        path_edges.add((path[index], path[index + 1]))
        # For undirected graphs we also add the reverse pair
        if not is_directed:
            path_edges.add((path[index + 1], path[index]))

    # Split every edge in the graph into two lists so we can draw them in
    # two passes (grey first, then red on top -- so the highlighted path
    # always sits visually above the rest of the graph).
    highlight_edges = []
    non_path_edges = []
    for u, v in nx_graph.edges():
        bucket = highlight_edges if (u, v) in path_edges else non_path_edges
        bucket.append((u, v))

    figure = pyplot.figure(figsize=(10, 8))
    node_colors = ["#ffb4a2" if node in path else "#a8d0e6" for node in nx_graph.nodes()]
    networkx.draw_networkx_nodes(nx_graph, layout, node_color=node_colors, node_size=900)
    networkx.draw_networkx_labels(nx_graph, layout, font_size=9)

    networkx.draw_networkx_edges(
        nx_graph, layout, edgelist=non_path_edges,
        edge_color="#cccccc", width=1.0,
    )
    networkx.draw_networkx_edges(
        nx_graph, layout, edgelist=highlight_edges,
        edge_color="#d62828", width=3.0,
    )

    edge_labels = {
        (u, v): f"{data['weight']:g}"
        for u, v, data in nx_graph.edges(data=True)
    }
    networkx.draw_networkx_edge_labels(nx_graph, layout, edge_labels=edge_labels, font_size=8)

    pyplot.title(f"Shortest path: {' -> '.join(path)}")
    pyplot.axis("off")
    pyplot.tight_layout()
    pyplot.savefig(output_file, dpi=150, bbox_inches="tight")
    print(f"Path figure saved to {output_file}")
    pyplot.show()
    pyplot.close(figure)


# =====================================================================
# Section 7: Menu and main()
# =====================================================================
def print_banner():
    """Print the program banner."""
    print("=" * 50)
    print("   Shortest Path Project")
    print("   Dijkstra & Bellman-Ford on Maryland Roads")
    print("=" * 50)


def print_menu():
    """Print the main menu options."""
    print()
    print("=" * 50)
    print("   Shortest Path Project")
    print("=" * 50)
    print("1. Show graph data (nodes and edges)")
    print("2. Run Dijkstra's algorithm")
    print("3. Run Bellman-Ford algorithm")
    print("4. Compare both algorithms")
    print("5. Visualize the graph")
    print("6. Visualize a shortest path")
    print("7. Load a different file")
    print("8. Quit")


def prompt_for_node(graph, prompt_text):
    nodes = sorted_node_names(graph)
    while True:
        choice = input(prompt_text).strip()
        if choice in graph:
            return choice
        print(f"Unknown node: {choice!r}")
        print(f"Available nodes: {', '.join(nodes)}")


def prompt_for_file_path(default_path="graph.txt"):
    """Prompt for an input file path, defaulting to graph.txt."""
    raw = input(f"Input file [{default_path}]: ").strip()
    # Windows Explorer's "Copy as path" wraps the path in double quotes;
    # strip a matching pair so pasted paths just work.
    if len(raw) >= 2 and raw[0] == raw[-1] and raw[0] in ('"', "'"):
        raw = raw[1:-1].strip()
    return raw if raw else default_path


def print_load_summary(graph, is_directed, file_path):
    """Print 'Loaded N nodes and M edges from <path>'."""
    edge_count = sum(len(neighbors) for neighbors in graph.values())
    if not is_directed:
        edge_count //= 2
    print(f"Loaded {len(graph)} nodes and {edge_count} edges from {file_path}.")


def show_graph_data(graph, is_directed):
    """Print the graph as a list of nodes and edges."""
    print(f"Directed: {is_directed}")
    print(f"Nodes ({len(graph)}): {', '.join(sorted_node_names(graph))}")

    # Undirected edges are stored twice in the adjacency dict (A->B AND
    # B->A), so a naive print would show every edge twice.
    seen = set()
    edges = []
    for from_node in sorted_node_names(graph):
        for to_node, weight in graph[from_node]:
            if not is_directed:
                key = tuple(sorted([from_node, to_node]))
                if key in seen:
                    continue
                seen.add(key)
            edges.append((from_node, to_node, weight))
    print(f"Edges ({len(edges)}):")
    for from_node, to_node, weight in edges:
        arrow = "->" if is_directed else "--"
        print(f"  {from_node} {arrow} {to_node}  ({weight:g})")


def confirm_dijkstra_with_negatives(graph):
    if not has_negative_weight(graph):
        return True
    print(
        "Warning: graph contains negative-weight edges. "
        "Dijkstra's algorithm is not guaranteed to be correct on such graphs."
    )
    answer = input("Continue with Dijkstra anyway? [Y/n]: ").strip().lower()
    return answer in ("", "y", "yes")


def format_path(path):
    """Format a list of node names as 'A -> B -> C', or '(no path)'."""
    return " -> ".join(path) if path else "(no path)"


def format_distance(distance):
    """Format a distance for display."""
    if distance == float("inf"):
        return "INF"
    if distance == float("-inf"):
        return "-INF"
    return f"{distance:g}"


def run_dijkstra_option(graph):
    """Handle menu option 2: prompt and run Dijkstra."""
    if not confirm_dijkstra_with_negatives(graph):
        return
    source = prompt_for_node(graph, "Source node: ")
    destination = prompt_for_node(graph, "Destination node: ")
    distance, path, runtime_ms = dijkstra(graph, source, destination)
    print()
    print(f"Source:      {source}")
    print(f"Destination: {destination}")
    print(f"Distance:    {format_distance(distance)}")
    print(f"Path:        {format_path(path)}")
    print(f"Runtime:     {runtime_ms:.3f} ms")
    if not path and distance == float("inf"):
        print(f"No path from {source} to {destination}")


def run_bellman_ford_option(graph):
    """Handle menu option 3: prompt and run Bellman-Ford."""
    source = prompt_for_node(graph, "Source node: ")
    destination = prompt_for_node(graph, "Destination node: ")
    distance, path, runtime_ms, negative_cycle = bellman_ford(graph, source, destination)
    print()
    print(f"Source:      {source}")
    print(f"Destination: {destination}")
    if negative_cycle is not None:
        print("Negative cycle detected -- no shortest path defined.")
        print(f"Cycle: {' -> '.join(negative_cycle)}")
    else:
        print(f"Distance:    {format_distance(distance)}")
        print(f"Path:        {format_path(path)}")
        if not path and distance == float("inf"):
            print(f"No path from {source} to {destination}")
    print(f"Runtime:     {runtime_ms:.3f} ms")


def run_comparison_option(graph):
    """Handle menu option 4: run both algorithms and compare."""
    source = prompt_for_node(graph, "Source node: ")
    destination = prompt_for_node(graph, "Destination node: ")

    if has_negative_weight(graph):
        print(
            "Note: graph has negative-weight edges. Dijkstra's result may be incorrect; "
            "running it anyway for comparison."
        )

    dij_distance, dij_path, dij_ms = dijkstra(graph, source, destination)
    bf_distance, bf_path, bf_ms, bf_cycle = bellman_ford(graph, source, destination)

    print()
    print(f"Source:      {source}")
    print(f"Destination: {destination}")
    print()
    header = f"{'Algorithm':<14} {'Distance':<10} {'Path':<50} {'Time (ms)':>10}"
    print(header)
    print("-" * len(header))
    print(
        f"{'Dijkstra':<14} {format_distance(dij_distance):<10} "
        f"{format_path(dij_path):<50} {dij_ms:>10.3f}"
    )
    if bf_cycle is not None:
        cycle_text = "Negative cycle: " + " -> ".join(bf_cycle)
        print(
            f"{'Bellman-Ford':<14} {'-INF':<10} {cycle_text:<50} {bf_ms:>10.3f}"
        )
    else:
        print(
            f"{'Bellman-Ford':<14} {format_distance(bf_distance):<10} "
            f"{format_path(bf_path):<50} {bf_ms:>10.3f}"
        )
    print()
    if bf_cycle is None and dij_distance == bf_distance:
        print("Both algorithms agree.")
    else:
        print("Algorithms differ -- see notes above (likely negative weights).")


def run_visualize_path_option(graph, is_directed):
    """Handle menu option 6: prompt and visualize a shortest path."""
    use_dijkstra = confirm_dijkstra_with_negatives(graph)
    source = prompt_for_node(graph, "Source node: ")
    destination = prompt_for_node(graph, "Destination node: ")
    if use_dijkstra:
        _, path, _ = dijkstra(graph, source, destination)
    else:
        _, path, _, negative_cycle = bellman_ford(graph, source, destination)
        if negative_cycle is not None:
            print("Negative cycle detected -- cannot visualize a shortest path.")
            return
    if not path:
        print(f"No path from {source} to {destination} -- nothing to draw.")
        return
    visualize_path(graph, is_directed, path)


def main():
    """Program entry point: load a graph and run the menu loop."""
    print_banner()
    file_path = prompt_for_file_path()
    try:
        graph, is_directed = load_graph(file_path)
    except (FileNotFoundError, ValueError) as error:
        print(f"Error loading {file_path}: {error}")
        sys.exit(1)
    print_load_summary(graph, is_directed, file_path)

    while True:
        print_menu()
        choice = input("Choose an option [1-8]: ").strip()
        if choice == "1":
            show_graph_data(graph, is_directed)
        elif choice == "2":
            run_dijkstra_option(graph)
        elif choice == "3":
            run_bellman_ford_option(graph)
        elif choice == "4":
            run_comparison_option(graph)
        elif choice == "5":
            visualize_graph(graph, is_directed)
        elif choice == "6":
            run_visualize_path_option(graph, is_directed)
        elif choice == "7":
            file_path = prompt_for_file_path(file_path)
            try:
                graph, is_directed = load_graph(file_path)
                print_load_summary(graph, is_directed, file_path)
            except (FileNotFoundError, ValueError) as error:
                print(f"Error loading {file_path}: {error}")
        elif choice == "8":
            print("Goodbye.")
            return
        else:
            print(f"Unknown option: {choice!r}")


if __name__ == "__main__":
    main()
