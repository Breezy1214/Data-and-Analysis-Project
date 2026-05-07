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
        raise FileNotFoundError("File not found: " + file_path)

    # `with` makes sure the file gets closed when this block ends.
    with input_file:
        line_number = 0
        for raw_line in input_file:
            line_number = line_number + 1

            # Drop anything after a `#` (a comment), then trim whitespace.
            line = raw_line.split("#", 1)[0].strip()
            if line == "":
                continue

            tokens = line.split()

            if not header_seen:
                if len(tokens) != 3:
                    raise ValueError(
                        "Line " + str(line_number) + ": header must be "
                        "'<node_count> <edge_count> <directed|undirected>'"
                    )
                try:
                    declared_node_count = int(tokens[0])
                    declared_edge_count = int(tokens[1])
                except ValueError:
                    raise ValueError(
                        "Line " + str(line_number) +
                        ": node and edge counts must be integers"
                    )
                direction = tokens[2].lower()
                if direction != "directed" and direction != "undirected":
                    raise ValueError(
                        "Line " + str(line_number) +
                        ": third header token must be 'directed' or 'undirected'"
                    )
                if direction == "directed":
                    is_directed = True
                else:
                    is_directed = False
                header_seen = True
                continue

            # Every non-header line should be: <from> <to> <weight>
            if len(tokens) != 3:
                raise ValueError(
                    "Line " + str(line_number) +
                    ": edge must be '<from> <to> <weight>'"
                )
            from_node = tokens[0]
            to_node = tokens[1]
            weight_text = tokens[2]
            try:
                weight = float(weight_text)
            except ValueError:
                raise ValueError(
                    "Line " + str(line_number) +
                    ": weight '" + weight_text + "' is not a number"
                )

            # Add an entry for from_node if it isn't already in the dict,
            # then record the edge from_node -> to_node.
            if from_node not in graph:
                graph[from_node] = []
            graph[from_node].append((to_node, weight))

            # For undirected graphs we also record the reverse edge
            # to_node -> from_node, since either direction is valid.
            # For directed graphs we still need to make sure to_node
            # appears as a key, even if it has no outgoing edges of
            # its own.
            if not is_directed:
                if to_node not in graph:
                    graph[to_node] = []
                graph[to_node].append((from_node, weight))
            else:
                if to_node not in graph:
                    graph[to_node] = []

            edges_loaded = edges_loaded + 1

    if not header_seen:
        raise ValueError("File contains no header line")
    if edges_loaded != declared_edge_count:
        print(
            "Warning: header declared " + str(declared_edge_count) +
            " edges but " + str(edges_loaded) + " were read."
        )
    if len(graph) != declared_node_count:
        print(
            "Warning: header declared " + str(declared_node_count) +
            " nodes but " + str(len(graph)) + " unique nodes were read."
        )

    return graph, is_directed


def has_negative_weight(graph):
    """Return True if any edge in the graph has a negative weight."""
    for neighbors in graph.values():
        for edge in neighbors:
            weight = edge[1]
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

    # Start every node at distance "infinity"; we'll lower these as
    # we find shorter paths from the source.
    distances = {}
    predecessors = {}
    for node in graph:
        distances[node] = float("inf")
        predecessors[node] = None
    distances[source] = 0.0

    # The priority queue holds (distance_so_far, node) pairs and
    # always pops the smallest distance first.
    priority_queue = []
    heapq.heappush(priority_queue, (0.0, source))

    while len(priority_queue) > 0:
        popped = heapq.heappop(priority_queue)
        current_distance = popped[0]
        current_node = popped[1]

        # heapq has no "decrease-key" operation, so each time we find
        # a shorter path to a node we PUSH a new entry instead of
        # updating the old one. That means stale, larger entries can
        # still be in the heap -- skip them when we see them.
        if current_distance > distances[current_node]:
            continue

        # Early exit: because we always pop the smallest distance
        # first, the moment the destination comes out of the heap its
        # distance is final. No later relaxation can improve it.
        if current_node == destination:
            break

        # Relaxation step: for every edge from current_node, check
        # whether going through current_node gives a shorter path to
        # the neighbor than what we have stored.
        for edge in graph[current_node]:
            neighbor = edge[0]
            edge_weight = edge[1]
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

    distances = {}
    predecessors = {}
    for node in graph:
        distances[node] = float("inf")
        predecessors[node] = None
    distances[source] = 0.0

    # Each pass relaxes information by one more edge along every
    # path, so after V-1 passes every shortest path has been fully
    # discovered (where V is the number of nodes). We don't need
    # the loop counter itself, only the number of repetitions, so
    # we use `_` as the variable name by convention.
    number_of_nodes = len(graph)
    for _ in range(number_of_nodes - 1):
        relaxed_any = False
        # Inner loop: try to relax EVERY edge in the graph this pass.
        for from_node in graph:
            for edge in graph[from_node]:
                to_node = edge[0]
                weight = edge[1]
                if distances[from_node] + weight < distances[to_node]:
                    distances[to_node] = distances[from_node] + weight
                    predecessors[to_node] = from_node
                    relaxed_any = True
        # If no edge was relaxed this pass, no later pass will relax
        # anything either -- we can stop early.
        if not relaxed_any:
            break

    # After V-1 passes, distances should be final. If we can STILL
    # relax an edge, that means a negative cycle is reachable.
    negative_cycle = None
    for from_node in graph:
        for edge in graph[from_node]:
            to_node = edge[0]
            weight = edge[1]
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
    # Walk back V steps; the counter itself isn't needed, only the
    # repetitions, so we use `_` by convention.
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

    # destination is unreachable from source.
    if predecessors[destination] is None and destination != source:
        return []

    # The list ends up in REVERSE order (destination first, source last).
    path = []
    cursor = destination
    while cursor is not None:
        path.append(cursor)
        if cursor == source:
            break  # stop before dereferencing predecessors[source] (None)
        cursor = predecessors[cursor]

    # Sanity check: if we never hit source, the chain dead-ended
    # somewhere else -- treat that as "unreachable" rather than
    # returning a partial path that doesn't actually start at source.
    if len(path) == 0 or path[-1] != source:
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
    for from_node in graph:
        for edge in graph[from_node]:
            to_node = edge[0]
            weight = edge[1]
            nx_graph.add_edge(from_node, to_node, weight=weight)
    return nx_graph


def format_weight(weight):
    """Format an edge weight, dropping the decimal part if whole."""
    if weight == int(weight):
        return str(int(weight))
    else:
        return str(weight)


def visualize_graph(graph, is_directed):
    """
    Draw the entire graph with weight labels on every edge using a
    spring layout. Opens a matplotlib window.
    """
    nx_graph = _build_networkx_graph(graph, is_directed)
    layout = networkx.spring_layout(nx_graph, seed=42)

    figure = pyplot.figure(figsize=(10, 8))
    networkx.draw_networkx_nodes(
        nx_graph, layout, node_color="#a8d0e6", node_size=900
    )
    networkx.draw_networkx_labels(nx_graph, layout, font_size=9)
    networkx.draw_networkx_edges(
        nx_graph, layout, edge_color="#888888", width=1.4
    )

    # Build a {(u, v): "weight"} dict so we can label every edge.
    edge_labels = {}
    for u, v, data in nx_graph.edges(data=True):
        edge_labels[(u, v)] = format_weight(data["weight"])
    networkx.draw_networkx_edge_labels(
        nx_graph, layout, edge_labels=edge_labels, font_size=8
    )

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

    # Build a set of edges that belong to the shortest path. We use
    # a set so we can quickly check "is this edge part of the path?".
    path_edges = set()
    for index in range(len(path) - 1):
        path_edges.add((path[index], path[index + 1]))
        # For undirected graphs we also add the reverse pair.
        if not is_directed:
            path_edges.add((path[index + 1], path[index]))

    # Split every edge into two lists so we can draw them in two
    # passes: grey first, then red on top. That way the highlighted
    # path always sits visually above the rest of the graph.
    highlight_edges = []
    non_path_edges = []
    for edge in nx_graph.edges():
        u = edge[0]
        v = edge[1]
        if (u, v) in path_edges:
            highlight_edges.append((u, v))
        else:
            non_path_edges.append((u, v))

    figure = pyplot.figure(figsize=(10, 8))

    # Pick a colour for each node: red-ish if it's on the path, blue otherwise.
    node_colors = []
    for node in nx_graph.nodes():
        if node in path:
            node_colors.append("#ffb4a2")
        else:
            node_colors.append("#a8d0e6")
    networkx.draw_networkx_nodes(
        nx_graph, layout, node_color=node_colors, node_size=900
    )
    networkx.draw_networkx_labels(nx_graph, layout, font_size=9)

    networkx.draw_networkx_edges(
        nx_graph, layout, edgelist=non_path_edges,
        edge_color="#cccccc", width=1.0,
    )
    networkx.draw_networkx_edges(
        nx_graph, layout, edgelist=highlight_edges,
        edge_color="#d62828", width=3.0,
    )

    edge_labels = {}
    for u, v, data in nx_graph.edges(data=True):
        edge_labels[(u, v)] = format_weight(data["weight"])
    networkx.draw_networkx_edge_labels(
        nx_graph, layout, edge_labels=edge_labels, font_size=8
    )

    pyplot.title("Shortest path: " + " -> ".join(path))
    pyplot.axis("off")
    pyplot.tight_layout()
    pyplot.savefig(output_file, dpi=150, bbox_inches="tight")
    print("Path figure saved to " + output_file)
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
        print("Unknown node: " + repr(choice))
        print("Available nodes: " + ", ".join(nodes))


def prompt_for_file_path(default_path="graph.txt"):
    """Prompt for an input file path, defaulting to graph.txt."""
    raw = input("Input file [" + default_path + "]: ").strip()
    # Windows Explorer's "Copy as path" wraps the path in double
    # quotes; strip a matching pair so pasted paths just work.
    if len(raw) >= 2 and raw[0] == raw[-1] and (raw[0] == '"' or raw[0] == "'"):
        raw = raw[1:-1].strip()
    if raw == "":
        return default_path
    else:
        return raw


def print_load_summary(graph, is_directed, file_path):
    """Print 'Loaded N nodes and M edges from <path>'."""
    edge_count = 0
    for neighbors in graph.values():
        edge_count = edge_count + len(neighbors)
    # Undirected graphs store every edge twice (A->B and B->A), so
    # divide by 2 to get the number of distinct edges.
    if not is_directed:
        edge_count = edge_count // 2
    print(
        "Loaded " + str(len(graph)) + " nodes and " +
        str(edge_count) + " edges from " + file_path + "."
    )


def show_graph_data(graph, is_directed):
    """Print the graph as a list of nodes and edges."""
    print("Directed: " + str(is_directed))
    print(
        "Nodes (" + str(len(graph)) + "): " +
        ", ".join(sorted_node_names(graph))
    )

    # Undirected edges are stored twice in the adjacency dict (A->B
    # AND B->A), so a naive print would show every edge twice. We
    # track edges we've already seen and skip the duplicates.
    seen = set()
    edges = []
    for from_node in sorted_node_names(graph):
        for edge in graph[from_node]:
            to_node = edge[0]
            weight = edge[1]
            if not is_directed:
                key = tuple(sorted([from_node, to_node]))
                if key in seen:
                    continue
                seen.add(key)
            edges.append((from_node, to_node, weight))

    print("Edges (" + str(len(edges)) + "):")
    if is_directed:
        arrow = "->"
    else:
        arrow = "--"
    for edge in edges:
        from_node = edge[0]
        to_node = edge[1]
        weight = edge[2]
        print(
            "  " + from_node + " " + arrow + " " + to_node +
            "  (" + format_weight(weight) + ")"
        )


def confirm_dijkstra_with_negatives(graph):
    if not has_negative_weight(graph):
        return True
    print(
        "Warning: graph contains negative-weight edges. "
        "Dijkstra's algorithm is not guaranteed to be correct on such graphs."
    )
    answer = input("Continue with Dijkstra anyway? [Y/n]: ").strip().lower()
    if answer == "" or answer == "y" or answer == "yes":
        return True
    else:
        return False


def format_path(path):
    """Format a list of node names as 'A -> B -> C', or '(no path)'."""
    if len(path) == 0:
        return "(no path)"
    else:
        return " -> ".join(path)


def format_distance(distance):
    """Format a distance for display."""
    if distance == float("inf"):
        return "INF"
    if distance == float("-inf"):
        return "-INF"
    return format_weight(distance)


def format_runtime_ms(runtime_ms):
    """Format a runtime in milliseconds with three decimal places."""
    return "{:.3f}".format(runtime_ms)


def run_dijkstra_option(graph):
    """Handle menu option 2: prompt and run Dijkstra."""
    if not confirm_dijkstra_with_negatives(graph):
        return
    source = prompt_for_node(graph, "Source node: ")
    destination = prompt_for_node(graph, "Destination node: ")
    distance, path, runtime_ms = dijkstra(graph, source, destination)
    print()
    print("Source:      " + source)
    print("Destination: " + destination)
    print("Distance:    " + format_distance(distance))
    print("Path:        " + format_path(path))
    print("Runtime:     " + format_runtime_ms(runtime_ms) + " ms")
    if len(path) == 0 and distance == float("inf"):
        print("No path from " + source + " to " + destination)


def run_bellman_ford_option(graph):
    """Handle menu option 3: prompt and run Bellman-Ford."""
    source = prompt_for_node(graph, "Source node: ")
    destination = prompt_for_node(graph, "Destination node: ")
    distance, path, runtime_ms, negative_cycle = bellman_ford(
        graph, source, destination
    )
    print()
    print("Source:      " + source)
    print("Destination: " + destination)
    if negative_cycle is not None:
        print("Negative cycle detected -- no shortest path defined.")
        print("Cycle: " + " -> ".join(negative_cycle))
    else:
        print("Distance:    " + format_distance(distance))
        print("Path:        " + format_path(path))
        if len(path) == 0 and distance == float("inf"):
            print("No path from " + source + " to " + destination)
    print("Runtime:     " + format_runtime_ms(runtime_ms) + " ms")


def run_comparison_option(graph):
    """Handle menu option 4: run both algorithms and compare."""
    source = prompt_for_node(graph, "Source node: ")
    destination = prompt_for_node(graph, "Destination node: ")

    if has_negative_weight(graph):
        print(
            "Note: graph has negative-weight edges. Dijkstra's result may "
            "be incorrect; running it anyway for comparison."
        )

    dij_distance, dij_path, dij_ms = dijkstra(graph, source, destination)
    bf_distance, bf_path, bf_ms, bf_cycle = bellman_ford(
        graph, source, destination
    )

    print()
    print("Source:      " + source)
    print("Destination: " + destination)
    print()

    # Build a fixed-width table by padding each column to a set width.
    header = (
        "Algorithm".ljust(14) + " " +
        "Distance".ljust(10) + " " +
        "Path".ljust(50) + " " +
        "Time (ms)".rjust(10)
    )
    print(header)
    print("-" * len(header))

    dij_row = (
        "Dijkstra".ljust(14) + " " +
        format_distance(dij_distance).ljust(10) + " " +
        format_path(dij_path).ljust(50) + " " +
        format_runtime_ms(dij_ms).rjust(10)
    )
    print(dij_row)

    if bf_cycle is not None:
        cycle_text = "Negative cycle: " + " -> ".join(bf_cycle)
        bf_row = (
            "Bellman-Ford".ljust(14) + " " +
            "-INF".ljust(10) + " " +
            cycle_text.ljust(50) + " " +
            format_runtime_ms(bf_ms).rjust(10)
        )
    else:
        bf_row = (
            "Bellman-Ford".ljust(14) + " " +
            format_distance(bf_distance).ljust(10) + " " +
            format_path(bf_path).ljust(50) + " " +
            format_runtime_ms(bf_ms).rjust(10)
        )
    print(bf_row)

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
    # We only need the path here, not the distance or runtime, so
    # we pull index 1 out of the result tuple.
    if use_dijkstra:
        dijkstra_result = dijkstra(graph, source, destination)
        path = dijkstra_result[1]
    else:
        bellman_result = bellman_ford(graph, source, destination)
        path = bellman_result[1]
        negative_cycle = bellman_result[3]
        if negative_cycle is not None:
            print("Negative cycle detected -- cannot visualize a shortest path.")
            return
    if len(path) == 0:
        print(
            "No path from " + source + " to " + destination +
            " -- nothing to draw."
        )
        return
    visualize_path(graph, is_directed, path)


def main():
    """Program entry point: load a graph and run the menu loop."""
    print_banner()
    file_path = prompt_for_file_path()
    try:
        graph, is_directed = load_graph(file_path)
    except (FileNotFoundError, ValueError) as error:
        print("Error loading " + file_path + ": " + str(error))
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
                print("Error loading " + file_path + ": " + str(error))
        elif choice == "8":
            print("Goodbye.")
            return
        else:
            print("Unknown option: " + repr(choice))


if __name__ == "__main__":
    main()
