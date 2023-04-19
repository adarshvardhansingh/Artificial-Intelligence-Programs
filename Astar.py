import heapq
import math

def astar(start, goal, graph):
    """
    Finds the shortest path between `start` and `goal` in `graph` using A* algorithm.

    Arguments:
        start (any): the starting node
        goal (any): the goal node
        graph (dict): a dictionary representing the graph, where the keys are the nodes and the values are dictionaries
                      containing the neighbors and their distances.

    Returns:
        list: a list of nodes representing the shortest path from `start` to `goal` in `graph`.
              If no path exists, an empty list is returned.
    """
    # Priority queue to store the nodes to be expanded
    queue = []

    # Dictionary to store the cost of the shortest path found so far to each node
    g_scores = {start: 0}

    # Dictionary to store the estimated cost to reach the goal from each node
    f_scores = {start: heuristic(start, goal)}

    # Dictionary to store the previous node in the shortest path from start to each node
    prev_nodes = {}

    # Add the start node to the queue
    heapq.heappush(queue, (f_scores[start], start))

    while queue:
        # Get the node with the lowest f_score
        _, curr_node = heapq.heappop(queue)

        # If we have reached the goal, return the path
        if curr_node == goal:
            path = []
            while curr_node in prev_nodes:
                path.append(curr_node)
                curr_node = prev_nodes[curr_node]
            path.append(start)
            path.reverse()
            return path

        # Expand the current node and update the queue and f_scores
        for neighbor, cost in graph[curr_node].items():
            # Compute the tentative g_score for the neighbor
            tentative_g_score = g_scores[curr_node] + cost

            # If the neighbor has not been visited yet, add it to the queue and f_scores
            if neighbor not in g_scores:
                g_scores[neighbor] = tentative_g_score
                f_scores[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                prev_nodes[neighbor] = curr_node
                heapq.heappush(queue, (f_scores[neighbor], neighbor))
            # If the neighbor has been visited and the new path is better, update its g_score, f_score, and prev_node
            elif tentative_g_score < g_scores[neighbor]:
                g_scores[neighbor] = tentative_g_score
                f_scores[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                prev_nodes[neighbor] = curr_node
                # If the neighbor is already in the queue, update its priority
                for i, (priority, node) in enumerate(queue):
                    if node == neighbor:
                        queue[i] = (f_scores[neighbor], neighbor)
                        heapq.heapify(queue)
                        break

    # If we have exhausted all nodes and haven't reached the goal, return an empty list
    return []

def heuristic(node, goal):
    """
    Computes the Euclidean distance between `node` and `goal`.

    Arguments:
        node (tuple): the coordinates of the node
        goal (tuple): the coordinates of the goal

    Returns:
        float: the Euclidean distance between `node` and `goal`.
    """
    return math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)

# Define the graph
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'D':                 3, 'E': 2},
    'C': {'A': 4, 'F': 5},
    'D': {'B': 3, 'G': 7},
    'E': {'B': 2, 'H': 2},
    'F': {'C': 5, 'I': 3},
    'G': {'D': 7, 'H': 2},
    'H': {'E': 2, 'G': 2, 'I': 2},
    'I': {'F': 3, 'H': 2}
}

# Define the start and goal nodes
start = 'A'
goal = 'I'

# Find the shortest path using A* algorithm
shortest_path = astar(start, goal, graph)

# Print the result
print(shortest_path)