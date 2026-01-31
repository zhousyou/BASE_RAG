import heapq

def heuristic(a, b):
    """Manhattan distance heuristic"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(grid, start, goal):
    """
    A* search algorithm implementation
    
    Args:
        grid: 2D list representing the grid (0 = free, 1 = obstacle)
        start: tuple (row, col) representing starting position
        goal: tuple (row, col) representing goal position
    
    Returns:
        list of tuples representing the path from start to goal, or None if no path exists
    """
    rows, cols = len(grid), len(grid[0])
    
    # Priority queue for open set
    open_set = [(0, start)]
    
    # Dictionary to store the cost of reaching each node
    g_score = {start: 0}
    
    # Dictionary to store the came_from information for path reconstruction
    came_from = {}
    
    # Set to keep track of visited nodes
    closed_set = set()
    
    while open_set:
        # Get the node with the lowest f_score
        current_f, current = heapq.heappop(open_set)
        
        # If we reached the goal
        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        # Mark as visited
        closed_set.add(current)
        
        # Check all neighbors
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Check bounds
            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue
            
            # Skip obstacles
            if grid[neighbor[0]][neighbor[1]] == 1:
                continue
            
            # Skip visited nodes
            if neighbor in closed_set:
                continue
            
            # Calculate tentative g_score
            tentative_g_score = g_score[current] + 1
            
            # If neighbor not in open_set or better path found
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                # Update scores
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
    
    # No path found
    return None

# Example usage
if __name__ == "__main__":
    # Example grid (0 = free, 1 = obstacle)
    grid = [
        [0, 0, 0, 0, 1],
        [1, 1, 0, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]
    
    start = (0, 0)
    goal = (4, 4)
    
    path = a_star_search(grid, start, goal)
    
    if path:
        print("Path found:", path)
    else:
        print("No path found")