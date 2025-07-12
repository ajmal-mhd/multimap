from scripts.db_interface import get_all_pairs

def find_path(start, end):
    wormholes = get_all_pairs()
    # Build adjacency list
    graph = {}
    for a, b in wormholes:
        graph.setdefault(a, []).append(b)

    # DFS to find a path
    def dfs(current, path, visited):
        if current == end:
            return path
        visited.add(current)
        for neighbor in graph.get(current, []):
            if neighbor not in visited:
                result = dfs(neighbor, path + [neighbor], visited)
                if result:
                    return result
        visited.remove(current)
        return None

    return dfs(start, [start], set())


# === Example Usage ===
if __name__ == '__main__':
    
    start_room = 'room1'
    end_room = 'room3'

    path = find_path(start_room, end_room)
    print(path)  # Example output: ['room1', 'room2', 'room3']
