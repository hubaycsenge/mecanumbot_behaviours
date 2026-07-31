import cv2
import numpy as np
import math
import heapq
import itertools
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def save_waypoints_yaml(output_path, waypoints_world):
    with open(output_path, "w", encoding="utf-8") as yaml_file:
        yaml_file.write("waypoints:\n")
        for x, y in waypoints_world:
            yaml_file.write(f"  - [{x:.3f}, {y:.3f}]\n")


def generate_patrol_route(
    pgm_path,
    yaml_resolution,
    yaml_origin,
    num_rays=36,
    num_waypoints=None,
    min_dist_px=None,
    max_dist_px=None,
):
    """
    Calculates visibility scores, extracts distinct high-visibility waypoints,
    converts them to Nav2 coordinates, and draws the planned route.
    """
    print(f"Loading map from {pgm_path}...")
    map_image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if map_image is None:
        print("Error: Could not load the PGM image. Check the file path.")
        return None, None, None

    height, width = map_image.shape
    _, grid = cv2.threshold(map_image, 250, 255, cv2.THRESH_BINARY)

    visibility_scores = np.zeros((height, width), dtype=np.float32)
    angles = np.linspace(0, 2 * math.pi, num_rays, endpoint=False)
    ray_dirs = [(math.cos(a), math.sin(a)) for a in angles]

    # --- AUTOMATION LOGIC ---
    # Auto-calculate parameters based on real-world meter equivalents if not provided
    if max_dist_px is None:
        # Assume a max useful camera/LiDAR range of 15 meters
        max_dist_px = int(15.0 / yaml_resolution)
        print(f"Auto-calculated max_dist_px: {max_dist_px} px (15.0m limit)")

    if min_dist_px is None:
        # Require waypoints to be at least 4.5 meters apart to avoid clustering
        min_dist_px = int(4.5 / yaml_resolution)
        print(f"Auto-calculated min_dist_px: {min_dist_px} px (4.5m separation)")

    if num_waypoints is None:
        # Calculate total free area in square meters
        free_pixels = np.sum(grid == 255)
        free_area_m2 = free_pixels * (yaml_resolution**2)
        # Assign 1 waypoint per 25 square meters of explorable space (minimum of 3 points)
        num_waypoints = max(3, int(free_area_m2 / 25.0))
        print(
            f"Auto-calculated num_waypoints: {num_waypoints} (Total free area: {free_area_m2:.1f} m^2)"
        )
    # ------------------------

    print("Calculating visibility scores (this may take a moment)...")
    for y in range(height):
        for x in range(width):
            if grid[y, x] == 255:
                score = 0
                for dx, dy in ray_dirs:
                    curr_x, curr_y = float(x), float(y)
                    while True:
                        curr_x += dx
                        curr_y += dy
                        ix, iy = int(curr_x), int(curr_y)
                        if (
                            ix < 0
                            or iy < 0
                            or ix >= width
                            or iy >= height
                            or grid[iy, ix] == 0
                        ):
                            break
                        if np.sqrt((curr_x - x) ** 2 + (curr_y - y) ** 2) > max_dist_px:
                            break  # too far, stop counting
                        score += 1
                visibility_scores[y, x] = score

    # Extract High-Visibility Waypoints (with distance suppression)
    print(f"Extracting top {num_waypoints} distinct waypoints...")
    waypoints_pixel = []
    waypoints_world = []

    temp_scores = visibility_scores.copy()
    origin_x, origin_y = yaml_origin

    for _ in range(num_waypoints):
        # Find the absolute highest score left on the map
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(temp_scores)

        if max_val == 0:
            print("Warning: Ran out of free space before finding all waypoints.")
            break

        px, py = max_loc
        waypoints_pixel.append((px, py))

        # Convert to Nav2 World Coordinates
        world_x = origin_x + (px * yaml_resolution)
        world_y = origin_y + ((height - py) * yaml_resolution)
        waypoints_world.append((world_x, world_y))

        # --- NEW: LINE-OF-SIGHT SUPPRESSION ---
        # Cast dense rays to find the exact boundary of what this point can see
        dense_angles = np.linspace(0, 2 * math.pi, 360, endpoint=False)
        polygon_pts = []
        for a in dense_angles:
            dx, dy = math.cos(a), math.sin(a)
            curr_x, curr_y = float(px), float(py)
            while True:
                next_x, next_y = curr_x + dx, curr_y + dy
                ix, iy = int(next_x), int(next_y)

                # Stop if we hit a wall or edge
                if ix < 0 or iy < 0 or ix >= width or iy >= height or grid[iy, ix] == 0:
                    polygon_pts.append((int(curr_x), int(curr_y)))
                    break
                # Stop if we exceed max distance
                if np.sqrt((next_x - px) ** 2 + (next_y - py) ** 2) > max_dist_px:
                    polygon_pts.append((int(next_x), int(next_y)))
                    break

                curr_x, curr_y = next_x, next_y

        # Zero out the score of the ENTIRE visible polygon
        # This forces the next waypoint into unseen "shadows" (like behind walls)
        pts = np.array(polygon_pts, np.int32)
        cv2.fillPoly(temp_scores, [pts], 0)

        # Suppress this neighborhood so the next waypoint is in a different location (fallback)
        cv2.circle(temp_scores, (px, py), min_dist_px, 0, -1)

    # Visualizing the Map and Route
    print("Generating visual map check...")
    display_map = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)

    for i, (px, py) in enumerate(waypoints_pixel):
        # Draw the waypoint as a solid red dot
        cv2.circle(display_map, (px, py), 5, (0, 0, 255), -1)

        # Number the waypoint
        cv2.putText(
            display_map,
            str(i + 1),
            (px + 8, py - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2,
        )

        # Draw a green line to connect the path
        if i > 0:
            prev_px, prev_py = waypoints_pixel[i - 1]
            cv2.line(display_map, (prev_px, prev_py), (px, py), (0, 255, 0), 2)

    # Connect the last waypoint back to the first to close the patrol loop
    if len(waypoints_pixel) > 1:
        cv2.line(display_map, waypoints_pixel[-1], waypoints_pixel[0], (0, 255, 0), 2)

    # Print YAML ready output
    print("\n--- Nav2 Waypoints Generated ---")
    print("waypoints:")
    for wp in waypoints_world:
        print(f"  - {{x: {wp[0]:.3f}, y: {wp[1]:.3f}}}")

    return waypoints_pixel, waypoints_world, display_map


def heuristic(a, b):
    # Euclidean distance heuristic for A*
    return math.hypot(b[0] - a[0], b[1] - a[1])


def calculate_drivable_distance(map_path, start_px, goal_px, resolution=0.05):
    """
    Calculates the shortest obstacle-free path distance between two pixels.
    """
    # 1. Load and threshold the map
    map_image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if map_image is None:
        print(f"Error: Map {map_path} not found")
        return np.inf

    _, grid = cv2.threshold(map_image, 250, 255, cv2.THRESH_BINARY)

    # Check if start or goal are inside a wall
    # Return np.inf instead of a string to prevent math crashes later!
    if grid[start_px[1], start_px[0]] == 0:
        print(f"Error: Start {start_px} is inside an obstacle on {map_path}.")
        return np.inf
    if grid[goal_px[1], goal_px[0]] == 0:
        print(f"Error: Goal {goal_px} is inside an obstacle on {map_path}.")
        return np.inf

    # 2. Setup A* Pathfinding variables
    # 8-way movement: (dx, dy)
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]

    open_set = []
    heapq.heappush(open_set, (0, start_px))

    came_from = {}
    g_score = {start_px: 0}

    # 3. Run A* Search
    while open_set:
        _, current = heapq.heappop(open_set)

        # If we reached the goal, reconstruct the path
        if current == goal_px:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_px)
            path.reverse()

            # Convert pixel distance to meters
            pixel_distance = g_score[goal_px]
            meters_distance = pixel_distance * resolution

            return pixel_distance

        # Check all adjacent pixels
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)

            # Ensure within map bounds
            if 0 <= neighbor[0] < grid.shape[1] and 0 <= neighbor[1] < grid.shape[0]:

                # Ensure pixel is free (White / 255)
                if grid[neighbor[1], neighbor[0]] == 255:

                    # Cost is 1 for straight, 1.414 for diagonal
                    cost = math.hypot(dx, dy)
                    tentative_g_score = g_score[current] + cost

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + heuristic(neighbor, goal_px)
                        heapq.heappush(open_set, (f_score, neighbor))

    return np.inf


def create_order(wpp, map_path):
    cost_matrix = np.zeros((len(wpp), len(wpp)))
    for i in range(len(wpp)):
        for j in range(len(wpp)):
            if i != j:
                if i < j:  # Calculate only once for each pair
                    # Pass the correct map_path parameter dynamically!
                    d = calculate_drivable_distance(map_path, wpp[i], wpp[j])
                    print(f"Distance from {i} to {j}: {d}")
                    cost_matrix[i][j] = d
                    cost_matrix[j][i] = d  # Symmetric matrix
            else:
                cost_matrix[i][j] = 0
    print("Cost Matrix:")
    print(cost_matrix)
    return cost_matrix


def held_karp(distance_matrix):
    """
    Solves the TSP using the Held-Karp Dynamic Programming algorithm.
    :param distance_matrix: A 2D list or numpy array of distances between waypoints.
    :return: A tuple of (optimal_path, minimum_distance)
    """
    n = len(distance_matrix)

    # Memoization table: maps a tuple (visited_set, last_node) to (min_cost, previous_node)
    memo = {}

    # Initialize the base cases: Path from the starting node (0) to all other nodes
    for i in range(1, n):
        memo[((1 << i) | 1, i)] = (distance_matrix[0][i], 0)

    # Iterate through all subset sizes of waypoints (from 2 to N)
    for subset_size in range(2, n):
        for subset in itertools.combinations(range(1, n), subset_size):
            bits = 1
            for bit in subset:
                bits |= 1 << bit

            for k in subset:
                prev_bits = bits & ~(1 << k)
                res = []
                for m in subset:
                    if m == 0 or m == k:
                        continue
                    cost = memo[(prev_bits, m)][0] + distance_matrix[m][k]
                    res.append((cost, m))

                memo[(bits, k)] = min(res)

    all_visited_bits = (1 << n) - 1
    res = []
    for k in range(1, n):
        cost = memo[(all_visited_bits, k)][0] + distance_matrix[k][0]
        res.append((cost, k))

    optimal_cost, last_node = min(res)

    path = []
    curr_bits = all_visited_bits
    curr_node = last_node

    while curr_node != 0:
        path.append(curr_node)
        next_node = memo[(curr_bits, curr_node)][1]
        curr_bits = curr_bits & ~(1 << curr_node)
        curr_node = next_node

    path.append(0)
    path.reverse()
    path.append(0)

    return path, optimal_cost


def generate_waypoints(
    base_name, min_dist_px=None, max_dist_px=None, num_waypoints=None
):
    """
    Generates waypoints and saves them to a YAML file.
    """
    description_base_path = get_package_share_directory("mecanumbot_description")
    out_path = os.path.join(
        description_base_path, "maps", base_name, base_name + "_waypoints.yaml"
    )
    in_features_path = os.path.join(
        description_base_path, "maps", base_name, base_name + ".yaml"
    )
    in_map_path = os.path.join(
        description_base_path, "maps", base_name, base_name + ".pgm"
    )

    with open(in_features_path, "r") as stream:
        features = yaml.safe_load(stream)

    wpp, wpw, wpmap = generate_patrol_route(
        pgm_path=in_map_path,
        yaml_resolution=features["resolution"],
        yaml_origin=features["origin"][:-1],
        num_waypoints=num_waypoints,
        min_dist_px=min_dist_px,
        max_dist_px=max_dist_px,
    )

    if wpp is not None:
        dist_mx = create_order(wpp, in_map_path)

        optimal_route, total_distance = held_karp(dist_mx)

        ordered_waypoints = [wpw[index] for index in optimal_route[:-1]]
        save_waypoints_yaml(out_path, ordered_waypoints)

        print(f"Optimal Patrol Route (Indices): {optimal_route}")
        print(f"Total Patrol Distance: {total_distance:.2f} units")
