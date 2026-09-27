def partition_rectangle(width_m, height_m, vehicle_count):
    """
    Divide a rectangular search region into equal vertical strips.

    Origin:
        southwest corner = (0, 0)

    Returns:
        [
            (x_min, y_min, x_max, y_max),
            ...
        ]
    """

    zone_width = width_m / vehicle_count

    zones = []

    for i in range(vehicle_count):
        x_min = i * zone_width
        x_max = (i + 1) * zone_width

        zones.append(
            (
                x_min,
                0,
                x_max,
                height_m,
            )
        )

    return zones

def generate_lawnmower_path(zone, lane_spacing_m=10):
    """
    Generate a simple back-and-forth coverage route.

    Coordinates are local metres:
        x = east
        y = north
    """

    x_min, y_min, x_max, y_max = zone

    waypoints = []

    y = y_min
    left_to_right = True

    while y <= y_max:

        if left_to_right:
            waypoints.append((x_min, y))
            waypoints.append((x_max, y))
        else:
            waypoints.append((x_max, y))
            waypoints.append((x_min, y))

        left_to_right = not left_to_right
        y += lane_spacing_m

    # Ensure final northern edge gets covered.
    if waypoints and waypoints[-1][1] < y_max:

        if left_to_right:
            waypoints.append((x_min, y_max))
            waypoints.append((x_max, y_max))
        else:
            waypoints.append((x_max, y_max))
            waypoints.append((x_min, y_max))

    return waypoints