import math


EARTH_RADIUS_M = 6_378_137.0


def geo_to_local(
    latitude: float,
    longitude: float,
    origin_latitude: float,
    origin_longitude: float,
) -> tuple[float, float]:
    """
    Convert latitude/longitude into local east/north metres
    relative to the mission origin.

    SwarmSearch operates over small local areas, so the
    equirectangular approximation is sufficient here.

    Returns:
        (east_m, north_m)
    """

    latitude_rad = math.radians(
        latitude
    )

    longitude_rad = math.radians(
        longitude
    )

    origin_latitude_rad = math.radians(
        origin_latitude
    )

    origin_longitude_rad = math.radians(
        origin_longitude
    )

    delta_latitude = (
        latitude_rad
        - origin_latitude_rad
    )

    delta_longitude = (
        longitude_rad
        - origin_longitude_rad
    )

    mean_latitude = (
        latitude_rad
        + origin_latitude_rad
    ) / 2.0

    north_m = (
        delta_latitude
        * EARTH_RADIUS_M
    )

    east_m = (
        delta_longitude
        * EARTH_RADIUS_M
        * math.cos(mean_latitude)
    )

    return east_m, north_m