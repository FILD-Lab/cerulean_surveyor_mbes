import pymap3d


def enu_to_lat_lon(east, north, up, datum_lat, datum_lon, datum_alt):
    """Convert map-frame ENU point(s) to (lat, lon), given the vehicle's lla_datum.

    The map frame follows REP-105 (x=East, y=North, z=Up) relative to the
    lla_datum origin established by lat_lon_to_map_frame.cpp, so this is a
    plain ENU-to-geodetic conversion. east/north/up may be scalars or
    numpy arrays; pymap3d broadcasts either way.
    """
    lat, lon, _alt = pymap3d.enu2geodetic(east, north, up, datum_lat, datum_lon, datum_alt)
    return lat, lon
