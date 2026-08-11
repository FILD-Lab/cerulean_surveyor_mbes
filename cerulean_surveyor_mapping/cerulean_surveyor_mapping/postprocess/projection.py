import numpy as np
from pyproj import CRS, Transformer

WGS84 = CRS.from_epsg(4326)


def utm_crs_for(lons, lats):
    """Pick the UTM CRS (WGS84 / UTM zone) appropriate for the mean location of the data."""
    mean_lon = float(np.mean(lons))
    mean_lat = float(np.mean(lats))
    zone = int((mean_lon + 180) / 6) + 1
    epsg = (32600 if mean_lat >= 0 else 32700) + zone
    return CRS.from_epsg(epsg)


def project_to_utm(lats, lons):
    """Project lat/lon (WGS84, degrees) to UTM easting/northing (meters).

    Returns (easting, northing, crs).
    """
    crs = utm_crs_for(lons, lats)
    transformer = Transformer.from_crs(WGS84, crs, always_xy=True)
    easting, northing = transformer.transform(lons, lats)
    return np.asarray(easting), np.asarray(northing), crs
