"""Minimal stub for pyned2lla - only needed to allow NatNetClient to import.

The diagnostic tool never calls ned_to_gps(), so the actual implementation
is not required.
"""

class wgs84:
    pass


def ned2lla(lat, lon, alt, n, e, d, ellipsoid):
    """Stub - raises if called."""
    raise NotImplementedError("pyned2lla stub: install pyned2lla for GPS conversion")
