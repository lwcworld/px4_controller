#!/usr/bin/env python
import math

class coordinate_transform():
    def __init__(self):
        self.a = 6378137
        self.b = 6356752.3142
        self.f = (self.a - self.b) / self.a
        self.e_sq = self.f * (2 - self.f)

    def geodetic_to_ecef(self, lat, lon, h):
        # (lat, lon) in WSG-84 degrees
        # h in meters
        lamb = math.radians(lat)
        phi = math.radians(lon)
        s = math.sin(lamb)
        N = self.a / math.sqrt(1 - self.e_sq * s * s)

        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        x = (h + N) * cos_lambda * cos_phi
        y = (h + N) * cos_lambda * sin_phi
        z = (h + (1 - self.e_sq) * N) * sin_lambda

        return x, y, z


    def ecef_to_enu(self, x, y, z, lat0, lon0, h0):
        lamb = math.radians(lat0)
        phi = math.radians(lon0)
        s = math.sin(lamb)
        N = self.a / math.sqrt(1 - self.e_sq * s * s)

        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)

        x0 = (h0 + N) * cos_lambda * cos_phi
        y0 = (h0 + N) * cos_lambda * sin_phi
        z0 = (h0 + (1 - self.e_sq) * N) * sin_lambda

        xd = x - x0
        yd = y - y0
        zd = z - z0

        xEast = -sin_phi * xd + cos_phi * yd
        yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd
        zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd

        return xEast, yNorth, zUp


    def geodetic_to_enu(self, lat, lon, h, lat_ref, lon_ref, h_ref):
        x, y, z = self.geodetic_to_ecef(lat, lon, h)

        return self.ecef_to_enu(x, y, z, lat_ref, lon_ref, h_ref)

transform= coordinate_transform()