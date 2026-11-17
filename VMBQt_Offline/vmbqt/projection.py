"""Explicit, reversible map projections. Z is always an absolute WGS84 height.

We deliberately do not silently guess a projection from the first OSM node.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, asdict
from pathlib import Path

from pyproj import CRS, Transformer
import yaml


@dataclass
class ProjectionConfig:
    projector_type: str = "Local"
    latitude: float = 35.6762
    longitude: float = 139.6503
    mgrs_grid: str = "54SUE"
    scale_factor: float = 0.9996
    vertical_datum: str = "WGS84"

    def to_dict(self):
        return asdict(self)

    def autoware_dict(self):
        result = {"projector_type": self.projector_type}
        if self.projector_type == "Local":
            return result
        result["vertical_datum"] = self.vertical_datum
        if self.projector_type == "MGRS":
            result["mgrs_grid"] = self.mgrs_grid
        else:
            result["map_origin"] = {
                "latitude": self.latitude, "longitude": self.longitude,
            }
            if self.projector_type == "TransverseMercator":
                result["scale_factor"] = self.scale_factor
        return result

    @classmethod
    def from_yaml(cls, path):
        value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("投影文件必须是 YAML 字典")
        origin = value.get("map_origin", {}) or {}
        return cls(value.get("projector_type", "Local"),
                   float(origin.get("latitude", 35.6762)),
                   float(origin.get("longitude", 139.6503)),
                   str(value.get("mgrs_grid", "54SUE")),
                   float(value.get("scale_factor", .9996)),
                   value.get("vertical_datum", "WGS84"))

    def write_yaml(self, path):
        # Validate before writing so unsupported datum/type cannot masquerade as supported.
        Projector(self)
        from .storage import atomic_text
        atomic_text(path, yaml.safe_dump(self.autoware_dict(), sort_keys=False))


class Projector:
    def __init__(self, config: ProjectionConfig):
        self.config = config
        self.offset = (0., 0.)
        self.transformer = None
        if config.projector_type == "Local":
            return
        if config.vertical_datum != "WGS84":
            raise ValueError("本版本只转换 WGS84 椭球高；不进行 EGM2008 大地水准面换算")
        if not (-80 <= config.latitude <= 84 and -180 <= config.longitude <= 180):
            raise ValueError("投影原点超出有效范围")
        if config.projector_type == "MGRS":
            zone, south, easting, northing = self._mgrs_origin(config.mgrs_grid)
            crs = CRS.from_dict({"proj": "utm", "zone": zone, "south": south})
            self.offset = easting, northing
        elif config.projector_type == "LocalCartesianUTM":
            zone = self.utm_zone(config.latitude, config.longitude)
            crs = CRS.from_dict({"proj": "utm", "zone": zone,
                                 "south": config.latitude < 0})
        elif config.projector_type == "TransverseMercator":
            if not (0.5 <= config.scale_factor <= 1.5):
                raise ValueError("scale_factor 必须在 0.5 至 1.5 之间")
            crs = CRS.from_proj4(
                f"+proj=tmerc +lat_0={config.latitude} +lon_0={config.longitude} "
                f"+k={config.scale_factor} +x_0=0 +y_0=0 +ellps=WGS84 +units=m")
        else:
            raise ValueError(f"未实现的投影类型：{config.projector_type}")
        self.transformer = Transformer.from_crs(4326, crs, always_xy=True)
        self.inverse = Transformer.from_crs(crs, 4326, always_xy=True)
        if config.projector_type == "LocalCartesianUTM":
            self.offset = self.transformer.transform(config.longitude, config.latitude)

    @staticmethod
    def utm_zone(latitude, longitude):
        zone = min(60, int((longitude + 180) / 6) + 1)
        # GeographicLib standard-zone exceptions used by Lanelet2 UTM.
        if 56 <= latitude < 64 and 3 <= longitude < 12:
            return 32
        if 72 <= latitude < 84 and 0 <= longitude < 42:
            return 31 if longitude < 9 else 33 if longitude < 21 else 35 if longitude < 33 else 37
        return zone

    @staticmethod
    def _mgrs_origin(grid):
        match = re.fullmatch(r"(\d{1,2})([C-HJ-NP-X])([A-HJ-NP-Z])([A-HJ-NP-V])",
                             grid.strip().upper())
        if not match:
            raise ValueError("MGRS 格式示例：54SUE（带号、纬度带、100km 方格）")
        zone = int(match[1]); band, col, row = match.groups()[1:]
        if not 1 <= zone <= 60:
            raise ValueError("MGRS 带号必须为 1–60")
        cols = ("ABCDEFGH", "JKLMNPQR", "STUVWXYZ")[(zone - 1) % 3]
        rows = "ABCDEFGHJKLMNPQRSTUV" if zone % 2 else "FGHJKLMNPQRSTUVABCDE"
        if col not in cols or row not in rows:
            raise ValueError("100km 方格字母与 MGRS 带号不匹配")
        min_northing = dict(zip("CDEFGHJKLMNPQRSTUVWX", [
            1100000, 2000000, 2800000, 3700000, 4600000, 5500000, 6400000,
            7300000, 8200000, 9100000, 0, 800000, 1700000, 2600000,
            3500000, 4400000, 5300000, 6200000, 7000000, 7900000]))[band]
        northing = rows.index(row) * 100000
        while northing < min_northing:
            northing += 2000000
        return zone, band < "N", (cols.index(col) + 1) * 100000., float(northing)

    def forward(self, latitude, longitude):
        if self.transformer is None:
            raise ValueError("Local 投影不能把经纬度转换为 XY；请先选择真实投影或提供 local_x/local_y")
        x, y = self.transformer.transform(longitude, latitude, errcheck=True)
        x -= self.offset[0]; y -= self.offset[1]
        self._check_mgrs(x, y)
        return x, y

    def reverse(self, x, y):
        if self.transformer is None:
            return 0., 0.
        self._check_mgrs(x, y)
        lon, lat = self.inverse.transform(x + self.offset[0], y + self.offset[1],
                                          errcheck=True)
        return lat, lon

    def _check_mgrs(self, x, y):
        if self.config.projector_type == "MGRS" and not (
                -0.001 <= x <= 100000.001 and -0.001 <= y <= 100000.001):
            raise ValueError("点超出指定的 MGRS 100km 方格；请检查坐标系或使用其他投影")
