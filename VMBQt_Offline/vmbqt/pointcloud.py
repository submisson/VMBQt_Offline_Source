"""Bounded PCD import. Original files remain unchanged; display samples are explicit.

Supports PCD ascii, binary and PCL's field-major binary_compressed/LZF layout.
"""
from dataclasses import dataclass
from pathlib import Path
import math
import struct
import numpy as np


@dataclass
class Cloud:
    source: str
    xyz: np.ndarray
    intensity: np.ndarray
    rgb: np.ndarray | None
    total: int
    visible: bool = True


def lzf_decompress(data, expected, limit=512 * 1024 * 1024):
    if expected < 0 or expected > limit:
        raise ValueError("压缩点云解压大小超限（默认 512MiB），请先分块")
    output = bytearray(); i = 0
    while i < len(data):
        control = data[i]; i += 1
        if control < 32:
            count = control + 1
            if i + count > len(data) or len(output) + count > expected:
                raise ValueError("损坏的 LZF 字面量")
            output.extend(data[i:i+count]); i += count
        else:
            count = control >> 5
            offset = (control & 31) << 8
            if count == 7:
                if i >= len(data):
                    raise ValueError("损坏的 LZF 长度")
                count += data[i]; i += 1
            if i >= len(data):
                raise ValueError("损坏的 LZF 偏移")
            offset += data[i]; i += 1
            count += 2
            start = len(output) - offset - 1
            if start < 0 or len(output) + count > expected:
                raise ValueError("LZF 引用越界")
            for j in range(count):
                output.append(output[start+j])
    if len(output) != expected:
        raise ValueError(f"LZF 解压长度错误：{len(output)} != {expected}")
    return bytes(output)


def read_pcd(path, max_points=300000):
    path = Path(path)
    max_points = max(1, int(max_points))
    with path.open("rb") as stream:
        header = {}
        for _ in range(1000):
            line = stream.readline(16384)
            if not line:
                raise ValueError("PCD 缺少 DATA 头")
            line = line.decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            key, *values = line.split()
            header[key.upper()] = values
            if key.upper() == "DATA":
                break
        else:
            raise ValueError("PCD 头过长")
        data_offset = stream.tell()
        fields = header.get("FIELDS", header.get("FIELD", []))
        sizes = list(map(int, header.get("SIZE", [])))
        types = header.get("TYPE", [])
        counts = list(map(int, header.get("COUNT", ["1"]*len(fields))))
        if not fields or not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError("PCD FIELDS/SIZE/TYPE/COUNT 不匹配")
        if len(set(fields)) != len(fields):
            raise ValueError("PCD 含重复字段名")
        if not {"x", "y", "z"}.issubset(fields):
            raise ValueError("PCD 必须包含 x、y、z 字段")
        n = int(header.get("POINTS", [str(int(header.get("WIDTH", [0])[0]) *
                                             int(header.get("HEIGHT", [1])[0]))])[0])
        if n < 1 or n > 2_000_000_000:
            raise ValueError("PCD POINTS 数量无效")
        formats = []
        for name, size, typ, count in zip(fields, sizes, types, counts):
            if size not in (1, 2, 4, 8) or typ not in ("I", "U", "F") or count < 1 or count > 1024:
                raise ValueError("PCD 字段类型不受支持")
            code = {"I": "i", "U": "u", "F": "f"}[typ] + str(size)
            try:
                dtype = np.dtype("<" + code)
            except TypeError as exc:
                raise ValueError("PCD 数值类型无效") from exc
            formats.append((name, dtype) if count == 1 else (name, dtype, (count,)))
        dtype = np.dtype(formats)
        if any(counts[fields.index(name)] != 1 for name in ("x", "y", "z")):
            raise ValueError("PCD x/y/z 的 COUNT 必须是 1")
        step = max(1, math.ceil(n / max_points))
        mode = header["DATA"][0].lower()
        if mode == "binary":
            if path.stat().st_size - data_offset < n * dtype.itemsize:
                raise ValueError("PCD binary 数据被截断")
            data = np.memmap(path, dtype=dtype, mode="r", offset=data_offset, shape=(n,))[::step]
            values = {f: np.array(data[f], copy=True) for f in fields}
        elif mode == "binary_compressed":
            raw_sizes = stream.read(8)
            if len(raw_sizes) != 8:
                raise ValueError("PCD 压缩大小头被截断")
            compressed, uncompressed = struct.unpack("<II", raw_sizes)
            if uncompressed != n * dtype.itemsize or compressed > 512*1024*1024:
                raise ValueError("PCD 压缩长度与字段定义不一致或超限")
            if compressed > path.stat().st_size - stream.tell():
                raise ValueError("PCD 压缩数据被截断")
            decoded = lzf_decompress(stream.read(compressed), uncompressed)
            values = {}; offset = 0
            for name, size, count in zip(fields, sizes, counts):
                field_dtype = dtype.fields[name][0]
                values[name] = np.frombuffer(decoded, dtype=field_dtype, count=n,
                                              offset=offset)[::step].copy()
                offset += n * size * count
        elif mode == "ascii":
            rows = []; columns = sum(counts); row = 0
            for line in stream:
                if not line.strip() or line.lstrip().startswith(b"#"):
                    continue
                if row >= n:
                    raise ValueError("PCD ASCII 行数超过 POINTS")
                if row % step == 0:
                    array = np.fromstring(line.decode("ascii"), sep=" ")
                    if len(array) != columns:
                        raise ValueError(f"PCD 第 {row+1} 行列数错误")
                    rows.append(array)
                row += 1
            if row != n:
                raise ValueError("PCD ASCII 数据行数少于 POINTS")
            array = np.array(rows); values = {}; col = 0
            for name, count in zip(fields, counts):
                values[name] = array[:, col] if count == 1 else array[:, col:col+count]
                col += count
        else:
            raise ValueError(f"不支持的 PCD DATA：{mode}")
    xyz = np.column_stack([values[f] for f in ("x", "y", "z")]).astype(np.float64)
    mask = np.isfinite(xyz).all(axis=1)
    xyz = xyz[mask]
    intensity = np.asarray(values.get("intensity", np.zeros(len(mask))), dtype=float)
    if intensity.ndim != 1:
        intensity = intensity[:, 0]
    intensity = np.nan_to_num(intensity[mask]).astype(np.float32)
    rgb = None
    color = "rgb" if "rgb" in values else "rgba" if "rgba" in values else None
    if color:
        packed = np.asarray(values[color])
        if packed.ndim != 1:
            raise ValueError("PCD rgb/rgba 的 COUNT 必须为 1")
        idx = fields.index(color)
        if types[idx] == "F":
            if sizes[idx] != 4:
                raise ValueError("PCD 浮点 rgb 必须为 float32")
            packed = packed.astype("<f4").view("<u4")
        else:
            packed = packed.astype("<u4")
        rgb = np.column_stack([(packed >> 16) & 255, (packed >> 8) & 255,
                               packed & 255]).astype(np.uint8)[mask]
    if not len(xyz):
        raise ValueError("PCD 没有有限坐标点")
    return Cloud(str(path.resolve()), xyz, intensity, rgb, n)


def height_at(clouds, xy, radius=1.):
    best_distance = radius * radius
    result = None
    for cloud in clouds:
        if not cloud.visible:
            continue
        distance = ((cloud.xyz[:, :2] - np.asarray(xy)[:2])**2).sum(axis=1)
        index = int(np.argmin(distance))
        if distance[index] <= best_distance:
            best_distance = distance[index]
            result = float(cloud.xyz[index, 2])
    return result
