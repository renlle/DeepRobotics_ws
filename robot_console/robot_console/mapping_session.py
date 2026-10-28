"""累计建图的文件事务：独立目录试建，确认后切换，旧地图保留为备份。"""
import math
import os
from pathlib import Path
import shutil
import uuid


def validate_map(directory):
    directory = Path(directory)
    for name in ('map_3d.pcd', 'sc_database.txt', 'optimized_poses.txt'):
        path = directory / name
        if not path.is_file() or path.stat().st_size == 0:
            raise ValueError(f'缺少有效历史地图文件: {path}')
    with (directory / 'sc_database.txt').open() as stream:
        # 检查完整自定义文本格式，不能只凭首行帧数接收截断数据库。
        lines = (line.split() for line in stream if line.strip())
        try:
            count = int(next(lines)[0])
            for index in range(count):
                pose = next(lines)
                if len(pose) != 7 or int(pose[0]) != index or not all(math.isfinite(float(v)) for v in pose[1:]):
                    raise ValueError('ScanContext 帧编号或位姿无效。')
                rows, cols = map(int, next(lines))
                if not (0 < rows <= 1000 and 0 < cols <= 1000):
                    raise ValueError('ScanContext 矩阵尺寸无效。')
                for _ in range(rows):
                    row = next(lines)
                    if len(row) != cols or not all(math.isfinite(float(v)) for v in row):
                        raise ValueError('ScanContext 描述子无效。')
            if next(lines, None) is not None:
                raise ValueError('ScanContext 数据超出声明帧数。')
        except (StopIteration, IndexError) as exc:
            raise ValueError('ScanContext 数据库被截断。') from exc
    with (directory / 'optimized_poses.txt').open() as stream:
        poses = [line.split() for line in stream if line.strip()]
    if count <= 0 or len(poses) != count:
        raise ValueError('历史轨迹帧数与 ScanContext 数据库不一致，请先检查地图完整性。')
    if any(len(pose) != 12 or not all(math.isfinite(float(v)) for v in pose) for pose in poses):
        raise ValueError('历史轨迹存在无效位姿。')
    scans = {int(p.stem) for p in (directory / 'Scans').glob('*.pcd') if p.stem.isdigit()}
    if scans != set(range(count)):
        raise ValueError('历史 Scans 编号不连续或与轨迹不一致，不能安全追加关键帧。')
    return count


class MappingSession:
    def __init__(self, directory):
        self.original = Path(directory).resolve()
        self.initial_count = validate_map(self.original)
        suffix = uuid.uuid4().hex[:12]
        self.staging = self.original.with_name(f'.{self.original.name}.cumulative-{suffix}')
        self.backup = self.original.with_name(f'.{self.original.name}.before-cumulative-{suffix}')

    def prepare(self):
        # 完整复制，避免硬链接被后端覆盖写时连带损坏原图。
        size = sum(p.stat().st_size for p in self.original.rglob('*') if p.is_file())
        if shutil.disk_usage(self.original.parent).free < size * 2 + 512 * 1024 * 1024:
            raise OSError('磁盘空间不足，无法为累计建图准备副本及新增地图。')
        shutil.copytree(self.original, self.staging, ignore=shutil.ignore_patterns('.map_backup'))
        for name in ('.cumulative_complete', '.cumulative_anchored'):
            (self.staging / name).unlink(missing_ok=True)
        # 只重置副本产物的时间，避开系统时间调整和文件系统时间精度差异。
        for name in ('map_3d.pcd', 'map_2d.pgm', 'map_2d.yaml', 'sc_database.txt', 'optimized_poses.txt'):
            path = self.staging / name
            if path.is_file():
                os.utime(path, ns=(0, 0))

    def validate_result(self):
        if not (self.staging / '.cumulative_complete').is_file():
            raise ValueError('后端未完成对齐和保存，原地图保留，本次结果不自动合并。')
        count = validate_map(self.staging)
        if count <= self.initial_count:
            raise ValueError('本次没有新增有效关键帧，原地图保留。')
        for name in ('map_3d.pcd', 'map_2d.pgm', 'map_2d.yaml', 'sc_database.txt', 'optimized_poses.txt'):
            path = self.staging / name
            if not path.is_file() or path.stat().st_size == 0 or path.stat().st_mtime_ns == 0:
                raise ValueError(f'本次地图未完整写入: {name}')

    def commit(self):
        self.validate_result()
        self.original.rename(self.backup)
        try:
            self.staging.rename(self.original)
        except OSError:
            self.backup.rename(self.original)
            raise
        return self.backup

    def discard(self):
        if self.staging.exists():
            shutil.rmtree(self.staging)
