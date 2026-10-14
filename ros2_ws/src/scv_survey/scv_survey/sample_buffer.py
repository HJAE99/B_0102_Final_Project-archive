"""IF-08 샘플의 CSV 선저장·번호 복원·재발행 큐 (ROS 무관)."""

import csv
import fcntl
import math
import os
from collections import deque
from dataclasses import asdict, dataclass, fields
from pathlib import Path


@dataclass(frozen=True)
class BufferedSample:
    seq: int
    stamp_sec: int
    stamp_nanosec: int
    robot_id: str
    run_id: int
    x: float
    y: float
    yaw: float
    pose_ok: bool
    detected: bool
    rssi_dbm: int

    @property
    def raw_key(self):
        return self.robot_id, self.stamp_sec, self.stamp_nanosec


CSV_FIELDS = [f.name for f in fields(BufferedSample)]


def sample_from_row(row):
    if set(row) != set(CSV_FIELDS) or any(v is None for v in row.values()):
        raise ValueError('불완전한 샘플 행')
    values = dict(row)
    for key in ('seq', 'stamp_sec', 'stamp_nanosec', 'run_id', 'rssi_dbm'):
        values[key] = int(row[key])
    for key in ('x', 'y', 'yaw'):
        values[key] = float(row[key])
        if not math.isfinite(values[key]):
            raise ValueError(f'{key}: 유효하지 않은 좌표')
    for key in ('pose_ok', 'detected'):
        if row[key] not in ('0', '1'):
            raise ValueError(f'{key}: 0 또는 1이어야 함')
        values[key] = row[key] == '1'
    if not (0 <= values['seq'] < 2**32 and 0 <= values['run_id'] < 2**32
            and 0 <= values['stamp_nanosec'] < 10**9):
        raise ValueError('seq·run_id·시각 범위 오류')
    return BufferedSample(**values)


class SampleBuffer:
    def __init__(self, path, robot_id):
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.samples = []
        self.raw_keys = set()
        self.robot_id = robot_id
        self.next_seq = 0
        self.failed = False
        self.file = self.path.open('a+', encoding='utf-8', newline='')
        try:
            # 같은 로봇의 두 버퍼가 같은 파일에서 번호를 만들지 못하게 한다.
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.file.seek(0)
            reader = csv.DictReader(self.file)
            if reader.fieldnames is not None:
                if reader.fieldnames != CSV_FIELDS:
                    raise ValueError('survey_buffer CSV 헤더가 다릅니다')
                for row in reader:
                    sample = sample_from_row(row)
                    if sample.robot_id != robot_id or sample.seq < self.next_seq:
                        raise ValueError('다른 로봇 또는 중복·역순 seq')
                    if sample.raw_key in self.raw_keys:
                        raise ValueError('중복 측정 시각')
                    self.samples.append(sample)
                    self.raw_keys.add(sample.raw_key)
                    self.next_seq = sample.seq + 1
                # 마지막 행의 부분 쓰기도 조용히 건너뛰지 않는다.
                self.file.seek(0)
                if not self.file.read().endswith('\n'):
                    raise ValueError('CSV 마지막 행이 완성되지 않았습니다')
            self.file.seek(0, os.SEEK_END)
            self.writer = csv.DictWriter(self.file, fieldnames=CSV_FIELDS, lineterminator='\n')
            if reader.fieldnames is None:
                self.writer.writeheader()
                self._flush()
        except Exception:
            self.file.close()
            raise

    def _flush(self):
        self.file.flush()
        os.fsync(self.file.fileno())

    def append(self, sample):
        if self.failed:
            raise OSError('이전 CSV 쓰기가 실패했습니다 — 파일을 확인하고 재시작하세요')
        if sample.seq != self.next_seq or sample.robot_id != self.robot_id:
            raise ValueError('로봇 또는 다음 seq가 일치하지 않습니다')
        if sample.raw_key in self.raw_keys:
            raise ValueError('이미 저장한 원본 샘플입니다')
        row = asdict(sample)
        row.update(pose_ok=int(sample.pose_ok), detected=int(sample.detected))
        sample_from_row({k: str(v) for k, v in row.items()})
        try:
            self.writer.writerow(row)
            self._flush()
        except OSError:
            self.failed = True
            raise
        self.samples.append(sample)
        self.raw_keys.add(sample.raw_key)
        self.next_seq += 1

    def select(self, run_id, from_seq):
        return [s for s in self.samples if s.run_id == run_id and s.seq >= from_seq]

    def last_seq(self, run_id):
        return next((s.seq for s in reversed(self.samples) if s.run_id == run_id), None)

    def close(self):
        self.file.close()


class ReplayQueue:
    def __init__(self, rate_hz=20.0):
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError('sync_rate_hz는 양수여야 합니다')
        self.interval = 1.0 / min(rate_hz, 20.0)
        self.queue = deque()
        self.keys = set()
        self.last_sent = None

    def schedule(self, samples):
        count = 0
        for sample in samples:
            key = sample.run_id, sample.seq
            if key not in self.keys:
                self.queue.append(sample)
                self.keys.add(key)
                count += 1
        return count

    def pop_ready(self, now):
        if not self.queue or (self.last_sent is not None and now - self.last_sent < self.interval):
            return None
        sample = self.queue.popleft()
        self.keys.remove((sample.run_id, sample.seq))
        self.last_sent = now
        return sample
