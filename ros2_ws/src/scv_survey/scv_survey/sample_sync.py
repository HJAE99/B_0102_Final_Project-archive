"""수신 번호와 버퍼 전체 스냅샷 수를 비교한다 (ROS 무관).

seq는 로봇 수명 동안 증가하므로 회차 첫 번호가 0이라고 가정하지 않는다.
최대 번호만으로 완료 판단하지 않고 전체 sync 응답의 count를 함께 사용한다.
"""


class SampleSync:
    def __init__(self):
        self.seen = set()
        self.last_rx = None
        self.snapshot_last = None
        self.snapshot_count = None
        self.last_request = None
        self.future = None

    def observe(self, seq, now):
        reconnected = self.last_rx is not None and now - self.last_rx >= 3.0
        gap = bool(self.seen) and seq > max(self.seen) + 1
        self.seen.add(seq)
        self.last_rx = now
        if reconnected or gap:
            self.snapshot_count = None

    def snapshot(self, last_seq, count):
        self.snapshot_last, self.snapshot_count = last_seq, count

    def complete(self):
        return self.snapshot_count is not None and len(
            {seq for seq in self.seen if seq <= self.snapshot_last}) >= self.snapshot_count

    def from_seq(self, full=False):
        if full or self.snapshot_count is None or not self.complete():
            return 0       # 첫 샘플·관제 재시작·스냅샷 부족은 전체 범위로 복구
        return self.snapshot_last + 1
