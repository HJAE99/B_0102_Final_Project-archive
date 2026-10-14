"""IF-08·09: TF 위치 결합 → CSV 선저장 → 발행, 회차별 범위 재전송."""

import time
from pathlib import Path

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException

from scv_msgs.msg import NetSample, RobotStatus
from scv_msgs.srv import SyncSamples
from scv_survey.manual_tagger import ManualTagger
from scv_survey.sample_buffer import BufferedSample, ReplayQueue, SampleBuffer


def sample_message(sample, resent=False):
    msg = NetSample()
    for name in ('seq', 'robot_id', 'run_id', 'x', 'y', 'yaw', 'pose_ok', 'detected', 'rssi_dbm'):
        setattr(msg, name, getattr(sample, name))
    msg.stamp.sec, msg.stamp.nanosec = sample.stamp_sec, sample.stamp_nanosec
    msg.resent = resent
    return msg


class SurveyBuffer(ManualTagger):
    def __init__(self):
        super().__init__(node_name='survey_buffer', pose_source='tf', run_id=0)
        try:
            if self.pose_source != 'tf':
                raise ValueError('survey_buffer는 tf 모드 전용입니다')
            self.run_id = 0     # 파라미터 고정값 대신 status가 회차를 정한다.
            self.robot_id = self.declare_parameter(
                'robot_id', self.get_namespace().strip('/') or 'robot4').value
            path = self.declare_parameter(
                'buffer_csv_path', str(Path('~/scv_survey_logs') / f'{self.robot_id}_buffer.csv')).value
            self.buffer = SampleBuffer(path, self.robot_id)
            self.seq = self.buffer.next_seq
            self.sample_runs = {}
            self.pending_keys = set()
            rate = float(self.declare_parameter('sync_rate_hz', 20.0).value)
            self.replay = ReplayQueue(rate)
            self.create_subscription(RobotStatus, 'status', self.on_status, 10)
            self.create_service(SyncSamples, 'survey/sync', self.on_sync)
            self.create_timer(0.05, self.send_replay,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info(
                f'버퍼 {self.buffer.path}, 복원 {len(self.buffer.samples)}개, 다음 seq={self.seq}, '
                f'sync ≤ {min(rate, 20.0):g} Hz; status 전에는 run_id=0')
        except Exception:
            self.destroy_node()
            raise

    def on_status(self, msg):
        if msg.robot_id == self.robot_id:
            self.run_id = msg.run_id

    def on_raw(self, raw):
        if raw.robot_id != self.robot_id:
            self.get_logger().warning('다른 로봇의 NetRaw를 제외합니다', once=True)
            return
        key = raw.robot_id, raw.stamp.sec, raw.stamp.nanosec
        # 재시작 때 RAW_QOS(transient_local)가 다시 준 과거 측정은 새 번호로 저장하지 않는다.
        if key in self.buffer.raw_keys or key in self.pending_keys:
            return
        self.pending_keys.add(key)
        self.sample_runs[id(raw)] = self.run_id
        super().on_raw(raw)

    def publish(self, raw, ok):
        sample = BufferedSample(
            self.seq, raw.stamp.sec, raw.stamp.nanosec, raw.robot_id,
            self.sample_runs.pop(id(raw)), *(float(v) for v in self.pose),
            bool(ok), raw.detected, raw.rssi_dbm)
        # 기록 실패 시 예외로 종료한다. 저장하지 않은 샘플을 발행하거나 번호를 초기화하지 않는다.
        self.buffer.append(sample)
        self.seq = self.buffer.next_seq
        self.pending_keys.remove(sample.raw_key)
        self.pub.publish(sample_message(sample))

    def on_sync(self, request, response):
        last_seq = self.buffer.last_seq(request.run_id)
        response.ok = last_seq is not None
        response.last_seq = last_seq if last_seq is not None else 0
        samples = self.buffer.select(request.run_id, request.from_seq)
        # 스냅샷의 대상 수를 알려 줘야 관제가 선두 누락도 확인할 수 있다.
        # 이미 큐에 있는 대상은 중복 예약하지 않지만 응답 수에는 포함한다.
        response.count = len(samples)
        scheduled = self.replay.schedule(samples)
        self.get_logger().info(
            f'sync run_id={request.run_id} from_seq={request.from_seq}: '
            f'대상 {response.count}개, 새 예약 {scheduled}개, last_seq={response.last_seq}')
        return response

    def send_replay(self):
        sample = self.replay.pop_ready(time.monotonic())
        if sample is not None:
            self.pub.publish(sample_message(sample, resent=True))

    def destroy_node(self):
        if hasattr(self, 'buffer'):
            self.buffer.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SurveyBuffer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
