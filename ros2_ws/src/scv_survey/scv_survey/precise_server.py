"""precise_server (로봇 PC, 묶음 시험용 임시 · IF-04): {ns}/survey/precise 액션 서버.

goal 을 받으면 duration_s 동안 {ns}/survey/raw 를 모아 검출값의 중앙값·IQR·검출 수를 돌려준다
(검출이 없으면 n_samples 0). rssi_scanner 가 이 서버를 맡게 되면 이 파일은 지운다.
"""

import statistics
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from scv_msgs.action import PreciseMeasure
from scv_msgs.msg import NetRaw
from scv_survey.rssi_scanner import RAW_QOS


class PreciseServer(Node):
    def __init__(self):
        super().__init__('precise_server')
        cb = ReentrantCallbackGroup()
        self.values = None      # 모으는 중일 때만 list
        self.create_subscription(NetRaw, 'survey/raw', self.on_raw, RAW_QOS, callback_group=cb)
        ActionServer(self, PreciseMeasure, 'survey/precise', self.execute,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)

    def on_raw(self, m: NetRaw):
        if self.values is not None and m.detected:
            self.values.append(m.rssi_dbm)

    def execute(self, gh):
        self.values, t0 = [], time.time()
        while time.time() - t0 < gh.request.duration_s and not gh.is_cancel_requested:
            gh.publish_feedback(PreciseMeasure.Feedback(elapsed_s=int(time.time() - t0)))
            time.sleep(1.0)
        v, self.values = self.values, None
        res = PreciseMeasure.Result(n_samples=len(v))
        if v:
            res.rssi_median = float(statistics.median(v))
            if len(v) >= 2:
                q = statistics.quantiles(v, n=4)
                res.rssi_iqr = float(q[2] - q[0])
        self.get_logger().info(f'정밀 측정: median={res.rssi_median:.1f} iqr={res.rssi_iqr:.1f} n={res.n_samples}')
        if gh.is_cancel_requested:
            gh.canceled()
        else:
            gh.succeed()
        return res


def main(args=None):
    rclpy.init(args=args)
    node = PreciseServer()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
