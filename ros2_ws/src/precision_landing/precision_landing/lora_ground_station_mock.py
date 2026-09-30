"""
Mock Ground Station — simulates the receiving end of the LoRa link
(what would run on a ground-based LoRa receiver connected to the
dispatcher's laptop). For testing the drone-side LoRa protocol node
without physical hardware.
"""
import rclpy
from rclpy.node import Node
from precision_landing.lora_transport_mock import MockLoraTransport


class LoraGroundStationMock(Node):
    def __init__(self):
        super().__init__('lora_ground_station_mock')
        self.transport = MockLoraTransport(node_role="ground")
        self.create_timer(0.2, self.poll_incoming)
        self.get_logger().info("Mock LoRa Ground Station listening...")

    def poll_incoming(self):
        packet = self.transport.receive_packet(timeout=0.05)
        if packet:
            self.get_logger().info(f"[GROUND] Received via LoRa: {packet}")

    def destroy_node(self):
        self.transport.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoraGroundStationMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

