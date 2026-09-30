"""
Mock LoRa Transport Layer — simulates a physical LoRa module (e.g. SX1262)
using a local UDP loopback instead of real UART/SPI hardware.
Swap this module for a real serial/SPI driver once hardware is available;
the LoraProtocolNode interface (send_packet/receive_packet) stays the same.
"""
import socket
import struct
import threading
import queue

MOCK_LORA_PORT = 47001  # arbitrary local port simulating the "air interface"
PACKET_FORMAT = "<BIfffBBB"  # uint8, uint32, 3x float32, uint8, uint8, uint8 = 20 bytes
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

# Simulate realistic LoRa constraints
SIMULATED_LATENCY_SEC = 0.3   # LoRa air time is much slower than WiFi
SIMULATED_MAX_RANGE_LOSS = False  # set True to simulate packet loss for testing


class MockLoraTransport:
    def __init__(self, node_role="drone"):
        """node_role: 'drone' or 'ground' -- determines send/recv port pairing."""
        self.node_role = node_role
        self.recv_port = MOCK_LORA_PORT if node_role == "ground" else MOCK_LORA_PORT + 1
        self.send_port = MOCK_LORA_PORT + 1 if node_role == "ground" else MOCK_LORA_PORT

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", self.recv_port))
        self.sock.settimeout(0.5)

        self.rx_queue = queue.Queue()
        self._running = True
        self._rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._rx_thread.start()

    def _receive_loop(self):
        while self._running:
            try:
                data, _ = self.sock.recvfrom(1024)
                self.rx_queue.put(data)
            except socket.timeout:
                continue
            except OSError:
                break

    def send_packet(self, msg_type, timestamp, lat, lon, alt, battery_pct, status_flags):
        checksum = (msg_type ^ battery_pct ^ status_flags) & 0xFF
        packet = struct.pack(
            PACKET_FORMAT, msg_type, timestamp, lat, lon, alt,
            battery_pct, status_flags, checksum
        )
        import time
        time.sleep(SIMULATED_LATENCY_SEC)  # simulate LoRa air time
        self.sock.sendto(packet, ("127.0.0.1", self.send_port))

    def receive_packet(self, timeout=0.1):
        """Returns a decoded dict, or None if nothing received within timeout."""
        try:
            data = self.rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

        if len(data) != PACKET_SIZE:
            return None  # malformed packet, drop it

        msg_type, timestamp, lat, lon, alt, battery_pct, status_flags, checksum = \
            struct.unpack(PACKET_FORMAT, data)

        expected_checksum = (msg_type ^ battery_pct ^ status_flags) & 0xFF
        if checksum != expected_checksum:
            return None  # corrupted packet, drop it

        return {
            "msg_type": msg_type,
            "timestamp": timestamp,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "battery_pct": battery_pct,
            "status_flags": status_flags,
        }

    def close(self):
        self._running = False
        self.sock.close()
