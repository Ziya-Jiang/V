#!/usr/bin/env python3
"""Read-only Lingkong node probe. Sends status queries, never motion commands."""

import argparse
import socket
import struct
import time


FRAME_FORMAT = "=IB3x8s"
QUERY_CODES = (0x9A, 0x92, 0x9C)


def request(bus, node_id, command, timeout_s=0.08):
    can_id = 0x140 + node_id
    payload = bytes([command]) + bytes(7)
    bus.send(struct.pack(FRAME_FORMAT, can_id, 8, payload))
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        bus.settimeout(max(0.001, deadline - time.monotonic()))
        try:
            raw = bus.recv(16)
        except TimeoutError:
            return None
        response_id, dlc, data = struct.unpack(FRAME_FORMAT, raw)
        if (
            response_id & 0x7FF == can_id
            and dlc == 8
            and data[0] == command
        ):
            return data
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="can1")
    parser.add_argument("--node", type=int, help="只检查一个节点；默认扫描 1..32")
    args = parser.parse_args()
    nodes = [args.node] if args.node is not None else range(1, 33)
    if any(node < 1 or node > 32 for node in nodes):
        parser.error("node 必须在 1..32")

    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.bind((args.interface,))
    found = []
    for node in nodes:
        status1 = request(bus, node, QUERY_CODES[0])
        if status1 is None:
            continue
        found.append(node)
        temperature = int.from_bytes(status1[1:2], "little", signed=True)
        voltage = int.from_bytes(status1[2:4], "little", signed=True) / 100.0
        current = int.from_bytes(status1[4:6], "little", signed=True) / 100.0
        angle_data = request(bus, node, QUERY_CODES[1])
        status2 = request(bus, node, QUERY_CODES[2])
        angle = (
            int.from_bytes(angle_data[1:8], "little", signed=True) / 100.0
            if angle_data is not None
            else None
        )
        print(
            f"node={node} temperature={temperature}C voltage={voltage:.2f}V "
            f"current={current:.2f}A motor={status1[6]} "
            f"error=0x{status1[7]:02X} angle_deg={angle}"
        )
        if status2 is not None:
            torque = int.from_bytes(status2[2:4], "little", signed=True)
            speed = int.from_bytes(status2[4:6], "little", signed=True)
            encoder = int.from_bytes(status2[6:8], "little", signed=False)
            print(
                f"  torque_raw={torque} speed_dps={speed} encoder={encoder}"
            )
    print(f"found_nodes={found}")
    bus.close()


if __name__ == "__main__":
    main()
