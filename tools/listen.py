#!/usr/bin/env python3
"""Listen to Peloton BLE Bridge multicast streams and pretty-print them."""

import socket
import struct
import sys
import json
from datetime import datetime

MULTICAST_IP = "239.255.42.99"
PORTS = {
    41234: "heartbeat",
    41235: "debug",
}

socks = []
for port, label in PORTS.items():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", port))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                 struct.pack("4sL", socket.inet_aton(MULTICAST_IP), socket.INADDR_ANY))
    s.setblocking(False)
    socks.append((s, label))

print(f"Listening on {MULTICAST_IP} ports {list(PORTS)} — Ctrl-C to stop", flush=True)

import select
try:
    while True:
        readable, _, _ = select.select([s for s, _ in socks], [], [], 1.0)
        for s in readable:
            label = dict(zip([s for s, _ in socks], [l for _, l in socks]))[s]
            data = s.recv(512).decode(errors="replace").strip()
            ts = datetime.now().strftime("%H:%M:%S")
            if label == "heartbeat":
                try:
                    d = json.loads(data)
                    calib = f"{d['calib']}/31" if 'calib' in d else "?"
                    print(f"[{ts}] cadence={d['cadence']:3} rpm  power={d['power']:4} W  "
                          f"resist={d['resistance']:3}%  calib={calib}  rx={d['rxBytes']}", flush=True)
                except json.JSONDecodeError:
                    print(f"[{ts}] {label}: {data}", flush=True)
            else:
                print(f"[{ts}] {data}", flush=True)
except KeyboardInterrupt:
    print("\nDone.")
finally:
    for s, _ in socks:
        s.close()
