"""rosbridge に何も購読しない websocket の接続を 1 本張り続ける（/client_count を 1 以上に保つ）。

UI を閉じて CPU を計測するとき、Standby で「クライアント 0 -> 電源 OFF」にならないようにする。Ctrl-C で切断。
rosbridge の生存確認（websocket の ping）に pong で答える（答えないと数秒で切られる）。
"""

import base64
import os
import socket
import struct
import time


def recv_exact(s, n):
    buf = b''
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('closed')
        buf += chunk
    return buf


def send_frame(s, opcode, payload=b''):
    # クライアントからのフレームはマスク必須（RFC 6455）。payload は ping と同じ（125 バイト以下）
    mask = os.urandom(4)
    head = bytes([0x80 | opcode, 0x80 | len(payload)]) + mask
    s.sendall(head + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))


def session():
    s = socket.create_connection(('127.0.0.1', 9090), timeout=5)
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall(
        (
            'GET / HTTP/1.1\r\nHost: localhost:9090\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
            f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n'
        ).encode()
    )
    resp = b''
    while b'\r\n\r\n' not in resp:
        resp += recv_exact(s, 1)
    print(resp.split(b'\r\n')[0].decode(), flush=True)
    s.settimeout(None)
    while True:
        b0, b1 = recv_exact(s, 2)
        opcode, n = b0 & 0x0F, b1 & 0x7F
        if n == 126:
            n = struct.unpack('>H', recv_exact(s, 2))[0]
        elif n == 127:
            n = struct.unpack('>Q', recv_exact(s, 8))[0]
        payload = recv_exact(s, n)  # サーバからのフレームはマスクなし
        if opcode == 0x9:  # ping -> pong
            send_frame(s, 0xA, payload)
        elif opcode == 0x8:  # close
            raise ConnectionError('closed by server')


while True:
    try:
        session()
    except (OSError, ConnectionError) as e:
        print('retry:', e, flush=True)
    time.sleep(2)
