"""Future raw UDP transport, deliberately disconnected from the GUI in v0.1.

This moves bytes, NOT application messages. It adds no made-up CRC or packet IDs.
Call from a network worker or bounded event loop after implementing the ICD.
"""
from __future__ import annotations
import socket


class UdpTransport:
    def __init__(self) -> None:
        self._socket: socket.socket | None = None
        self.peer: tuple[str, int] | None = None

    def open(self, bind_host: str, bind_port: int, peer_host: str, peer_port: int) -> None:
        if not 0 <= bind_port <= 65535 or not 1 <= peer_port <= 65535:
            raise ValueError('Invalid UDP port.')
        self.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setblocking(False)
            sock.bind((bind_host, bind_port))
        except OSError:
            sock.close()
            raise
        self._socket, self.peer = sock, (peer_host, peer_port)

    def send(self, payload: bytes) -> None:
        if self._socket is None or self.peer is None:
            raise RuntimeError('UDP transport is closed.')
        if not isinstance(payload, bytes) or not payload or len(payload) > 65507:
            raise ValueError('Invalid datagram. Fragmentation policy belongs in the ICD.')
        self._socket.sendto(payload, self.peer)
        # UDP send is not an acknowledgement and MUST NOT update 'applied' state.

    def receive_available(self, max_packets: int = 32) -> list[tuple[bytes, tuple[str, int]]]:
        if self._socket is None:
            return []
        packets = []
        for _ in range(max_packets):
            try:
                packets.append(self._socket.recvfrom(65535))
            except BlockingIOError:
                break
        return packets

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
        self._socket, self.peer = None, None
