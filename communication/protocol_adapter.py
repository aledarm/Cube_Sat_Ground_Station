"""Insertion point for Ricardo's ICD. No guessed byte layout or fake CRC.

Implementation checklist:
- version, message ID, command ID, timestamps and clock domain;
- units, axes, sign conventions and body/world reference;
- byte order, lengths, framing, numeric types and CRC parameters;
- ACK = received vs accepted vs applied; reject/error responses;
- bounded retry policy and idempotency, particularly for non-idempotent commands;
- stale/duplicate/out-of-order telemetry; explicit device capabilities;
- image encoding, fragments/reassembly, frame ID and timestamp;
- tested E-STOP behavior, local hardware protection and loss-of-link state.

Never map an absent measurement to 0.0 or confirm a command merely because
sendto() succeeded. The GUI must use 'unknown', 'pending' or 'not supported'.
"""
from core.models import Command, CommandResult, Telemetry


class ProtocolNotConfigured(RuntimeError):
    pass


class ProtocolAdapter:
    def encode_command(self, command: Command) -> bytes:
        raise ProtocolNotConfigured('Real commands disabled: TMTC ICD not implemented.')

    def decode_telemetry(self, datagram: bytes) -> Telemetry:
        raise ProtocolNotConfigured('Real telemetry disabled: TMTC ICD not implemented.')

    def decode_ack(self, datagram: bytes) -> CommandResult:
        raise ProtocolNotConfigured('Real ACK handling disabled: TMTC ICD not implemented.')

    def decode_camera_frame(self, datagrams: list[bytes]):
        raise ProtocolNotConfigured('Camera transport/encoding not defined by the ICD yet.')
