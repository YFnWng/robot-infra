"""Owned latest-only snapshot mailbox for hardware-free process replay.

Atomic files in a private local temporary directory are deliberately a test
transport, not a production IPC API. Only the parent's own workers may write
here; pickle must never be loaded from external/untrusted sessions.
"""
from dataclasses import dataclass
import hashlib
import pickle
import time


MAX_BYTES = 4*1024*1024


@dataclass(frozen=True)
class ReplaySnapshot:
    sequence: int
    published_monotonic_s: float
    source_monotonic_s: float
    contract_sha256: str
    inputs: tuple


def publish(directory, snapshot):
    payload = pickle.dumps(snapshot, protocol=5)
    if len(payload) > MAX_BYTES:
        raise ValueError("replay snapshot exceeds mailbox bound")
    temporary = directory/"snapshot.tmp"
    temporary.write_bytes(hashlib.sha256(payload).digest()+payload)
    temporary.replace(directory/"snapshot.bin")
    return len(payload)


def receive(directory, contract_sha256):
    path = directory/"snapshot.bin"
    if not path.exists():
        return None
    with path.open('rb') as stream:
        data = stream.read(MAX_BYTES+33)
    if len(data) > MAX_BYTES+32 or len(data) < 32:
        raise ValueError("invalid replay snapshot size")
    digest, payload = data[:32], data[32:]
    if hashlib.sha256(payload).digest() != digest:
        raise ValueError("replay snapshot checksum mismatch")
    snapshot = pickle.loads(payload)  # Private parent-owned mailbox only.
    if (not isinstance(snapshot, ReplaySnapshot) or snapshot.sequence < 1
            or snapshot.contract_sha256 != contract_sha256
            or not 0 <= snapshot.source_monotonic_s <= snapshot.published_monotonic_s <= time.monotonic()):
        raise ValueError("invalid replay snapshot contract or timestamps")
    return snapshot
