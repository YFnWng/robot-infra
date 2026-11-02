import time

import pytest
import torch

from runtime_supervision.replay_exchange import ReplaySnapshot, publish, receive


def snapshot(sequence=1, contract='model'):
    now = time.monotonic()
    return ReplaySnapshot(sequence, now, now-.01, contract,
                          (torch.tensor([1., 2.]), {'history': [3., 4.]}, None))


def test_latest_complete_snapshot(tmp_path):
    assert receive(tmp_path, 'model') is None
    original = snapshot()
    assert publish(tmp_path, original) > 0
    decoded = receive(tmp_path, 'model')
    assert torch.equal(decoded.inputs[0], original.inputs[0])
    assert decoded.inputs[1:] == original.inputs[1:]
    publish(tmp_path, snapshot(2))
    assert receive(tmp_path, 'model').sequence == 2
    assert list(tmp_path.iterdir()) == [tmp_path/'snapshot.bin']


def test_contract_and_checksum_fail_closed(tmp_path):
    publish(tmp_path, snapshot())
    with pytest.raises(ValueError, match='contract'):
        receive(tmp_path, 'other')
    path = tmp_path/'snapshot.bin'
    path.write_bytes(path.read_bytes()[:-1]+b'!')
    with pytest.raises(ValueError, match='checksum'):
        receive(tmp_path, 'model')


def test_future_timestamp_rejected(tmp_path):
    publish(tmp_path, ReplaySnapshot(1, time.monotonic()+10, time.monotonic(), 'model', ()))
    with pytest.raises(ValueError, match='timestamps'):
        receive(tmp_path, 'model')


def test_bounded_mailbox(tmp_path, monkeypatch):
    from runtime_supervision import replay_exchange
    monkeypatch.setattr(replay_exchange, 'MAX_BYTES', 16)
    with pytest.raises(ValueError, match='bound'):
        publish(tmp_path, snapshot())
    (tmp_path/'snapshot.bin').write_bytes(b'x'*50)
    with pytest.raises(ValueError, match='size'):
        receive(tmp_path, 'model')
