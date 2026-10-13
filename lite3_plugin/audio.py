"""Audio command encoding for the Lite3 plugin.

The Lite3's speaker is on the Motion Host. EMOS and this plugin run on the
compute board, so audio is streamed to the Motion Host over UDP as raw
mono F32LE PCM and played there (see the plugin README for the receiver-side
gstreamer command).
"""

import base64
import queue
import threading
import time
from io import BytesIO
from typing import List, Optional, Union

from ros_sugar.robot import UdpTransport


def encode_audio(
    output: Union[bytes, str],
    block_size: int = 1024,
    expected_rate: int = 16000,
    logger=None,
) -> List[bytes]:
    """Decode an audio blob into raw mono F32LE PCM blocks for UDP streaming.

    :param output: Encoded audio (wav/flac/...) as bytes, or a base64 string.
    :param block_size: Frames per UDP packet.
    :param expected_rate: Sample rate the Motion Host receiver is configured
        for; a mismatch is logged (audio would play at the wrong speed).
    :returns: List of raw float32 little-endian mono PCM blocks.
    """
    try:
        import numpy as np
        from soundfile import SoundFile
    except ModuleNotFoundError as e:
        raise ModuleNotFoundError(
            "Lite3 audio streaming requires 'soundfile' "
            "(pip install soundfile)."
        ) from e

    raw = output if isinstance(output, bytes) else base64.b64decode(output)
    if not raw:
        return []

    blocks: List[bytes] = []
    with SoundFile(BytesIO(raw)) as f:
        if logger is not None and f.samplerate != expected_rate:
            logger.warning(
                f"Audio sample rate {f.samplerate} Hz != expected "
                f"{expected_rate} Hz; the Motion Host would play it at the "
                "wrong speed. Set AUDIO_SAMPLE_RATE to the TTS model's rate."
            )
        for block in f.blocks(block_size, dtype="float32", always_2d=True):
            # Downmix to mono so the wire format is deterministic.
            if block.shape[1] > 1:
                block = block.mean(axis=1)
            blocks.append(np.ascontiguousarray(block, dtype="<f4").tobytes())
    return blocks


class PacedUdpTransport(UdpTransport):
    """A send-only UDP transport that sends one datagram every ``interval``
    seconds from a thread of its own.
    """

    def __init__(self, name: str, *, interval: float, **kwargs) -> None:
        super().__init__(name, **kwargs)
        self.interval = interval
        self._queue: "queue.Queue[bytes]" = queue.Queue()
        self._sender: Optional[threading.Thread] = None
        self._sender_stop = threading.Event()

    def send(self, payload: Union[bytes, List[bytes]]) -> bool:
        if self.send_to is None:
            return False
        if self._send_sock is None:
            self.open_egress()
        if self._sender is None or not self._sender.is_alive():
            self._sender_stop.clear()
            self._sender = threading.Thread(
                target=self._send_loop, name=f"udp-{self.name}-pace", daemon=True
            )
            self._sender.start()
        datagrams = payload if isinstance(payload, (list, tuple)) else (payload,)
        for datagram in datagrams:
            self._queue.put(datagram)
        return True

    def _send_loop(self) -> None:
        due = time.monotonic()
        while not self._sender_stop.is_set():
            try:
                datagram = self._queue.get(timeout=0.5)
            except queue.Empty:
                due = time.monotonic()
                continue
            try:
                self._send_sock.sendto(datagram, self.send_to)
            except OSError:
                pass
            due += self.interval
            delay = due - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                due = time.monotonic()  # behind: no burst to catch up

    def close(self) -> None:
        self._sender_stop.set()
        if self._sender is not None:
            self._sender.join(timeout=2.0)
            self._sender = None
        self._queue = queue.Queue()
        super().close()
