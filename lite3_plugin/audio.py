"""Audio command encoding for the Lite3 plugin.

The Lite3's speaker is on the Motion Host. EMOS and this plugin run on the
compute board, so audio is streamed to the Motion Host over UDP as raw
mono F32LE PCM and played there (see the plugin README for the receiver-side
gstreamer command).
"""

import base64
from io import BytesIO
from typing import List, Union


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
