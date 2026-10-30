"""
Pytest runner for AM32 black-box test vectors.

Usage:
    cd tests/blackbox
    python -m pytest test_vectors.py -v

Or from repo root:
    python -m pytest tests/blackbox/test_vectors.py -v
"""

import pytest
from pathlib import Path
from harness import AM32Harness, run_test_vectors

VECTORS_DIR = Path(__file__).parent / "vectors"


@pytest.fixture
def harness():
    import os
    exe = os.environ.get("AM32_HARNESS", None)
    with AM32Harness(exe_path=exe) as h:
        yield h


def get_vector_files():
    """Discover all .txt vector files.

    Fails fast when no vectors are discovered so missing test data
    (e.g. due to path typos or CI misconfiguration) cannot silently
    skip the parametrized tests.
    """
    files = sorted(VECTORS_DIR.glob("*.txt"))
    assert files, f"No vector files found in {VECTORS_DIR!s}"
    return files


# Vectors with known Rust vs C behavioral differences (pending investigation)
XFAIL_VECTORS = {
    "desync_recovery",  # Rust doesn't reset zero_crosses on desync (different main_loop ordering)
}


@pytest.mark.parametrize(
    "vector_file",
    get_vector_files(),
    ids=lambda p: p.stem,
)
def test_vector(harness, vector_file):
    """Run a single test vector file."""
    if vector_file.stem in XFAIL_VECTORS:
        pytest.xfail(f"Known Rust vs C difference: {vector_file.stem}")
    run_test_vectors(harness, vector_file)


# Convenience: also run direct API tests

class TestHarnessAPI:
    """Basic sanity tests for the harness itself."""

    def test_start_stop(self, harness):
        state = harness.state()
        assert state["armed"] == 0
        assert state["running"] == 0
        assert state["tick"] == 0

    def test_single_tick(self, harness):
        s1 = harness.tick()
        assert s1["tick"] == 1
        assert s1["signaltimeout"] == 1

    def test_bulk_ticks(self, harness):
        state = harness.ticks(100)
        assert state["tick"] == 100

    def test_config(self, harness):
        harness.config(armed=1)
        state = harness.state()
        assert state["armed"] == 1

    def test_reset(self, harness):
        harness.config(armed=1)
        harness.reset()
        state = harness.state()
        assert state["armed"] == 0

    def test_arm_convenience(self, harness):
        state = harness.arm()
        assert state["armed"] == 1

    def test_servo_override_survives_repeated_raw_captures(self, harness):
        harness.config(inputSet=1, servoPwm=1, dshot=0,
                       input_pin_state=0, dma_0=1000, dma_1=2500)
        for expected in [250, 500, 750]:
            state = harness.tick(transfer=1)
            assert state["servoPwm"] == 1
            assert state["newinput"] == expected
            assert state["signaltimeout"] == 1

    def test_crsf_raw_serial_validation_and_recovery(self, harness):
        def frame(throttle):
            channels = [1811] * 16
            channels[2] = throttle
            packed = sum(value << (11 * i) for i, value in enumerate(channels))
            payload = bytes([0x16]) + packed.to_bytes(22, "little")
            crc = 0
            for byte in payload:
                crc ^= byte
                for _ in range(8):
                    crc = ((crc << 1) ^ (0xd5 if crc & 0x80 else 0)) & 255
            return bytes([0xc8, 24]) + payload + bytes([crc])

        harness.crsf(frame(992))
        harness.config(inputSet=0, signaltimeout=99)
        packet = frame(172)
        state = harness.crsf(packet[:-1])
        assert (state["newinput"], state["signaltimeout"], state["inputSet"]) == (1024, 99, 0)
        state = harness.crsf([packet[-1] ^ 1])
        assert (state["newinput"], state["signaltimeout"], state["inputSet"]) == (1024, 99, 0)
        state = harness.crsf([0xc8, 2, 0x16, 0xd3])
        assert (state["newinput"], state["signaltimeout"], state["inputSet"]) == (1024, 99, 0)
        for throttle, expected in [(172, 0), (1811, 2047), (992, 1024)]:
            state = harness.crsf(frame(throttle))
            assert (state["newinput"], state["signaltimeout"], state["inputSet"]) == (expected, 0, 1)

    def test_empty_crsf_input_returns_current_state(self, harness):
        harness.config(newinput=1234, signaltimeout=17)
        assert harness.crsf([]) == harness.state()
