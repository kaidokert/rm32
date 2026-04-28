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
