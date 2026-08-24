"""pytest configuration for SafeRoom.

The package is installed (``pip install -e ".[dev]"``), so no path juggling
is needed here.
"""

import numpy as np
import pytest


@pytest.fixture(autouse=True)
def _seeded_rng():
    """Make the synthetic thermal fixtures deterministic.

    ``tests/test_ir_confirmer.py`` builds frames with ``np.random.normal``.
    Left unseeded, the noise occasionally pushes a blob across a decision
    threshold and the assertion flips, so the suite is intermittently red for
    reasons unrelated to the code under test.
    """
    np.random.seed(0)
