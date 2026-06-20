"""Run all unittest modules under tests/ with FreeCAD loaded."""

import os
import sys
import unittest


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if root not in sys.path:
        sys.path.insert(0, root)
    loader = unittest.TestLoader()
    suite = loader.discover(
        start_dir=os.path.join(root, "tests"),
        pattern="test_*.py",
    )
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


sys.exit(main())
