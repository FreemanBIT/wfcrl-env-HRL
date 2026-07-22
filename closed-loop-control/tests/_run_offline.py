"""Offline test runner: executes test_* functions without requiring pytest.

Provides the two pytest features the unit tests use — ``tmp_path`` (a
per-test temporary Path) and ``pytest.raises`` — so the suite runs in an
environment where pytest is not installed. In a normal environment just use
``pytest tests/`` instead.
"""
from __future__ import annotations

import inspect
import sys
import tempfile
import traceback
from contextlib import contextmanager
from pathlib import Path


class _Raises:
    def __init__(self, exc):
        self.exc = exc

    def __enter__(self):
        return self

    def __exit__(self, et, ev, tb):
        if et is None:
            raise AssertionError(f"DID NOT RAISE {self.exc.__name__}")
        return issubclass(et, self.exc)


# make a fake "pytest" module available for import in the tests
class _PytestShim:
    raises = staticmethod(lambda exc: _Raises(exc))


sys.modules.setdefault("pytest", _PytestShim())


def run_module(modname: str) -> bool:
    mod = __import__(modname, fromlist=["*"])
    tests = [(k, v) for k, v in vars(mod).items()
             if k.startswith("test_") and callable(v)]
    passed = 0
    for name, fn in tests:
        sig = inspect.signature(fn)
        kwargs = {}
        tmp = None
        if "tmp_path" in sig.parameters:
            tmp = Path(tempfile.mkdtemp())
            kwargs["tmp_path"] = tmp
        try:
            fn(**kwargs)
            print(f"PASS  {name}")
            passed += 1
        except AssertionError as e:
            print(f"FAIL  {name}: {e}")
        except Exception as e:
            print(f"ERROR {name}: {type(e).__name__}: {e}")
            traceback.print_exc()
    print(f"\n{passed}/{len(tests)} passed in {modname}")
    return passed == len(tests)


if __name__ == "__main__":
    ok = True
    for m in sys.argv[1:] or ["tests.test_units"]:
        ok = run_module(m) and ok
    sys.exit(0 if ok else 1)
