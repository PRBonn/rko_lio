# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# equivalent to cpp/rko_lio/core/profiler.hpp
import atexit
import time
from collections import defaultdict


class ScopedProfiler:
    _profile_data = defaultdict(lambda: {"count": 0, "total": 0.0, "max_time": 0.0})

    def __init__(self, name):
        self.name = name

    def __enter__(self):
        self.start = time.perf_counter()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        elapsed = (time.perf_counter() - self.start) * 1000  # ms
        data = ScopedProfiler._profile_data[self.name]
        data["count"] += 1
        data["total"] += elapsed
        data["max_time"] = max(data["max_time"], elapsed)

    @staticmethod
    def print_results():
        for name, info in ScopedProfiler._profile_data.items():
            avg_ms = info["total"] / info["count"] if info["count"] else 0.0
            avg_hz = 1000.0 / avg_ms if avg_ms > 0 else 0.0
            max_ms = info["max_time"]
            max_hz = 1000.0 / max_ms if max_ms > 0 else 0.0
            print(
                f"{name} - Profiling results:\n"
                f"  Execution count: {info['count']}\n"
                f"  Average time: {avg_ms:.2f}ms\n"
                f"  Average frequency: {avg_hz:.2f}Hz\n"
                f"  Max (worst-case) time: {max_ms:.2f}ms\n"
                f"  Worst-case frequency: {max_hz:.2f}Hz\n"
            )


atexit.register(ScopedProfiler.print_results)
