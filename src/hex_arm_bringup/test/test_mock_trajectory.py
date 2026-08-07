#!/usr/bin/env python3
from trajectory_test_common import run


if __name__ == "__main__":
    run("mock", launch_timeout=30.0, goal_timeout=15.0)

