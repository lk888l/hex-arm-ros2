#!/usr/bin/env python3
from trajectory_test_common import run


if __name__ == "__main__":
    run("gz", launch_timeout=90.0, goal_timeout=30.0)

