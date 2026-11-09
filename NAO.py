#!/usr/bin/env python3
"""
NAO Autonomous Striker Controller Entry Point
Direct execution wrapper for Webots and standalone invocation.
"""

import os
import sys

# Add controllers/NAO to sys.path so modules and motions can be loaded seamlessly
controller_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "controllers", "NAO")
if os.path.isdir(controller_dir):
    sys.path.insert(0, controller_dir)
    target_script = os.path.join(controller_dir, "NAO.py")
    if os.path.isfile(target_script):
        with open(target_script, "rb") as f:
            code = compile(f.read(), target_script, "exec")
            exec(code, globals(), locals())
    else:
        print(f"[ERROR] Target controller not found at: {target_script}", file=sys.stderr)
        sys.exit(1)
else:
    print(f"[ERROR] Controller directory not found at: {controller_dir}", file=sys.stderr)
    sys.exit(1)
