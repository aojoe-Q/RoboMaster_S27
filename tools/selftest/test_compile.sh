#!/bin/bash
cd "$(dirname "$0")"
echo "=== Testing USB selftest limits ==="
python run_selftest.py 2>&1 | head -20

echo ""
echo "=== Testing control selftest compilation ==="
python run_control_selftest.py 2>&1 | head -40
