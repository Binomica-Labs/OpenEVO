#!/bin/bash
cd "$(dirname "$0")"
echo "Starting OpenEvo Interface (June 11, 2026 - V1)..."
echo "Browser will open at http://localhost:8080"
(sleep 4 && open http://localhost:8080) &
python3 2026-06-11_OpenEvo_Interface_V1.py
