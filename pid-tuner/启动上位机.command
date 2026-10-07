#!/bin/zsh
set -eu
app_directory=${0:A:h}
if command -v uv >/dev/null 2>&1; then
    exec uv run --script "$app_directory/app.py" --open
fi
exec python3 "$app_directory/app.py" --open
