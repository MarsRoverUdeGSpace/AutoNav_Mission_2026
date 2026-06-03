#!/usr/bin/env bash
set -euo pipefail

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "error: not inside a git repo" >&2
  exit 1
fi

git config alias.tct '!tools/test_commit_tag.sh'
echo "added alias: git tct \"commit message\" \"tag\" [\"tag message\"]"
