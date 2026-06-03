#!/usr/bin/env bash
set -euo pipefail

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "error: not inside a git repo" >&2
  exit 1
fi

if [[ $# -lt 2 ]]; then
  echo "usage: tools/test_commit_tag.sh \"commit message\" \"tag\" [\"tag message\"]" >&2
  exit 1
fi

commit_msg="$1"
tag_name="$2"
tag_msg="${3:-tested: ${tag_name}}"

if git diff --cached --quiet; then
  echo "error: no staged changes. stage with: git add -p" >&2
  exit 1
fi

branch="$(git rev-parse --abbrev-ref HEAD)"

git commit -m "${commit_msg}"
git tag -a "${tag_name}" -m "${tag_msg}"
git push origin "${branch}"
git push origin "${tag_name}"

echo "done: ${branch} @ ${tag_name}"
