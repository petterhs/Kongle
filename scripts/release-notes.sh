#!/usr/bin/env bash
set -euo pipefail

tag=${1:?release tag required}
output=${2:?output file required}
previous_stable=$(
  git tag --merged HEAD --list 'v[0-9]*' |
    grep -E '^v[0-9]+\.[0-9]+\.[0-9]+$' |
    grep -Fvx "$tag" |
    sort -V |
    tail -n 1 || true
)

# Use one synthetic release section. Existing RC/dev tags must not truncate
# stable notes, and each candidate should show the full delta since stable.
if [[ -n "$previous_stable" ]]; then
  git-cliff --config cliff.toml --tag-pattern '^$' --ignore-tags '^$' \
    --tag "$tag" --output "$output" "$previous_stable..HEAD"
else
  git-cliff --config cliff.toml --tag-pattern '^$' --ignore-tags '^$' \
    --tag "$tag" --output "$output"
fi
