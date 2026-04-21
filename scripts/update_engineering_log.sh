#!/usr/bin/env bash
set -euo pipefail

COUNT="${1:-20}"
README_FILE="README.md"
START_MARKER="<!-- ENGINEERING_LOG_START -->"
END_MARKER="<!-- ENGINEERING_LOG_END -->"

if [[ ! -f "${README_FILE}" ]]; then
    echo "Error: ${README_FILE} not found in current directory."
    exit 1
fi

if ! grep -Fq "${START_MARKER}" "${README_FILE}" || ! grep -Fq "${END_MARKER}" "${README_FILE}"; then
    echo "Error: Engineering log markers are missing in ${README_FILE}."
    exit 1
fi

tmp_log="$(mktemp)"
tmp_readme="$(mktemp)"
trap 'rm -f "${tmp_log}" "${tmp_readme}"' EXIT

has_commits="false"
if git rev-parse --is-inside-work-tree >/dev/null 2>&1 && git rev-parse --verify HEAD >/dev/null 2>&1; then
    git --no-pager log -n "${COUNT}" --date=short --pretty=format:'- %ad | %h | %s' > "${tmp_log}"
    has_commits="true"
else
    echo "- No commits yet. Create your first commit and rerun this script." > "${tmp_log}"
fi

awk -v start="${START_MARKER}" -v end="${END_MARKER}" -v logfile="${tmp_log}" '
BEGIN {
    in_block = 0
}
{
    if ($0 == start) {
        print
        while ((getline line < logfile) > 0) {
            print line
        }
        in_block = 1
        next
    }
    if ($0 == end) {
        in_block = 0
        print
        next
    }
    if (!in_block) {
        print
    }
}
' "${README_FILE}" > "${tmp_readme}"

mv "${tmp_readme}" "${README_FILE}"
if [[ "${has_commits}" == "true" ]]; then
    echo "Engineering log updated with latest ${COUNT} commit message(s)."
else
    echo "Engineering log updated with placeholder message (no commits found)."
fi
