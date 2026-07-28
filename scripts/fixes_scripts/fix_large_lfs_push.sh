#!/usr/bin/env bash
set -euo pipefail

# Configure repo-local Git LFS settings to improve reliability for large pushes.

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info() {
    echo -e "${YELLOW}$*${NC}"
}

ok() {
    echo -e "${GREEN}$*${NC}"
}

err() {
    echo -e "${RED}$*${NC}" >&2
}

if ! command -v git >/dev/null 2>&1; then
    err "git is not installed or not in PATH."
    exit 1
fi

if ! command -v git-lfs >/dev/null 2>&1; then
    err "git-lfs is not installed or not in PATH."
    exit 1
fi

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    err "Run this script inside a Git repository."
    exit 1
fi

if ! git remote get-url origin >/dev/null 2>&1; then
    err "No 'origin' remote found."
    exit 1
fi

ORIGIN_URL="$(git remote get-url origin)"

# Build Azure DevOps HTTPS LFS URL when origin is SSH:
# git@ssh.dev.azure.com:v3/ORG/PROJECT/REPO
# -> https://dev.azure.com/ORG/PROJECT/_git/REPO/info/lfs
infer_lfs_url_from_origin() {
    local origin="$1"

    if [[ "$origin" =~ ^git@ssh\.dev\.azure\.com:v3/([^/]+)/([^/]+)/([^/]+)$ ]]; then
        local org="${BASH_REMATCH[1]}"
        local project="${BASH_REMATCH[2]}"
        local repo="${BASH_REMATCH[3]}"
        echo "https://dev.azure.com/${org}/${project}/_git/${repo}/info/lfs"
        return 0
    fi

    if [[ "$origin" =~ ^ssh\.dev\.azure\.com:v3/([^/]+)/([^/]+)/([^/]+)$ ]]; then
        local org="${BASH_REMATCH[1]}"
        local project="${BASH_REMATCH[2]}"
        local repo="${BASH_REMATCH[3]}"
        echo "https://dev.azure.com/${org}/${project}/_git/${repo}/info/lfs"
        return 0
    fi

    return 1
}

set_local_if_needed() {
    local key="$1"
    local wanted="$2"
    local current
    current="$(git config --local --get "$key" || true)"

    if [[ "$current" == "$wanted" ]]; then
        ok "No change: $key already '$wanted'"
    else
        git config --local "$key" "$wanted"
        ok "Set: $key='$wanted'"
    fi
}

info "Applying Git LFS reliability settings (repo-local)..."

LFS_URL="$(git config --local --get lfs.url || true)"
if [[ -z "$LFS_URL" ]]; then
    if inferred="$(infer_lfs_url_from_origin "$ORIGIN_URL")"; then
        LFS_URL="$inferred"
        info "Inferred LFS URL from origin: $LFS_URL"
    else
        info "Could not infer Azure DevOps LFS URL from origin. Keeping existing lfs.url as-is."
    fi
fi

if [[ -n "$LFS_URL" ]]; then
    set_local_if_needed "lfs.url" "$LFS_URL"
fi

set_local_if_needed "lfs.locksverify" "false"
set_local_if_needed "lfs.concurrenttransfers" "1"
set_local_if_needed "http.version" "HTTP/1.1"

echo
info "Effective settings:"
git config --local --get-regexp '^lfs\.url$|^lfs\.locksverify$|^lfs\.concurrenttransfers$|^http\.version$' || true

echo
ok "Done. Retry push with: git push origin $(git branch --show-current)"
