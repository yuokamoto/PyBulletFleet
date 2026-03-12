#!/usr/bin/env bash
# setup_skills.sh — Install PyBulletFleet domain skills as symlinks
#
# Usage:
#   ./setup_skills.sh              # Install (create symlinks)
#   ./setup_skills.sh --uninstall  # Remove symlinks
#   ./setup_skills.sh --status     # Show current state
#
# Skills are stored in this repo at .copilot/skills/ and symlinked
# into ~/.copilot/skills/ so all agent platforms can discover them.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_SKILLS_DIR="${REPO_DIR}/.copilot/skills"
TARGET_DIR="${HOME}/.copilot/skills"

# All PyBulletFleet skills
SKILLS=(
    "working-with-pybullet-fleet"
    "pybullet-performance-workflow"
    "adding-sim-entities"
    "pybullet-collision-tuning"
    "integrating-with-uso"
)

usage() {
    echo "Usage: $0 [--uninstall|--status]"
    echo ""
    echo "  (no args)     Install skills (create symlinks)"
    echo "  --uninstall   Remove skill symlinks"
    echo "  --status      Show installation status"
    exit 1
}

install_skills() {
    mkdir -p "${TARGET_DIR}"
    local installed=0 skipped=0

    for skill in "${SKILLS[@]}"; do
        local src="${REPO_SKILLS_DIR}/${skill}"
        local dst="${TARGET_DIR}/${skill}"

        if [[ ! -d "${src}" ]]; then
            echo "  WARN: ${skill} — source not found at ${src}, skipping"
            skipped=$((skipped + 1))
            continue
        fi

        if [[ -L "${dst}" ]]; then
            local current_target
            current_target="$(readlink -f "${dst}")"
            local expected_target
            expected_target="$(readlink -f "${src}")"
            if [[ "${current_target}" == "${expected_target}" ]]; then
                echo "  OK:   ${skill} — already linked"
                installed=$((installed + 1))
                continue
            else
                echo "  UPDATE: ${skill} — repointing symlink"
                rm "${dst}"
            fi
        elif [[ -e "${dst}" ]]; then
            echo "  WARN: ${skill} — ${dst} exists but is not a symlink, skipping"
            skipped=$((skipped + 1))
            continue
        fi

        ln -s "${src}" "${dst}"
        echo "  LINK: ${skill} → ${src}"
        installed=$((installed + 1))
    done

    echo ""
    echo "Done: ${installed} installed, ${skipped} skipped"
}

uninstall_skills() {
    local removed=0

    for skill in "${SKILLS[@]}"; do
        local dst="${TARGET_DIR}/${skill}"

        if [[ -L "${dst}" ]]; then
            rm "${dst}"
            echo "  REMOVED: ${skill}"
            removed=$((removed + 1))
        elif [[ -e "${dst}" ]]; then
            echo "  SKIP:    ${skill} — exists but is not a symlink (not managed by us)"
        else
            echo "  SKIP:    ${skill} — not installed"
        fi
    done

    echo ""
    echo "Done: ${removed} removed"
}

show_status() {
    echo "PyBulletFleet Skills Status"
    echo "Source:  ${REPO_SKILLS_DIR}"
    echo "Target:  ${TARGET_DIR}"
    echo ""

    for skill in "${SKILLS[@]}"; do
        local src="${REPO_SKILLS_DIR}/${skill}"
        local dst="${TARGET_DIR}/${skill}"

        if [[ -L "${dst}" ]]; then
            local target
            target="$(readlink "${dst}")"
            echo "  ✓ ${skill} → ${target}"
        elif [[ -d "${dst}" ]]; then
            echo "  ⚠ ${skill} — directory (not symlink)"
        elif [[ ! -d "${src}" ]]; then
            echo "  ✗ ${skill} — source missing"
        else
            echo "  ✗ ${skill} — not installed"
        fi
    done
}

# Main
case "${1:-}" in
    --uninstall) uninstall_skills ;;
    --status)    show_status ;;
    --help|-h)   usage ;;
    "")          install_skills ;;
    *)           usage ;;
esac
