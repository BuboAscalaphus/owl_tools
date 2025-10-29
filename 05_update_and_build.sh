#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/config.sh"

# Optional: set OWL_META_URL to auto-clone/update your meta repo on the HOST
#   export OWL_META_URL=https://github.com/you/owl_meta.git
OWL_META_URL="${OWL_META_URL:-}"

# --- 0) Sanity: workspace exists on host (bind-mounted into container) ---
mkdir -p "$WS"   # e.g., ~/owl_ws  (host)  ↔  /home/dev/ws (container)

# --- 1) Ensure owl_meta is present/updated on the HOST (if URL provided) ---
if [[ -n "$OWL_META_URL" ]]; then
  if [[ ! -d "$WS/owl_meta/.git" && ! -d "$WS/src/owl_meta/.git" ]]; then
    echo "📦 Cloning owl_meta on host → $WS/owl_meta"
    git clone "$OWL_META_URL" "$WS/owl_meta"
  else
    # prefer the one under src/ if present; otherwise update root-level owl_meta
    if [[ -d "$WS/src/owl_meta/.git" ]]; then
      echo "🔄 Updating host $WS/src/owl_meta"
      git -C "$WS/src/owl_meta" pull --ff-only || true
    else
      echo "🔄 Updating host $WS/owl_meta"
      git -C "$WS/owl_meta" pull --ff-only || true
    fi
  fi
fi

# --- 2) Locate the .repos file on the HOST ---
REPOS_FILE=""
if [[ -f "$WS/src/owl_meta/owl_stack.repos" ]]; then
  REPOS_FILE="$WS/src/owl_meta/owl_stack.repos"
elif [[ -f "$WS/owl_meta/owl_stack.repos" ]]; then
  REPOS_FILE="$WS/owl_meta/owl_stack.repos"
fi

# --- 3) Import any NEW repos into $WS/src on the HOST (idempotent) ---
if [[ -n "$REPOS_FILE" ]]; then
  echo "🧾 Importing missing repos from $REPOS_FILE (host)"
  mkdir -p "$WS/src"
  (cd "$WS" && vcs import --skip-existing src < "$REPOS_FILE" || true)
else
  echo "ℹ️  No owl_stack.repos found under $WS/src/owl_meta/ or $WS/owl_meta/."
fi

# --- 4) Pull existing repos on the HOST (so container does no Git at all) ---
if [[ -d "$WS/src" ]]; then
  echo "🔄 Pulling existing repos in $WS/src (host)"
  (cd "$WS" && vcs pull src || true)
fi

# --- 5) Now only do deps + build INSIDE the container ---
echo "🚢 Using container '$NAME' for rosdep + colcon (no Git in container)"
docker exec -it "$NAME" bash -lc '
  set -e
  cd ~/ws
  echo "📦 rosdep install"
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
  echo "🛠  colcon build"
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install
'


