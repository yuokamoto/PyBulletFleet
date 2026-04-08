#!/usr/bin/env bash
# ============================================================================
# screen_capture.sh — Record a PyBullet GUI window via external screen capture
#
# Launches a demo script with GUI enabled, waits for the PyBullet window to
# appear, then records it using ffmpeg (x11grab) or kazam. The simulation
# runs at its real speed — no getCameraImage() overhead.
#
# Ideal for performance-showcase videos where DataMonitor FPS/RTF overlay
# is visible proof of real-time execution.
#
# Requirements (Ubuntu):
#   sudo apt install ffmpeg xdotool          # recommended
#   sudo apt install kazam                    # alternative (--kazam flag)
#
# Usage:
#   # Record single demo — output auto-named from script
#   #  → docs/media/100robots_grid_demo.mp4
#   ./scripts/screen_capture.sh examples/scale/100robots_grid_demo.py
#
#   # Custom output, duration, and extra demo args
#   ./scripts/screen_capture.sh -o perf_demo.mp4 -d 15 \
#       examples/scale/100robots_grid_demo.py --mode=mixed
#
#   # Use kazam instead of ffmpeg
#   ./scripts/screen_capture.sh --kazam examples/scale/100robots_grid_demo.py
#
#   # Multi-demo with interactive pause between demos
#   ./scripts/screen_capture.sh --multi \
#       examples/scale/100robots_grid_demo.py \
#       examples/scale/100robots_cube_patrol_demo.py
#
#   # Pause/resume during recording: press Enter or send SIGUSR1
#   #   kill -USR1 <pid>
# ============================================================================
set -euo pipefail

# ── Defaults ──────────────────────────────────────────────────────────────
OUTPUT=""                        # empty = auto-derive from demo script name
OUTPUT_DIR="docs/media"          # default output directory
DURATION=10
FPS=30
WINDOW_TITLE="Bullet Physics"
RECORDER="ffmpeg"               # ffmpeg | kazam
WINDOW_WAIT_TIMEOUT=15          # seconds to wait for window
STABILIZE_DELAY=2               # seconds after window appears before recording
MULTI_MODE=false                # run multiple demos sequentially
DEMO_SCRIPTS=()
DEMO_ARGS=()

# ── Parse arguments ───────────────────────────────────────────────────────
usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS] DEMO_SCRIPT [DEMO_ARGS...]
       $(basename "$0") [OPTIONS] --multi DEMO1.py DEMO2.py ...

Record the PyBullet GUI window while running a demo script.

Options:
  -o, --output FILE     Output file (default: auto from script name)
  --output-dir DIR      Output directory (default: $OUTPUT_DIR)
  -d, --duration SECS   Recording duration per demo (default: $DURATION)
  -f, --fps FPS         Capture FPS (default: $FPS)
  -t, --title PATTERN   Window title to match (default: "$WINDOW_TITLE")
  --kazam               Use kazam instead of ffmpeg
  --wait SECS           Seconds to wait for window (default: $WINDOW_WAIT_TIMEOUT)
  --delay SECS          Delay after window appears before recording (default: $STABILIZE_DELAY)
  --multi               Multi-demo mode: each positional arg is a demo script
  -h, --help            Show this help

Pause/Resume (ffmpeg mode):
  During recording, press Enter to pause/resume.
  From another terminal:  kill -USR1 <script_pid>

Output Naming:
  By default, output is auto-derived from the demo script filename:
    examples/scale/100robots_grid_demo.py  ->  docs/media/100robots_grid_demo.mp4

Examples:
  $(basename "$0") examples/scale/100robots_grid_demo.py
  $(basename "$0") -o perf.mp4 -d 15 examples/scale/100robots_grid_demo.py
  $(basename "$0") --multi -d 8 demo1.py demo2.py demo3.py
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -o|--output)     OUTPUT="$2"; shift 2 ;;
        --output-dir)    OUTPUT_DIR="$2"; shift 2 ;;
        -d|--duration)   DURATION="$2"; shift 2 ;;
        -f|--fps)        FPS="$2"; shift 2 ;;
        -t|--title)      WINDOW_TITLE="$2"; shift 2 ;;
        --kazam)         RECORDER="kazam"; shift ;;
        --wait)          WINDOW_WAIT_TIMEOUT="$2"; shift 2 ;;
        --delay)         STABILIZE_DELAY="$2"; shift 2 ;;
        --multi)         MULTI_MODE=true; shift ;;
        -h|--help)       usage ;;
        --)              shift; break ;;  # end of options
        -*)              echo "Unknown option: $1"; usage ;;
        *)
            if [[ "$MULTI_MODE" == true ]]; then
                # In multi mode, all positional args are demo scripts
                DEMO_SCRIPTS+=("$1")
            else
                if [[ ${#DEMO_SCRIPTS[@]} -eq 0 ]]; then
                    DEMO_SCRIPTS+=("$1")
                else
                    DEMO_ARGS+=("$1")
                fi
            fi
            shift
            ;;
    esac
done

# Collect remaining positional args after "--" (or end of while loop)
for arg in "$@"; do
    if [[ "$MULTI_MODE" == true ]]; then
        DEMO_SCRIPTS+=("$arg")
    else
        if [[ ${#DEMO_SCRIPTS[@]} -eq 0 ]]; then
            DEMO_SCRIPTS+=("$arg")
        else
            DEMO_ARGS+=("$arg")
        fi
    fi
done

if [[ ${#DEMO_SCRIPTS[@]} -eq 0 ]]; then
    echo "ERROR: No demo script specified."
    usage
fi

for script in "${DEMO_SCRIPTS[@]}"; do
    if [[ ! -f "$script" ]]; then
        echo "ERROR: Script not found: $script"
        exit 1
    fi
done

# ── Derive output filename from demo script ──────────────────────────────
derive_output() {
    local script="$1"
    local base
    base=$(basename "$script" .py)
    echo "${OUTPUT_DIR}/${base}.mp4"
}

# ── Dependency checks ─────────────────────────────────────────────────────
check_cmd() {
    if ! command -v "$1" &>/dev/null; then
        echo "ERROR: '$1' not found. Install with: sudo apt install $1"
        exit 1
    fi
}

if [[ "$RECORDER" == "ffmpeg" ]]; then
    check_cmd ffmpeg
elif [[ "$RECORDER" == "kazam" ]]; then
    check_cmd kazam
    check_cmd dbus-send
fi

# ── Utility: find window by title ─────────────────────────────────────────
find_window_id() {
    local pattern="$1"
    if command -v xdotool &>/dev/null; then
        xdotool search --name "$pattern" 2>/dev/null | head -1
    else
        # Fallback: enumerate all windows via xprop on root
        local root_children
        root_children=$(xprop -root _NET_CLIENT_LIST 2>/dev/null \
            | grep -oP '0x[0-9a-fA-F]+' || true)
        for wid in $root_children; do
            local name
            name=$(xprop -id "$wid" WM_NAME 2>/dev/null \
                | sed -n 's/.*= "\(.*\)"/\1/p' || true)
            if [[ "$name" == *"$pattern"* ]]; then
                echo "$wid"
                return 0
            fi
        done
    fi
    return 1
}

get_window_geometry() {
    local wid="$1"
    local info
    info=$(xwininfo -id "$wid" 2>/dev/null)
    local x y w h
    x=$(echo "$info" | grep "Absolute upper-left X:" | awk '{print $NF}')
    y=$(echo "$info" | grep "Absolute upper-left Y:" | awk '{print $NF}')
    w=$(echo "$info" | grep "Width:"  | awk '{print $NF}')
    h=$(echo "$info" | grep "Height:" | awk '{print $NF}')
    echo "$x $y $w $h"
}

# ── Pause / Resume support (ffmpeg) ──────────────────────────────────────
# ffmpeg pauses/resumes via SIGSTOP/SIGCONT.
# We toggle on USR1 or interactive Enter key.
FFMPEG_PID=""
PAUSED=false

toggle_pause() {
    if [[ -z "$FFMPEG_PID" ]] || ! kill -0 "$FFMPEG_PID" 2>/dev/null; then
        return
    fi
    if [[ "$PAUSED" == false ]]; then
        kill -STOP "$FFMPEG_PID" 2>/dev/null || true
        PAUSED=true
        echo "  ** PAUSED  (press Enter or send USR1 to resume)"
    else
        kill -CONT "$FFMPEG_PID" 2>/dev/null || true
        PAUSED=false
        echo "  >> RESUMED"
    fi
}

# Handle USR1 signal for remote pause/resume
trap 'toggle_pause' USR1

# ── Interactive key reader (background) ───────────────────────────────────
KEY_READER_PID=""

start_key_reader() {
    if [[ -t 0 ]]; then
        (
            while true; do
                read -r -s -n 1 2>/dev/null || break
                kill -USR1 $$ 2>/dev/null || break
            done
        ) &
        KEY_READER_PID=$!
    fi
}

stop_key_reader() {
    if [[ -n "${KEY_READER_PID:-}" ]]; then
        kill "$KEY_READER_PID" 2>/dev/null || true
        wait "$KEY_READER_PID" 2>/dev/null || true
        KEY_READER_PID=""
    fi
}

# ── Record one demo ──────────────────────────────────────────────────────
record_one_demo() {
    local demo_script="$1"
    shift
    local demo_args=("$@")
    local out_file

    if [[ -n "$OUTPUT" ]]; then
        out_file="$OUTPUT"
    else
        out_file=$(derive_output "$demo_script")
    fi
    mkdir -p "$(dirname "$out_file")"

    local demo_base
    demo_base=$(basename "$demo_script")

    echo ""
    echo "=== Recording: $demo_base ==="
    echo "  Script:   $demo_script ${demo_args[*]:-}"
    echo "  Output:   $out_file"
    echo "  Duration: ${DURATION}s @ ${FPS}fps"
    echo "  Recorder: $RECORDER"
    if [[ -t 0 ]]; then
        echo "  Tip: Press Enter to pause/resume recording"
    fi
    echo ""

    echo "[1/5] Launching demo with GUI..."
    python3 "$demo_script" "${demo_args[@]}" &
    local demo_pid=$!

    # Cleanup for this demo
    local record_pid=""
    demo_cleanup() {
        stop_key_reader
        kill "$demo_pid" 2>/dev/null || true
        if [[ -n "${record_pid:-}" ]]; then
            kill -CONT "$record_pid" 2>/dev/null || true
            kill "$record_pid" 2>/dev/null || true
        fi
        if [[ "$RECORDER" == "kazam" ]]; then
            killall kazam 2>/dev/null || true
        fi
        wait 2>/dev/null || true
        FFMPEG_PID=""
        PAUSED=false
    }
    trap demo_cleanup EXIT

    # ── Wait for window ───────────────────────────────────────────────
    echo "[2/5] Waiting for window '$WINDOW_TITLE' (timeout: ${WINDOW_WAIT_TIMEOUT}s)..."
    local window_id=""
    local elapsed=0
    while [[ $elapsed -lt $WINDOW_WAIT_TIMEOUT ]]; do
        window_id=$(find_window_id "$WINDOW_TITLE" || true)
        if [[ -n "$window_id" ]]; then
            break
        fi
        sleep 0.5
        elapsed=$((elapsed + 1))
    done

    if [[ -z "$window_id" ]]; then
        echo "ERROR: Window '$WINDOW_TITLE' not found within ${WINDOW_WAIT_TIMEOUT}s"
        demo_cleanup
        trap - EXIT
        return 1
    fi
    echo "  Found window: $window_id"

    # ── Raise window to foreground ────────────────────────────────────
    # x11grab captures a screen *region*, not a specific window.
    # If the PyBullet window is behind another window (e.g. VS Code),
    # the wrong content gets recorded. Bring it to front first.
    if command -v xdotool &>/dev/null; then
        xdotool windowactivate --sync "$window_id" 2>/dev/null || true
        xdotool windowraise "$window_id" 2>/dev/null || true
        xdotool windowfocus --sync "$window_id" 2>/dev/null || true
        echo "  Raised window to foreground"
    fi

    echo "[3/5] Waiting ${STABILIZE_DELAY}s for window to stabilize..."
    sleep "$STABILIZE_DELAY"

    # ── Verify demo process + re-find window after stabilize delay ────
    # The demo may crash/finish during the delay, or PyBullet may have
    # recreated the window (new X11 ID). Re-search is more robust than
    # checking the stale window_id with xdotool getwindowname, which can
    # fail on raw OpenGL windows that lack standard WM properties.
    if ! kill -0 "$demo_pid" 2>/dev/null; then
        echo "ERROR: Demo process (PID $demo_pid) exited during stabilize delay."
        echo "  The demo finished or crashed before recording could start."
        echo "  Try: increase sim_duration or decrease delay in demos.yaml"
        demo_cleanup
        trap - EXIT
        return 1
    fi
    # Re-find the window (handles recreated windows / stale IDs)
    local new_window_id
    new_window_id=$(find_window_id "$WINDOW_TITLE" || true)
    if [[ -z "$new_window_id" ]]; then
        echo "ERROR: Window '$WINDOW_TITLE' not found after stabilize delay."
        echo "  The demo may have crashed. Check demo output above for errors."
        demo_cleanup
        trap - EXIT
        return 1
    fi
    if [[ "$new_window_id" != "$window_id" ]]; then
        echo "  Window ID changed: $window_id → $new_window_id (re-raising)"
        window_id="$new_window_id"
        if command -v xdotool &>/dev/null; then
            xdotool windowactivate --sync "$window_id" 2>/dev/null || true
            xdotool windowraise "$window_id" 2>/dev/null || true
            xdotool windowfocus --sync "$window_id" 2>/dev/null || true
        fi
    fi

    # ── Get window geometry (after raise — position may have changed) ─
    local win_x win_y win_w win_h
    read -r win_x win_y win_w win_h <<< "$(get_window_geometry "$window_id")"
    echo "  Window geometry: ${win_w}x${win_h}+${win_x}+${win_y}"

    # Ensure even dimensions for H.264
    win_w=$(( (win_w / 2) * 2 ))
    win_h=$(( (win_h / 2) * 2 ))

    # Validate geometry — ffmpeg silently fails on 0x0
    if [[ "$win_w" -le 0 || "$win_h" -le 0 ]]; then
        echo "ERROR: Invalid window geometry: ${win_w}x${win_h}"
        demo_cleanup
        trap - EXIT
        return 1
    fi

    # ── Start recording ───────────────────────────────────────────────
    echo "[4/5] Recording for ${DURATION}s..."

    local ffmpeg_log
    ffmpeg_log=$(mktemp "/tmp/screen_capture_ffmpeg_XXXXXX.log")

    if [[ "$RECORDER" == "ffmpeg" ]]; then
        ffmpeg -y \
            -video_size "${win_w}x${win_h}" \
            -framerate "$FPS" \
            -f x11grab \
            -i "${DISPLAY:-:0}.0+${win_x},${win_y}" \
            -t "$DURATION" \
            -c:v libx264 \
            -preset fast \
            -crf 23 \
            -pix_fmt yuv420p \
            "$out_file" \
            </dev/null 2>"$ffmpeg_log" &
        record_pid=$!
        FFMPEG_PID=$record_pid

        start_key_reader

    elif [[ "$RECORDER" == "kazam" ]]; then
        kazam --silent &
        sleep 2
        dbus-send --session --type=method_call \
            --dest=org.kazam /org/kazam org.kazam.start_recording 2>/dev/null || {
            echo "WARNING: DBus start failed. Trying kazam CLI..."
            killall kazam 2>/dev/null || true
            sleep 1
            kazam -a "${win_x},${win_y},${win_w},${win_h}" --silent &
            record_pid=$!
            sleep 2
            dbus-send --session --type=method_call \
                --dest=org.kazam /org/kazam org.kazam.start_recording 2>/dev/null || true
        }
        (
            sleep "$DURATION"
            dbus-send --session --type=method_call \
                --dest=org.kazam /org/kazam org.kazam.stop_recording 2>/dev/null || true
            sleep 2
            dbus-send --session --type=method_call \
                --dest=org.kazam /org/kazam org.kazam.quit 2>/dev/null || true
        ) &
        record_pid=$!
    fi

    wait "$record_pid" 2>/dev/null || true

    # ── Done ──────────────────────────────────────────────────────────
    echo "[5/5] Recording complete."
    stop_key_reader

    kill "$demo_pid" 2>/dev/null || true
    wait "$demo_pid" 2>/dev/null || true
    FFMPEG_PID=""
    PAUSED=false
    trap - EXIT

    if [[ -f "$out_file" ]]; then
        local size
        size=$(du -h "$out_file" | cut -f1)
        echo ""
        echo "  OK: $out_file ($size)"
    else
        echo ""
        echo "  FAIL: Output file not found: $out_file"
        if [[ -f "${ffmpeg_log:-}" ]]; then
            echo "  ffmpeg log (last 10 lines):"
            tail -10 "$ffmpeg_log" 2>/dev/null | sed 's/^/    /' || true
        fi
        if [[ "$RECORDER" == "kazam" ]]; then
            echo "    Kazam saves to ~/Videos/ by default. Check there."
            ls -lt ~/Videos/*.mp4 2>/dev/null | head -3 || true
        fi
    fi
    rm -f "${ffmpeg_log:-}" 2>/dev/null || true
}

# ── Main ──────────────────────────────────────────────────────────────────
echo "=== PyBullet Screen Capture ==="
echo "  PID: $$ (send USR1 to pause/resume)"

if [[ "$MULTI_MODE" == true ]]; then
    echo "  Mode: multi-demo (${#DEMO_SCRIPTS[@]} demos)"
    echo ""

    for i in "${!DEMO_SCRIPTS[@]}"; do
        script="${DEMO_SCRIPTS[$i]}"
        n=$((i + 1))
        total=${#DEMO_SCRIPTS[@]}

        echo ""
        echo "--- Demo $n/$total ---"

        # For multi-mode with explicit -o, append index to filename
        if [[ -n "$OUTPUT" && $total -gt 1 ]]; then
            saved_output="$OUTPUT"
            local_ext="${OUTPUT##*.}"
            local_base="${OUTPUT%.*}"
            OUTPUT="${local_base}_${n}.${local_ext}"
            record_one_demo "$script"
            OUTPUT="$saved_output"
        else
            record_one_demo "$script"
        fi

        # Prompt between demos (if not the last one)
        if [[ $n -lt $total ]]; then
            echo ""
            echo "--------------------------------------"
            echo "  Next: $(basename "${DEMO_SCRIPTS[$((i + 1))]}")"
            echo "  Press Enter to start, or Ctrl+C to stop."
            echo "--------------------------------------"
            read -r
        fi
    done

    echo ""
    echo "=== All ${#DEMO_SCRIPTS[@]} demos recorded ==="
else
    record_one_demo "${DEMO_SCRIPTS[0]}" "${DEMO_ARGS[@]}"
fi

echo ""
echo "Done."
echo "  Play:     xdg-open <file>"
echo "  To GIF:   ffmpeg -i <file> -vf 'fps=10,scale=640:-1' output.gif"
