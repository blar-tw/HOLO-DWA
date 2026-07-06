#!/usr/bin/env bash
#
# exp.sh - experiment driver on top of run_lab.sh, for scripted iteration:
#
#   ./exp.sh up [goal_x goal_y]   # kill + fresh detached stack (batch starts
#                                 #   by itself). Env: N_RUNS=5 RUN_TIMEOUT=90
#                                 #   HEADLESS=1 COLLISION_DIST=0.3
#   ./exp.sh collect <name>       # wait for the current batch to finish, then
#                                 #   write logs/exp/<name>/ (summary, csv,
#                                 #   report, pane capture, config snapshot)
#   ./exp.sh go <name>            # reset scanner (picks up code edits) + collect
#   ./exp.sh status               # one-shot: processes, run count, pane tail
#   ./exp.sh down                 # tear everything down
#
# A batch is "finished" when summary.jsonl grew by N_RUNS lines since the
# marker (written by up/go). collect times out after EXP_TIMEOUT seconds
# (default 900) and reports whatever is there.

set -euo pipefail
LAB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${LAB_DIR}"
LOG_DIR="${LAB_DIR}/logs"
SUMMARY="${LOG_DIR}/summary.jsonl"
MARKER="${LOG_DIR}/.exp_marker"          # "<offset_lines> <epoch> <n_runs>"
SESSION="${SESSION:-holo-dwa-lab}"

N_RUNS="${N_RUNS:-5}"
RUN_TIMEOUT="${RUN_TIMEOUT:-90}"
HEADLESS="${HEADLESS:-1}"
COLLISION_DIST="${COLLISION_DIST:-0.3}"
EXP_TIMEOUT="${EXP_TIMEOUT:-900}"

summary_lines() { [[ -f "${SUMMARY}" ]] && wc -l < "${SUMMARY}" || echo 0; }

write_marker() { echo "$(summary_lines) $(date +%s) ${1}" > "${MARKER}"; }

pane_tail() {
  local n="${1:-25}"
  local pane_file="${LOG_DIR}/.scan_pane"
  [[ -f "${pane_file}" ]] || { echo "(no scanner pane recorded)"; return; }
  tmux capture-pane -p -t "$(cat "${pane_file}")" -S -2000 2>/dev/null | \
    sed '/^$/d' | tail -n "${n}" || echo "(pane capture failed)"
}

case "${1:-}" in
  up)
    shift || true
    ./run_lab.sh kill >/dev/null 2>&1 || true
    sleep 1
    NO_ATTACH=1 HEADLESS="${HEADLESS}" N_RUNS="${N_RUNS}" RUN_TIMEOUT="${RUN_TIMEOUT}" \
      COLLISION_DIST="${COLLISION_DIST}" ./run_lab.sh "$@"
    write_marker "${N_RUNS}"
    echo "Stack launching (headless=${HEADLESS}, n_runs=${N_RUNS}, run_timeout=${RUN_TIMEOUT}s)."
    echo "Batch starts automatically in ~30 s. Then: ./exp.sh collect <name>"
    ;;

  go)
    [[ -n "${2:-}" ]] || { echo "usage: ./exp.sh go <name>" >&2; exit 1; }
    # keep the n_runs the stack was launched with (baked into restart helper)
    LAUNCH_NRUNS="$(awk '{print $3}' "${MARKER}" 2>/dev/null || echo "${N_RUNS}")"
    write_marker "${LAUNCH_NRUNS}"
    ./run_lab.sh reset
    exec "$0" collect "$2"
    ;;

  collect)
    [[ -n "${2:-}" ]] || { echo "usage: ./exp.sh collect <name>" >&2; exit 1; }
    NAME="$2"
    [[ -f "${MARKER}" ]] || { echo "ERROR: no marker - run './exp.sh up' first" >&2; exit 1; }
    read -r OFFSET T0 WANT < "${MARKER}"
    TARGET=$((OFFSET + WANT))
    echo "Waiting for ${WANT} run(s): summary ${OFFSET} -> ${TARGET} lines (timeout ${EXP_TIMEOUT}s)..."
    waited=0
    while :; do
      have="$(summary_lines)"
      if (( have >= TARGET )); then
        echo "Batch complete: ${have} lines (+$((have - OFFSET)))."
        break
      fi
      if (( waited >= EXP_TIMEOUT )); then
        echo "TIMEOUT after ${waited}s with $((have - OFFSET))/${WANT} runs - collecting anyway."
        break
      fi
      if (( waited % 60 == 0 && waited > 0 )); then
        echo "  ... ${waited}s, $((have - OFFSET))/${WANT} runs done"
      fi
      sleep 5; waited=$((waited + 5))
    done
    sleep 2   # let the last summary line / csv rows flush

    DEST="${LOG_DIR}/exp/${NAME}"
    mkdir -p "${DEST}"
    # new summary lines only
    if [[ -f "${SUMMARY}" ]]; then
      tail -n +"$((OFFSET + 1))" "${SUMMARY}" > "${DEST}/summary.jsonl"
    fi
    # session CSVs written since the marker
    find "${LOG_DIR}" -maxdepth 1 -name "session_*.csv" -newermt "@${T0}" \
      -exec cp {} "${DEST}/" \; 2>/dev/null || true
    # config snapshot: full planner copy + the dwa_config block of the node
    cp dwa_core.py "${DEST}/dwa_core.py.snap"
    sed -n '/self.dwa_config = dwa_core.Config()/,/goal_threshold/p' scanner_lab.py > "${DEST}/node_config.txt"
    pane_tail 120 > "${DEST}/pane.txt"
    {
      echo "===== experiment: ${NAME} ====="
      echo "marker offset=${OFFSET} t0=${T0} want=${WANT} collected=$(( $(summary_lines) - OFFSET ))"
      echo
      ./analyze.py "${DEST}/summary.jsonl" || true
      echo
      for csv in "${DEST}"/session_*.csv; do
        [[ -e "${csv}" ]] && python3 report.py "${csv}" || true
      done
    } > "${DEST}/report.txt" 2>&1
    echo "Report -> ${DEST}/report.txt"
    ;;

  status)
    echo "--- processes ---"
    pgrep -af 'gz sim|MicroXRCEAgent|parameter_bridge|scanner_lab\.py' | sed 's/^/  /' || echo "  (none)"
    pgrep -x px4 >/dev/null && echo "  px4 running" || echo "  px4 NOT running"
    echo "--- summary.jsonl: $(summary_lines) lines total ---"
    [[ -f "${MARKER}" ]] && echo "  marker: $(cat "${MARKER}")  (offset epoch n_runs)"
    echo "--- scanner pane tail ---"
    pane_tail 15 | sed 's/^/  /'
    ;;

  down)
    ./run_lab.sh kill
    ;;

  *)
    sed -n '3,20p' "$0"
    ;;
esac
