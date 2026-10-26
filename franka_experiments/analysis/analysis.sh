#!/bin/bash
# Run the a-posteriori analyses of one experiment run, on the host or in the container.
#
#   source analysis.sh {left|right|both} [RUN]     Kalman filter: kf_compare.py + whiteness_test.py
#   source analysis.sh mpc [RUN]                   MPC: mpc_analysis.py
#   (or: bash analysis.sh ...)
#
# RUN is a run folder name, or <controller>/<run> (default: the latest run; for mpc, the latest
# mpc or mpc_cbf one). The logs are read from ~/Desktop/Thesis/experiment_logs, or in the
# container from /ros2_ws/experiment_logs (mounted from the host), and the plots and summary
# CSVs go to graphs_thesis next to them:
#   graphs_thesis/<run>/{savitzky_golay,whiteness_test}/
#   graphs_thesis/<controller>/<run>/mpc_analysis/
# The Python used needs pandas, scipy and matplotlib: $KF_PYTHON if set, else the 'floods'
# conda env, else python3 (the container's has them).

_run_analysis() {
    if [[ $# -lt 1 || $# -gt 2 || ! "$1" =~ ^(left|right|both|mpc)$ ]]; then
        echo "Usage: source analysis.sh {left|right|both|mpc} [RUN]"
        return 1
    fi
    local what="$1" here py
    here="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
    local run=()
    [[ -n "$2" ]] && run=("$2")

    for py in "${KF_PYTHON:-}" "$HOME/miniconda3/envs/floods/bin/python" python3; do
        [[ -n "$py" ]] && "$py" -c "import pandas, scipy, matplotlib" 2>/dev/null && break
        py=""
    done
    if [[ -z "$py" ]]; then
        echo "No Python with pandas + scipy + matplotlib found (set KF_PYTHON)."
        return 1
    fi

    if [[ "$what" == mpc ]]; then
        echo "Running mpc_analysis.py ${run[*]}..."
        "$py" "$here/mpc_analysis.py" "${run[@]}" || return 1
        echo "MPC analysis completed"
        return 0
    fi

    echo "Running kf_compare.py --side $what ${run[*]}..."
    "$py" "$here/kf_compare.py" "${run[@]}" --side "$what" || return 1

    echo "Running whiteness_test.py --side $what ${run[*]}..."
    "$py" "$here/whiteness_test.py" "${run[@]}" --side "$what" || return 1

    echo "Analysis completed for: $what"
}

_run_analysis "$@"
_analysis_status=$?
unset -f _run_analysis
# 'return' when sourced, 'exit' when run with bash
return $_analysis_status 2>/dev/null || exit $_analysis_status