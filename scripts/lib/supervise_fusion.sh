# Bash 5.1+: monitor required children, including after replay finishes.
supervise_fusion() {
    local replay_pid="$1" log_dir="$2"
    shift 2
    local active=("$replay_pid" "$@") ended status pid save_pid=""
    while ((${#active[@]})); do
        ended=''
        if wait -n -p ended "${active[@]}"; then status=0; else status=$?; fi
        if [[ "$ended" == "$replay_pid" && "$status" == 0 ]]; then
            echo "Replay finished; SLAM may still be processing queued scans. Logs: $log_dir"
            echo "Replay succeeded; preparing map save."
            local remaining=()
            for pid in "${active[@]}"; do
                [[ "$pid" == "$ended" ]] || remaining+=("$pid")
            done
            active=("${remaining[@]}")
            if [[ "${AUTO_SAVE:-0}" == 1 ]]; then
                python3 scripts/auto_save_fusion_map.py --logs "$log_dir" --map-id "$MAP_ID" > "$log_dir/save.log" 2>&1 &
                save_pid=$!
                pids+=("$save_pid")
                active+=("$save_pid")
            fi
        elif [[ -n "$save_pid" && "$ended" == "$save_pid" && "$status" == 0 ]]; then
            cat "$log_dir/save.log"
            echo "Windows remain open for inspection. Ctrl+C to exit."
            local remaining=()
            for pid in "${active[@]}"; do
                [[ "$pid" == "$ended" ]] || remaining+=("$pid")
            done
            active=("${remaining[@]}")
        else
            echo "Fusion system stopped: PID=${ended:-unknown}, exit=$status. Logs: $log_dir" >&2
            echo "Check slam.log, fusion.log, rviz.log and replay.log." >&2
            [[ -f "$log_dir/save.log" ]] && tail -10 "$log_dir/save.log" >&2
            return 1
        fi
    done
}
