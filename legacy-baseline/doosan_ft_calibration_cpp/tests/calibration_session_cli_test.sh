#!/bin/sh
set -eu
tmp_dir=$(mktemp -d /tmp/ft-calibration-cli-XXXXXX)
trap 'rm -rf -- "$tmp_dir"' EXIT

./build/ft_calibration demo "$tmp_dir/samples.csv" > /dev/null
printf 'ft_auto_session_v1\nstate=INCOMPLETE\ncaptured_poses=20\nexpected_poses=36\n' > "$tmp_dir/samples.csv.status"
if ./build/ft_calibration calibrate "$tmp_dir/samples.csv" "$tmp_dir/incomplete.txt" > "$tmp_dir/log" 2>&1; then
    printf 'Incomplete session was incorrectly accepted\n' >&2
    exit 1
fi
case "$(cat "$tmp_dir/log")" in
    *"Automatic capture is incomplete"*) ;;
    *) cat "$tmp_dir/log" >&2; exit 1 ;;
esac

printf 'ft_auto_session_v1\nstate=FITTING\ncaptured_poses=35\nexpected_poses=35\n' > "$tmp_dir/samples.csv.status"
if ./build/ft_calibration calibrate "$tmp_dir/samples.csv" "$tmp_dir/badcount.txt" > "$tmp_dir/log" 2>&1; then
    printf 'Session with a wrong row count was incorrectly accepted\n' >&2
    exit 1
fi
case "$(cat "$tmp_dir/log")" in
    *"row count does not match"*) ;;
    *) cat "$tmp_dir/log" >&2; exit 1 ;;
esac

printf 'ft_auto_session_v1\nstate=FITTING\ncaptured_poses=36\nexpected_poses=36\n' > "$tmp_dir/samples.csv.status"
./build/ft_calibration calibrate "$tmp_dir/samples.csv" "$tmp_dir/model.txt" > "$tmp_dir/log" 2>&1
./build/ft_calibration validate "$tmp_dir/samples.csv" "$tmp_dir/model.txt" valid > /dev/null
