#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BINARY="$SCRIPT_DIR/plannersim"
RUNS=5
TOTAL=0
DONE=0

for n in $(seq 500 1500 25000) 25000; do
    for scenario in 1 2 3; do
        for alg in 0 1; do
            TOTAL=$((TOTAL + 1))
        done
    done
done

for n in $(seq 500 1500 25000) 25000; do
    for scenario in 1 2 3; do
        for alg in 0 1; do
            DONE=$((DONE + 1))
            ALG_NAME="A*"
            [ "$alg" -eq 1 ] && ALG_NAME="RRT*"
            case $scenario in
                1) SCENE_NAME="particles_only" ;;
                2) SCENE_NAME="static_obstacles" ;;
                3) SCENE_NAME="dynamic_obstacles" ;;
            esac
            echo "[$DONE/$TOTAL] particles=$n  scenario=$SCENE_NAME  algorithm=$ALG_NAME  runs=$RUNS"
            "$BINARY" "$n" "$scenario" "$alg" "$RUNS" --headless
        done
    done
done

echo "Benchmark complete. Results in $SCRIPT_DIR/results.csv"
