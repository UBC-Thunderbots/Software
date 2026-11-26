#!/usr/bin/env bash

# TODO REMOVE THIS LATER ON, OR REWRITE TO BE BETTER

VERBOSE=0

log() {
  if [ $VERBOSE -eq 1 ]; then
    echo $1
  fi
}

log "Removing epoch timestamps from logs"

for f in /tmp/tbots_*/test/*/avah_sim.log; do
  log "..."
  sed -I "" "/epoch_timestamp_seconds/d" "$f"
done

log "Checking for diffs between test runs"

HAS_DIFFS=0

for run1 in /tmp/tbots_*/; do
  for run2 in /tmp/tbots_*/; do
    if [ $run1 != $run2 ]; then
      for t in ${run1}test/*/avah_sim.log; do
        rel_log="${t#$run1}"
        log1="$run1$rel_log"
        log2="$run2$rel_log"

        if [ -f "$log1" ]; then
          log "Comparing $log1 $log2"
          diff $log1 $log2
          if [ $? -eq 1 ]; then
            HAS_DIFFS=1
            log "DIFFS FOUND BETWEEN: $log1 $log2"
          fi
        fi
      done
    fi
  done
done

if [ $HAS_DIFFS -eq 0 ]; then
  echo "No diffs found :)"
else
  echo "DIFFS FOUND! :(((("
fi
