#!/usr/bin/env bash
# Prove a UART can talk to itself before trusting anything measured through it.
#
# Two hardware conclusions in docs/ were wrong because every capture was taken
# through a broken receiver. Run this FIRST when debugging any serial link here.
#
# Wiring: jumper the port's TX pin to its OWN RX pin and unplug everything else.
#         For /dev/ttyTHS1 that is 40-pin header pin 8 -> pin 10.
#
# Usage:  bash scripts/uart_loopback_test.sh [-d DEV] [-b BAUD] [-r REPS] [-s "SIZES"]
#
# Reads:  EXACT ECHO at every size  -> the port is healthy.
#         ALL ZEROS above some size -> the data path is broken; see
#                                      docs/xrce-agent-nx.md.
set -u

DEV=/dev/ttyTHS1
BAUD=115200
REPS=3
SIZES="1 4 8 16 32 64"

while getopts "d:b:r:s:h" o; do
  case "$o" in
    d) DEV=$OPTARG ;;
    b) BAUD=$OPTARG ;;
    r) REPS=$OPTARG ;;
    s) SIZES=$OPTARG ;;
    h) sed -n '2,20p' "$0"; exit 0 ;;
    *) exit 2 ;;
  esac
done

if [ ! -e "$DEV" ]; then echo "no such device: $DEV" >&2; exit 1; fi

# The agent (or anything else) must not hold the port -- a second reader steals
# bytes and the result is meaningless.
if systemctl is-active --quiet dds-agent 2>/dev/null; then
  echo "ERROR: dds-agent is running and owns $DEV." >&2
  echo "       sudo systemctl stop dds-agent   # then re-run" >&2
  exit 1
fi

TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT

stty -F "$DEV" "$BAUD" raw -echo -echoe -echok -crtscts min 0 time 20

# printable payload with no zero bytes, so any 0x00 in the output is corruption
gen() {
  python3 -c "import sys;n=int(sys.argv[1]);sys.stdout.write(''.join(chr(33+(i%90)) for i in range(n)))" "$1"
}

echo "device=$DEV baud=$BAUD reps=$REPS"
printf '%6s %5s %6s %8s  %s\n' size rep got nonzero verdict
echo "----------------------------------------------------------"
fail=0
for n in $SIZES; do
  for r in $(seq 1 "$REPS"); do
    msg=$(gen "$n")
    out=$TMP/lb.bin; rm -f "$out"
    # reader must be open BEFORE the write, or it races and reads nothing
    timeout 3 head -c "$n" "$DEV" > "$out" &
    reader=$!
    python3 -c "import time; time.sleep(0.3)"
    printf '%s' "$msg" > "$DEV"
    wait $reader 2>/dev/null
    got=$(stat -c %s "$out" 2>/dev/null || echo 0)
    nz=$(python3 -c "print(sum(1 for c in open('$out','rb').read() if c))" 2>/dev/null || echo 0)
    if [ "$got" = "0" ]; then v="nothing back"; fail=1
    elif cmp -s <(printf '%s' "$msg") "$out"; then v="EXACT ECHO"
    elif [ "$nz" = "0" ]; then v="ALL ZEROS"; fail=1
    else v="partial ($nz nonzero)"; fail=1
    fi
    printf '%6s %5s %6s %8s  %s\n' "$n" "$r" "$got" "$nz" "$v"
  done
done

echo
if [ "$fail" = "0" ]; then
  echo "PASS -- $DEV echoes cleanly. Anything measured through it can be trusted."
else
  echo "FAIL -- $DEV corrupts its own data. Do NOT trust captures taken through it."
  echo "        See docs/xrce-agent-nx.md."
fi
exit $fail
