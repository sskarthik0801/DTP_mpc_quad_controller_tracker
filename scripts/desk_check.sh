#!/usr/bin/env bash
# desk_check.sh -- the four one-liners a flight day starts with, as one check with a verdict.
# Read-only. Exit 0 = every line PASS. Used by the GCS Flight day card ("Desk check") and by
# docs/flight-day-runbook.md step 1. The fault card is its own check (clear_fault_logs.py).
#
#   bash scripts/desk_check.sh
fail=0
line() { printf '  %-4s %-28s %s\n' "$1" "$2" "$3"; [ "$1" = FAIL ] && fail=1; return 0; }

en=$(systemctl is-enabled logloader 2>/dev/null); en=${en:-not installed}
case "$en" in
    disabled|not\ installed|masked) line PASS "ARK logloader" "$en (it looped for a day on 2026-09-12 and filled the disk)";;
    *) line FAIL "ARK logloader" "$en -- sudo systemctl disable --now logloader (docs/fact-ark-logloader-loop.md)";;
esac

free_gb=$(df -BG --output=avail / | tail -1 | tr -dc '0-9')
if [ "${free_gb:-0}" -ge 20 ]; then line PASS "root disk" "${free_gb} GB free (need >= 20; Stage A records ~12 MB/s)"
else line FAIL "root disk" "${free_gb} GB free -- need >= 20 GB"; fi

cur=$(cat /sys/class/devfreq/17000000.gpu/cur_freq 2>/dev/null)
max=$(cat /sys/class/devfreq/17000000.gpu/max_freq 2>/dev/null)
if [ -n "$cur" ] && [ "$cur" = "$max" ]; then line PASS "jetson_clocks" "GPU pinned at $((cur/1000000)) MHz"
elif [ -z "$cur" ]; then line WARN "jetson_clocks" "cannot read the GPU devfreq node"; fail=1
else line FAIL "jetson_clocks" "GPU at $((cur/1000000)) of $((max/1000000)) MHz -- run: sudo jetson_clocks"; fi

swap_mb=$(awk '/SwapTotal/{t=$2} /SwapFree/{f=$2} END{print int((t-f)/1024)}' /proc/meminfo)
if [ "$swap_mb" -le 64 ]; then line PASS "swap" "${swap_mb} MB used"
else line WARN "swap" "${swap_mb} MB used before the stack is even up -- a reboot clears it (then sudo jetson_clocks); watch it stays flat with the stack running"; fi

if ip -brief link show eno1 2>/dev/null | grep -q " UP " && ping -c 1 -W 1 192.168.1.154 >/dev/null 2>&1; then
    line PASS "MID360 link" "eno1 up, 192.168.1.154 answers"
else line FAIL "MID360 link" "eno1 down or the LiDAR does not answer -- scripts/network_setup.sh, LiDAR power"; fi

for s in dds-agent mavlink-router; do
    st=$(systemctl is-active "$s" 2>/dev/null)
    [ "$st" = active ] && line PASS "$s" "active" || line FAIL "$s" "$st -- sudo systemctl restart $s (after an FC power cycle, mavlink-router does not reconnect by itself)"
done

echo
if [ "$fail" = 0 ]; then echo "DESK CHECK: PASS"; else echo "DESK CHECK: FAIL -- fix the lines above"; fi
exit $fail
