# ARK OS `logloader` re-uploaded one log ~21,000 times — cause, fix, and what was kept

**2026-09-13. Fixed: service disabled, duplicates deleted, disk back to 364 GB free.**

## What it did

ARK OS's `logloader.service` downloads ULogs from the FC over MAVLink and uploads them to the local
PX4 Flight Review (`flight-review.service`, port 5006).

**The loop.**
- From **2026-09-12 19:03 EDT** it re-uploaded **one** log, `LOG0135_2026-09-12T23:03:08Z.ulg`,
  every ~2 s.
- It resumed after every reboot, including 2026-09-13.

**What it cost:**

| | |
|---|---|
| Flight Review entries for that one log | **21,022** (20,945 × 17,416,192 B, plus 77 zero-byte) |
| `/var/lib/ark-os/flight-review` | **344 GB**, growing ~500 MB/min |
| Root disk | **95 % used, 23 GB free** at 16:23 on 2026-09-13 |
| CPU | ~42 % of a core for logloader alone, plus Flight Review parsing each upload |
| Flight stack | a **3.9 s `/lidar_slam/odom` delivery stall** on the bench full graph while it ran (stamps on time, arrival late, at two independent subscribers); none over 126 ms in a 20-min soak with it stopped |

**Every 2026-09-12 Stage A flight ran with it looping.** That makes it the first candidate with a
mechanism for that day's undiagnosed odometry-age problem (`m5-m6-plan.md` §3 G0.2 note). Not
proven.

## Why — a bug in the installed logloader, not the SD card

**The installed binary.**
- `/usr/lib/ark-os/bin/logloader`, built 2026-07-23.
- Config `/etc/ark-os/logloader.toml`: MAVLink `udp://:14551`, local server only, remote upload off.
- It keeps its state in `/var/lib/ark-os/logs/local_server.db` and `remote_server.db`.
- Its source is the legacy version in `~/ARK-OS/services/logloader/logloader` (parent of commit
  `558188e`, `src/ServerInterface.cpp`).

**The bug:**
1. **A log's identity is `hash(date + "_" + size)`** (`generate_uuid`).
2. **It downloaded LOG0135 at 19:03, within a minute of PX4 opening it**, so the staged copy
   `/var/lib/ark-os/logs/logs/LOG0135_2026-09-12T23:03:08Z.ulg` is **truncated: 17,416,192 B**.
3. **The FC later listed the finished log at 18,378,936 B.** That is a new identity, so it became a
   new row (`afd6313c…`) marked downloaded — a file with that name already existed — but not
   uploaded.
4. **`upload_log()` recomputes the identity from the *local file's* size.** So each successful
   upload marked the OLD row (`4b4ccf3c…`, already uploaded), and the new row stayed
   `uploaded = 0`.
5. **The next pass picked the same row again.** Forever.

**Removing the log from the FC's SD card would not have stopped it.** The loop is driven by that
database row and the truncated file on the Jetson.

**The same bug can bite any log fetched while PX4 is still writing it** — for example with
logloader running while the aircraft is armed and logging.

## What was done (2026-09-13, 16:27–16:45)

1. **logloader stopped and disabled.**
   - `sudo systemctl disable logloader` removed it from both `multi-user.target.wants` and
     `ark-os.target.wants`.
2. **The stuck row marked uploaded.**
   - `afd6313c…` (the only `uploaded = 0` row) set to 1 in `local_server.db`, so re-enabling
     cannot resume this loop.
   - Both databases were backed up first to `~/backups/logloader-db-20260913/`.
3. **Flight Review cleaned while stopped.** `logs.sqlite` backed up to
   `~/backups/flight-review-db-20260913/`.
   - **Verified first:** all 20,945 full-size copies sha256-identical to the staged file
     (`72094af0…`).
   - **Deleted 21,021 entries with Flight Review's own delete steps** (`edit_entry.py
     delete_log_entry`): the `.ulg`, `cache/img/<id>.png`, `cache/kml/<id>.kml`, and the
     `LogsGenerated` and `Logs` rows.
   - **Kept the first upload:** Id `4606cfb5-d98e-4bef-a763-f6d53ba5eb01`, 2026-09-12 19:03:56.
   - `Vehicle.LatestLogId` re-pointed from a deleted duplicate to LOG0146 (`61777cc5…`), the newest
     real log. `VACUUM`, integrity OK.
4. **Restarted and checked.** `/browse` 200, 176 logs, the kept LOG0135 opens.
   **364.8 GB freed. Root disk 18 % used, 364 GB free.**

## Kept on purpose

- **The FC's copy of LOG0135: `/fs/microsd/log/2026-09-12/23_02_29.ulg`, 18,378,936 B.**
  - It is the **only complete copy**; the Jetson's is truncated.
  - It never drove the loop.
  - Delete it later with `python3 scripts/px4/px4_shell.py "rm /fs/microsd/log/2026-09-12/23_02_29.ulg"`
    if it isn't wanted.
- **LOG0081's two Flight Review entries** (09-07). Their contents differ, so they are two
  downloads, not duplicates.
- **`Vehicle.FlightTime`** was not recomputed.

## Checking, undoing, re-enabling

```bash
systemctl is-enabled logloader     # expect: disabled -- an ARK OS update or the ARK UI could turn it back on
df -h /                            # the flight stack and bags share this disk
journalctl -u logloader --since "2 min ago" | grep -c Uploading   # ~0 if it is ever running
```

**Re-enable:** `sudo systemctl enable --now logloader`. It will not loop on LOG0135 again. Watch
for the pattern above if a log is fetched mid-write.

**Undo the database edits:** stop both services, then copy the backups back into
`/var/lib/ark-os/logs/` and `/var/lib/ark-os/flight-review/data/`, owned by `jetson`. The deleted
duplicate files are not recoverable, and did not need to be: they were byte-identical to the kept
entry and to the staged file.
