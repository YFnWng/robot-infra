# Exact WSL ↔ Windows-host clock sync (for offline data fusion)

Goal: make the WSL clock track the **Windows host clock to microseconds** so the
ZED recorder (Windows, epoch-ns timestamps) and the ROS rosbag / encoder / EM
streams (WSL, epoch-ns timestamps) share **one absolute timeline with no offset**.
Offline fusion then aligns purely by timestamp (interpolation) — no per-session
clock handshake, no cross-correlation required (those become fallbacks only).

## Why PTP, not NTP
`systemd-timesyncd` / `ntpdate` sync WSL to *internet* NTP servers, and Windows
syncs to its *own* source independently — the two clocks track "true time" but
drift apart by ms–tens of ms. WSL2 instead exposes the **Hyper-V PTP clock**,
which IS the Windows host system clock:

    /dev/ptp0  (alias /dev/ptp_hyperv),  /sys/class/ptp/ptp0/clock_name == "hyperv"

Disciplining WSL's `CLOCK_REALTIME` to this PHC makes WSL time == host time == the
ZED's timestamp reference. Absolute correctness vs UTC is irrelevant — we only
need both streams on the *same* clock, which they now are.

Prerequisites (all present on this machine): WSL2 kernel (`6.18.*-microsoft`),
`systemd=true` in `/etc/wsl.conf`, and the `hyperv` PHC device above.

## Setup (run once)
```bash
sudo apt update && sudo apt install -y chrony
sudo systemctl disable --now systemd-timesyncd          # network-NTP daemon conflicts
sudo tee /etc/chrony/conf.d/hyperv-phc.conf >/dev/null <<'EOF'
refclock PHC /dev/ptp_hyperv poll 2 dpoll -3 offset 0 stratum 1 prefer
makestep 0.1 3
EOF
sudo sed -i 's/^pool /#pool /; s/^server /#server /' /etc/chrony/chrony.conf   # drop network sources
sudo systemctl enable --now chrony && sudo systemctl restart chrony
```
chrony is `enable`d, so systemd restarts it whenever WSL launches (a few-second
unsynced window at startup until it re-locks).

## Verify
```bash
chronyc sources -v      # PHC0 must be the selected source:  #* PHC0 ...
chronyc tracking        # 'System time' + 'RMS offset' should be microseconds
```
Healthy result looks like:
```
#* PHC0            1   2   17    6   +43us[+38us] +/- 1598ns
System time     : 0.000014201 seconds slow of NTP time   # ~14 us
RMS offset      : 0.000008297 seconds                     # ~8 us
Leap status     : Normal
```
Offsets in **µs** (not ~37 s) confirm the PHC reports UTC cleanly — no leap/TAI
correction needed. One-line health check:
```bash
chronyc tracking | awk -F': *' '/RMS offset|System time/{print $1": "$2}'
```

## Caveats
- **Sleep/resume** is the classic drift trigger — re-check `chronyc sources`
  before a long collection run. chrony re-disciplines automatically (`makestep`
  hard-steps if it ever jumps >100 ms).
- `/dev/ptp0` is root-only; chrony opens it as root at startup. If `chronyc
  sources` shows PHC0 with no samples, add a udev rule:
  `KERNEL=="ptp0", MODE="0660", GROUP="_chrony"`.
- If a *constant* multi-second bias ever appears (leap-second handling), set it
  via the refclock `offset` parameter.
