#!/usr/bin/env bash
# Boot-time RT tuning for the Franka control PC.  Everything here is lost on
# reboot, which is why it lives in a systemd unit rather than in a README:
#
#   1. the robot NIC's IRQ is moved onto an isolated core, so the FCI packets
#      are serviced away from iwlwifi/nvme (CPU4 was measured at 6.2 ms stalls);
#   2. the CPU governor goes to 'performance' -- 'powersave' lets an idle core
#      sit at a low P-state, and the ramp-up costs more than a 1 ms deadline;
#   3. irqbalance is stopped, otherwise it undoes step 1 within seconds.
#
# The kernel-cmdline half (isolcpus / nohz_full / rcu_nocbs / irqaffinity /
# max_cstate) is permanent and lives in /etc/default/grub -- not here.
set -uo pipefail

NIC="${FRANKA_NIC:-enp129s0}"      # interface facing the robot
IRQ_CPU="${FRANKA_IRQ_CPU:-2}"     # isolated core for the NIC IRQ
rc=0

log() { echo "[franka-rt-tuning] $*"; }

# ── 1. NIC IRQ → isolated core ───────────────────────────────────────────────
# IRQ NUMBERS ARE NOT STABLE ACROSS REBOOTS: this box moved enp129s0 from 171
# to 148 on a single reboot, which silently invalidated a hardcoded fix.
# Always resolve by device name.
mapfile -t irqs < <(awk -v nic="$NIC" '$NF == nic { sub(":", "", $1); print $1 }' /proc/interrupts)

if (( ${#irqs[@]} == 0 )); then
    log "WARNING: no IRQ found for '$NIC' -- is the interface up? Skipping step 1."
    rc=1
else
    for irq in "${irqs[@]}"; do
        if echo "$IRQ_CPU" > "/proc/irq/$irq/smp_affinity_list" 2>/dev/null; then
            log "IRQ $irq ($NIC) -> CPU$(cat /proc/irq/"$irq"/smp_affinity_list)"
        else
            log "WARNING: could not set affinity for IRQ $irq"
            rc=1
        fi
    done
fi

# ── 2. governor ──────────────────────────────────────────────────────────────
if command -v cpupower >/dev/null 2>&1; then
    cpupower frequency-set -g performance >/dev/null 2>&1 \
        && log "governor -> performance" \
        || { log "WARNING: cpupower failed"; rc=1; }
else
    # cpupower is not installed on every image; sysfs does the same job.
    for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        [[ -w $g ]] && echo performance > "$g" 2>/dev/null
    done
    log "governor -> performance (via sysfs, cpupower not installed)"
fi

# ── 3. irqbalance ────────────────────────────────────────────────────────────
if systemctl is-active --quiet irqbalance; then
    systemctl stop irqbalance && log "irqbalance stopped"
else
    log "irqbalance already inactive"
fi

exit $rc
