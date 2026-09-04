# RT tuning for the Franka control PC

Three layers, only the first is permanent by itself.

## 1. Kernel cmdline — permanent, `/etc/default/grub`

```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3 irqaffinity=0,1,4-23 intel_idle.max_cstate=1 processor.max_cstate=1"
```

Then `sudo update-grub && sudo reboot`. Verify with `cat /sys/devices/system/cpu/isolated` → `2-3`.

Cores 2 and 3 are P-cores measured clean (10-28 µs with `cyclictest`). CPU4 carries
the `iwlwifi` and `nvme0q2` IRQs and was reproducibly hitting **6230 µs**, which is
six missed FCI deadlines in a row → `communication_constraints_violation`.

## 2. This systemd unit — resets on every boot without it

```bash
sudo install -m 755 franka-rt-tuning.sh /usr/local/sbin/franka-rt-tuning.sh
sudo install -m 644 franka-rt-tuning.service /etc/systemd/system/franka-rt-tuning.service
sudo systemctl daemon-reload
sudo systemctl enable --now franka-rt-tuning.service
systemctl status franka-rt-tuning.service
```

Overrides, if the hardware changes:

```bash
sudo systemctl edit franka-rt-tuning.service
# [Service]
# Environment=FRANKA_NIC=enp1s0
# Environment=FRANKA_IRQ_CPU=2
```

## 3. Pinning the control loop — done by the launch file

`torque_control_stack.launch.py` runs `scripts/pin_rt_thread.sh` a second after the
controller spawner, moving **only** the `SCHED_FIFO` thread of `ros2_control_node`
onto CPU3. Disable with `rt_pin_cpu:=''`, change core with `rt_pin_cpu:=2`.

`isolcpus` only *empties* a core — nothing runs there until something is pinned to
it. Without step 3 the RT thread was measured on CPU5, free to migrate onto CPU4.

## Checks

```bash
cat /proc/cmdline                                    # isolcpus present
grep enp129s0 /proc/interrupts                       # counters on CPU2
cat /sys/devices/system/cpu/cpu2/cpufreq/scaling_governor   # performance
cyclictest -m -p 90 -i 200 -d 0 -D 45 -a 3 -t 1 -q   # Max < 50 us (was 6230)
ps -L -o tid,psr,cls,rtprio -p $(pgrep -f controller_manager/ros2_control_node) | grep FF
```
