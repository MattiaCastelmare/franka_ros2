#!/bin/bash
# Host-side: stack the new policy (top) over v10c 2.5M (bottom), same seeds, frame-synced.   stack_v11.sh TAG
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"/runs/videos
for sc in dynamic static; do
  ffmpeg -y -loglevel error -i sac_v11_ft_prec_$1_${sc}_raw.mp4 -i sac_v10c_2.5M_${sc}_raw.mp4 \
    -filter_complex "[0]scale=1440:-2[a];[1]scale=1440:-2[b];[a][b]vstack" -c:v libx264 -crf 28 -preset slow -pix_fmt yuv420p \
    -movflags +faststart v11_vs_v10c_${sc}.mp4
done
ls -la v11_vs_v10c_*.mp4
