#!/bin/bash
# Download CON-1 / CON-2 (GrandTour dev missions) LiDAR, IMU and prism reference.
set -u
HF=https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset/resolve/main
for pair in 2024-12-09-09-34-43:con-1 2024-12-09-09-41-46:con-2; do
  folder=${pair%%:*}; short=${pair##*:}; D=/home/user/bench/refine171/data/$short
  mkdir -p $D/z $D/ref
  for f in stim320_imu hesai_points; do
    [ -f $D/z/$f/.complete ] && continue
    for k in 1 2 3 4; do
      rm -rf $D/z.part; mkdir -p $D/z.part
      if curl -sS -L --fail "$HF/$folder/data/$f.tar" | tar xf - -C $D/z.part; then
        rm -rf $D/z/$f; mv $D/z.part/$f $D/z/$f; touch $D/z/$f/.complete; break
      fi
      echo "$short $f attempt $k failed"; sleep $((60*k))
    done
  done
  [ -d $D/ref/prism_position ] || (curl -sS -L --fail "$HF/$folder/data/prism_position.tar" | tar xf - -C $D/ref)
  echo "$short fetched"
done
echo FETCH DONE
