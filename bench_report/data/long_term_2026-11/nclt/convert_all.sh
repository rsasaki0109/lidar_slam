set -u
cd /home/user/bench/nclt
B=https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu
for d in 2012-01-15 2012-05-26 2012-05-11 2012-06-15 2012-02-18 2012-11-16 2012-11-04 2012-11-17; do
  [ -f bags/$d/metadata.yaml ] && { echo "$d exists"; continue; }
  rm -rf bags/$d
  mkdir -p sen/$d
  curl -s $B/sensor_data/${d}_sen.tar.gz | tar -xzO --wildcards "*/ms25.csv" > sen/$d/ms25.csv
  w=$(python3 -c "import json;w=json.load(open('windows.json'))['windows_utime']['$d'];print(w[0],w[1])")
  set -- $w
  start=$(date +%s)
  python3 -I /home/user/lidar_slam_ros2/scripts/convert_nclt_session.py $B/velodyne_data/${d}_vel.tar.gz sen/$d/ms25.csv bags/$d --point-step 6 --from-utime $1 --to-utime $2 > logs_$d.txt 2>&1
  echo "$d exit $? $(( $(date +%s) - start )) s $(du -sh bags/$d | cut -f1) $(tail -1 logs_$d.txt)"
done
