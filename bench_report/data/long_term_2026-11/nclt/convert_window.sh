# convert.sh DAY FROM_UTIME TO_UTIME NAME : stream one window of an NCLT session into bags/NAME
set -u
cd /home/user/bench/nclt
B=https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu
d=$1; name=$4
[ -f bags/$name/metadata.yaml ] && { echo "$name exists"; exit 0; }
rm -rf bags/$name; mkdir -p sen/$d bags
[ -s sen/$d/ms25.csv ] || curl -s $B/sensor_data/${d}_sen.tar.gz | tar -xzO --wildcards "*/ms25.csv" > sen/$d/ms25.csv
start=$(date +%s)
python3 -I /home/user/lidar_slam/scripts/convert_nclt_session.py $B/velodyne_data/${d}_vel.tar.gz sen/$d/ms25.csv bags/$name --point-step 6 --from-utime $2 --to-utime $3 > logs/convert_$name.txt 2>&1
echo "$name exit $? $(( $(date +%s) - start )) s $(du -sh bags/$name | cut -f1) $(tail -1 logs/convert_$name.txt)"
