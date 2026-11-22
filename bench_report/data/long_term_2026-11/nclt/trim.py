"""Copy a converted NCLT bag from a given time (s after its first message) on."""
import sys
from rosbags.rosbag2 import CompressionFormat, CompressionMode, Reader, Writer
from rosbags.typesys import Stores, get_typestore
TS = get_typestore(Stores.ROS2_HUMBLE)
src, dst, skip = sys.argv[1], sys.argv[2], float(sys.argv[3])
with Reader(src) as reader:
    start = reader.start_time + int(skip * 1e9)
    writer = Writer(dst, version=8); writer.set_compression(CompressionMode.MESSAGE, CompressionFormat.ZSTD); writer.open()
    out = {c.topic: writer.add_connection(c.topic, c.msgtype, typestore=TS) for c in reader.connections}
    n = 0
    for c, t, raw in reader.messages():
        if t >= start:
            writer.write(out[c.topic], t, raw); n += 1
    writer.close(); print('messages', n, 'start', start / 1e9)
