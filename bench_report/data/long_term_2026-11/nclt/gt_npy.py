"""groundtruth_DAY.csv -> gt1hz/DAY.npy (utime, x, y, z; one row per second); with --full also gt/DAY.npy (all rows)."""
import sys
import numpy as np
root, full, days = sys.argv[1], sys.argv[2] == 'full', sys.argv[3:]
for d in days:
    a = np.loadtxt(f'{root}/gt_csv/groundtruth_{d}.csv', delimiter=',')
    a = a[np.isfinite(a).all(1)]
    if full:
        np.save(f'{root}/gt/{d}.npy', a)
    s = (a[:, 0] // 1_000_000).astype(np.int64)
    _, first = np.unique(s, return_index=True)
    np.save(f'{root}/gt1hz/{d}.npy', a[first, :4])
    print(d, len(a), len(first))
