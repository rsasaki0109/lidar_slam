# KITTI splat drive (RobotNativeEngine example)

An Ackermann car drives through a 3D Gaussian Splatting scene of KITTI raw drive 0018 in
[RobotNativeEngine](https://github.com/rsasaki0109/RobotNativeEngine) (RNE). The splat is
trained with `visloc-rs` `gsplat_train` on a COLMAP scene whose camera poses come from
lidar_slam (`scripts/export_kitti_colmap.py`), so it is metric and gravity-aligned.

This directory is a copy of the RNE example crate `examples/160_kitti_splat_drive` (it builds
only inside an RNE checkout):

1. Copy `Cargo.toml` and `main.rs` to `<rne>/examples/160_kitti_splat_drive/` and add that
   path to the RNE workspace `members`.
2. `cargo build --release -p kitti_splat_drive --example 160_kitti_splat_drive`
3. Run with `--manifest kitti0018.rne.splat.toml --path <path.csv> --out <dir>`; on a
   machine without a GPU, set `VK_ICD_FILENAMES` to Mesa lavapipe.

`kitti0018.rne.splat.toml` rotates the z-up SLAM frame -90 deg about X into RNE's y-up
frame. RNE draws the splat into an sRGB target without undoing the splat's sRGB colours, so
images come out washed out; until that is fixed in RNE, convert the PLY's colours to linear
first (the `f_dc` term exactly, the `f_rest` terms to first order).
