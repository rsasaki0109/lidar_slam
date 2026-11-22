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
frame. Two RNE patches fix its splat colour pass; apply them with `git am`:

- `0001-*.patch`: the splat was drawn through an sRGB view of the hybrid target, encoding
  its (already sRGB) colours twice and washing them out.
- `0002-*.patch`: wgpu-3dgs-viewer 0.2 takes 2f as the focal length for the 2D covariance
  and an `exp(-q^2)` falloff, so every splat was sqrt(2) too wide; a per-splat size of
  1/sqrt(2) cancels it. With it, visloc-rs localizes 115 of 172 RNE front-camera frames
  (median 5 cm); before it, none.

## Visual-localization closed loop (`161_kitti_splat_visloc_loop`, `visloc_loop.rs`)

The car steers on its estimated pose: dead reckoning from wheel speed and steering (2 %
scale error, 0.003 rad/s yaw-rate bias) between visual fixes, a fix every 0.5 s from the
front camera rendered at the true pose by visloc-rs `gsplat_render_server` and localized by
`basalt_euroc_localize_demo` (`--min-inliers 15`) against a map built from splat renders at
the SLAM camera poses (`render_tum_seq.py`, `tum_rgbd_save_map --keep-world`). Fixes more
than 3 m from the estimate are rejected.

| run (KITTI 0018, first 290 m, 7 m/s) | visual fixes | cross-track mean / max | estimate error mean / max |
|---|---|---|---|
| odometry only | - | 5.55 / 17.17 m | 5.97 / 17.37 m |
| visual closed loop | 65 / 86 | 0.07 / 0.71 m | 0.13 / 0.82 m |
