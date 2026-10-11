"""Pixel -> ground position for a downward-looking camera (flat-ground assumption)."""
import math
import numpy as np


def rot_ned_from_body(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp,     sr * cp,                cr * cp]])


# Nadir camera, image top = drone nose: image-right = body-right,
# image-down = body-backward, optical axis = body-down.
R_BODY_FROM_CAM = np.array([[0.0, -1.0, 0.0],
                            [1.0,  0.0, 0.0],
                            [0.0,  0.0, 1.0]])


def intrinsics_from_hfov(width_px, height_px, hfov_deg):
    fx = (width_px / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return (fx, fx, width_px / 2.0, height_px / 2.0)


def camera_attitude(meta, cam_cfg):
    """Attitude to use for projection.
    'stabilised_nadir': gimbal keeps the camera pointing straight down, so only yaw matters.
    'fixed_body': camera rigidly mounted on the drone, use the drone's roll/pitch too."""
    yaw = meta["yaw"] + math.radians(cam_cfg.get("image_top_yaw_offset_deg", 0.0))
    if cam_cfg.get("gimbal", "stabilised_nadir") == "stabilised_nadir":
        return (0.0, 0.0, yaw)
    return (meta["roll"], meta["pitch"], yaw)


def pixel_to_ned_offset(u, v, K, attitude, agl):
    """Return (north_m, east_m) of the ground point seen at pixel (u, v), relative to the drone."""
    fx, fy, cx, cy = K
    ray_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    ray_ned = rot_ned_from_body(*attitude) @ R_BODY_FROM_CAM @ ray_cam
    if ray_ned[2] <= 1e-6:
        return None
    s = agl / ray_ned[2]
    return float(s * ray_ned[0]), float(s * ray_ned[1])


def offset_to_latlon(lat, lon, north_m, east_m):
    dlat = north_m / 111_320.0
    dlon = east_m / (111_320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon
