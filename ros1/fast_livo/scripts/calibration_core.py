"""Validate live simulated camera optics and convert TF matrices to FAST-LIVO2."""
import numpy as np


def camera_parameters(info, image, now, max_age=.5):
    """Accept an unrectified, undistorted pinhole image and its matching info."""
    width, height = int(info['width']), int(info['height'])
    # The configured three-level eight-pixel patch pyramid needs a useful
    # interior; tiny images can also make upstream's 17-row grid divide by zero.
    if width < 64 or height < 64 or (width, height) != (image['width'], image['height']):
        raise ValueError('CameraInfo and image dimensions are invalid or differ')
    if info['frame_id'] != image['frame_id'] or not info['frame_id']:
        raise ValueError('CameraInfo and image optical frames differ')
    stamps = np.asarray([info['stamp'], image['stamp'], now], dtype=float)
    if (not np.isfinite(stamps).all() or min(stamps) <= 0
            or abs(stamps[0] - stamps[1]) > 1.e-7
            or not -.01 <= now - stamps[0] <= max_age):
        raise ValueError('CameraInfo and image must have matching, current simulation stamps')
    if image['encoding'] not in ('rgb8', 'bgr8', 'mono8'):
        raise ValueError('unsupported visual image encoding: ' + image['encoding'])
    if info['distortion_model'] != 'plumb_bob':
        raise ValueError('simulation calibration requires the plumb_bob pinhole model')
    distortion = np.asarray(info['D'], dtype=float)
    if distortion.shape != (5,) or not np.isfinite(distortion).all() or np.any(np.abs(distortion) > 1.e-12):
        raise ValueError('simulation calibration requires five zero distortion coefficients')
    k = np.asarray(info['K'], dtype=float)
    if k.shape != (9,) or not np.isfinite(k).all():
        raise ValueError('camera matrix K must contain nine finite values')
    k = k.reshape(3, 3)
    if (k[0, 0] <= 0 or k[1, 1] <= 0 or not 0 <= k[0, 2] < width
            or not 0 <= k[1, 2] < height
            or not np.allclose([k[0, 1], k[1, 0], *k[2]], [0, 0, 0, 0, 1], atol=1.e-10, rtol=0)):
        raise ValueError('camera K is not a supported calibrated pinhole matrix')
    r, p = np.asarray(info['R'], dtype=float), np.asarray(info['P'], dtype=float)
    expected_p = np.column_stack((k, np.zeros(3)))
    if (r.shape != (9,) or not np.allclose(r.reshape(3, 3), np.eye(3), atol=1.e-10, rtol=0)
            or p.shape != (12,) or not np.allclose(p.reshape(3, 4), expected_p, atol=1.e-7, rtol=0)):
        raise ValueError('rectified or shifted camera projections are unsupported in this simulation path')
    roi = info['roi']
    if (info['binning_x'] not in (0, 1) or info['binning_y'] not in (0, 1)
            or roi['x_offset'] or roi['y_offset']
            or roi['width'] not in (0, width) or roi['height'] not in (0, height)):
        raise ValueError('binned or cropped CameraInfo requires an adjusted camera model')
    return dict(cam_model='Pinhole', cam_width=width, cam_height=height, scale=1.,
                cam_fx=float(k[0, 0]), cam_fy=float(k[1, 1]),
                cam_cx=float(k[0, 2]), cam_cy=float(k[1, 2]),
                cam_d0=0., cam_d1=0., cam_d2=0., cam_d3=0.)


def extrinsic_parameters(imu_from_lidar, camera_from_lidar):
    """TF lookup(target, source) directly supplies T_IL and T_CL upstream uses."""
    matrices = []
    for value in (imu_from_lidar, camera_from_lidar):
        matrix = np.asarray(value, dtype=float)
        if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1.e-9, rtol=0)):
            raise ValueError('sensor TF is not a finite homogeneous transform')
        rotation = matrix[:3, :3]
        if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.e-6, rtol=0)
                or abs(np.linalg.det(rotation) - 1.) > 1.e-6):
            raise ValueError('sensor TF rotation is not a proper rigid rotation')
        matrices.append(matrix)
    imu, camera = matrices
    return dict(extrinsic_T=imu[:3, 3].tolist(), extrinsic_R=imu[:3, :3].reshape(-1).tolist(),
                Pcl=camera[:3, 3].tolist(), Rcl=camera[:3, :3].reshape(-1).tolist())
