"""Measured terrain contracts; no ROS or simulator required."""
import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'ros1/exploration/scripts'
sys.path.insert(0, str(SCRIPTS))


def mapping():
    # A missing implementation is an explicit failing behavior during RED.
    assert (SCRIPTS / 'mapping_core.py').is_file(), 'native terrain implementation missing'
    import mapping_core
    return mapping_core


def floor_patch(x0=-.55, x1=.55, y0=-.35, y1=.35, z=0.):
    return np.array([(x, y, z) for x in np.arange(x0, x1+.001, .05)
                     for y in np.arange(y0, y1+.001, .05)])


def test_a_supported_plane_does_not_fill_an_unseen_footprint_hole():
    core = mapping()
    terrain = core.ObservedTerrain()
    points = floor_patch()
    points = points[~((points[:, 0] >= .099) & (points[:, 0] < .199)
                     & (points[:, 1] >= .099) & (points[:, 1] < .199))]
    terrain.update('camera', 10., points, (0., 0., .43, 0.))
    assert terrain.floor_z == pytest.approx(0., abs=.002)
    assert terrain.footprint_known((0., 0., .43, 0.))[0] is False
    values, center = terrain.grid()
    assert np.count_nonzero(np.isfinite(values)) < points.shape[0]
    assert terrain.cell_elevation(.15, .15) != terrain.cell_elevation(.15, .15)


def test_both_sources_keep_equal_and_interleaved_original_stamps():
    terrain = mapping().ObservedTerrain()
    points = floor_patch()
    assert terrain.update('lidar', 10., points, (0., 0., .43, 0.))
    assert terrain.update('camera', 10., points, (0., 0., .43, 0.))
    assert terrain.update('lidar', 10.2, points, (0., 0., .43, 0.))
    assert terrain.update('camera', 10.1, points, (0., 0., .43, 0.))
    assert not terrain.update('camera', 10., points, (0., 0., .43, 0.))
    status = terrain.status(10.3, (0., 0., .43, 0.))
    assert status['sources'] == {'lidar': {'stamp': 10.2, 'count': 2},
                                 'camera': {'stamp': 10.1, 'count': 2}}
    assert {'stamp', 'frame_id', 'floor_z', 'cells', 'footprint_known', 'reason', 'sources'} <= status.keys()
    assert status['frame_id'] == 'lio_map'


def test_invalid_payload_cannot_update_source_liveness_or_create_ground():
    terrain = mapping().ObservedTerrain()
    for points in ([[1, 2]], [[0, 0, math.nan]], [[math.inf, 0, 0]], []):
        assert not terrain.update('camera', 1., points, (0., 0., .43, 0.))
    assert not terrain.update('camera', math.nan, floor_patch(), (0., 0., .43, 0.))
    assert not terrain.update('camera', 1., floor_patch(), (math.nan, 0., .43, 0.))
    assert terrain.status(2., (0., 0., .43, 0.))['sources']['camera']['count'] == 0
    assert terrain.floor_z is None


@pytest.mark.parametrize('height', [.08, .45, -.12])
def test_low_obstacles_body_obstacles_and_holes_override_history(height):
    terrain = mapping().ObservedTerrain()
    terrain.update('camera', 1., floor_patch(), (0., 0., .43, 0.))
    assert terrain.footprint_known((0., 0., .43, 0.))[0]
    terrain.update('lidar', 1., [[.15, .15, height]], (0., 0., .43, 0.))
    assert not terrain.footprint_known((0., 0., .43, 0.))[0]
    assert math.isnan(terrain.cell_elevation(.15, .15))


def test_real_hits_persist_after_translation_without_fresh_support_claim():
    terrain = mapping().ObservedTerrain()
    # Camera observes only the strip ahead. Base later moves onto that strip.
    terrain.update('camera', 1., floor_patch(1., 2.2), (0., 0., .43, 0.))
    assert not terrain.footprint_known((0., 0., .43, 0.))[0]
    assert terrain.footprint_known((1.6, 0., .43, 0.))[0]
    terrain.update('camera', 2., floor_patch(2.6, 3.6), (1.6, 0., .43, 0.))
    assert terrain.footprint_known((1.6, 0., .43, 0.))[0]
    assert terrain.status(20., (1.6, 0., .43, 0.))['sources']['camera']['stamp'] == 2.


def test_footprint_covers_all_touched_cells_and_rotation_not_only_center():
    terrain = mapping().ObservedTerrain()
    terrain.update('camera', 1., floor_patch(-.45, .45, -.25, .25), (0., 0., .43, 0.))
    assert terrain.footprint_known((0., 0., .43, 0.))[0]
    # Rotating the .75 x .45 footprint reaches beyond y=.25.
    assert not terrain.footprint_known((0., 0., .43, math.pi/4))[0]
    terrain.update('camera', 2., floor_patch(-.7, .7, -.7, .7), (0., 0., .43, 0.))
    assert terrain.footprint_known((0., 0., .43, math.pi/4))[0]


def test_interpolated_path_rejects_unknown_between_known_endpoints():
    terrain = mapping().ObservedTerrain()
    points = np.concatenate((floor_patch(-.6, .6), floor_patch(1.4, 2.6)))
    terrain.update('camera', 1., points, (0., 0., .43, 0.))
    assert terrain.footprint_known((0., 0., .43, 0.))[0]
    assert terrain.footprint_known((2., 0., .43, 0.))[0]
    valid, reason, index = terrain.validate_path([(0., 0., .43, 0.), (2., 0., .43, 0.)])
    assert not valid and 'unknown' in reason and index == 1
    assert not terrain.validate_path([])[0]
    assert not terrain.validate_path([(0., math.nan, .43, 0.)])[0]


def test_native_filter_preserves_ray_frame_and_removes_body_and_range():
    core = mapping()
    transform = np.eye(4)
    transform[0, 3] = .2
    points = np.array([[1., .2, .1], [1.01, .2, .1], [0., 0., 0.],
                       [100., 0., 0.], [np.nan, 1., 1.], [.1, 0., 0.]])
    out = core.filter_native_points(points, transform, .15, 20., .05)
    np.testing.assert_allclose(out, [[1., .2, .1]])
    np.testing.assert_allclose(core.transform_points(out, transform), [[1.2, .2, .1]])


def test_bounded_transform_queue_does_not_block_other_source_or_use_latest_tf():
    core = mapping()
    queue = core.TransformQueue(max_per_source=2, max_age=1.)
    queue.enqueue('lidar', 10., 'l10', 0.)
    queue.enqueue('camera', 10., 'c10', 0.)
    seen = []
    def lookup(source, stamp):
        seen.append((source, stamp))
        return None if source == 'lidar' else 'camera_tf_at_10'
    assert queue.ready(lookup, .1) == [('camera', 10., 'c10', 'camera_tf_at_10')]
    assert ('camera', 10.) in seen
    queue.enqueue('lidar', 11., 'l11', .1)
    queue.enqueue('lidar', 12., 'l12', .2)
    assert queue.pending('lidar') == 2
    ready = queue.ready(lambda source, stamp: stamp, .3)
    assert [item[1] for item in ready] == [11., 12.]
    queue.enqueue('camera', 20., 'expired', 0.)
    assert queue.ready(lambda *_: 'tf', 2.) == []


def test_imu_pose_and_twist_are_converted_at_the_base_origin():
    core = mapping()
    mount = np.eye(4)
    mount[:3, 3] = [.261, .02329, .10088]
    imu = np.array([[0., -1., 0., 1.], [1., 0., 0., 2.],
                    [0., 0., 1., 3.], [0., 0., 0., 1.]])
    base = core.imu_to_base_pose(imu, mount)
    np.testing.assert_allclose(base[:3, 3], [1.02329, 1.739, 2.89912])
    linear, angular = core.imu_to_base_twist([1., 0., 0.], [0., 0., 2.], mount)
    np.testing.assert_allclose(linear, [1.04658, -.522, 0.])
    np.testing.assert_allclose(angular, [0., 0., 2.])


def test_grid_codec_has_official_column_major_layout_and_preserves_nan():
    assert (SCRIPTS / 'grid_map_codec.py').is_file(), 'GridMap encoder missing'
    from grid_map_codec import encode_layer, decode_layer, grid_position
    values = np.array([[10., 11., 12.], [20., 21., np.nan]], dtype=np.float32)
    data, dimensions = encode_layer(values)
    assert dimensions == [('column_index', 3, 6), ('row_index', 2, 2)]
    np.testing.assert_allclose(data, [10., 20., 11., 21., 12., np.nan])
    np.testing.assert_allclose(decode_layer(data, dimensions), values)
    np.testing.assert_allclose(grid_position((0, 0), values.shape, .5, (1., -2.)), [1.25, -1.5])
    np.testing.assert_allclose(grid_position((1, 2), values.shape, .5, (1., -2.)), [.75, -2.5])


def test_organized_cloud_unpacking_respects_row_padding_and_rejects_bad_buffers():
    core = mapping()
    # Each row has two float XYZ points and four bytes of row padding.
    import struct
    payload = struct.pack('<ffffff4sffffff4s', 1., 2., 3., 4., 5., 6., b'pad!',
                          7., 8., 9., 10., 11., 12., b'pad!')
    fields = [('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1)]
    points = core.unpack_xyz(payload, 2, 2, 12, 28, fields)
    np.testing.assert_allclose(points, [[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]])
    for wrong in [payload[:16], b'']:
        with pytest.raises(ValueError):
            core.unpack_xyz(wrong, 2, 2, 12, 28, fields)
    with pytest.raises(ValueError):
        core.unpack_xyz(payload, 2, 2, 12, 28, fields[:2])


def test_initialization_uses_stance_prior_instead_of_a_dominant_low_obstacle_top():
    terrain = mapping().ObservedTerrain()
    ground = floor_patch(-.5, .5, -.3, .3)
    obstacle = np.repeat(floor_patch(1., 2., -.5, .5, .08), 4, axis=0)
    terrain.update('camera', 1., np.concatenate((ground, obstacle)), (0., 0., .43, 0.))
    assert terrain.floor_z == pytest.approx(0., abs=.002)
    assert terrain.footprint_known((0., 0., .43, 0.))[0]
    assert math.isnan(terrain.cell_elevation(1.5, 0.))
    uninitialized = mapping().ObservedTerrain()
    uninitialized.update('camera', 1., obstacle, (0., 0., .43, 0.))
    assert uninitialized.floor_z is None


def test_grid_world_positions_match_actual_return_cells_and_keep_far_cells_nan():
    terrain = mapping().ObservedTerrain(length=2.)
    terrain.update('camera', 1., floor_patch(-.4, .4, -.3, .3), (0., 0., .43, 0.))
    values, center = terrain.grid()
    assert center == (0., 0.)
    assert values.shape == (20, 20)
    # World (.15,.15) is official matrix index (8,8), not (11,11).
    assert values[8, 8] == 0.
    assert math.isnan(values[1, 1])
    points = terrain.ground_points()
    assert np.all(points[:, :2] >= [-.451, -.351])
    assert np.all(points[:, :2] <= [.451, .351])
    assert not terrain.footprint_known((10., 10., .43, 0.))[0]


def test_obstacle_seen_before_floor_initialization_survives_later_ground_hits():
    terrain = mapping().ObservedTerrain()
    terrain.update('lidar', 1., [[.15, .15, .08]], (0., 0., .43, 0.))
    terrain.update('camera', 1.1, floor_patch(), (0., 0., .43, 0.))
    assert math.isnan(terrain.cell_elevation(.15, .15))
    assert not terrain.validate_path([(0., 0., .43, 0.)])[0]
    assert math.isnan(terrain.cell_elevation(2., 2.))


def test_preinitialization_buffer_overflow_cannot_forget_unclassified_hits():
    terrain = mapping().ObservedTerrain(preinitialization_limit=2)
    terrain.update('lidar', 1., [[.15, .15, .08], [.25, .15, .08], [.35, .15, .08]],
                   (0., 0., .43, 0.))
    terrain.update('camera', 1.1, floor_patch(), (0., 0., .43, 0.))
    for x in (.15, .25, .35):
        assert math.isnan(terrain.cell_elevation(x, .15))


def test_gyro_buffer_uses_nearest_acquisition_stamp_and_rejects_wrong_or_stale_input():
    samples = mapping().GyroBuffer('mid360_imu_link', max_age=.025, capacity=2)
    assert not samples.add(1., 'base', [0., 0., .3], np.eye(3))
    assert samples.nearest(1.) is None
    assert samples.add(1., 'mid360_imu_link', [0., 0., .3], np.eye(3)*.01)
    assert samples.nearest(1.03) is None
    assert samples.add(1.02, 'mid360_imu_link', [0., 0., .4], np.eye(3)*.02)
    angular, covariance = samples.nearest(1.019)
    np.testing.assert_allclose(angular, [0., 0., .4])
    np.testing.assert_allclose(covariance, np.eye(3)*.02)
    assert samples.add(2., 'mid360_imu_link', [0., 0., .5], np.eye(3))
    assert samples.add(2.1, 'mid360_imu_link', [0., 0., .5], np.eye(3))
    assert samples.nearest(1.) is None
    assert not samples.add(2.01, 'mid360_imu_link', [0., 0., math.nan], np.eye(3))
