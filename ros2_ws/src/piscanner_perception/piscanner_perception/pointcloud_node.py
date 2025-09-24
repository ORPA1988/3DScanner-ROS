"""ROS2-Knoten zur Fusion von LiDAR-, Motor- und Kameradaten."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32, Header
from std_srvs.srv import Trigger

from .color_maps import intensity_to_rgb
from .geometry import compute_lidar_points, rotate_to_world, vector_to_pano_uv


@dataclass
class ScanData:
    """Zwischenspeicher für die zuletzt generierte Punktwolke."""

    points: np.ndarray
    colors_rgb: np.ndarray
    colors_bgr: np.ndarray
    intensities: np.ndarray
    lidar_image: np.ndarray


class PointCloudBuilderNode(Node):
    """Kombiniert die Sensordaten zu einer farbigen Punktwolke."""

    def __init__(self) -> None:
        super().__init__("pointcloud_builder_node")

        self.declare_parameter("lidar_offset_y", 0.05)
        self.declare_parameter("lidar_mount_height", 0.25)
        self.declare_parameter("camera_offset", [0.0, -0.05, 0.2])
        self.declare_parameter("camera_fov_deg", 180.0)
        self.declare_parameter("save_directory", str(Path.home() / "piscanner_data"))
        self.declare_parameter("lidar_image_size", 512)

        self._bridge = CvBridge()
        self._latest_motor_angle = 0.0
        self._latest_camera_image: Optional[np.ndarray] = None
        self._latest_scan: Optional[ScanData] = None

        self._pointcloud_publisher = self.create_publisher(PointCloud2, "/piscanner_perception/pointcloud", 10)
        self._image_publisher = self.create_publisher(Image, "/piscanner_perception/lidar_image", 10)

        self.create_subscription(LaserScan, "/stl27l/scan", self._on_scan, 10)
        self.create_subscription(Float32, "/piscanner_control/motor_angle", self._on_motor_angle, 10)
        self.create_subscription(Image, "/piscanner_camera/panorama", self._on_camera_image, 10)

        self._save_service = self.create_service(Trigger, "/piscanner_perception/save_snapshot", self._handle_save_request)

        save_directory = Path(self.get_parameter("save_directory").value)
        save_directory.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Speichere Daten unter: {save_directory}")

    def _on_motor_angle(self, msg: Float32) -> None:
        self._latest_motor_angle = float(msg.data)

    def _on_camera_image(self, msg: Image) -> None:
        self._latest_camera_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = np.array(msg.ranges, dtype=np.float32)
        if np.any(~np.isfinite(ranges)):
            ranges = np.nan_to_num(ranges, nan=msg.range_max)

        intensities = np.array(msg.intensities if msg.intensities else np.ones_like(ranges), dtype=np.float32)
        intensities = np.clip(intensities, 0.0, 1.0)

        x_local, z_local = compute_lidar_points(ranges, msg.angle_min, msg.angle_increment)
        points_world = rotate_to_world(
            x_local,
            z_local,
            self._latest_motor_angle,
            float(self.get_parameter("lidar_offset_y").value),
            float(self.get_parameter("lidar_mount_height").value),
        )

        colors_rgb = intensity_to_rgb(intensities)
        colors_bgr = colors_rgb[:, ::-1]

        if self._latest_camera_image is not None:
            camera_offset = np.array(self.get_parameter("camera_offset").value, dtype=np.float32)
            vectors = points_world - camera_offset
            u, v = vector_to_pano_uv(
                vectors,
                float(self.get_parameter("camera_fov_deg").value),
                self._latest_camera_image.shape[1],
                self._latest_camera_image.shape[0],
            )
            sampled = self._latest_camera_image[v, u]
            colors_bgr = sampled
            colors_rgb = sampled[:, ::-1]

        lidar_image = self._create_lidar_image(points_world, colors_bgr, msg.range_max)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id or "piscanner_frame"

        cloud_msg = self._create_pointcloud_msg(points_world, colors_rgb, intensities, header)
        self._pointcloud_publisher.publish(cloud_msg)

        image_msg = self._bridge.cv2_to_imgmsg(lidar_image, encoding="bgr8")
        image_msg.header = header
        self._image_publisher.publish(image_msg)

        self._latest_scan = ScanData(points_world, colors_rgb, colors_bgr, intensities, lidar_image)

    def _create_pointcloud_msg(
        self,
        points: np.ndarray,
        colors_rgb: np.ndarray,
        intensities: np.ndarray,
        header: Header,
    ) -> PointCloud2:
        rgb_int = (colors_rgb[:, 0].astype(np.uint32) << 16) | (colors_rgb[:, 1].astype(np.uint32) << 8) | colors_rgb[:, 2].astype(np.uint32)
        cloud_points = [
            (float(p[0]), float(p[1]), float(p[2]), float(i), int(rgb))
            for p, i, rgb in zip(points, intensities, rgb_int)
        ]
        fields = [
            PointField(name=name, offset=offset, datatype=datatype, count=1)
            for name, offset, datatype in [
                ("x", 0, PointField.FLOAT32),
                ("y", 4, PointField.FLOAT32),
                ("z", 8, PointField.FLOAT32),
                ("intensity", 12, PointField.FLOAT32),
                ("rgb", 16, PointField.UINT32),
            ]
        ]
        return point_cloud2.create_cloud(header, fields, cloud_points)

    def _create_lidar_image(self, points: np.ndarray, colors_bgr: np.ndarray, range_max: float) -> np.ndarray:
        size = int(self.get_parameter("lidar_image_size").value)
        image = np.zeros((size, size, 3), dtype=np.uint8)
        center = size // 2
        scale = (size / 2 - 1) / max(range_max, 1e-3)
        u = np.clip((points[:, 0] * scale + center).astype(int), 0, size - 1)
        v = np.clip((points[:, 2] * scale + center).astype(int), 0, size - 1)
        image[v, u] = colors_bgr
        return image

    def _handle_save_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._latest_scan is None:
            response.success = False
            response.message = "Noch keine Daten zum Speichern vorhanden"
            return response

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        directory = Path(self.get_parameter("save_directory").value)
        pcd_path = directory / f"scan_{timestamp}.pcd"
        image_path = directory / f"scan_{timestamp}.png"

        self._write_pcd(pcd_path, self._latest_scan.points, self._latest_scan.colors_rgb, self._latest_scan.intensities)

        from PIL import Image as PilImage

        PilImage.fromarray(self._latest_scan.lidar_image).save(image_path)

        response.success = True
        response.message = f"Gespeichert: {pcd_path.name}, {image_path.name}"
        return response

    def _write_pcd(self, path: Path, points: np.ndarray, colors_rgb: np.ndarray, intensities: np.ndarray) -> None:
        with open(path, "w", encoding="utf-8") as f:
            f.write("# .PCD v0.7 - Point Cloud Data\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z intensity rgb\n")
            f.write("SIZE 4 4 4 4 4\n")
            f.write("TYPE F F F F U\n")
            f.write("COUNT 1 1 1 1 1\n")
            f.write(f"WIDTH {points.shape[0]}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {points.shape[0]}\n")
            f.write("DATA ascii\n")
            for p, intensity, color in zip(points, intensities, colors_rgb):
                rgb = int(color[0]) << 16 | int(color[1]) << 8 | int(color[2])
                f.write(f"{p[0]:.5f} {p[1]:.5f} {p[2]:.5f} {intensity:.5f} {rgb}\n")


def main() -> None:
    rclpy.init()
    node = PointCloudBuilderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Pointcloud-Node beendet (CTRL+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
