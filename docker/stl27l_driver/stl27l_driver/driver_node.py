"""ROS2-Treiberknoten für den STL27L-LiDAR.

Dieses Skript dient als Beispiel, wie ein herstellerspezifischer LiDAR-Treiber in
ROS2 eingebunden werden kann. Da der originale Treiber proprietär ist, generiert dieser
Knoten synthetische Daten, die aber denselben Nachrichtentyp (`sensor_msgs.msg.LaserScan`)
verwenden. So lässt sich das restliche System entwickeln und testen, bevor die echte
Hardware integriert wird.
"""

from __future__ import annotations

import math
import random
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class STL27LDriverNode(Node):
    """Einfacher ROS2-Knoten, der LaserScan-Daten publiziert.

    Der reale STL27L liefert Intensitätswerte und Distanzen in einer vertikalen Ebene.
    Um dieses Verhalten zu imitieren, erzeugt der Knoten eine sinusförmige Testumgebung
    mit verrauschten Messwerten. Sämtliche Parameter sind ausführlich kommentiert, damit
    Anpassungen für reale Sensoren leicht fallen.
    """

    def __init__(self) -> None:
        # Der Konstruktor initialisiert den Node mit einem sprechenden Namen.
        super().__init__("stl27l_driver_node")

        # Parameter erlauben es, die Scanfrequenz oder die Auflösung ohne Codeänderungen
        # anzupassen. Für echte Sensoren sollten die Standardwerte mit den Datenblättern
        # abgeglichen werden.
        self.declare_parameter("scan_frequency", 10.0)
        self.declare_parameter("min_range", 0.05)
        self.declare_parameter("max_range", 12.0)
        self.declare_parameter("angle_min", -math.pi / 2)
        self.declare_parameter("angle_max", math.pi / 2)
        self.declare_parameter("angle_increment", math.radians(0.5))

        # Publisher für den LaserScan. Die Queuegröße 10 ist für einen Demo-Knoten ausreichend.
        self._publisher = self.create_publisher(LaserScan, "/stl27l/scan", 10)

        # Timer sorgt für periodische Veröffentlichung der Daten.
        scan_frequency = self.get_parameter("scan_frequency").value
        self._timer = self.create_timer(1.0 / scan_frequency, self._publish_scan)

        self.get_logger().info("STL27L Demo-Treiber gestartet")

    def _create_scan(self) -> Tuple[np.ndarray, np.ndarray]:
        """Erzeugt synthetische Reichweiten- und Intensitätsdaten.

        Die Funktion liefert zwei NumPy-Arrays: eines für die Distanzen und eines für die
        Intensitäten. Durch die separate Funktion ist der Code leichter testbar.
        """

        angle_min = self.get_parameter("angle_min").value
        angle_max = self.get_parameter("angle_max").value
        angle_increment = self.get_parameter("angle_increment").value

        angles = np.arange(angle_min, angle_max + angle_increment, angle_increment)

        # Synthetische Umgebung: eine Kombination aus Sinusflächen und zufälligen Objekten.
        base_distance = 2.0 + np.sin(angles * 3.0) * 0.2
        random_objects = 0.2 * np.exp(-((angles - 0.5) ** 2) / 0.1)
        distances = base_distance + random_objects

        # Rauschen hinzufügen, um realistischere Messwerte zu simulieren.
        noise = np.random.normal(0.0, 0.02, size=distances.shape)
        distances = np.clip(distances + noise, self.get_parameter("min_range").value, self.get_parameter("max_range").value)

        # Intensitäten basieren auf der Entfernung: nahe Objekte reflektieren typischerweise stärker.
        intensities = np.clip(1.0 / (distances ** 2), 0.0, 1.0)

        return distances.astype(np.float32), intensities.astype(np.float32)

    def _publish_scan(self) -> None:
        """Erstellt und veröffentlicht eine neue `LaserScan`-Nachricht."""

        distances, intensities = self._create_scan()
        angle_min = self.get_parameter("angle_min").value
        angle_max = self.get_parameter("angle_max").value
        angle_increment = self.get_parameter("angle_increment").value

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "stl27l_lidar"
        scan_msg.angle_min = float(angle_min)
        scan_msg.angle_max = float(angle_max)
        scan_msg.angle_increment = float(angle_increment)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self.get_parameter("scan_frequency").value
        scan_msg.range_min = float(self.get_parameter("min_range").value)
        scan_msg.range_max = float(self.get_parameter("max_range").value)
        scan_msg.ranges = distances.tolist()
        scan_msg.intensities = intensities.tolist()

        self._publisher.publish(scan_msg)


def main() -> None:
    """Initialisiert rclpy und startet den Node."""

    rclpy.init()
    node = STL27LDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("STL27L Demo-Treiber beendet (CTRL+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
