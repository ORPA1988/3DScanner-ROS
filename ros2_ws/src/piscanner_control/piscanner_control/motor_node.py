"""ROS2-Knoten zur Steuerung des Schrittmotors.

Der Knoten abstrahiert die Hardwareansteuerung und stellt Topics sowie Services
bereit, damit andere ROS2-Knoten (z. B. die Punktwolkenfusion) die aktuelle Motorposition
nutzen können. Umfangreiche Kommentare erläutern die Arbeitsweise.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

from piscanner_control.srv import ConfigureScan
from .stepper_driver import StepperConfig, StepperDriver


class SimulatedStepperDriver:
    """Einfache Simulation, falls keine GPIO-Hardware verfügbar ist."""

    def __init__(self, config: StepperConfig) -> None:
        self._config = config
        self._current_angle = 0.0

    @property
    def current_angle(self) -> float:
        return self._current_angle

    def rotate_to(self, target_angle: float, angular_velocity: float) -> None:
        # Die Simulation schläft entsprechend der benötigten Zeit und setzt den Winkel.
        angle_difference = abs(target_angle - self._current_angle)
        time_needed = angle_difference / max(angular_velocity, 1e-3)
        time.sleep(time_needed)
        self._current_angle = target_angle

    def enable(self) -> None:  # pragma: no cover - trivial
        return

    def disable(self) -> None:  # pragma: no cover - trivial
        return

    def cleanup(self) -> None:  # pragma: no cover - trivial
        return


@dataclass
class ScanConfiguration:
    """Zwischenspeicher für konfigurierte Scanparameter."""

    start_angle: float
    end_angle: float
    angular_velocity: float


class StepperMotorNode(Node):
    """ROS2-Node, der den Schrittmotor kontrolliert und den Winkel veröffentlicht."""

    def __init__(self) -> None:
        super().__init__("stepper_motor_node")

        # Parameter für Pinbelegung und Getriebeuntersetzung.
        self.declare_parameter("step_pin", 23)
        self.declare_parameter("dir_pin", 24)
        self.declare_parameter("enable_pin", 18)
        self.declare_parameter("steps_per_revolution", 200.0 * (1.0 + 38.0 / 14.0))
        self.declare_parameter("max_speed", 2.0)

        config = StepperConfig(
            step_pin=int(self.get_parameter("step_pin").value),
            dir_pin=int(self.get_parameter("dir_pin").value),
            enable_pin=int(self.get_parameter("enable_pin").value),
            steps_per_revolution=float(self.get_parameter("steps_per_revolution").value),
            max_speed=float(self.get_parameter("max_speed").value),
        )

        # Versuche die echte Hardware anzusprechen, falle andernfalls auf Simulation zurück.
        try:
            self._driver = StepperDriver(config)
            self.get_logger().info("Hardware-Treiber aktiviert")
        except RuntimeError:
            self._driver = SimulatedStepperDriver(config)
            self.get_logger().warning("GPIO-Treiber nicht verfügbar, starte Simulation")

        self._angle_publisher = self.create_publisher(Float32, "/piscanner_control/motor_angle", 10)
        self._scan_command_sub = self.create_subscription(Bool, "/piscanner_control/scan_command", self._scan_command_callback, 10)
        self._home_service = self.create_service(Trigger, "/piscanner_control/home_motor", self._handle_home_request)
        self._configure_service = self.create_service(ConfigureScan, "/piscanner_control/configure_scan", self._handle_configure_request)

        self._status_timer = self.create_timer(0.1, self._publish_angle)

        self._scan_configuration: Optional[ScanConfiguration] = None
        self._scan_thread: Optional[threading.Thread] = None
        self._scan_active = threading.Event()

    def _publish_angle(self) -> None:
        """Veröffentlicht regelmäßig den aktuellen Winkel."""

        msg = Float32()
        msg.data = float(self._driver.current_angle)
        self._angle_publisher.publish(msg)

    def _handle_home_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Referenziert den Motor (setzt Winkel auf 0)."""

        self._driver.enable()
        # In einem realen Aufbau würde hier ein Endschalter angefahren.
        self._driver.rotate_to(0.0, self._driver.current_angle if self._driver.current_angle != 0 else 1.0)
        self._driver.disable()
        response.success = True
        response.message = "Motor auf 0 rad gesetzt"
        return response

    def _handle_configure_request(self, request: ConfigureScan.Request, response: ConfigureScan.Response) -> ConfigureScan.Response:
        """Speichert die Konfiguration für den nächsten Scan."""

        if request.start_angle_rad >= request.end_angle_rad:
            response.success = False
            response.message = "Startwinkel muss kleiner als Endwinkel sein"
            return response

        if request.angular_velocity_rad_s <= 0.0:
            response.success = False
            response.message = "Die Winkelgeschwindigkeit muss positiv sein"
            return response

        self._scan_configuration = ScanConfiguration(
            start_angle=float(request.start_angle_rad),
            end_angle=float(request.end_angle_rad),
            angular_velocity=float(request.angular_velocity_rad_s),
        )
        response.success = True
        response.message = "Scankonfiguration übernommen"
        return response

    def _scan_command_callback(self, msg: Bool) -> None:
        """Reagiert auf Start-/Stop-Kommandos."""

        if msg.data and not self._scan_active.is_set():
            if self._scan_configuration is None:
                self.get_logger().error("Scan gestartet, aber keine Konfiguration gesetzt")
                return
            self._scan_active.set()
            self._scan_thread = threading.Thread(target=self._execute_scan, daemon=True)
            self._scan_thread.start()
        elif not msg.data:
            self._scan_active.clear()

    def _execute_scan(self) -> None:
        """Führt den Scan in einem Hintergrundthread aus."""

        if self._scan_configuration is None:
            return

        config = self._scan_configuration
        self._driver.enable()

        current_target = config.start_angle
        self._driver.rotate_to(current_target, config.angular_velocity)
        current_target = config.end_angle

        # Solange der Scan aktiv ist, fahre zum Endwinkel und wieder zurück.
        while self._scan_active.is_set():
            self._driver.rotate_to(current_target, config.angular_velocity)
            current_target = config.start_angle if current_target == config.end_angle else config.end_angle

        self._driver.disable()

    def destroy_node(self) -> bool:
        """Sorgt für sauberes Herunterfahren der Threads und Hardware."""

        self._scan_active.clear()
        if self._scan_thread and self._scan_thread.is_alive():
            self._scan_thread.join(timeout=1.0)
        self._driver.cleanup()
        return super().destroy_node()


def main() -> None:
    """Initialisiert rclpy und startet den Node."""

    rclpy.init()
    node = StepperMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Schrittmotor-Knoten beendet (CTRL+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
