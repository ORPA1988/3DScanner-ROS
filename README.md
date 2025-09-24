# 3DScanner-ROS Gesamtpaket

<!--
    Diese README-Datei dient als zentrale Einstiegshilfe für Anwenderinnen und Anwender,
    die den in diesem Repository enthaltenen ROS2-basierten 3D-Scanner
    auf einem Raspberry Pi 5 mit LiDAR, Schrittmotor und HQ-Kamera aufbauen möchten.
    Die Kommentare innerhalb der README sollen jeden Schritt nachvollziehbar machen.
-->

Die folgenden Abschnitte führen Schritt für Schritt durch Hardwareaufbau, Softwareinstallation
und Nutzung der Beispielknoten. Alle Inhalte wurden mit Fokus auf Einsteigerinnen und Einsteiger
in ROS2 und Robotik geschrieben und erläutern daher viele Details ausführlich.

## Inhaltsverzeichnis

1. [Überblick](#überblick)
2. [Hardwareaufbau](#hardwareaufbau)
3. [Softwareübersicht](#softwareübersicht)
4. [Vorbereitung des Raspberry Pi](#vorbereitung-des-raspberry-pi)
5. [Docker-basierter LiDAR-Treiber](#docker-basierter-lidar-treiber)
6. [ROS2-Workspace aufsetzen](#ros2-workspace-aufsetzen)
7. [Pakete und Knoten](#pakete-und-knoten)
8. [Kalibrierung und Parameter](#kalibrierung-und-parameter)
9. [Datenerfassung und Speicherung](#datenerfassung-und-speicherung)
10. [Fehlersuche](#fehlersuche)
11. [Weiterführende Ressourcen](#weiterführende-ressourcen)

## Überblick

<!--
    Dieser Abschnitt erklärt, was das System leisten soll.
-->

Der 3D-Scanner kombiniert einen zweidimensionalen LiDAR (STL27L), einen NEMA17-Schrittmotor
mit Getriebe und eine Raspberry Pi HQ-Kamera mit Fischaugenobjektiv. Der LiDAR liefert
Entfernungs- und Intensitätsdaten in einer vertikalen Ebene (XZ), während der Schrittmotor
die gesamte Einheit um die Y-Achse dreht. Durch die Verknüpfung der Laser-Scans mit der
aktuellen Motorposition entsteht eine 3D-Punktwolke. Die Kamera liefert ergänzend
Farbinformationen (Vertexfärbung), die auf die Punktwolke projiziert werden.

## Hardwareaufbau

<!--
    Die folgenden Schritte beschreiben den mechanischen und elektrischen Aufbau.
-->

1. **LiDAR-Montage**: Der STL27L wird so befestigt, dass seine Scan-Ebene der globalen XZ-Ebene
   entspricht und die 0°-Richtung auf -X (Westen) zeigt. Beachten Sie den seitlichen
   Versatz in Richtung +Y.
2. **Schrittmotor und Getriebe**: Verbinden Sie den NEMA17 mit dem A4988-Treiber.
   Das Planetengetriebe besitzt eine Untersetzung von \(1 + \frac{38}{14}\). Notieren Sie
   die resultierenden Schritte pro Umdrehung (siehe Softwareparameter).
3. **Kameraausrichtung**: Die Raspberry Pi HQ-Kamera wird mit dem 180°-Fischauge so montiert,
   dass sie entlang -Y blickt (Süd). Die Kamera befindet sich leicht in -Y-Richtung versetzt.
4. **Verkabelung**: Nutzen Sie hochwertige Kabel für Versorgung und Signalübertragung.
   - Der LiDAR wird über USB mit dem Pi verbunden.
   - Der A4988 erhält STEP, DIR, ENABLE Signale vom Pi (GPIO) sowie separate Versorgung.
   - Die Kamera wird über das offizielle CSI-Kabel angeschlossen.

## Softwareübersicht

<!--
    Hier geben wir einen Überblick über die Softwarekomponenten.
-->

Das Repository liefert einen vollständigen ROS2-Workspace mit folgenden Paketen:

- `piscanner_control`: Steuert den Schrittmotor, veröffentlicht die aktuelle Drehposition
  und bietet Services zur Scanplanung.
- `piscanner_perception`: Verarbeitet LiDAR-Scans, kombiniert sie mit Motorwinkeln
  und Kamerabildern zu farbcodierten 3D-Punktwolken und 2D-Intensitätskarten.
- `piscanner_bringup`: Enthält Launch-Dateien, Parameter und Beispielkonfigurationen
  für einen kompletten Scanlauf.

Zusätzlich sind Dockerdateien vorhanden, um den herstellerspezifischen LiDAR-Treiber
in einem Container auszuführen und via ROS2-Bridge in das System einzubinden.

## Vorbereitung des Raspberry Pi

<!--
    Dieser Abschnitt erläutert die Basiskonfiguration von Raspberry Pi OS 64-bit.
-->

1. **Raspberry Pi OS installieren**: Nutzen Sie den Raspberry Pi Imager und wählen Sie
   die 64-bit Variante.
2. **System aktualisieren**:
   ```bash
   sudo apt update && sudo apt full-upgrade
   ```
3. **ROS2 (Humble) installieren**: Folgen Sie der offiziellen Anleitung unter
   <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>.
4. **Zusatzpakete installieren**:
   ```bash
   sudo apt install python3-colcon-common-extensions python3-pip docker.io docker-compose
   sudo usermod -aG docker $USER
   ```
5. **Neustart durchführen**, damit Dockerberechtigungen aktiv werden.

## Docker-basierter LiDAR-Treiber

<!--
    Wir beschreiben, wie der STL27L per Docker eingebunden wird.
-->

1. Wechseln Sie in das Verzeichnis `docker/` und bauen Sie das Image:
   ```bash
   cd docker
   docker build -t stl27l_driver .
   ```
2. Starten Sie den Container, der den LiDAR-Treiber und eine ROS2-Bridge bereitstellt:
   ```bash
   docker compose up
   ```
3. Der Container veröffentlicht den ROS2-Topic `/stl27l/scan` vom Typ `sensor_msgs/msg/LaserScan`.
   Testen Sie die Verbindung mit:
   ```bash
   ros2 topic echo /stl27l/scan
   ```

## ROS2-Workspace aufsetzen

<!--
    Schritt-für-Schritt-Anleitung zur Einrichtung des Workspace.
-->

1. **Workspace kopieren**: Klonen Sie dieses Repository auf den Pi.
2. **Abhängigkeiten installieren**:
   ```bash
   cd ros2_ws
   rosdep install --from-paths src -y --ignore-src
   ```
3. **Workspace bauen**:
   ```bash
   colcon build --symlink-install
   ```
4. **Umgebung laden**:
   ```bash
   source install/setup.bash
   ```

## Pakete und Knoten

<!--
    Dieser Abschnitt beschreibt die wichtigsten Nodes und ihre Topics/Services.
-->

### `piscanner_control`

- **Node `stepper_motor_node`** (`piscanner_control/motor_node.py`):
  - Subscribes: `/piscanner_control/scan_command` (`std_msgs/Bool`) zum Starten/Stoppen.
  - Publishes: `/piscanner_control/motor_angle` (`std_msgs/Float32`) – aktuelle Motorposition.
  - Services:
    - `/piscanner_control/home_motor` (`std_srvs/Trigger`) zum Referenzieren.
    - `/piscanner_control/configure_scan` (`piscanner_control/srv/ConfigureScan`) für Scankonfiguration.

### `piscanner_perception`

- **Node `pointcloud_builder_node`** (`piscanner_perception/pointcloud_node.py`):
  - Subscribes: `/stl27l/scan`, `/piscanner_control/motor_angle`, `/piscanner_camera/panorama`.
  - Publishes: `/piscanner_perception/pointcloud` (`sensor_msgs/PointCloud2`),
    `/piscanner_perception/lidar_image` (`sensor_msgs/Image`).
  - Services:
    - `/piscanner_perception/save_snapshot` (`std_srvs/Trigger`) zum Speichern aktueller Daten.

### `piscanner_bringup`

- **Launch-Datei `piscanner.launch.py`** startet alle Nodes, lädt Parameter und stellt sicher,
  dass Topics korrekt verbunden sind.

## Kalibrierung und Parameter

<!--
    Hinweise zur Parametrierung zur exakten Rekonstruktion.
-->

- **Offsets**: Die Parameter `lidar_offset_y`, `camera_offset`, `lidar_mount_height`
  werden in `piscanner_bringup/config/scanner_parameters.yaml` hinterlegt.
- **Motorauflösung**: Aus der Getriebeuntersetzung ergibt sich
  \(\text{SchritteProUmdrehung} = 200 \times (1 + 38/14)\).
- **Kamerakalibrierung**: Nutzen Sie `camera_calibration` aus ROS2, um die Fischaugenparameter
  zu bestimmen und in `camera_info` zu speichern.

## Datenerfassung und Speicherung

<!--
    Wir erklären, wie 2D-Bilder und 3D-Punktwolken gespeichert werden.
-->

1. Starten Sie `ros2 launch piscanner_bringup piscanner.launch.py`.
2. Warten Sie, bis der Motor referenziert wurde.
3. Rufen Sie `ros2 service call /piscanner_perception/save_snapshot std_srvs/srv/Trigger {}`
   auf, um die aktuelle Punktwolke (`.pcd`) und das 2D-LiDAR-Bild (`.png`) im Verzeichnis
   `~/piscanner_data/` abzulegen.

## Fehlersuche

<!--
    Tipps zur Problembehebung.
-->

- **Keine LiDAR-Daten**: Prüfen Sie, ob der Docker-Container läuft und ob der Benutzer Zugriff
  auf das USB-Gerät hat (`sudo usermod -aG dialout $USER`).
- **Motor bewegt sich nicht**: Kontrollieren Sie die Versorgungsspannung, die DIR/STEP-Pins
  und die A4988-Strombegrenzung.
- **Punktwolke verzerrt**: Überprüfen Sie die Winkelrichtung des Motors und die LiDAR-Offsets.
- **Farbprojektion fehlerhaft**: Stellen Sie sicher, dass die Kamera ein aktuelles Panorama
  veröffentlicht und die Kalibrierparameter korrekt sind.

## Weiterführende Ressourcen

- Offenes Beispielprojekt PiLiDAR: <https://github.com/ORPA1988/PiLiDAR>
- ROS2 Dokumentation: <https://docs.ros.org/en/humble/>
- Point Cloud Library: <https://pointclouds.org/>

Viel Erfolg beim Nachbau und viel Spaß beim Experimentieren mit dem 3D-Scanner!
