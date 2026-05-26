#!/usr/bin/env bash
set -eo pipefail

XACRO_PATH="${1:-/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/xacro/gazebo.xacro}"

if [ ! -f "$XACRO_PATH" ]; then
  echo "gazebo.xacro not found: $XACRO_PATH" >&2
  exit 1
fi

python3 - "$XACRO_PATH" <<'PY'
import pathlib
import re
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text()
marker = "    <!-- Foot contacts. -->"
if marker not in text:
    raise SystemExit("insertion marker not found")

text = re.sub(
    r'\n    <gazebo reference="RGB_camera_link">.*?\n    </gazebo>\n\n'
    r'    <gazebo reference="D435_camera_link">.*?\n    </gazebo>\n\n'
    r'(?=    <!-- Foot contacts\. -->)',
    '\n',
    text,
    flags=re.S,
)

camera_xml = r'''
    <gazebo reference="RGB_camera_link">
        <sensor name="race2026_rgb_camera" type="camera">
            <pose>0 0 0.02 0 0.24 0</pose>
            <always_on>true</always_on>
            <update_rate>10</update_rate>
            <visualize>false</visualize>
            <camera>
                <horizontal_fov>1.047</horizontal_fov>
                <image>
                    <width>320</width>
                    <height>240</height>
                    <format>R8G8B8</format>
                </image>
                <clip>
                    <near>0.03</near>
                    <far>20.0</far>
                </clip>
            </camera>
            <plugin name="race2026_rgb_camera_plugin" filename="libgazebo_ros_camera.so">
                <ros>
                    <remapping>image_raw:=/rgb_camera/image_raw</remapping>
                    <remapping>camera_info:=/rgb_camera/camera_info</remapping>
                </ros>
                <camera_name>rgb_camera</camera_name>
                <frame_name>RGB_camera_link</frame_name>
            </plugin>
        </sensor>
    </gazebo>

    <gazebo reference="D435_camera_link">
        <sensor name="race2026_d435_depth_camera" type="depth">
            <pose>0 0 0.02 0 0.24 0</pose>
            <always_on>true</always_on>
            <update_rate>8</update_rate>
            <visualize>false</visualize>
            <camera>
                <horizontal_fov>1.047</horizontal_fov>
                <image>
                    <width>320</width>
                    <height>240</height>
                    <format>R8G8B8</format>
                </image>
                <clip>
                    <near>0.08</near>
                    <far>8.0</far>
                </clip>
            </camera>
            <plugin name="race2026_d435_depth_camera_plugin" filename="libgazebo_ros_camera.so">
                <ros>
                    <remapping>image_raw:=/D435_camera/image_raw</remapping>
                    <remapping>camera_info:=/D435_camera/camera_info</remapping>
                    <remapping>depth/image_raw:=/D435_camera/depth/image_raw</remapping>
                    <remapping>depth/camera_info:=/D435_camera/depth/camera_info</remapping>
                    <remapping>points:=/D435_camera/points</remapping>
                </ros>
                <camera_name>D435_camera</camera_name>
                <frame_name>D435_camera_link</frame_name>
                <min_depth>0.08</min_depth>
                <max_depth>8.0</max_depth>
            </plugin>
        </sensor>
    </gazebo>

'''

path.write_text(text.replace(marker, camera_xml + marker))
print(f"patched 2026 camera sensors into {path}")
PY
