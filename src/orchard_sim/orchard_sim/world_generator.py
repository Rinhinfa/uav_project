#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
from rclpy.node import Node


class OrchardWorldGenerator(Node):
    def __init__(self) -> None:
        super().__init__("orchard_world_generator")
        self.declare_parameter("rows", 8)
        self.declare_parameter("trees_per_row", 24)
        self.declare_parameter("row_spacing", 5.0)
        self.declare_parameter("tree_spacing", 3.0)
        self.declare_parameter("tree_radius", 0.35)
        self.declare_parameter("include_uav_models", False)
        self.declare_parameter("uav_count", 3)
        self.declare_parameter("uav_row_spacing", 3.0)
        self.declare_parameter("uav_start_x", -2.0)
        self.declare_parameter("uav_start_z", 1.0)
        self.declare_parameter("output_sdf", "/tmp/orchard_world.sdf")

        self._generate_once()

    def _generate_once(self) -> None:
        rows = int(self.get_parameter("rows").value)
        trees_per_row = int(self.get_parameter("trees_per_row").value)
        row_spacing = float(self.get_parameter("row_spacing").value)
        tree_spacing = float(self.get_parameter("tree_spacing").value)
        tree_radius = float(self.get_parameter("tree_radius").value)
        include_uav_models = bool(self.get_parameter("include_uav_models").value)
        uav_count = int(self.get_parameter("uav_count").value)
        uav_row_spacing = float(self.get_parameter("uav_row_spacing").value)
        uav_start_x = float(self.get_parameter("uav_start_x").value)
        uav_start_z = float(self.get_parameter("uav_start_z").value)
        output_sdf = Path(str(self.get_parameter("output_sdf").value))

        models = []
        tree_id = 0
        y_offset = -0.5 * (rows - 1) * row_spacing
        x_offset = -0.5 * (trees_per_row - 1) * tree_spacing
        for row in range(rows):
            y = y_offset + row * row_spacing
            for col in range(trees_per_row):
                x = x_offset + col * tree_spacing
                models.append(self._tree_model_block(tree_id, x, y, tree_radius))
                tree_id += 1
        if include_uav_models:
            models.extend(
                self._uav_model_blocks(
                    uav_count=uav_count,
                    row_spacing=uav_row_spacing,
                    start_x=uav_start_x,
                    start_z=uav_start_z,
                )
            )

        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="orchard_world">
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.60 0.75 0.95 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 80 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.35 0.15 -1.0</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="ground_link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
          <material>
            <ambient>0.30 0.45 0.30 1</ambient>
            <diffuse>0.35 0.50 0.35 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    {' '.join(models)}
  </world>
</sdf>
"""
        output_sdf.parent.mkdir(parents=True, exist_ok=True)
        output_sdf.write_text(sdf, encoding="utf-8")
        self.get_logger().info(f"Generated orchard world: {output_sdf}")

    @staticmethod
    def _tree_model_block(tree_id: int, x: float, y: float, radius: float) -> str:
        canopy_r = round(radius * 2.2, 2)
        return f"""
<model name="tree_{tree_id}">
  <static>true</static>
  <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
  <link name="trunk">
    <collision name="trunk_col">
      <pose>0 0 1.0 0 0 0</pose>
      <geometry><cylinder><radius>{radius:.2f}</radius><length>2.0</length></cylinder></geometry>
    </collision>
    <visual name="trunk_vis">
      <pose>0 0 1.0 0 0 0</pose>
      <geometry><cylinder><radius>{radius:.2f}</radius><length>2.0</length></cylinder></geometry>
      <material>
        <ambient>0.40 0.25 0.10 1</ambient>
        <diffuse>0.50 0.30 0.15 1</diffuse>
        <specular>0.05 0.04 0.02 1</specular>
      </material>
    </visual>
  </link>
  <link name="canopy">
    <collision name="canopy_col">
      <pose>0 0 2.9 0 0 0</pose>
      <geometry><sphere><radius>{canopy_r}</radius></sphere></geometry>
    </collision>
    <visual name="canopy_vis">
      <pose>0 0 2.9 0 0 0</pose>
      <geometry><sphere><radius>{canopy_r}</radius></sphere></geometry>
      <material>
        <ambient>0.15 0.48 0.12 1</ambient>
        <diffuse>0.22 0.56 0.16 1</diffuse>
        <specular>0.03 0.10 0.03 1</specular>
      </material>
    </visual>
  </link>
</model>
"""

    @staticmethod
    def _uav_model_blocks(
        uav_count: int, row_spacing: float, start_x: float, start_z: float
    ) -> list[str]:
        # 每架无人机独立颜色
        colours = [
            (0.10, 0.20, 0.85),  # uav_1 — 蓝
            (0.10, 0.65, 0.25),  # uav_2 — 绿
            (0.90, 0.42, 0.05),  # uav_3 — 橙
            (0.58, 0.10, 0.82),  # uav_4 — 紫
        ]
        models = []
        y_center = 0.5 * max(0, uav_count - 1) * row_spacing

        # 机臂半长（box 长 0.54m，末端到中心 0.27m，在 ±45° 方向）
        arm_half = 0.27
        # 旋翼安装点（±45° 分解到 XY）
        rt = arm_half * math.cos(math.radians(45))  # ≈ 0.191

        for i in range(max(0, uav_count)):
            y = i * row_spacing - y_center
            r, g, b = colours[i % len(colours)]

            # 旋翼 XML（4 个薄圆盘）
            rotor_tips = [(rt, rt), (-rt, rt), (-rt, -rt), (rt, -rt)]
            rotor_xml = "\n".join(
                f"""      <visual name="rotor_{j}_vis">
        <pose>{tx:.4f} {ty:.4f} 0.030 0 0 0</pose>
        <geometry><cylinder><radius>0.140</radius><length>0.010</length></cylinder></geometry>
        <material>
          <ambient>0.08 0.08 0.08 0.70</ambient>
          <diffuse>0.12 0.12 0.12 0.70</diffuse>
        </material>
      </visual>"""
                for j, (tx, ty) in enumerate(rotor_tips)
            )

            # 起落架 XML（4 根细腿，从机身底部向下）
            leg_len = 0.18
            leg_xml = "\n".join(
                f"""      <visual name="leg_{j}_vis">
        <pose>{tx:.4f} {ty:.4f} {-0.05 - leg_len / 2:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>{leg_len:.3f}</length></cylinder></geometry>
        <material>
          <ambient>0.20 0.20 0.20 1</ambient>
          <diffuse>0.25 0.25 0.25 1</diffuse>
        </material>
      </visual>"""
                for j, (tx, ty) in enumerate(rotor_tips)
            )

            models.append(f"""
<model name="uav_{i + 1}">
  <static>false</static>
  <pose>{start_x:.2f} {y:.2f} {start_z:.2f} 0 0 0</pose>
  <link name="base_link">
    <gravity>false</gravity>
    <inertial>
      <mass>1.5</mass>
      <inertia>
        <ixx>0.025</ixx><iyy>0.025</iyy><izz>0.045</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
      </inertia>
    </inertial>
    <collision name="body_col">
      <geometry><box><size>0.28 0.28 0.10</size></box></geometry>
    </collision>
    <!-- 机身 -->
    <visual name="body_vis">
      <geometry><box><size>0.28 0.28 0.10</size></box></geometry>
      <material>
        <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
        <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
        <specular>0.40 0.40 0.40 1</specular>
      </material>
    </visual>
    <!-- 机臂 X 型，两根交叉横杆 -->
    <visual name="arm_ne_sw_vis">
      <pose>0 0 0 0 0 0.7854</pose>
      <geometry><box><size>0.54 0.042 0.032</size></box></geometry>
      <material><ambient>0.22 0.22 0.22 1</ambient><diffuse>0.28 0.28 0.28 1</diffuse></material>
    </visual>
    <visual name="arm_nw_se_vis">
      <pose>0 0 0 0 0 2.3562</pose>
      <geometry><box><size>0.54 0.042 0.032</size></box></geometry>
      <material><ambient>0.22 0.22 0.22 1</ambient><diffuse>0.28 0.28 0.28 1</diffuse></material>
    </visual>
    <!-- 旋翼 -->
{rotor_xml}
    <!-- 起落架 -->
{leg_xml}
  </link>
</model>
""")
        return models


def main() -> None:
    rclpy.init()
    node = OrchardWorldGenerator()
    rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
