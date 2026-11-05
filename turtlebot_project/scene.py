"""
Build the room and a simplified Burger model rather than the official robot model.
"""
from pathlib import Path
import cv2
from .model import MARKER_SIDE
from .config import OBSTACLES

def create_scene(path):
    pattern = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 7, 6)
    cells = []
    pitch = MARKER_SIDE / 6
    # The tag x axis points along world -y, its y axis along +z and its face towards -x.
    for row in range(6):
        for col in range(6):
            if pattern[row, col] == 0:
                y = -(col - 2.5) * pitch
                z = 0.22 + (2.5 - row) * pitch
                cells.append(f'<geom name="tag_{row}_{col}" type="box" pos="2.993 {y} {z}" size="0.001 {pitch/2} {pitch/2}" rgba="0 0 0 1" contype="0" conaffinity="0"/>')
    
    # Each obstacle becomes box 0.5m tall, with the base at z=0. The position is the centre of the box.
    obstacle_xml = []
    for i, (x0, x1, y0, y1) in enumerate(OBSTACLES):
        obstacle_xml.append(f'<geom name="obstacle_{i}" type="box" '
                            f'pos="{(x0+x1)/2} {(y0+y1)/2} 0.25" ' 
                            f'size="{(x1-x0)/2} {(y1-y0)/2} 0.25" ' # MuJoCo box size is half the extent in each dimension; 0.25 becomes 0.5m cube
                            f'rgba="0.5 0.5 0.5 1"/>'
                            )
        
    xml = '''<mujoco model="turtlebot3_burger_camera_baseline">
  <compiler angle="radian"/>
  <option timestep="0.002" integrator="implicitfast" cone="elliptic" noslip_iterations="5"/>
  <visual><global offwidth="960" offheight="720"/><headlight ambient="0.6 0.6 0.6" diffuse="0.5 0.5 0.5"/></visual>
  <default><geom friction="0.9 0.005 0.0001" condim="4" solref="0.006 1" solimp="0.99 0.999 0.001"/><joint damping="0.001" armature="0.002"/></default>
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" diffuse="0.7 0.7 0.7"/>
    <geom name="floor" type="plane" size="5 4 0.1" friction="0.001 0.00001 0.00001" rgba="0.75 0.78 0.8 1"/>
    <geom name="wall_back" type="box" pos="-2.6 0 0.4" size="0.05 2 0.4" rgba="0.7 0.7 0.7 1"/>
    <geom name="wall_left" type="box" pos="0.3 2 0.4" size="2.9 0.05 0.4" rgba="0.7 0.7 0.7 1"/>
    <geom name="wall_right" type="box" pos="0.3 -2 0.4" size="2.9 0.05 0.4" rgba="0.7 0.7 0.7 1"/>
    <geom name="wall_dock" type="box" pos="3.2 0 0.4" size="0.05 2 0.4" rgba="0.7 0.7 0.7 1"/>
    <geom name="dock_board" type="box" pos="3 0 0.22" size="0.005 0.18 0.18" rgba="1 1 1 1"/>
    TAG_CELLS
    OBSTACLES
    <camera name="overview" pos="0.2 -5.5 5.5" xyaxes="1 0 0 0 0.707 0.707"/>
    <geom name="charge_light" type="sphere" pos="3 0 0.65" size="0.045" 
      rgba="0.4 0.4 0.4 1" contype="0" conaffinity="0" group="2"/>
    <body name="base" pos="-1.8 -0.65 0.07">
      <freejoint name="base_free"/>
      <site name="lidar" pos="0 0 0.16" size="0.005" rgba="1 0 0 1"/>
      <inertial pos="-0.025 0 0" mass="1" diaginertia="0.006 0.006 0.008"/>
      <geom name="chassis" type="box" size="0.07 0.065 0.025" rgba="0.1 0.5 0.25 1"/>
      <geom name="top" type="cylinder" pos="0 0 0.06" size="0.055 0.01" mass="0.1" rgba="0.1 0.2 0.2 1"/>
      <geom name="camera_shell" type="box" pos="0.06 0 0.15" size="0.012 0.025 0.015" mass="0.03" rgba="0.08 0.08 0.08 1"/>
      <camera name="front" pos="0.06 0 0.15" xyaxes="0 -1 0 0 0 1" fovy="50"/>
      <body name="left_wheel" pos="0 0.08 -0.037">
        <joint name="left" type="hinge" axis="0 1 0"/>
        <geom name="left_tire" type="cylinder" size="0.033 0.009" quat="0.70710678 0.70710678 0 0" mass="0.05" rgba="0.08 0.08 0.08 1"/>
      </body>
      <body name="right_wheel" pos="0 -0.08 -0.037">
        <joint name="right" type="hinge" axis="0 1 0"/>
        <geom name="right_tire" type="cylinder" size="0.033 0.009" quat="0.70710678 0.70710678 0 0" mass="0.05" rgba="0.08 0.08 0.08 1"/>
      </body>
      <geom name="caster" type="sphere" pos="-0.075 0 -0.06" size="0.01" mass="0.02" friction="0.001 0.00001 0.00001"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="left_motor" joint="left" kv="0.15" ctrlrange="-6.5 6.5" forcerange="-0.12 0.12"/>
    <velocity name="right_motor" joint="right" kv="0.15" ctrlrange="-6.5 6.5" forcerange="-0.12 0.12"/>
  </actuator>
  <sensor><jointpos name="left_encoder" joint="left"/><jointpos name="right_encoder" joint="right"/></sensor>
</mujoco>'''.replace('TAG_CELLS', '\n'.join(cells)).replace('OBSTACLES', '\n'.join(obstacle_xml))
    import xml.etree.ElementTree as ET
    root = ET.fromstring(xml)
    # All robot geoms, including wheel descendendts belong to group 1
    # THe lidar ray mask will ignore group 1, so the robot will not see itself
    for geom in root.find(".//body[@name='base']").iter("geom"):
        geom.set('group', '1')
    xml = ET.tostring(root, encoding='unicode')
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(xml, encoding='utf8')
    return Path(path)
