import importlib.util
from pathlib import Path
import unittest

from launch.actions import DeclareLaunchArgument


LAUNCH_PATH = Path(__file__).parents[1] / "launch" / "integrated_robot_o6_bringup.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("integrated_robot_o6_bringup", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _launch_defaults(module):
    defaults = {}
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            value = entity.default_value
            defaults[entity.name] = value[0].text if hasattr(value[0], "text") else str(value)
    return defaults


class IntegratedRobotO6ModesTest(unittest.TestCase):
  def test_shared_bus_is_the_default_and_independent_defaults_use_can6_can7(self):
    module = _load_launch_module()
    defaults = _launch_defaults(module)

    self.assertEqual(defaults["o6_hardware_mode"], "shared_bus")
    self.assertEqual(defaults["right_o6_can_interface"], "can6")
    self.assertEqual(defaults["left_o6_can_interface"], "can7")


  def test_independent_mode_rejects_overlap_with_robot_can_interfaces(self):
    module = _load_launch_module()

    with self.assertRaisesRegex(RuntimeError, "separate CAN interfaces"):
      module._validate_o6_configuration(
          "independent",
          {"can1", "can0"},
          {"can7", "can0"},
      )

  def test_independent_mode_rejects_overlap_with_head_or_lift_can(self):
    module = _load_launch_module()

    with self.assertRaisesRegex(RuntimeError, "separate CAN interfaces"):
      module._validate_o6_configuration(
          "independent",
          {"can1", "can0", "can2", "can3", "can4", "can5"},
          {"can7", "can2"},
      )


  def test_shared_bus_mode_accepts_arm_bus_for_each_hand(self):
    module = _load_launch_module()

    self.assertEqual(module._validate_o6_configuration(
      "shared_bus",
      {"can1", "can0"},
      {"can7", "can6"},
    ), "shared_bus")

  def test_independent_mode_creates_two_standalone_o6_drivers(self):
    module = _load_launch_module()

    class Context:
      values = {
          "o6_hardware_mode": "independent",
          "use_fake_hardware": "false",
          "right_o6_can_interface": "can6",
          "left_o6_can_interface": "can7",
          "right_o6_can_id": "0x27",
          "left_o6_can_id": "0x28",
      }

      def perform_substitution(self, substitution):
        return self.values[substitution.variable_name[0].text]

    nodes = module._independent_o6_nodes(Context())
    drivers = [
      node for node in nodes
      if getattr(node, "_Node__node_executable", None) == "o6_driver_node"
    ]
    self.assertEqual(len(drivers), 2)
    self.assertEqual({node._Node__package for node in drivers}, {"hands_hardware"})

  def test_independent_fake_mode_keeps_state_mapping_without_can_drivers(self):
    module = _load_launch_module()

    class Context:
      values = {
          "o6_hardware_mode": "independent",
          "use_fake_hardware": "true",
          "right_o6_can_interface": "can6",
          "left_o6_can_interface": "can7",
          "right_o6_can_id": "0x27",
          "left_o6_can_id": "0x28",
      }

      def perform_substitution(self, substitution):
        return self.values[substitution.variable_name[0].text]

    nodes = module._independent_o6_nodes(Context())
    executables = [node._Node__node_executable for node in nodes]
    self.assertNotIn("o6_driver_node", executables)
    self.assertIn("o6_joint_state_mapper_node", executables)
    self.assertIn("joint_state_merger", executables)


if __name__ == "__main__":
  unittest.main()
