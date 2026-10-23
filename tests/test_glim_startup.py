"""Launch wiring tests without starting GLIM, DDS or a robot."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import run_online_deploy


class GlimStartupTest(unittest.TestCase):
    def test_deploy_default_and_override(self):
        for extra, expected in [
            ([], ROOT / "glim/config"),
            (["--glim-config", "/tmp/custom-glim"], Path("/tmp/custom-glim")),
        ]:
            args = ["launcher", "--environment", "sim",
                    "--navigation-localization-source", "glim"] + extra
            with patch.object(sys, "argv", args), patch.object(
                run_online_deploy, "run_processes", return_value=0
            ) as run:
                self.assertEqual(run_online_deploy.main(), 0)
            command = run.call_args.args[0][0]
            self.assertEqual(command[command.index("--glim-config")+1], str(expected))

    def test_simulator_localization_does_not_pass_glim_config(self):
        with patch.object(sys, "argv", ["launcher", "--environment", "sim",
                  "--navigation-localization-source", "simulator"]), patch.object(
                  run_online_deploy, "run_processes", return_value=0) as run:
            run_online_deploy.main()
        self.assertNotIn("--glim-config", run.call_args.args[0][0])

    def test_ros_launch_default(self):
        from launch.actions import DeclareLaunchArgument
        spec = importlib.util.spec_from_file_location("glim_launch", ROOT / "glim/launch/glim.launch.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        argument = next(e for e in description.entities if isinstance(e, DeclareLaunchArgument))
        self.assertEqual(argument.name, "config_path")
        self.assertEqual("".join(s.text for s in argument.default_value), str(ROOT / "glim/config"))


if __name__ == "__main__":
    unittest.main()
