"""Record the cell working, because watching it is what actually finds problems.

Every collision in this cell's history was found by a person watching a video, while the
instruments said it was clean. So this drives the arms through the real station sequence
and films it, rather than asserting anything.

Frames are pulled from `overview-cam` on a task that runs while the arms move, so the
video is a record of the run rather than a slideshow of poses. Each frame is captioned
with the phase and the wall-clock time into the run, which is what makes it possible to
say "at 0:14 the forearm clips the tray" and have that mean something afterwards.

    .venv/bin/python probes/record_cell.py [output.mp4]
"""

import asyncio
import io
import json
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from obstacle_check import box, tool_transform  # noqa: E402


TOOL_LENGTH_MM = 120.0


def grasp_height(props, part_pose):
    """Flange height that puts the cup on the part's top face.

    Derived from what the sim reports the part to be, not from a remembered number: the
    part changed from an 80 mm carton to a 6 mm steel blank, and a hard-coded half-height
    would have aimed the cup 37 mm inside it.
    """
    part = next((p for p in props if p["label"] == "part"), None)
    half = (part["dims_mm"][2] / 2.0) if part else 0.0
    return part_pose["position_mm"][2] + half + TOOL_LENGTH_MM
from PIL import Image, ImageDraw  # noqa: E402
from viam.components.arm import Arm  # noqa: E402
from viam.components.camera import Camera  # noqa: E402
from viam.components.gripper import Gripper  # noqa: E402
from viam.components.generic import Generic  # noqa: E402
from viam.proto.common import (  # noqa: E402
    GeometriesInFrame, Pose, PoseInFrame, WorldState,
)
from viam.robot.client import RobotClient  # noqa: E402
from viam.services.motion import MotionClient  # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
STATIONS = json.loads((Path(__file__).resolve().parent / "stations.json").read_text())
HOMES = {"arm-a": (300.0, -380.0, 900.0), "arm-b": (300.0, 380.0, 900.0)}
FPS = 12
rate = float(FPS)
OUT = Path(sys.argv[1] if len(sys.argv) > 1 else
           ".scratch/runs/qc-cell.mp4").resolve()


class Recorder:
    """Pulls frames while something else drives the arms."""

    def __init__(self, camera, directory: Path):
        self.camera = camera
        self.directory = directory
        self.caption = "starting"
        self.count = 0
        self.failures = 0
        self._stop = False
        self._t0 = time.perf_counter()

    async def run(self):
        while not self._stop:
            try:
                images, _ = await self.camera.get_images()
                if not images:
                    raise RuntimeError("camera returned no images")
                # NamedImage carries encoded bytes plus a mime type, not a PIL image.
                frame = Image.open(io.BytesIO(images[0].data))
            except Exception as exc:  # noqa: BLE001
                # A dropped frame must not end the take, but a camera that never
                # works must not be mistaken for one: count the failures and say so.
                self.failures += 1
                if self.failures == 1:
                    print(f"  camera error: {type(exc).__name__}: {str(exc)[:90]}")
                await asyncio.sleep(0.1)
                continue
            self._stamp(frame.convert("RGB"))
            # Pace the capture. Rendering happens on the sim thread, so pulling frames
            # flat out competes with stepping physics - and it showed: a pick that the
            # reliability probe lands 5/5 dropped the carton every time it was filmed.
            # The encode still uses the rate actually achieved, so the video plays at
            # real time rather than looking sped up.
            await asyncio.sleep(1.0 / FPS)

    def _stamp(self, frame):
        elapsed = time.perf_counter() - self._t0
        draw = ImageDraw.Draw(frame)
        label = f"{int(elapsed // 60)}:{elapsed % 60:04.1f}   {self.caption}"
        draw.rectangle([0, frame.height - 34, frame.width, frame.height],
                       fill=(0, 0, 0))
        draw.text((14, frame.height - 25), label, fill=(255, 255, 255))
        frame.save(self.directory / f"f{self.count:05d}.jpg", quality=88)
        self.count += 1

    def elapsed(self) -> float:
        return time.perf_counter() - self._t0

    def stop(self):
        self._stop = True


async def main():
    frames = OUT.parent / "frames"
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    frames.mkdir(parents=True, exist_ok=True)

    robot = await RobotClient.at_address(
        FQDN,
        RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]
        ),
    )
    try:
        motion = MotionClient.from_robot(robot, "builtin")
        world = Generic.from_robot(robot, "sim-world")
        camera = Camera.from_robot(robot, "overview-cam")

        await world.do_command({"command": "reset"})
        await asyncio.sleep(3)

        props = list((await world.do_command({"command": "obstacles"}))["obstacles"])
        def world_state(with_part: bool):
            """The carton is an obstacle right up until you mean to touch it.

            Approach with it declared, so the arm routes around it instead of swiping it
            off the belt; drop it from the set for the final descent and the carry, which
            are contacts by intention rather than collisions.
            """
            chosen = props if with_part else [p for p in props if p["label"] != "part"]
            return WorldState(
                obstacles=[GeometriesInFrame(
                    reference_frame="world",
                    geometries=[box(p["label"], p["center_mm"], p["dims_mm"])
                                for p in chosen],
                )],
                transforms=[tool_transform("arm-a"), tool_transform("arm-b")],
            )

        state = world_state(True)

        recorder = Recorder(camera, frames)
        task = asyncio.create_task(recorder.run())

        async def go(arm, position, caption, with_part=True):
            recorder.caption = caption
            try:
                await motion.move(
                    component_name=arm,
                    destination=PoseInFrame(reference_frame="world", pose=Pose(
                        x=position[0], y=position[1], z=position[2],
                        o_x=0, o_y=0, o_z=-1, theta=0)),
                    world_state=world_state(with_part),
                )
            except Exception as exc:  # noqa: BLE001
                recorder.caption = f"{caption} - FAILED"
                print(f"  {caption}: {str(exc)[:70]}")
                await asyncio.sleep(1.0)

        # Open with a real pick, because that is the part worth watching: the suction
        # either carries the carton or it does not, and the video says which.
        gripper = Gripper.from_robot(robot, "suction-a")
        part = (await world.do_command(
            {"command": "prop_poses", "names": ["part"]}))["props"]["part"]
        px, py, pz = part["position_mm"]
        contact = grasp_height(props, part)

        def at(z):
            return (px, py, z)

        await asyncio.sleep(1.0)
        recorder.caption = "arm-b clears"
        await go("arm-b", HOMES["arm-b"], "arm-b clears")
        await go("arm-a", at(contact + 150), "arm-a  approach the carton")
        await go("arm-a", at(contact), "arm-a  down to the carton", with_part=False)
        recorder.caption = "suction-a  grab"
        caught = await gripper.grab()
        await asyncio.sleep(0.6)
        await go("arm-a", at(contact + 260), f"arm-a  lift (holding={caught})",
                 with_part=False)
        await go("arm-a", (px - 120, py + 220, contact + 260), "arm-a  carry",
                 with_part=False)
        recorder.caption = "suction-a  release"
        await gripper.open()
        await asyncio.sleep(1.5)

        for station, (arm, position) in STATIONS["stations"].items():
            if station.startswith("FORBIDDEN"):
                continue
            moving = arm if arm.startswith("arm") else f"arm-{arm}"
            other = "arm-b" if moving == "arm-a" else "arm-a"
            await go(other, HOMES[other], f"{other} clears")
            await go(moving, position, f"{moving}  {station}")
            print(f"  {station}: {recorder.count} frames so far")
        recorder.caption = "done"
        await asyncio.sleep(1.0)

        recorder.stop()
        await task
        global rate
        rate = recorder.count / max(recorder.elapsed(), 0.001)
        print(f"\ncaptured {recorder.count} frames over {recorder.elapsed():.0f}s "
              f"= {rate:.1f} fps ({recorder.failures} camera errors)")
        if recorder.count == 0:
            raise RuntimeError(
                "no frames captured - there is nothing to encode. "
                f"{recorder.failures} camera errors; see the message above."
            )
    finally:
        await robot.close()

    subprocess.run([
        "ffmpeg", "-y", "-loglevel", "error", "-framerate", f"{rate:.3f}",
        "-i", str(frames / "f%05d.jpg"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", str(OUT),
    ], check=True)
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    size = OUT.stat().st_size / 1e6
    print(f"wrote {OUT}  ({size:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
