"""Film a QC round.

Drives `probes/qc/round.py` - the same code the tests will - rather than a sequence of
its own, so the video always shows what the round actually does. The caption follows the
trace as it fills, so a frame can be matched to the step that produced it.

A round that fails still gets filmed to the end of what it managed: the failure is the
part worth watching.

    .venv/bin/python probes/record_round.py [good|bad] [out.mp4]
"""

import asyncio
import io
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from PIL import Image, ImageDraw  # noqa: E402
from qc.layout import Cell  # noqa: E402
from qc.round import Clients, Trace, run_round  # noqa: E402
from viam.components.arm import Arm  # noqa: E402
from viam.components.camera import Camera  # noqa: E402
from viam.components.generic import Generic  # noqa: E402
from viam.components.gripper import Gripper  # noqa: E402
from viam.robot.client import RobotClient  # noqa: E402
from viam.services.motion import MotionClient  # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
FPS = 10


async def main() -> int:
    tray = sys.argv[1] if len(sys.argv) > 1 else "good"
    out = Path(sys.argv[2] if len(sys.argv) > 2
               else ".scratch/runs/qc-round.mp4").resolve()
    frames = out.parent / "round_frames"
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    frames.mkdir(parents=True, exist_ok=True)

    robot = await RobotClient.at_address(
        FQDN,
        RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    clients = Clients(
        motion=MotionClient.from_robot(robot, "builtin"),
        world=Generic.from_robot(robot, "sim-world"),
        arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
        grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                  "arm-b": Gripper.from_robot(robot, "suction-b")})
    camera = Camera.from_robot(robot, "overview-cam")
    trace = Trace()
    started = time.perf_counter()
    filming = True
    count = {"n": 0}

    async def film():
        while filming:
            try:
                images, _ = await camera.get_images()
                frame = Image.open(io.BytesIO(images[0].data)).convert("RGB")
            except Exception:  # noqa: BLE001
                await asyncio.sleep(0.1)
                continue
            elapsed = time.perf_counter() - started
            # The caption reads from the trace as it fills, so it names the step that is
            # actually happening rather than a script's idea of the order.
            stage = trace.names[-1] if trace.names else "starting"
            draw = ImageDraw.Draw(frame)
            draw.rectangle([0, frame.height - 34, frame.width, frame.height],
                           fill=(0, 0, 0))
            draw.text((14, frame.height - 25),
                      f"{int(elapsed // 60)}:{elapsed % 60:04.1f}   {stage}",
                      fill=(255, 255, 255))
            frame.save(frames / f"f{count['n']:05d}.jpg", quality=88)
            count["n"] += 1
            await asyncio.sleep(1.0 / FPS)

    task = asyncio.create_task(film())
    failure = None
    try:
        await run_round(clients, Cell.from_fragment(), verdict_tray=tray, trace=trace)
    except Exception as exc:  # noqa: BLE001
        failure = str(exc)[:200]
    finally:
        filming = False
        await asyncio.sleep(0.3)
        task.cancel()
        await robot.close()

    for stage in trace.stages:
        def tidy(value):
            if isinstance(value, (list, tuple)):
                return [round(x) if isinstance(x, (int, float)) else x for x in value]
            return value

        detail = {k: tidy(v) for k, v in stage.items()
                  if k not in ("stage", "error")}
        print(f"  {stage['stage']:>24}  {detail}")
    if failure:
        print(f"\n  round did not finish: {failure}")

    took = time.perf_counter() - started
    rate = count["n"] / max(took, 0.001)
    subprocess.run([
        "ffmpeg", "-y", "-loglevel", "error", "-framerate", f"{rate:.3f}",
        "-i", str(frames / "f%05d.jpg"), "-c:v", "libx264", "-pix_fmt", "yuv420p",
        "-crf", "20", str(out)], check=True)
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    print(f"\nwrote {out} ({out.stat().st_size / 1e6:.1f} MB, "
          f"{count['n']} frames over {took:.0f}s)")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
