import asyncio
import threading
from types import SimpleNamespace

from viam.proto.app.robot import ComponentConfig

from isaac_module import sdk_patches
from isaac_module.models.world import IsaacWorld
import isaac_module.models.world as world_model


def test_resource_creator_runs_outside_module_event_loop(monkeypatch):
    creator_thread = None
    registered = []

    def creator(config, dependencies):
        nonlocal creator_thread
        creator_thread = threading.get_ident()
        return SimpleNamespace(logger=SimpleNamespace())

    class Module:
        server = SimpleNamespace(register=registered.append)

        async def _get_dependencies(self, dependencies):
            return {}

    monkeypatch.setattr(
        sdk_patches.Registry, "lookup_resource_creator", lambda api, model: creator
    )
    monkeypatch.setattr(sdk_patches, "update_log_level", lambda logger, level: None)
    request = SimpleNamespace(
        dependencies=[],
        config=ComponentConfig(
            name="resource", api="rdk:component:generic", model="erh:isaac-sim:world"
        ),
    )
    asyncio.run(sdk_patches._add_resource(Module(), request))

    assert creator_thread != threading.get_ident()
    assert len(registered) == 1



def test_world_commands_run_outside_module_event_loop(monkeypatch):
    calls = []

    class Sim:
        def status(self):
            calls.append(("status", threading.get_ident()))
            return {"booted": True}

        def play(self):
            calls.append(("play", threading.get_ident()))

        def pause(self):
            calls.append(("pause", threading.get_ident()))

        def reset(self):
            calls.append(("reset", threading.get_ident()))

        def add_usd_reference(self, usd_path, prim_path, position):
            calls.append(("add_usd", threading.get_ident()))

    sim = Sim()
    monkeypatch.setattr(world_model.SimManager, "get", lambda: sim)

    async def commands():
        world = IsaacWorld("world")
        assert await world.do_command({"command": "status"}) == {"booted": True}
        await world.do_command({"command": "play"})
        await world.do_command({"command": "pause"})
        await world.do_command({"command": "reset"})
        await world.do_command(
            {
                "command": "add_usd",
                "usd_path": "asset.usd",
                "prim_path": "/World/asset",
            }
        )

    asyncio.run(commands())

    assert [name for name, _ in calls] == ["status", "play", "pause", "reset", "add_usd"]
    assert all(thread_id != threading.get_ident() for _, thread_id in calls)
