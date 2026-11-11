r"""
Builds /Game/Maps/RigidColliders: the level of the rigid collider comparison (Bullet vs Chaos).

The scenes are pure Chrono rigid bodies (shape pairs, stacks, a ramp slide, a bounce test) built on the physics thread, so the level
only needs a light, a sky and an AChronoBackendComparison actor with the collider tests switched on (vehicle and the other rigid
scenes off). Play it, or run headless:

    UnrealEditor.exe F:\Workspace\UnrealChrono\UnrealChrono.uproject /Game/Maps/RigidColliders -game -nullrhi -nosound -ChronoCompareQuit

    UnrealEditor-Cmd.exe F:\Workspace\UnrealChrono\UnrealChrono.uproject -run=pythonscript -script=F:\Workspace\UnrealChrono\Scripts\create_collider_level.py -unattended -nosplash

Delete Content/Maps/RigidColliders.umap first to rebuild it.
"""
import unreal

LEVEL = "/Game/Maps/RigidColliders"

EAL = unreal.EditorAssetLibrary


def log(msg):
    unreal.log("CreateColliderLevel: %s" % msg)


def spawn(cls, location, rotation=(0, 0, 0)):
    return unreal.EditorLevelLibrary.spawn_actor_from_class(cls, unreal.Vector(*location), unreal.Rotator(*rotation))


def main():
    EAL.make_directory("/Game/Maps")
    levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    if EAL.does_asset_exist(LEVEL):
        EAL.delete_asset(LEVEL)
    levels.new_level(LEVEL)
    world = unreal.EditorLevelLibrary.get_editor_world()

    spawn(unreal.DirectionalLight, (0, 0, 1000), (0, -45, 40)).set_actor_label("Sun")
    spawn(unreal.SkyAtmosphere, (0, 0, 0)).set_actor_label("SkyAtmosphere")
    sky = spawn(unreal.SkyLight, (0, 0, 500))
    sky.set_actor_label("SkyLight")
    sky.get_component_by_class(unreal.SkyLightComponent).set_editor_property("real_time_capture", True)

    # a visible floor under the arena (the physics ground is a Chrono body; this one is only for the eye)
    floor = spawn(unreal.StaticMeshActor, (0, 12000, -5))
    floor.set_actor_label("Floor")
    floor.set_actor_scale3d(unreal.Vector(60, 60, 0.1))
    floor.static_mesh_component.set_static_mesh(EAL.load_asset("/Engine/BasicShapes/Cube.Cube"))
    floor.static_mesh_component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)

    comparison = spawn(unreal.ChronoBackendComparison, (0, 0, 200))
    comparison.set_actor_label("ColliderComparison")
    comparison.set_editor_property("run_vehicle_tests", False)
    comparison.set_editor_property("run_rigid_body_tests", False)
    comparison.set_editor_property("run_collider_tests", True)

    world.get_world_settings().set_editor_property("default_game_mode", unreal.UnrealChronoGameMode)
    levels.save_current_level()
    log("saved %s" % LEVEL)


try:
    main()
except Exception as exc:
    import traceback
    log("FAILED: %s\n%s" % (exc, traceback.format_exc()))
