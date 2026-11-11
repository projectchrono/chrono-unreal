r"""
Builds /Game/Maps/BackendComparison: the level of the Bullet vs Chaos comparison test.

It contains the geometry the test car drives over (a long asphalt slab, a ramp and two bumps, all primitive shapes tagged
for the Bullet export) and an AChronoBackendComparison actor that runs the whole test when the level is played.

    UnrealEditor-Cmd.exe F:\Workspace\UnrealChrono\UnrealChrono.uproject -run=pythonscript -script=F:\Workspace\UnrealChrono\Scripts\create_comparison_level.py -unattended -nosplash

Delete Content/Maps/BackendComparison.umap first to rebuild it.
"""
import unreal

LEVEL = "/Game/Maps/BackendComparison"
CONTENT_DIR = "/Game/TerrainDeform"

EAL = unreal.EditorAssetLibrary
TOOLS = unreal.AssetToolsHelpers.get_asset_tools()


def log(msg):
    unreal.log("CreateComparisonLevel: %s" % msg)


def physical_material(name, terrain_type, friction):
    path = "%s/%s" % (CONTENT_DIR, name)
    if EAL.does_asset_exist(path):
        pm = EAL.load_asset(path)
    else:
        EAL.make_directory(CONTENT_DIR)
        pm = TOOLS.create_asset(name, CONTENT_DIR, unreal.VehiclePhysicalMaterial, unreal.PhysicalMaterialFactoryNew())
    pm.set_editor_property("material_type", terrain_type)
    pm.set_editor_property("friction", friction)
    EAL.save_loaded_asset(pm)
    return pm


def spawn(cls, location, rotation=(0, 0, 0)):
    return unreal.EditorLevelLibrary.spawn_actor_from_class(cls, unreal.Vector(*location), unreal.Rotator(*rotation))


def box(label, location_cm, size_m, material, rotation=(0, 0, 0)):
    """Static box from the engine cube (100 cm), size in metres. rotation = (roll, pitch, yaw)."""
    actor = spawn(unreal.StaticMeshActor, location_cm, rotation)
    actor.set_actor_label(label)
    actor.set_actor_scale3d(unreal.Vector(*size_m))
    comp = actor.static_mesh_component
    comp.set_static_mesh(EAL.load_asset("/Engine/BasicShapes/Cube.Cube"))
    body = comp.get_editor_property("body_instance")
    body.set_editor_property("phys_material_override", material)
    comp.set_editor_property("body_instance", body)
    # The Bullet backend only exports geometry whose actor AND component carry this tag
    actor.tags = [unreal.Name("ChronoStatic")]
    comp.component_tags = [unreal.Name("ChronoStatic")]
    return actor


def main():
    EAL.make_directory("/Game/Maps")
    asphalt = physical_material("PM_Asphalt", unreal.TerrainType.ASPHALT, 0.9)

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

    # ---- the geometry both backends see: x is the driving direction, the car starts at the origin ----------------------
    box("Ground", (25000, 0, -50), (600, 60, 1), asphalt)                                      # top at z = 0, x -50 .. 550 m
    box("Ramp", (1200, 0, 4), (8, 6, 0.2), asphalt, rotation=(0, 1.5, 0))                       # 1.5 degree ramp, 21 cm drop at its end
    box("Bump_1", (3000, 0, 4), (0.4, 6, 0.08), asphalt)                                       # 8 cm speed bumps
    box("Bump_2", (3500, 0, 4), (0.4, 6, 0.08), asphalt)

    comparison = spawn(unreal.ChronoBackendComparison, (0, 0, 200))
    comparison.set_actor_label("BackendComparison")

    world.get_world_settings().set_editor_property("default_game_mode", unreal.UnrealChronoGameMode)
    levels.save_current_level()
    log("saved %s" % LEVEL)


try:
    main()
except Exception as exc:
    import traceback
    log("FAILED: %s\n%s" % (exc, traceback.format_exc()))
