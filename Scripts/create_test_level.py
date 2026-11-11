r"""
Builds /Game/Maps/TerrainDeformTest: a Nanite landscape that deforms under the wheels of the primitive test car,
plus a few primitive shapes (road slab, ramp, boxes, a sphere and a cylinder) for the car to drive over.

Run it headless (the editor must not be open on this project):

    UnrealEditor-Cmd.exe F:\Workspace\UnrealChrono\UnrealChrono.uproject -run=pythonscript -script=F:\Workspace\UnrealChrono\Scripts\create_test_level.py -unattended -nosplash

or from the editor's Python console / Tools > Execute Python Script.

The terrain deformation assets (/Game/TerrainDeform: render targets, parameter collection, tread texture, material function and
the landscape material) are shipped with the project; this script only creates the physical materials, the level and the
actors. To rebuild the level delete Content/Maps/TerrainDeformTest.umap first, then run it again.
"""
import math
import unreal

LEVEL = "/Game/Maps/TerrainDeformTest"
CONTENT_DIR = "/Game/TerrainDeform"

COMPONENTS = 8              # landscape components per side (126 quads each) -> 1008 quads
QUAD_SIZE_CM = 100.0        # 1 m per quad -> a 1 km square
Z_SCALE = 100.0
MAX_DISPLACEMENT_CM = 50.0  # world cm of deformation that maps to the whole 0..1 range of the material's Displacement pin

EAL = unreal.EditorAssetLibrary
MEL = unreal.MaterialEditingLibrary
TOOLS = unreal.AssetToolsHelpers.get_asset_tools()


def log(msg):
    unreal.log("CreateTestLevel: %s" % msg)


def make_physical_material(name, terrain_type, friction):
    path = "%s/%s" % (CONTENT_DIR, name)
    if EAL.does_asset_exist(path):
        pm = EAL.load_asset(path)
    else:
        pm = TOOLS.create_asset(name, CONTENT_DIR, unreal.VehiclePhysicalMaterial, unreal.PhysicalMaterialFactoryNew())
    pm.set_editor_property("material_type", terrain_type)
    pm.set_editor_property("friction", friction)
    EAL.save_loaded_asset(pm)
    return pm


def spawn(cls, location, rotation=(0, 0, 0)):
    return unreal.EditorLevelLibrary.spawn_actor_from_class(cls, unreal.Vector(*location), unreal.Rotator(*rotation))


def spawn_primitive(label, mesh_path, location, scale, rotation=(0, 0, 0), phys_material=None, color=None):
    """A static mesh actor made from one of the engine's basic shapes. rotation = (roll, pitch, yaw)."""
    actor = spawn(unreal.StaticMeshActor, location, rotation)
    actor.set_actor_label(label)
    actor.set_actor_scale3d(unreal.Vector(*scale))
    comp = actor.static_mesh_component
    comp.set_static_mesh(EAL.load_asset(mesh_path))
    if phys_material:
        body = comp.get_editor_property("body_instance")
        body.set_editor_property("phys_material_override", phys_material)
        comp.set_editor_property("body_instance", body)
    # The Bullet backend only exports geometry whose actor AND component carry this tag
    actor.tags = [unreal.Name("ChronoStatic")]
    comp.component_tags = [unreal.Name("ChronoStatic")]
    return actor


def main():
    EAL.make_directory(CONTENT_DIR)
    EAL.make_directory("/Game/Maps")

    sand = make_physical_material("PM_DeformSand", unreal.TerrainType.SAND, 0.6)
    asphalt = make_physical_material("PM_Asphalt", unreal.TerrainType.ASPHALT, 0.9)
    rock = make_physical_material("PM_Rock", unreal.TerrainType.ROCK, 0.85)

    # Landscape material: the displacement magnitude is in the landscape's LOCAL space, so divide by the Z scale
    mat = EAL.load_asset("%s/M_TerrainDeformLandscape" % CONTENT_DIR)
    mat.set_editor_property("enable_tessellation", True)
    scaling = mat.get_editor_property("displacement_scaling")
    scaling.set_editor_property("magnitude", MAX_DISPLACEMENT_CM / Z_SCALE)
    scaling.set_editor_property("center", 0.5)
    mat.set_editor_property("displacement_scaling", scaling)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)

    levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    if EAL.does_asset_exist(LEVEL):
        EAL.delete_asset(LEVEL)
    levels.new_level(LEVEL)
    world = unreal.EditorLevelLibrary.get_editor_world()

    # ---- lighting ------------------------------------------------------------------------------------------------
    sun = spawn(unreal.DirectionalLight, (0, 0, 1000), (0, -45, 40))
    sun.set_actor_label("Sun")
    spawn(unreal.SkyAtmosphere, (0, 0, 0)).set_actor_label("SkyAtmosphere")
    sky_light = spawn(unreal.SkyLight, (0, 0, 500))
    sky_light.set_actor_label("SkyLight")
    sky_light.get_component_by_class(unreal.SkyLightComponent).set_editor_property("real_time_capture", True)
    spawn(unreal.ExponentialHeightFog, (0, 0, 0)).set_actor_label("HeightFog")

    # ---- deformable landscape ------------------------------------------------------------------------------------
    landscape = unreal.TestLevelBuilder.create_test_landscape(world, mat, sand, COMPONENTS, QUAD_SIZE_CM, Z_SCALE, 150.0, 10000.0)
    if not landscape:
        raise RuntimeError("could not create the landscape")
    landscape.set_editor_property("enable_nanite", True)
    landscape.tags = [unreal.Name("ChronoStatic")]   # exported by the Bullet backend (the Chaos backend uses the level directly)
    # Virtual shadow maps cache pages and only invalidate them when geometry moves; displacement driven by a texture does
    # not count, so stale cached shadows appear as dark patches unless the landscape bypasses the cache.
    for comp in landscape.get_components_by_class(unreal.LandscapeComponent):
        comp.set_editor_property("shadow_cache_invalidation_behavior", unreal.ShadowCacheInvalidationBehavior.ALWAYS)

    cube = "/Engine/BasicShapes/Cube.Cube"
    sphere = "/Engine/BasicShapes/Sphere.Sphere"
    cylinder = "/Engine/BasicShapes/Cylinder.Cylinder"

    # ---- primitive shapes ----------------------------------------------------------------------------------------
    # Engine basic shapes are 100 cm across, so the scale is the size in metres.
    # Hard road slab (does not deform): 40 m x 8 m, 20 cm above the sand, 30 m ahead of the start
    spawn_primitive("RoadSlab", cube, (5500, 0, 0), (40, 8, 0.4), phys_material=asphalt)
    # Gentle ramp (2 degrees) from the sand up onto the slab
    spawn_primitive("Ramp", cube, (3200, 0, 10), (6, 8, 0.3), rotation=(0, 2.0, 0), phys_material=asphalt)
    # Steeper ramp (10 degrees) off to the side to jump from
    spawn_primitive("JumpRamp", cube, (1500, 2500, 60), (8, 5, 0.3), rotation=(0, 10, 0), phys_material=rock)
    # Boxes, a sphere and a cylinder to bump into
    for i in range(4):
        spawn_primitive("Box_%d" % i, cube, (1500 + i * 400, -600 + (i % 2) * 1200, 50), (1, 1, 1), rotation=(0, 0, 25 * i), phys_material=rock)
    spawn_primitive("Boulder", sphere, (1200, 900, 100), (2, 2, 2), phys_material=rock)
    spawn_primitive("Pillar", cylinder, (1200, -900, 150), (1, 1, 3), phys_material=rock)

    # ---- vehicle + physics world ---------------------------------------------------------------------------------
    car = spawn(unreal.PhysicsVehicle, (0, 0, 60))
    car.set_actor_label("TestCar")
    phys_world = spawn(unreal.CarPhysicsWorld, (0, 0, 0))
    phys_world.set_actor_label("CarPhysicsWorld")
    phys_world.set_editor_property("physics_vehicle", car)
    phys_world.set_editor_property("collision_backend", unreal.VehicleCollisionBackend.CHAOS)

    world.get_world_settings().set_editor_property("default_game_mode", unreal.UnrealChronoGameMode)

    levels.save_current_level()
    log("saved %s" % LEVEL)


try:
    main()
except Exception as exc:
    import traceback
    log("FAILED: %s\n%s" % (exc, traceback.format_exc()))
