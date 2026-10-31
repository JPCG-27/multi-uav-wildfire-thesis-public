"""Unreal Editor Python: import the Poly Haven flora downloaded by
Tools/download_flora.py into /Game/ThesisFlora, with auto-built materials
and Nanite enabled (the scans are high-poly; Nanite makes that free).

Run INSIDE the Unreal editor:
  Tools menu -> Execute Python Script... -> pick this file
  (or: Output Log -> cmd dropdown set to Python -> py "Tools/import_polyhaven.py")
Requires the "Python Editor Script Plugin" (enabled by default in UE 5.x).

After it finishes, assign in your ForestDresser actor:
  TreeMeshes:        SM_fir_sapling, SM_pine_sapling_small, SM_quiver_tree_02
  BushMeshes:        SM_searsia_lucida
  GroundCoverMeshes: SM_tree_stump_01, SM_boulder_01
"""

import json
import os
import unreal

RAW_DIR = os.path.join(unreal.SystemLibrary.get_project_directory(), "RawAssets", "polyhaven")
DEST_ROOT = "/Game/ThesisFlora"

ASSET_TOOLS = unreal.AssetToolsHelpers.get_asset_tools()
MEL = unreal.MaterialEditingLibrary

# Texture-map suffix -> (material property, sRGB, normal-map compression).
# Non-color masks stay sRGB-off and sample as Linear Color. UE 5.7 reports
# compile errors if roughness/alpha/mask samples keep the default Color sampler.
SUFFIX_RULES = {
    "diff":        ("base_color", True,  False),
    "nor_gl":      ("normal",     False, True),
    "rough":       ("roughness",  False, False),
    "alpha":       ("opacity",    False, False),
    "mask":        ("opacity",    False, False),
    "ao":          ("ao",         False, False),
}

BASE_COLOR_TINTS = {
    "boulder_01": (0.42, 0.42, 0.37),
    "rock_07": (0.40, 0.41, 0.36),
    "rock_09": (0.38, 0.39, 0.34),
    "rock_moss_set_01": (0.36, 0.42, 0.32),
    "rock_moss_set_02": (0.36, 0.42, 0.32),
}

LOW_SPECULAR_ASSETS = {
    "boulder_01",
    "rock_07",
    "rock_09",
    "rock_moss_set_01",
    "rock_moss_set_02",
}


def import_texture(file_path, dest_path):
    task = unreal.AssetImportTask()
    task.filename = file_path
    task.destination_path = dest_path
    task.automated = True
    task.save = True
    task.replace_existing = True
    ASSET_TOOLS.import_asset_tasks([task])
    paths = list(task.get_editor_property("imported_object_paths"))
    return unreal.EditorAssetLibrary.load_asset(paths[0]) if paths else None


def import_static_mesh(fbx_path, dest_path, asset_name):
    options = unreal.FbxImportUI()
    options.import_mesh = True
    options.import_as_skeletal = False
    options.import_animations = False
    options.import_materials = False     # we build cleaner ones ourselves
    options.import_textures = False
    options.static_mesh_import_data.combine_meshes = True
    options.static_mesh_import_data.generate_lightmap_u_vs = False
    options.static_mesh_import_data.auto_generate_collision = False

    task = unreal.AssetImportTask()
    task.filename = fbx_path
    task.destination_path = dest_path
    task.destination_name = "SM_" + asset_name
    task.automated = True
    task.save = True
    task.replace_existing = True
    task.options = options
    ASSET_TOOLS.import_asset_tasks([task])
    paths = list(task.get_editor_property("imported_object_paths"))
    return unreal.EditorAssetLibrary.load_asset(paths[0]) if paths else None


def build_material(asset_name, set_name, textures, dest_path):
    """textures: dict suffix -> Texture2D. Returns a masked two-sided material."""
    mat_name = "M_{}_{}".format(asset_name, set_name) if set_name else "M_" + asset_name
    mat_path = dest_path + "/" + mat_name
    if unreal.EditorAssetLibrary.does_asset_exist(mat_path):
        unreal.EditorAssetLibrary.delete_asset(mat_path)

    mat = ASSET_TOOLS.create_asset(mat_name, dest_path, unreal.Material, unreal.MaterialFactoryNew())
    if mat is None:
        return None

    mat.set_editor_property("two_sided", True)
    # Usage flags: required for HISM instancing and Nanite meshes; without them
    # the editor warns and packaged builds render the default material.
    mat.set_editor_property("used_with_instanced_static_meshes", True)
    try:
        mat.set_editor_property("used_with_nanite", True)
    except Exception:
        pass  # property name differs on some engine versions; editor auto-adds in PIE
    y = -300
    for suffix, tex in sorted(textures.items()):
        rule = SUFFIX_RULES.get(suffix)
        if rule is None or tex is None:
            continue
        prop, srgb, is_normal = rule
        tex.set_editor_property("srgb", srgb)
        if is_normal:
            tex.set_editor_property("compression_settings",
                                    unreal.TextureCompressionSettings.TC_NORMALMAP)
        unreal.EditorAssetLibrary.save_loaded_asset(tex)

        sample = MEL.create_material_expression(
            mat, unreal.MaterialExpressionTextureSample, -500, y)
        y += 280
        sample.texture = tex
        if is_normal:
            sample.sampler_type = unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL
        elif prop != "base_color":
            sample.sampler_type = unreal.MaterialSamplerType.SAMPLERTYPE_LINEAR_COLOR

        if prop == "base_color":
            tint = BASE_COLOR_TINTS.get(asset_name.lower())
            if tint:
                tint_node = MEL.create_material_expression(
                    mat, unreal.MaterialExpressionConstant3Vector, -250, y - 280)
                tint_node.set_editor_property(
                    "constant", unreal.LinearColor(tint[0], tint[1], tint[2], 1.0))
                mul = MEL.create_material_expression(
                    mat, unreal.MaterialExpressionMultiply, -40, y - 280)
                MEL.connect_material_expressions(sample, "RGB", mul, "A")
                MEL.connect_material_expressions(tint_node, "", mul, "B")
                MEL.connect_material_property(mul, "", unreal.MaterialProperty.MP_BASE_COLOR)
            else:
                MEL.connect_material_property(sample, "RGB", unreal.MaterialProperty.MP_BASE_COLOR)
        elif prop == "normal":
            MEL.connect_material_property(sample, "RGB", unreal.MaterialProperty.MP_NORMAL)
        elif prop == "roughness":
            MEL.connect_material_property(sample, "R", unreal.MaterialProperty.MP_ROUGHNESS)
        elif prop == "ao":
            MEL.connect_material_property(sample, "R", unreal.MaterialProperty.MP_AMBIENT_OCCLUSION)
        elif prop == "opacity":
            mat.set_editor_property("blend_mode", unreal.BlendMode.BLEND_MASKED)
            MEL.connect_material_property(sample, "R", unreal.MaterialProperty.MP_OPACITY_MASK)

    if asset_name.lower() in LOW_SPECULAR_ASSETS:
        spec = MEL.create_material_expression(mat, unreal.MaterialExpressionConstant, -40, y + 80)
        spec.set_editor_property("r", 0.12)
        MEL.connect_material_property(spec, "", unreal.MaterialProperty.MP_SPECULAR)

    MEL.recompile_material(mat)
    unreal.EditorAssetLibrary.save_loaded_asset(mat)
    return mat


def enable_nanite(mesh):
    ns = mesh.get_editor_property("nanite_settings")
    ns.enabled = True
    mesh.set_editor_property("nanite_settings", ns)


def main():
    manifest_path = os.path.join(RAW_DIR, "manifest.json")
    if not os.path.exists(manifest_path):
        unreal.log_error("No manifest at {} — run Tools/download_flora.py first.".format(manifest_path))
        return
    with open(manifest_path, "r", encoding="utf-8") as f:
        manifest = json.load(f)

    for asset, info in manifest.items():
        unreal.log("=== Importing {} ===".format(asset))
        adir = os.path.join(RAW_DIR, asset)
        dest = "{}/{}".format(DEST_ROOT, asset)

        # 1. Textures, grouped by material-set prefix ("twigs_diff" -> set "twigs").
        sets = {}
        for tex_file in info.get("textures", []):
            stem = os.path.splitext(tex_file)[0]
            matched = None
            for suffix in SUFFIX_RULES:
                if stem.endswith(suffix):
                    matched = suffix
                    break
            if not matched:
                continue
            set_name = stem[: -len(matched)].rstrip("_")
            tex = import_texture(os.path.join(adir, tex_file), dest + "/Textures")
            sets.setdefault(set_name, {})[matched] = tex

        # 2. Mesh.
        mesh = import_static_mesh(os.path.join(adir, info["fbx"]), dest, asset)
        if mesh is None:
            unreal.log_error("FBX import failed for {}".format(asset))
            continue

        # 3. Materials per texture set; assign to slots by name match.
        materials = {s: build_material(asset, s, texs, dest + "/Materials")
                     for s, texs in sets.items()}

        static_materials = list(mesh.get_editor_property("static_materials"))
        fallback = next(iter(materials.values()), None)
        for i, sm in enumerate(static_materials):
            slot = str(sm.material_slot_name).lower()
            chosen = None
            for set_name, m in materials.items():
                if set_name and (set_name.lower() in slot or slot in set_name.lower()):
                    chosen = m
                    break
            sm.material_interface = chosen or fallback
            static_materials[i] = sm
        mesh.set_editor_property("static_materials", static_materials)

        # 4. Nanite — makes the high-poly scans render cheaply, no LODs needed.
        try:
            enable_nanite(mesh)
        except Exception as e:
            unreal.log_warning("Nanite enable failed for {}: {}".format(asset, e))

        unreal.EditorAssetLibrary.save_loaded_asset(mesh)
        unreal.log("OK: {}/SM_{}".format(dest, asset))

    unreal.log("All done. Assign the SM_* meshes in your ForestDresser "
               "(TreeMeshes / BushMeshes / GroundCoverMeshes).")


if __name__ in ("__main__", "__builtin__", "builtins"):
    main()
