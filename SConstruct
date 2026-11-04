#!/usr/bin/env python
"""Build script for the Modular Vehicles GDExtension.

Usage (from the repository root):
    scons platform=windows target=editor            # editor build for development
    scons platform=windows target=template_debug    # debug export template build
    scons platform=windows target=template_release  # release export template build

godot-cpp lives in thirdparty/godot-cpp (git submodule). Compiled libraries land in
addons/modular_vehicles/bin/ and are referenced by modular_vehicles.gdextension.
"""

env = SConscript("thirdparty/godot-cpp/SConstruct")

MV_VERSION = "0.1.0"

env.Append(CPPPATH=["src/"])
env.Append(
    CPPDEFINES=[
        ("MV_VERSION", '\\"%s\\"' % MV_VERSION),
        ("MV_BUILD_TARGET", '\\"%s\\"' % env["target"]),
    ]
)
if env["precision"] == "double":
    env.Append(CPPDEFINES=["MV_DOUBLE_PRECISION"])

sources = Glob("src/*.cpp") + Glob("src/nodes/*.cpp") + Glob("src/resources/*.cpp")
if env["target"] == "editor":
    sources += Glob("src/editor/*.cpp")

# In-editor class reference (doc_classes/*.xml) is compiled into editor and debug builds.
doc_sources = Glob("doc_classes/*.xml")
if doc_sources and env["target"] in ["editor", "template_debug"]:
    sources.append(env.GodotCPPDocData("src/gen/doc_data.gen.cpp", source=doc_sources))

libname = "libmodular_vehicles"
bindir = "addons/modular_vehicles/bin"

if env["platform"] == "macos":
    library = env.SharedLibrary(
        "{0}/{1}.{2}.{3}.framework/{1}.{2}.{3}".format(bindir, libname, env["platform"], env["target"]),
        source=sources,
    )
else:
    library = env.SharedLibrary(
        "{}/{}{}{}".format(bindir, libname, env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(library)
