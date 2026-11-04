#!/usr/bin/env python
"""Build script for the Modular Vehicles GDExtension.

Usage (from the repository root):
    scons platform=windows target=editor            # editor build for development
    scons platform=windows target=template_debug    # debug export template build
    scons platform=windows target=template_release  # release export template build
    scons tests                                     # unit tests -> bin/tests/mv_tests(.exe)

godot-cpp lives in thirdparty/godot-cpp (git submodule). Compiled libraries land in
addons/modular_vehicles/bin/ and are referenced by modular_vehicles.gdextension.
"""

import os

MV_VERSION = "0.1.0"

# ---------------------------------------------------------------------------------------------
# Unit tests (doctest). `scons tests` compiles tests/unit/*.cpp with a plain SCons environment:
# only src/ (the std-only math headers) and thirdparty/doctest are on the include path and
# godot-cpp is never configured, compiled or linked. `precision=double` defines
# MV_DOUBLE_PRECISION exactly like the library build does. See docs/testing.md.
# ---------------------------------------------------------------------------------------------
test_env = Environment()
test_env.Append(CPPPATH=["src/", "thirdparty/doctest/"])
if ARGUMENTS.get("precision", "single") == "double":
    test_env.Append(CPPDEFINES=["MV_DOUBLE_PRECISION"])
if "msvc" in test_env["TOOLS"]:
    test_env.Append(CXXFLAGS=["/std:c++17", "/EHsc", "/W4", "/permissive-", "/utf-8"])
else:
    test_env.Append(CXXFLAGS=["-std=c++17", "-Wall", "-Wextra", "-Wpedantic"])

test_objects = [
    test_env.Object(target="bin/tests/obj/" + os.path.splitext(os.path.basename(str(src)))[0], source=src)
    for src in Glob("tests/unit/*.cpp")
]
test_program = test_env.Program(target="bin/tests/mv_tests", source=test_objects)
test_env.Alias("tests", test_program)

# When only the tests are requested, skip godot-cpp entirely (no submodule or platform needed).
if COMMAND_LINE_TARGETS == ["tests"]:
    Return()

# ---------------------------------------------------------------------------------------------
# GDExtension library
# ---------------------------------------------------------------------------------------------
env = SConscript("thirdparty/godot-cpp/SConstruct")

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
