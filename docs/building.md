# Building from source

Modular Vehicles is a C++ GDExtension built with [godot-cpp](https://github.com/godotengine/godot-cpp)
and SCons. End users do not need to build anything: release zips on GitHub contain compiled
libraries for Windows, Linux and macOS. Build from source if you want to change the C++ code or
target a platform that is not shipped.

## Prerequisites

| Tool | Version | Notes |
|---|---|---|
| Git | any recent | with submodule support |
| Python | 3.10+ | |
| SCons | 4.4+ (4.8 recommended) | `python -m pip install --upgrade scons` |
| Windows: Visual Studio 2022 Build Tools | MSVC v143 | "Desktop development with C++" workload. SCons locates it automatically, no Developer Prompt needed |
| Linux: GCC or Clang | C++17 | `build-essential` |
| macOS: Xcode command line tools | | universal builds need Xcode 12+ |
| Godot | 4.4+ | to run the demo project and verify the build |

## Get the source

```
git clone --recurse-submodules https://github.com/MarcosBottenbley/modular-vehicles-for-godot.git
cd modular-vehicles-for-godot
```

If you already cloned without submodules:

```
git submodule update --init --recursive
```

The godot-cpp submodule is pinned to `godot-4.4.1-stable`, which is why the extension declares
`compatibility_minimum = "4.4"`. Newer Godot 4.x editors load it unchanged.

## Build

Run from the repository root. Pick the target you need:

| Target | Used by |
|---|---|
| `editor` | the Godot editor while you develop |
| `template_debug` | debug exports (and the editor if no editor build exists) |
| `template_release` | release exports |

```
scons platform=windows target=editor
scons platform=windows target=template_debug
scons platform=windows target=template_release
```

Linux and macOS:

```
scons platform=linux target=editor
scons platform=macos arch=universal target=editor
```

Useful options:

| Option | Effect |
|---|---|
| `-j8` | parallel compile |
| `dev_build=yes debug_symbols=yes` | asserts and symbols for debugging the extension |
| `compiledb=yes` | writes `compile_commands.json` for clangd / IDEs |
| `precision=double` | double-precision build (must match a double-precision Godot) |
| `-c` | clean the chosen target |

The first build compiles godot-cpp itself and takes several minutes. Set the `SCONS_CACHE`
environment variable to a folder outside the repo to share compiled godot-cpp objects between
targets and checkouts.

## Output

Libraries are written to `addons/modular_vehicles/bin/` with godot-cpp's naming scheme, for
example `libmodular_vehicles.windows.editor.x86_64.dll` or
`libmodular_vehicles.linux.template_release.x86_64.so`. macOS builds produce a `.framework`
folder. These names must match the `[libraries]` entries in
`addons/modular_vehicles/modular_vehicles.gdextension`; if you add a platform, add a key there.

`bin/` is gitignored. Never commit compiled libraries.

## Verify

Open the repository root as a Godot project, or run headless:

```
godot --headless --path . --quit
```

A successful load prints `Modular Vehicles <version> (<target>) loaded` in the Output panel or
terminal, and `MVVehicle` appears in the editor's Create Node dialog.

## Windows notes

- The `.gdextension` sets `reloadable = true`, so the editor can hot-reload after a rebuild.
  If the linker reports it cannot open the output file, close the editor or the running game.
- Use `Godot_v4.x_win64_console.exe` for headless commands so you can see the output.

## macOS notes

Builds are not code-signed. Users who download a release may need to clear the quarantine
attribute once: `xattr -dr com.apple.quarantine addons/modular_vehicles/bin`.
