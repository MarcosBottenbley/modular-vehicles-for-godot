# doctest (vendored)

Single-header C++ unit-testing framework used by the `scons tests` target (`tests/unit/`).

| | |
|---|---|
| Version | 2.4.11 |
| File | `doctest.h` (the single-header release artifact, unmodified) |
| Source | https://github.com/doctest/doctest/releases/download/v2.4.11/doctest.h |
| Release page | https://github.com/doctest/doctest/releases/tag/v2.4.11 |
| SHA-256 | `44faa038e9c3f9728efbda143748d01124ea0a27f4bf78f35a15d8fab2e039fb` |
| License | MIT (Copyright (c) 2016-2023 Viktor Kirilov), see the header of `doctest.h` |

To upgrade: download the new `doctest.h` from the matching GitHub release, replace this file,
update the table above and run `scons tests`.

This folder is only on the include path of the unit-test binary; the GDExtension library never
includes it.
