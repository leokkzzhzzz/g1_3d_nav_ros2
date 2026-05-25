# open3d141 — pre-built Open3D 0.14.1 dependency

This directory contains the **headers** and **CMake configuration** for Open3D
0.14.1, used by `open3d_loc` for ICP-based global localization.

## What's in git

- `include/` — Open3D + 3rdparty C/C++ headers (~14 MB)
- `lib/cmake/Open3D/` — CMake config so downstream packages can find Open3D
- `install_deps.sh`, `readme.md` — original deepglint scripts
- `test_open3d/` — minimal CMake sanity test

## What's NOT in git

`lib/*.a` — the actual compiled static libraries (562 MB total, including a
single 495 MB `libOpen3D_3rdparty_assimp_assimp.a` that exceeds GitHub's
100 MB single-file limit).

## How to obtain the missing libraries

You need the full `lib/` directory to build `open3d_loc`. Three options:

### Option 1 — Use the pre-built `g1_nav_final:latest` image (easiest)

The Docker image already has everything baked in at `/root/3d_nav_g1/deps/open3d141/lib/`.
Just run inside the container; no separate Open3D install needed.

```bash
docker pull us-central1-docker.pkg.dev/dreamcontroltrain/g1-nav/3d_nav_g1:latest
```

### Option 2 — Build Open3D 0.14.1 from source

Open3D 0.14.1 from upstream:

```bash
git clone https://github.com/isl-org/Open3D.git
cd Open3D && git checkout v0.14.1
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$(pwd)/../../open3d141 \
      -DBUILD_SHARED_LIBS=OFF \
      -DBUILD_PYTHON_MODULE=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_UNIT_TESTS=OFF \
      ..
make -j$(nproc)
make install
```

This takes **~2 hours** on a Jetson Orin NX.

### Option 3 — Copy `lib/` from a teammate's machine (fastest if available)

```bash
# On a machine that has the libs:
tar -czf open3d141_lib.tar.gz -C 3d_nav_g1/deps/open3d141 lib
# Transfer (562 MB → ~200 MB compressed)
scp open3d141_lib.tar.gz target:/tmp/

# On target:
cd 3d_nav_g1/deps/open3d141
tar -xzf /tmp/open3d141_lib.tar.gz   # restores lib/
```

## Why is this not just an upstream Open3D apt package?

`open3d_loc` was built against Open3D 0.14.1 with `BUILD_SHARED_LIBS=OFF`, which
is not the default and not what apt provides. Static-linking against a known
ABI prevents version skew between deepglint code and Open3D releases. Switching
to system Open3D would require source-level changes in `open3d_loc`.
