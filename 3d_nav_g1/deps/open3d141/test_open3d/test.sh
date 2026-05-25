export DISPLAY=10.11.106.85:0.0

cmake -DOpen3D_DIR="{YOURPATH}/open3d141/lib/cmake/Open3D" \
-S ./open3d-cmake-find-package/ -B ./build

cmake --build ./build/ --config Release

./build/Draw