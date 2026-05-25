# depends
```bash
sh install_deps.sh
```

# test
if you are working with docker or a server without display, please modify the IP(you local IP) in test_open3d/test.sh for DISPLAY. Otherwise, please comment it out 
```bash
# for example
export DISPLAY=10.11.106.85:0.0
```

then modify the Open3D_DIR in test_open3d/test.sh with your current dir

```bash
cd test_open3d
sh test.sh
```
