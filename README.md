### overview
This is a package to load point cloud data from file and detect defect.
It's used to detect defect on plane.
It's simplely calculate the plane equation and transform the plane parallel to XoY plane.
Then calculate normal and curve. outliers is seen as results.

Note:
1. Input file can be .pcd/.ply/.PCD/.PLY format.
2. It will transform data to mm unit, like all data is 50.86 level instead of 0.05086.
3. There are also many useful tools like ply2pcd and pcd2ply, used to converte format between point cloud files.
4. It's a good example of cmake project.

#### to run the code
```
mkdir build
cd build
cmake ..
make -j8
sudo make install
make clean

./bin/normalEstimate ../data/xxx.pcd
```