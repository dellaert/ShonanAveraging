# Shonan Rotation Averaging

Shonan rotation averaging takes pairwise rotation measurements and returns a set of absolute rotation matrices that are a global minimizer of the measurement error, measured by the Frobenius norm. For more explanation, check out the corresponding [project website](https://dellaert.github.io/ShonanAveraging/index.html).

The code is now part of GTSAM 4.1, although we are planning a port to Ceres as well. The easiest way to get started is using the python wrapper interface in GTSAM, instructions below.

## [GTSAM in Colab Notebook]

The super-easy way to get run the code is via this [Colab notebook](https://github.com/dellaert/ShonanAveraging/blob/master/gtsam/ShonanAveraging.ipynb).

## [GTSAM](https://gtsam.org)

We have currently tested on Linux and MacOS. To get started on a **Linux** machine, python 3.6.9 is currently supported:
```bash
    conda create --name py369 python=3.6.9
    conda activate py369
```

On **MacOS**, we are tracking the homebrew version, which is 3.8:
```bash
    conda create --name py38 python=3.8
    conda activate py38
```

From that point on, everything should be the same.First, install matplotlib and gtsam:
```bash
    conda install matplotlib
    pip install gtsam
```
Clone this repo, cd to the `gtsam` subfolder, and run the test
```bash
    cd gtsam
    python test_ShonanAveraging.py 
```
You can then run the CLI using
```bash
    python ShonanAveragingCLI.py -i pose3example-grid.txt
```
The files follow the g2o coding convention, and should store SE(3) constraints rather than just SO(3). You can just store zero translations if you do not have them. 

The constraints are most conveniently given using `EDGE_SE3:QUAT` lines:
```
EDGE_SE3:QUAT i j x y z qx qy qz qw + 21 entries of 6*6 upper triangular covariance matrix
```

Optionally, you can give an initial estimate using `VERTEX` lines, but this is currently ignored:
```
VERTEX_SE3:QUAT i x y z qx qy qz qw
```

Here is a snippet from the example file `pose3example-grid.txt`:
```
EDGE_SE3:QUAT 0 1 1.00497 0.002077 -0.015539 -0.508004 0.250433 0.711222 -0.416386 2500 0 0 0 0 0 2500 0 0 0 0 2500 0 0 0 400 0 0 400 0 400
EDGE_SE3:QUAT 1 2 -0.200593 0.339956 -0.908079 -0.093598 0.151993 0.42829 0.885836 2500 0 0 0 0 0 2500 0 0 0 0 2500 0 0 0 400 0 0 400 0 400
...
VERTEX_SE3:QUAT 0 1.63791e-12 7.56548e-14 -3.02811e-12 5.35657e-13 2.43616e-13 9.71152e-14 1
VERTEX_SE3:QUAT 1 1.01609 0.00274307 -0.0351514 -0.499545 0.247735 0.723569 -0.406854
VERTEX_SE3:QUAT 2 1.99996 0.0304956 -0.040662 0.403501 -0.294714 -0.4254 0.754563
...
```


<!-- # [Ceres](http://ceres-solver.org/)

Install ceres as explained at [Ceres install page](http://ceres-solver.org/installation.html). On MacOS, I do

> brew install ceres-solver --HEAD -->


