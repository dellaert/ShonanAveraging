# Shonan Rotation Averaging

Shonan rotation averaging takes pairwise rotation measurements and returns a set of absolute rotation matrices that are a global minimizer of the measurement error, measured by the Frobenius norm:

$$\alpha$$

You can try it here: 

# [GTSAM](https://gtsam.org)

This should get you started:
    conda create --name py369 python=3.6.9
    conda activate py369
    pip install gtsam
    pip install matplotlib

Clone this repo, cd to the gtsam subfolder, and run the test
    cd gtsam
    python test_ShonanAveraging.py 

You can then run the CLI using
    python ShonanAveragingCLI.py pose3example-grid.txt

The files follow the g2o coding convention, and should store SE(3) constraints rather than just SO(3). You can just store zero translations if you do not have them.

<!-- # [Ceres](http://ceres-solver.org/)

Install ceres as explained at [Ceres install page](http://ceres-solver.org/installation.html). On MacOS, I do

> brew install ceres-solver --HEAD -->


