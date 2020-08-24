---
layout: page
title: Shonan Rotation Averaging
permalink: /
---

<div class="container-fluid">
<div id="header"></div>
<div id="media"></div>
</div>

#### Abstract

Shonan Rotation Averaging is a fast, simple, and elegant rotation averaging algorithm that is guaranteed to recover globally optimal solutions under mild assumptions on the measurement noise. Our method employs semidefinite relaxation in order to recover provably globally optimal solutions of the rotation averaging problem. In contrast to prior work, we show how to solve large-scale instances of these relaxations using manifold minimization on (only slightly) higher-dimensional rotation manifolds, re-using existing high-performance (but local) structure-from-motion pipelines. Our method thus preserves the speed and scalability of current SFM methods, while recovering globally optimal solutions.

#### Colab Notebook

Try it out quickly with this [minimal Colab notebook](https://github.com/dellaert/ShonanAveraging/blob/master/gtsam/ShonanAveraging.ipynb).

#### Spotlight Video

Below is the ECCV 2020 spotlight video, narrated by Frank, which gives a flavor for the method using "small world graphs", as inspired by the [paper by Kyle Wilson and David Bindel](https://arxiv.org/abs/2003.08310).  He also gives an overview of the results on real-world datasets, as well as the intuition behind the mathematical underpinnings of Shonan Averaging.

<div class="container-fluid">
<div  class="py-3">
<div id="video"></div>
</div>
</div>

<h4 class="pt-3">Small World Datasets</h4>

You can play around with the small world examples yourself, below. you can select graphs with either 20 or 40 nodes, where each node is connected to either 4, 8, or 16 neighboring nodes. The grid below will change to show the "Sankey diagrams" that illustrate how Shonan averaging navigates between saddle points of the objective function for different values of p, as in *SO(p)*. The grid shows the effect of measurements noise (columns correspond to 2, 5, 10, 25, and 50 degrees of added angular noise) and graph connectivity (varying the small world parameter p from 0.0 to 1.0).
<div id="matrix" class="pt-3"></div>

Above we always show the most difficult case, corresponding to very little connectivity (p=0.0) and a high degree of noise (50 degrees). The Sankey diagram, as Frank explains in the vide, shows what fraction of the 100 initial estimates terminate at a given SO(p) level. When terminating, the objective value is always the same, showing the method eventually converges to the global optimum in all cases.

<!-- #### Exploring The Prior

<div id="priors"></div>

#### Results
Pivot table with results:

<div id="pivot"></div> -->
