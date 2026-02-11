RA-L Supplementary Material
---------------------------

.. .. note::
..    This page serves as supplementary website for our RA-L submission.

System Pipeline
===============

.. figure:: images/pipeline.png
   :width: 100%
   :alt: System Pipeline Schematic

   **Fig 1:** Pipeline of the LiDAR-inertial odometry system.

The motion model computes average acceleration :math:`\bar{\mathbf{a}}` and angular velocity :math:`\bar{\boldsymbol{\omega}}` for the window of IMU measurements between two LiDAR scans. These control inputs are used to deskew the incoming LiDAR scan :math:`\mathcal{P}` into the deskewed scan :math:`\mathcal{P}^*`.

The deskewed scan is first downsampled to :math:`\mathcal{P}^*_{merge}` (which is later used to update the map). An optional second downsampling step produces :math:`\mathcal{Q}`; if this second step is disabled, :math:`\mathcal{Q} = \mathcal{P}^*_{merge}`.

For registration, :math:`\mathcal{Q}` is matched against the VDB-based map by computing data associations via a nearest neighbor search with an association threshold. The motion model provides the initial guess for the first iteration of the optimization, while subsequent iterations use the current best estimate.

The iterative optimization minimizes a joint cost: an ICP cost from the associations, and an orientation regularization cost. The final estimated pose :math:`\mathbf{T}` (after convergence) is used to transform :math:`\mathcal{P}^*_{merge}` and update the map.

Trajectory Plots
================

In this section, we present trajectory plots using ``evo_ape`` (Absolute Pose Error). The trajectories are colored based on error magnitude, where "hotter" colors (reds/yellows) indicate higher error.

Oxford Spires (Radcliffe)
^^^^^^^^^^^^^^^^^^^^^^^^^

.. figure:: images/oxfs_radcliffe_02_no_ar.png
   :width: 100%
   :alt: Oxford Spires Radcliffe - No AR

   **Fig 2:** Odometry result when adaptive regularization is disabled (``-AR`` ablation in the paper) on Oxford Spires Radcliffe sequence.

.. figure:: images/oxfs_radcliffe_02_full.png
   :width: 100%
   :alt: Oxford Spires Radcliffe - Full System

   **Fig 3:** Full odometry system result (``Ours`` ablation in the paper) on Oxford Spires Radcliffe sequence.

Shown above are two trajectory results when ablating the performance of the odometry system. The color scale represents the APE error. Please note that the range of error (color bar scale) is different in both plots; the full system trajectory has a lower total error than the no-regularization version. From this comparison, it can be seen that the full system (``Ours``) performs better on this sequence.

Car Dataset (Urban)
^^^^^^^^^^^^^^^^^^^

.. figure:: images/ipbcar_urban_no_ar.png
   :width: 100%
   :alt: Car Urban - No AR

   **Fig 4:** Odometry result when adaptive regularization is disabled (``-AR`` ablation in the paper) on Car (Urban) sequence.

.. figure:: images/ipbcar_urban_full.png
   :width: 100%
   :alt: Car Urban - Full System

   **Fig 5:** Full odometry system result (``Ours`` ablation in the paper) on Car (Urban) sequence.

Similar to the Oxford Spires results, we once again see the full system performs (``Ours``) better than the ablated version (``-AR``).

Car Dataset (Rural)
^^^^^^^^^^^^^^^^^^^

.. figure:: images/ipbcar_rural_comp_xy_multiple_methods.png
   :width: 100%
   :alt: Car Rural - Multiple Methods Comparison

   **Fig 8:** Trajectory comparison of multiple methods on the Car (Rural) sequence.

Here we plot the full XY trajectory results from all the methods which successfully ran on the rural sequence. The trajectories have been aligned (using Umeyama alignment) to the reference trajectory, which is defined in a GPS coordinate frame. Each approach experiences significant drift on this 50 km long sequence, and as a consequence of the alignment process, the trajectories appear well-aligned near the middle of the sequence but drift at the ends. Nevertheless, we see that our approach (red) is the one that remains closest to the reference trajectory overall.
