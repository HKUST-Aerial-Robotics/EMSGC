# EMSGC
This repository maintains the implementation of the paper "Event-based Motion Segmentation withSpatio-Temporal Graph Cuts".

### Related Publications

* **[Event-based Motion Segmentation with Spatio-Temporal Graph Cuts](https://arxiv.org/pdf/2012.08730.pdf)**, 
*Yi Zhou, Guillermo Gallego, Xiuyuan Lu, Siqi Liu and Shaojie Shen*, submitted to IEEE 
Transactions on Neural Network and Learning Systems (T-NNLS) 2021.

* **[Event-based Motion Segmentation by Cascaded Two-Level Multi-Model Fitting]()**, 
*Event-based  Motion  Segmentation  by  CascadedTwo-Level  Multi-Model  Fitting*, IROS 2021.


# 1. Installation
TODO

# 2. Usage
TODO

# 3. Parameters

- width, height: Image size
- num_events_per_image: The number of events involved in the ST volume
- contrast_measure: 0 -> VARIANCE_CONTRAST, 1 -> MEAN_SQUARE_CONTRAST
- use_polarity: Set "False" to ignore polarity (Only False is supported in this release)
- gaussian_smoothing_sigma: Sigma of the Gaussian for replacing the the Dirac function (see Eq.(3) and (4))
- verbosity: Set "1" for detailed output
- timestamp_begin: Starting timestamp
- timestamp_end: Ending timestamp
- timestamp_step: Temporal step
- DisplayResult: Set "True" for saving results to local files.
- SaveResult: Set "True" for displaying result with OpenCV

### MRF Parameters
- LAMBDA_data: Coefficient of data term
- LAMBDA_smooth: Coefficient of Potts model
- LAMBDA_label: Coefficient of MDL term
- NUM_ITER_SEGMENTATION: Number of interation for continuous update
- NUM_ITER_LABELING: Number of interation for discrete update (alpha-expension graph cut)

### Initialization
- HybridModelCombination: 0->Only2D, 1->Only3D, 2->Only4D, 3->Both2Dand3D, 4->Both2Dand4D, 5->Both3Dand4D, 6->All2D3D4D
- BracketRow, BracketCol: Number of divisions in each dimension.
- Affine_trans2d_max: Maximum translation
- Affine_scale_max: Maximum scale
- Affine_theta_max: Maximum theta
- division_exponent: Times of binary search.

# 5. Datasets
Data can be downloaded from the [here](https://drive.google.com/drive/folders/1KB4oUOQcPF9v1u9GEaCMowPkRHyVxodc?usp=sharing).

