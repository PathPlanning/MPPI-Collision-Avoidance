# Model Predictive Path Integral for Multi-Agent Collision Avoidance

## Decrtiption
<p align="center">
  <img src="img/mppi_demo.gif" width="500"/>
</p>


Implementation of distributed multi-agent collision avoidance algorithm based on the seminal sampling-based model predictive control algorithm, i.e., _MPPI_ [[1](https://homes.cs.washington.edu/~bboots/files/InformationTheoreticMPC.pdf)], that originally solves a single-agent problem. We enhance it by introducing safe distributions for the multi-agent setting that are derived from the _Optimal Reciprocal Collision Avoidance (ORCA)_ linear constraints [[2](https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf)], an established approach from the multi-agent navigation domain. 


This is supplementary code for paper "Model Predictive Path Integral for Decentralized Multi-Agent Collision Avoidance" which is under consideration in the PeerJ Computer Science journal. 


The pipeline of the proposed algorithm is presented on picture below. The red elements on scheme demonstrate the components that were added to MPPI procedure to adapt in for decentralized multi-agent scenario. Our approach introduсes a method for taking into account linear constraints in the sampling process. The corresponding component is marked with an asterisk on the scheme.

<p align="center">
  <img src="img/mppi_scheme.png" width="500"/>
</p>

_* Please note that this repository includes the implementation of the suggested method for differential-drive and car-like dynamics (these cases make it possible to reduce the problem of finding a new sampling distribution to a linear programming). In the future, it is planned to implement the full version of our method based on SOCP solver._

## Running the Implementation

Code is written in C++ and and includes a number of bindings to Python 3 (using [pybind11](https://github.com/pybind/pybind11) library).

To build and run the project you can use CMake, CMakeLists.txt file is available in the repo. Please note that the code relies on C++17 standard. Make sure that your compiler supports it. At the moment, the build and launch were tested only on Manjaro Linux using the GCC 13.2 C++ compiler and Python 3.11.

### Prerequisites

- CMake (3.20) and make (4.4.1)
- C++ compiler with C++17 standart support (e.g. GCC 13.2)
- Python 3.11 with next libraries:
    - numpy (1.26.3)
    - scipy (1.10.1)
    - matplotlib (3.7.1)
    - IPython (8.12)
- Jupyter Notebook (for visualization)


The repository also includes fetching of [or-tools](https://github.com/google/or-tools) (in `CMakeLists.txt` as `FetchContent`) and a number of submodules (in `external/` subdirectory)
- [xsimd](https://github.com/xtensor-stack/xsimd)
- [xtl](https://github.com/xtensor-stack/xtl)
- [xtensor](https://github.com/xtensor-stack/xtensor)
- [xtensor-blas](https://github.com/xtensor-stack/xtensor-blas)
- [pybind11](https://github.com/pybind/pybind11)
- [xtensor-python](https://github.com/xtensor-stack/xtensor-python)

It is also necessary to install all the dependencies (e.g. BLAS and LAPACK for [xtensor-blas](https://github.com/xtensor-stack/xtensor-blas)) required to build them.


### Download
Download current repository to your local machine. Use:
``` bash
 git clone --recurse-submodules git@github.com:PathPlanning/MPPI-Collision-Avoidance.git
```

### Build

Go to the directory with the project and create a subdirectory `build/release`. Go to the new subdirectory.

```bash
mkdir -p build/release
cd build/release
```

Use the CMake and make tools to build the project. You can use the `-j` flag to improve building peroformance, but this may cause errors in case of resource deficit (in this case, it is worth specifying the number of processor cores to build, e.g. `-j4`)

```bash
cmake ../../ -DCMAKE_BUILD_TYPE=Release 
make
```

or 

```bash
cmake ../../ -DCMAKE_BUILD_TYPE=Release 
make -j
```
### Launch an Example

Go to the example directory and run jupyter noterbook. After that choose `example.ipynb` file and run it.

```bash
cd ../../example
jupyter notebook
```

The notebook contains a circular scenario (see animation at the top), the size of the circle and the number of agents can be changed by updating variables `CIRC_R` and `AGENTS_NUM`.

```python
CIRC_R = 6 # size of the circle
AGENTS_NUM = 5 # number of agents
```


## References

1. [Williams, G., Wagener, N., Goldfain, B., Drews, P., Rehg, J. M., Boots, B., and Theodorou, E. A. (2017b). Information theoretic MPC for model-based reinforcement learning. In 2017 IEEE International Conference on Robotics and Automation (ICRA), pages 1714–1721](https://homes.cs.washington.edu/~bboots/files/InformationTheoreticMPC.pdf)
2. [Van Den Berg, J., Guy, S. J., Lin, M., and Manocha, D. (2011). Reciprocal n-body collision avoidance. In Robotics research, pages 3–19.](https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf)
