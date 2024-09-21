<<<<<<< HEAD
<div align="center">
    <h1>LiLoc</h1>
    <br />
    <a href=https://www.youtube.com/watch?v=txBE5ZRduEw>🎬Youtube</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://www.bilibili.com/video/BV1uatkeFEWL/?vd_source=7936e3be9727382a31661ae25224c8ad>🎬Bilibili</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/YixFeng/Block-Map-Based-Localization/blob/main/README.md#Installation">🛠️Installation</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://arxiv.org/abs/2409.10172>📑Paper</a>
  <br />
  <br />
</div>

![BMLoc_cover](doc/liloc.png)

In this work, we propose a versatile graph-based lifelong localization framework, <strong>LiLoc</strong> , which enhances its timeliness by maintaining a single central session while improves the accuracy through multi-modal factors between the central and subsidiary sessions. The main contributions are as follows:

- A graph-based framework for long-term localization featuring a flexible mode-switching mechanism, to achieve accurate multi-session localization.
- An adaptive submap joining strategy to dynamically manage (i.e., generate, select and update) prior submaps, reducing system memory consumption while ensuring the timeliness of prior knowledge.
- An egocentric factor graph (EFG) module to tightly couple multi-modal constrains, along with a novel propagation model to enhance  prior constrains by distributing weighted scan matching factors in joint factor-graph optimization (JFGO). 
- We achieve the competitive performance on public and custom datasets and the proposed system will be released for community use.


***
## Installation
### 1. Prerequisites
#### 1.1 System and third-party packages
- Ubuntu $\geq$ 18.04 (tested on Ubuntu 20.04)

- PCL $\geq$ 1.8 (tested on PCL 1.10)

- OpenCV $\geq$ 4.0 (tested on OpenCV 4.2)

- GTSAM $\geq$ 4.0.0 (tested on GTSAM 4.0.2)

#### 1.2 Other Packages
- [ndt_omp](https://github.com/koide3/ndt_omp)
- [better_fastlio2](https://github.com/Yixin-F/better_fastlio2) (Refer to the module of "pose initialization" in this open-source repositories, the new reconstructed code is comming soon.)

### 2. Build
```bash
cd <your workspace>/src
git clone https://github.com/koide3/ndt_omp
git clone https://github.com/YixFeng/Block-Map-Based-Localization

cd ..
catkin_make
source devel/setup.bash
```

## Run
We provide some [Block Maps](https://drive.google.com/file/d/1Z2K56jTkMOouZhM4c9JhPvqyDxGFiXSY/view?usp=drive_link) (Google Drive) to make your tests easy. 

- **M2DGR street_01**
  
![m2dgr_street_01](figs/m2dgr_street_01.png)

- **M2DGR street_02**

![m2dgr_street_02](figs/m2dgr_street_02.png)

### 1. NCLT
Download NCLT from [https://robots.engin.umich.edu/nclt/](https://robots.engin.umich.edu/nclt/)
```bash
roslaunch block_localization run_nclt.launch
```

### 2. M2DGR
Download M2DGR from [https://github.com/SJTU-ViSYS/M2DGR](https://github.com/SJTU-ViSYS/M2DGR)
```bash
roslaunch block_localization run_m2dgr.launch
```
*Remarks:*
Since BM-Loc is a map-based localization method, you need to provide the directory where maps are stored. Edit the parameter `globalmap_dir` in `config/*.yaml` files. 


## Citation
If you use any of this code, please cite our [paper](https://arxiv.org/pdf/2404.18192).

```bibtex
@article{feng2024block,
  title={Block-Map-Based Localization in Large-Scale Environment},
  author={Feng, Yixiao and Jiang, Zhou and Shi, Yongliang and Feng, Yunlong and Chen, Xiangyu and Zhao, Hao and Zhou, Guyue},
  journal={arXiv preprint arXiv:2404.18192},
  year={2024}
}
```

## Acknowledgements
Thanks for the open-source projects [hdl_localization](https://github.com/koide3/hdl_localization), [hdl_global_localization](https://github.com/koide3/hdl_localization) and [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM).
=======
### The paper, titled "LiLoc: Lifelong Localization using Adaptive Submap Joining and Egocentric Factor Graph" is now publicly available on [Arxiv](https://arxiv.org/abs/2409.10172).

### The reconstructed code is comming soon ...
>>>>>>> d950221b21a548fc472488a790e3f83315e4bb55
