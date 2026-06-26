<div align="center">
  <h1>
    <img src="assets/img/logo.png" alt="C2-Explorer Logo" width="70"/> C<sup>2</sup>-Explorer

  </h1>
  <h2>Contiguity-Driven Task Allocation with Connectivity-Aware Task Representation for Decentralized Multi-UAV Exploration</h2>
  <!-- <strong>
    Research Project / Preprint
  </strong> -->
  <!-- <br> -->
  <p align="center">
    <a href="https://tracylucia.github.io/" target="_blank">Xinlu Yan</a><sup>1,*</sup>,
    <a href="https://zager-zhang.github.io" target="_blank">Mingjie Zhang</a><sup>2,3,*</sup>,
    Yuhao Fang<sup>1</sup>,
    Yanke Sun<sup>1</sup>,
    <a href="https://personal.hkust-gz.edu.cn/junma/people-page.html" target="_blank">Jun Ma</a><sup>2</sup>,
    <a href="https://homepage.hit.edu.cn/youmingong" target="_blank">Youmin Gong</a><sup>1</sup>,
    <a href="https://robotics-star.com/people" target="_blank">Boyu Zhou</a><sup>3,†</sup>,
    <a href="https://homepage.hit.edu.cn/meijie" target="_blank">Jie Mei</a><sup>1,†</sup><br/><sub>&nbsp;</sub><br/>
    <sup>1</sup> Harbin Institute of Technology, Shenzhen<br/>
    <sup>2</sup> The Hong Kong University of Science and Technology (Guangzhou)<br/>
    <sup>3</sup> Southern University of Science and Technology<br/>
    <sup>*</sup> Equal Contribution &nbsp;&nbsp; <sup>†</sup> Co-corresponding Authors
  </p>
  <a href="https://arxiv.org/abs/2603.07699" target="_blank"><img alt="Paper" src="https://img.shields.io/badge/Paper-arXiv-b31b1b?logo=arxiv&logoColor=white"/></a>
  <a href="https://www.bilibili.com/video/BV1U5N3zgE4p/?share_source=copy_web&amp;vd_source=1801a551da967e1db6162c2de1380d70" target="_blank"><img alt="Video" src="https://img.shields.io/badge/Video-Bilibili-FB7299?logo=bilibili&amp;logoColor=white"/></a>
  <a href="https://robotics-star.com/C2-Explorer/" target="_blank"><img alt="Project Page" src="https://img.shields.io/badge/Project_Page-Website-4A90E2?logo=googlechrome&logoColor=white"/></a>
  <br/>

</div>

<p align="center" style="margin-top: 18px;">
  <img src="assets/img/cover.jpg" width="100%"/>
</p>

<p align="center">
  C<sup>2</sup>-Explorer is a <strong><em>decentralized</em></strong> multi-UAV exploration framework to enhance <strong><em>flexible</em></strong> and <strong><em>contiguous</em></strong> task allocation.
</p>

## 📢 News

- **[26/06/2026]**: 👩‍💻 Release the main algorithm of C<sup>2</sup>-Explorer.
- **[16/06/2026]**: 🎉 C<sup>2</sup>-Explorer is conditionally accepted to IROS 2026.

## 🤖 Real-World Experiments

<p align="center">
  <img src="assets/video/real-world.gif" width="80%"/>
</p>

## 🛠️ Installation

### Prerequisites

This repository is organized as a ROS1 catkin workspace and is primarily tested with:

- ROS Noetic (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04)
- PCL
- Eigen

#### For NLopt

```bash
git clone -b v2.7.1 https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

#### For LKH

```bash
wget http://akira.ruc.dk/~keld/research/LKH-3/LKH-3.0.6.tgz
tar xvfz LKH-3.0.6.tgz
cd LKH-3.0.6
make
sudo cp LKH /usr/local/bin
```

#### For MARSIM

```bash
sudo apt update
sudo apt install libglfw3-dev libglew-dev
```

### Compilation
<!-- 开源的时候换成：https://github.com/Robotics-STAR-Lab/C2-Explorer.git-->

```bash
cd ${YOUR_WORKSPACE_PATH}
git clone https://github.com/Robotics-STAR-Lab/C2-Explorer.git
catkin_make
```

## 🚀 Quick Start

Launch RViz in one terminal:

```bash
source ./devel/setup.bash
roslaunch exploration_manager rviz.launch
```

Launch the simulator in another terminal:

```bash
source ./devel/setup.bash
roslaunch exploration_manager open_plan_office.launch
```

### Note

- Trigger with the `2D Nav Goal` in Rviz when the terminal displays `wait for trigger`.

- You can replace `open_plan_office` with other maps. We provide another two test scenarios: `cubicle_office`, `octa_maze`.

- If you want to use your own `.pcd`, place the file under `src/MARSIM/map_generator/resource` and add the corresponding `.yaml` file under `src/swarm_exploration/exploration_manager/config/maps`.

## ✒️ Citation 

```bibtex
@misc{yan2026c2explorer,
      title={C$^2$-Explorer: Contiguity-Driven Task Allocation with Connectivity-Aware Task Representation for Decentralized Multi-UAV Exploration}, 
      author={Xinlu Yan and Mingjie Zhang and Yuhao Fang and Yanke Sun and Jun Ma and Youmin Gong and Boyu Zhou and Jie Mei},
      year={2026},
      eprint={2603.07699},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2603.07699}, 
}
```

## 🤓 Acknowledgments

We would like to express our gratitude to the following projects, which have provided significant support and inspiration for our work:

- [FALCON](https://github.com/HKUST-Aerial-Robotics/FALCON): An efficient framework for fast UAV exploration, from which our method for constructing topological maps draws inspiration.
- [RACER](https://github.com/SYSU-STAR/RACER): A Rapid Exploration Framework for Multiple UAVs.
- [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL): An Efficient Framework for Fast UAV Exploration.
- [MARSIM](https://github.com/hku-mars/MARSIM): A Light-weight Point-realistic Simulator for LiDAR-based UAVs.
