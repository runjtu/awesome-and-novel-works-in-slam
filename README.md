# awesome-and-novel-works-in-slam [![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

This repo contains a mostly cutting edge (new derives from old) list of **awesome and novel works in slam** <br>

If you find this repository useful, please consider STARing this list. Feel free to share this list with others! More comments will be found below, yet just opinions of mine. Let's make slamer great again.

---
## Overview

  - [3DGSNeRF](#3DGSNeRF)
  - [v2x](#v2x)
  - [Semantic](#semantic)
  - [novel](#novel)
  - [Largemodel](#largemodel)
  - [leaders](#leaders)
  - [journals](#journals)
  - [conferences](#conferences)
  - [Datasets](#Datasets)
  - [Competition](#Competition)
  - [Tools](#Tools)
  - [Sim](#Sim)
  - [BuySensors](#BuySensors)
  - [FindIntern](#FindIntern)
  - [WeMedia](#WeMedia)
  - [WhySDMap](#WhySDMap)
  - [WhyTraversability](#WhyTraversability)
  - [WhyLongTerm](#WhyLongTerm)

---

## 3DGSNeRF

Follow [[Jiheng Yang](https://github.com/yangjiheng/nerf_and_beyond_docs)] for more information on NeRF/3DGS/EVER, He will update news weekly, best Chinese community for NeRF/3DGS, I will do supplement when I have time.

Follow [[ai kwea](https://github.com/kwea123)] for more practical experience on NeRF, I will do supplement when I have time.

Follow [[Chongjie Ye](https://github.com/hugoycj)] for more practical experience on 3DGS, I will do supplement when I have time.

* **STORM**: "STORM: Spatio-Temporal Reconstruction Model for Large-Scale Outdoor Scenes", *arxiv 2024*.  [[Project](https://jiawei-yang.github.io/STORM/)] 

* **NaVILA**: "NaVILA: Legged Robot Vision-Language-Action Model for Navigation", *arxiv 2024*.  [[Paper](https://arxiv.org/pdf/2412.04453)] 

* **BeNeRF**: "BeNeRF: Neural Radiance Fields from a Single Blurry Image and Event Stream", *arxiv 2024*.  [[Paper](https://arxiv.org/pdf/2407.02174)] 

* **SLGaussian**: "SLGaussian: Fast Language Gaussian Splatting in Sparse Views", *arxiv 2024*.  [[Paper](https://arxiv.org/pdf/2412.08331)] 

* **HUGSIM**: "HUGSIM: A Real-Time, Photo-Realistic and Closed-Loop Simulator for Autonomous Driving", *arxiv 2024*.  [[Code](https://github.com/hyzhou404/HUGSIM)] 

* **GaussianRPG**: "GaussianRPG: 3D Gaussian Rendering PlayGround", *ECCV 2024*.  [[Code](https://github.com/GimpelZhang/GaussianRPG)] 

* **HSFM**: "Reconstructing People, Places, and Cameras", *arxiv 2024*.  [[Project](https://muelea.github.io/hsfm/)] 

* **Gaussian Splatting**: "3D Gaussian Splatting for Real-Time Radiance Field Rendering", *ACM Transactions on Graphics 2023*.  [[Paper](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/)] [[Code](https://github.com/graphdeco-inria/gaussian-splatting)]

* **Neural-Sim**: "Learning to Generate Training Data with NeRF", *ECCV 2022*.  [[Paper](https://arxiv.org/pdf/2207.11368.pdf)] [[Code](https://github.com/gyhandy/Neural-Sim-NeRF)] [[Webpage](https://fylwen.github.io/disp6d.html)]

* **iNeRF**: "Inverting Neural Radiance Fields for Pose Estimation", *IROS, 2021*. [[Paper](https://arxiv.org/pdf/2012.05877.pdf)] [[Code](https://github.com/yenchenlin/iNeRF-public)] [[Website](https://yenchenlin.me/inerf/)] [[Dataset](https://github.com/BerkeleyAutomation/dex-nerf-datasets)]

* **iMAP**: "Implicit Mapping and Positioning in Real-Time", *ICCV, 2021*. [[Paper](https://arxiv.org/abs/2103.12352)] [[Code](https://edgarsucar.github.io/iMAP/)]

* **SHINE-Mapping**: "Large-Scale 3D Mapping Using Sparse Hierarchical Implicit Neural Representations", *ICRA, 2023*. [[Paper](https://arxiv.org/pdf/2210.02299.pdf)] [[Code](https://github.com/PRBonn/SHINE_mapping)]

* **H2-Mapping**: "Real-time Dense Mapping Using Hierarchical Hybrid Representation", *RA-L, 2023*. [[Paper](https://arxiv.org/pdf/2306.03207.pdf)] [[Code](https://github.com/SYSU-STAR/H2-Mapping)]

* **LATITUDE**: Robotic Global Localization with Truncated Dynamic Low-pass Filter in City-scale NeRF, *ICRA,  2023*. [[Paper](https://arxiv.org/pdf/2209.09357.pdf)] [[Code](https://github.com/jike5/LATITUDE)]

* **NeuSE**: "Neural SE(3)-Equivariant Embedding for Consistent Spatial Understanding with Objects", *arXiv*. [[Paper](https://arxiv.org/pdf/2303.07308.pdf)] [[Code](https://neuse-slam.github.io/neuse/)]

* **ObjectFusion**: "Accurate object-level SLAM with neural object priors", *Graphical Models,  2022*. [[Paper](https://www.sciencedirect.com/science/article/pii/S1524070322000418)]

* **NDF_Change**: "Robust Change Detection Based on Neural Descriptor Fields", *IROS, 2022*. [[Paper](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9981246)]

* **LNDF**: "Local Neural Descriptor Fields: Locally Conditioned Object Representations for Manipulation", *ICRA, 2023*. [[Paper](https://arxiv.org/abs/2302.03573)] [[Webpage](https://elchun.github.io/lndf/)]

- **NeRF-LOAM**: Neural Implicit Representation for Large-Scale Incremental LiDAR Odometry and Mapping, *arXiv*. [[Paper](https://arxiv.org/pdf/2303.10709.pdf)] [[Code](https://github.com/JunyuanDeng/NeRF-LOAM)]

* **"Implicit Map Augmentation for Relocalization"**, *ECCV Workshop, 2022*. [[Paper](https://link.springer.com/chapter/10.1007/978-3-031-25066-8_36)]

*  **Co-SLAM**: Joint Coordinate and Sparse Parametric Encodings for Neural Real-Time SLAM, *CVPR, 2023*. [[Paper](https://arxiv.org/pdf/2304.14377.pdf)] [[Website](https://hengyiwang.github.io/projects/CoSLAM)]
*  Neural Implicit Dense Semantic SLAM, *arXiv, 2023*. [[Paper](https://arxiv.org/pdf/2304.14560.pdf)]

* **NeRF-Navigation**: "Vision-Only Robot Navigation in a Neural Radiance World", *ICRA, 2022*. [[Paper](https://mikh3x4.github.io/nerf-navigation/assets/NeRF_Navigation.pdf)] [[Code](https://github.com/mikh3x4/nerf-navigation)] [[Website](https://mikh3x4.github.io/nerf-navigation/)] 

* **ESDF**: "Sampling-free obstacle gradients and reactive planning in Neural Radiance Fields", *arXiv*. [[Paper](https://arxiv.org/abs/2205.01389)]

* **ClearDepth**: "ClearDepth: Enhanced Stereo Perception of Transparent Objects for Robotic Manipulation", *arXiv 2024*. [[Paper](https://arxiv.org/pdf/2409.08926)]

* **GS-LIVOM**: "Real-time Gaussian Splatting Assisted LiDAR-Inertial-Visual Odometry and Dense Mappings", *arXiv2024*. [[Code](https://github.com/xieyuser/GS-LIVOM)]




---
## v2x

Follow [[Siheng Chen](https://siheng-chen.github.io/)] for more information on V2X Perception, I will do supplement when I have time.

Follow [[Runsheng Xu](https://github.com/DerrickXuNu)] for more information on V2X Perception, I will do supplement when I have time.

* **HighwayEnv**: "An Environment for Autonomous Driving Decision-Making", *GitHub*. [[Code](https://github.com/Farama-Foundation/HighwayEnv)]

* **OpenLane-V2**: "The World's First Perception and Reasoning Benchmark for Scene Structure in Autonomous Driving.", *GitHub*. [[Code](https://github.com/OpenDriveLab/OpenLane-V2)]



---
## semantic

* **OPen3d**: "A Modern Library for 3D Data Processing", *arxiv*. [[Paper](https://arxiv.org/abs/1801.09847)] [[Code](https://github.com/isl-org/Open3D-ML)]

* **SuMa++**: "SuMa++: Efficient LiDAR-based Semantic SLAM", *IEEE/RSJ International Conference on Intelligent Robots and Systems 2019*. [[Paper](https://arxiv.org/abs/1801.09847)] [[Code](https://github.com/PRBonn/semantic_suma)]

* **Khronos**: "Khronos: A Unified Approach for Spatio-Temporal Metric-Semantic SLAM in Dynamic Environments", *Robotics: Science and Systems*. [[Paper](https://arxiv.org/abs/2402.13817)] [[Code](https://github.com/MIT-SPARK/Khronos)]


---
## novel

* **UniDistill**: "A Universal Cross-Modality Knowledge Distillation Framework for 3D Object Detection in Bird's-Eye View", *CVPR 2023*. [[Code](https://github.com/megvii-research/CVPR2023-UniDistill)]

* **Let Occ Flow**: "Let Occ Flow: Self-Supervised 3D Occupancy Flow Prediction", *CoRL 2024*. [[Code](https://github.com/eliliu2233/occ-flow)]

* **UniLoc**: "UniLoc: Towards Universal Place Recognition Using Any Single Modality", *arxiv 2024*. [[Paper](https://arxiv.org/pdf/2412.12079)]

* **Reliable-loc**: "Reliable LiDAR global localization using spatial verification and pose uncertainty", *arxiv 2024*. [[Code](https://github.com/zouxianghong/Reliable-loc)]

* **FAST-LIEO**: "FAST-LIEO", *github 2024*. [[Code](https://github.com/wsjpla/FAST-LIEO)]

* **TripletLoc**: "TripletLoc: One-Shot Global Localization using Semantic Triplet in Urban Environment", *arxiv2024*. [[Code](https://github.com/Weixin-Ma/tripletloc)]



---
## largemodel

* **DriveLikeAHuman**: "Rethinking Autonomous Driving with Large Language Models", *CVPR, 2023*. [[Code](https://github.com/PJLab-ADG/DriveLikeAHuman)]

* **learnLLM**[[Code](https://github.com/Lordog/dive-into-llms)]

* **ReAct: Synergizing Reasoning and Acting in Language Models**[[Code](https://github.com/ysymyth/ReAct)]

* **GraphRag**[[Code](https://github.com/microsoft/graphrag)]

* **DreamerV3**[[Code](https://github.com/danijar/dreamerv3)]

* **DinoV2**[[Code](https://github.com/facebookresearch/dinov2)]

* **CLIP**[[Code](https://github.com/openai/CLIP)]

* **llama**[[Code](https://github.com/meta-llama/llama)]

* **Gato**[[Paper](https://arxiv.org/pdf/2205.06175)]

* **Open the Black Box of Transformers**[[Paper](https://arxiv.org/abs/2311.13110)]

* **DeepSeek-V3**[[Code](https://github.com/deepseek-ai/DeepSeek-V3)]

* **Lotus**[[Code](https://github.com/EnVision-Research/Lotus)]



----
## leaders

### Industries and the off-campus

* **DeepMind** [[Homepage](https://github.com/google-deepmind)]

* **NVlabs** [[Homepage](https://github.com/NVlabs)]

* **NVIDIA Isaac** [[Homepage](https://github.com/nvidia-isaac)]

* **NVIDIA AI IOT** [[Homepage](https://github.com/NVIDIA-AI-IOT)]

* **ROS-device-drivers** [[Homepage](https://github.com/ros-drivers)]

* **dora-rs** [[Homepage](https://github.com/dora-rs)]

* **Eclipse zenoh** [[Homepage](https://github.com/eclipse-zenoh)]

* **ADLINK-ROS** [[Homepage](https://github.com/Adlink-ROS)]

* **eProsima** [[Homepage](https://github.com/eProsima)]

* **ANYbotics** [[Homepage](https://github.com/ANYbotics)]

* **Idiap Research Institute** [[Homepage](https://github.com/idiap)]

* **Waymo Research** [[Homepage](https://github.com/waymo-research)]

* **Bosch Research** [[Homepage](https://github.com/boschresearch)]

* **Motional** [[Homepage](https://github.com/nutonomy)]

* **valeo.ai** [[Homepage](https://github.com/valeoai)]

* **wayveai** [[Homepage](https://github.com/wayveai)]

* **Foxglove** [[Homepage](https://github.com/foxglove)]

* **rerun** [[Homepage](https://github.com/rerun-io/rerun/releases/tag/0.20.0)]

* **Udacity** [[Homepage](https://github.com/udacity)]
  
* **Nature Robots** [[Homepage](https://github.com/naturerobots)]
  
* **livox** [[Homepage](https://github.com/Livox-SDK)]

* **The International Digital Economy Academy** [[Homepage](https://github.com/IDEA-Research)]

* **ADG@PJLab** [[Homepage](https://github.com/PJLab-ADG)]

* **OpenDriveLab** [[Homepage](https://github.com/OpenDriveLab)]

* **OpenRobotLab** [[Homepage](https://github.com/OpenRobotLab)]

* **Alibaba AD Lab** [[Homepage](https://github.com/ADLab-AutoDrive)]

* **Sense-GVT** [[Homepage](https://github.com/Sense-GVT)]

* **Megvii Robot** [[Homepage](https://github.com/MegviiRobot)]

* **Megvii Research** [[Homepage](https://github.com/megvii-research)]

* **Baidu Apollo Auto** [[Homepage](https://github.com/ApolloAuto)]

* **Alibaba TongYi Vision Intelligence Lab** [[Homepage](https://github.com/ali-vilab)]

* **qcraftai** [[Homepage](https://github.com/qcraftai)]

* **haomoai** [[Homepage](https://github.com/haomo-ai)]

* **HUAWEI Noah's Ark Lab** [[Homepage](https://github.com/huawei-noah)]

* **Toyota Research Institute Machine Learning** [[Homepage](https://github.com/TRI-ML)]

### UK

* **Oxford Control Group** [[Homepage](https://github.com/oxfordcontrol)]

* **Oxford Dynamic Robot Systems Group** [[Homepage](https://github.com/ori-drs)]

* **Imperial College London Dyson Robotics Laboratory** [[Homepage](https://github.com/dyson-robotics-lab)]

* **UCL CDT in Foundational AI** [[Homepage](https://www.ucl.ac.uk/ai-centre/)]

* **UCL Robot Perception and Learning Lab** [[Homepage](https://github.com/RPL-CS-UCL)]

* **School of Informatics University of Edinburgh** [[Homepage](https://homepages.inf.ed.ac.uk/bwebb/)]


### Europe 

* **ETHZ Autonomous Systems Lab** [[Homepage](https://github.com/ethz-asl)]

* **ETHZ Robotic Systems Lab** [[Homepage](https://github.com/leggedrobotics)]

* **ETHZ Computer-Vision-and-Geometry-Lab** [[Homepage](https://github.com/cvg)]

* **ETHZ Visual Intelligence and Systems Group** [[Homepage](https://github.com/SysCV)]

* **ETHZ University of Cyprus Vision for Robotics Lab** [[Homepage](https://github.com/VIS4ROB-lab)]

* **ETHZ Learning and Adaptive Systems Group** [[Homepage](https://github.com/lasgroup)]

* **UZH Robotics and Perception Group** [[Homepage](https://github.com/uzh-rpg)]

* **EPFL CV LAB** [[Homepage](https://github.com/cvlab-epfl)]

* **TUM-Institue-of-Auotomative-Technology** [[Homepage](https://github.com/TUMFTM)]

* **TUM-Computer-Vision-Group** [[Homepage](https://github.com/tum-vision)]

* **TUM Smart Robotics Lab** [[Homepage](https://github.com/smartroboticslab)]

* **Flight Robotics and Perception Group (FRPG)** [[Homepage](https://github.com/robot-perception-group)]

* **Heidelberg Computer Vision and Learning Lab** [[Homepage](https://github.com/vislearn)]

* **UFR Robot Learning** [[Homepage](https://github.com/robot-learning-freiburg)]
  
* **University of Tübingen** [[Homepage](https://github.com/autonomousvision)]
  
* **Bonn Photogrammetry Robotics** [[Homepage](https://github.com/PRBonn)]
  
* **Karlsruhe Institute of Technology Institute of Measurement and Control Systems** [[Homepage](https://github.com/KIT-MRT)]

* **IGMR - RWTH Aachen University** [[Homepage](https://github.com/IGMR-RWTH)]

* **RWTH Aachen Institute of Automatic Control** [[Homepage](https://github.com/rwth-irt)]

* **RWTH Aachen Institut für Kraftfahrzeuge ika** [[Homepage](https://github.com/ika-rwth-aachen)]

* **Goettingen Data Fusion Group** [[Homepage](https://www.uni-goettingen.de/de/523154.html)]

* **Albert-Ludwigs-Universität Freiburg Robot Learning Lab** [[Homepage](https://github.com/robot-learning-freiburg)]

* **KTH Robotics Perception and Learning** [[Homepage](https://github.com/KTH-RPL)]

* **University of Turku Turku Intelligent Embedded and Robotic Systems Lab** [[Homepage](https://github.com/TIERS)]

* **NMBU Robotics** [[Homepage](https://github.com/NMBURobotics)]

* **TU Delft 3D geoinformation research group** [[Homepage](https://github.com/tudelft3d)]

* **TU Delft Intelligent Vehicles** [[Homepage](https://github.com/tudelft-iv)]

* **Poznan University of Technology Mobile Robots Lab** [[Homepage](https://github.com/LRMPUT)]

* **Sapienza University of Rome Robots Vision and Perception** [[Homepage](https://github.com/rvp-group)]

* **LTU Robotics & AI Engineering Group** [[Homepage](https://github.com/LTU-RAI)]

* **Control of Networked Systems Group at University of Klagenfurt** [[Homepage](https://github.com/aau-cns/)]

* **Vision for Robotics and Autonomous Systems Czech Technical University in Prague (CTU)** [[Homepage](https://github.com/ctu-vras)]


### Notrh America

* **Stanford Autonomous Systems Lab** [[Homepage](https://github.com/StanfordASL)]

* **Stanford Robotics Embodied Artificial Intelligence Lab REAL** [[Homepage](https://github.com/real-stanford)]

* **Stanford Vision and Learning Lab** [[Homepage](https://github.com/StanfordVL)]

* **Stanford Computational Vision and Geometry Lab** [[Homepage](https://github.com/cvgl)]

* **Stanford Center for Research on Foundation Models** [[Homepage](https://github.com/stanford-crfm)]

* **MIT-SPARKs** [[Homepage](https://github.com/MIT-SPARK)]

* **MIT Driverless Perception Team** [[Homepage](https://github.com/cv-core)]

* **MIT CSAIL Computer Vision** [[Homepage](https://github.com/CSAILVision)]

* **MIT's Marine Robotics Group** [[Homepage](https://github.com/MarineRoboticsGroup)]

* **MIT HAN Lab** [[Homepage](https://github.com/mit-han-lab)]

* **MIT Aerospace Controls Laboratory** [[Homepage](https://github.com/mit-acl)]

* **CMU Robotics Institute AirLab** [[Homepage](https://github.com/castacks)]

* **CMU Robotic Exploration Lab** [[Homepage](https://github.com/RoboticExplorationLab)]

* **CMU Robot Perception Lab** [[Homepage](https://github.com/rpl-cmu)]

* **Learning and Control for Agile Robotics Lab at CMU** [[Homepage](https://github.com/LeCAR-Lab)]

* **Princeton Computational Imaging** [[Homepage](https://github.com/princeton-computational-imaging)]

* **UPenn Perception Action Learning Group** [[Homepage](https://github.com/penn-pal-lab)]

* **UPenn Kumar Robotics** [[Homepage](https://github.com/KumarRobotics)]

* **UCB Model Predictive Control Laboratory** [[Homepage](https://github.com/MPC-Berkeley)]

* **Berkeley Automation Lab** [[Homepage](https://github.com/BerkeleyAutomation)]

* **UCLA Verifiable & Control-Theoretic Robotics Laboratory** [[Homepage](https://github.com/vectr-ucla)]

* **UCLA Mobility Lab** [[Homepage](https://github.com/ucla-mobility)]

* **Cornell Tech Kuleshov Group** [[Homepage](https://github.com/kuleshov-group)]

* **UCSD Existential Robotics Lab** [[Homepage](https://github.com/ExistentialRobotics)]

* **UCSD Hao Su's Lab** [[Homepage](https://github.com/haosulab)]

* **Umich Ford Center for Autonomous Vehicles FCAV** [[Homepage](https://github.com/umautobots)]

* **Umich The Autonomous Robotic Manipulation Lab studies motion planning manipulation human-robot collaboration** [[Homepage](https://github.com/UM-ARM-Lab)]

* **Umich Dynamic Legged Locomotion Robotics Lab** [[Homepage](https://github.com/UMich-BipedLab)]

* **Umich Computational Autonomy and Robotics Laboratory** [[Homepage](https://github.com/UMich-CURLY)]

* **NYU AI4CE Lab** [[Homepage](https://github.com/ai4ce)]

* **GaTech BORG Lab** [[Homepage](https://github.com/borglab)]

* **GaTech Intelligent Vision and Automation Laboratory IVA Lab** [[Homepage](https://github.com/ivalab)]

* **Texas Austin Autonomous Mobile Robotics Laboratory** [[Homepage](https://github.com/ut-amrl)]

* **Texas Austin Visual Informatics Group** [[Homepage](https://github.com/VITA-Group)]

* **University of Delaware Robot Perception Navigation Group RPNG** [[Homepage](https://github.com/rpng)]

* **Virginia Tech Transportation Institute** [[Homepage](https://github.com/VTTI)]

* **SIT Robust Field Autonomy Lab** [[Homepage](https://github.com/RobustFieldAutonomyLab)]

* **University at Buffalo Spatial AI & Robotics Lab** [[Homepage](https://github.com/sair-lab)]

* **Toronto STARS Laboratory** [[Homepage](https://github.com/utiasSTARS)]

* **UBC Computer Vision Group** [[Homepage](https://github.com/ubc-vision)]

* **UWaterloo CL2** [[Homepage](https://github.com/CL2-UWaterloo)]

* **École Polytechnique de Montréal Making Innovative Space Technology Lab** [[Homepage](https://github.com/MISTLab)]

* **Université Laval Northern Robotics Laboratory** [[Homepage](https://github.com/norlab-ulaval/)]

  
### Asia 

* **Yonsei Computational Intelligence Laboratory** [[Homepage](https://github.com/yonsei-cilab)]

* **SNU RPM** [[Homepage](https://github.com/irapkaist)]

* **KAIST Urban Robotics Lab** [[Homepage](https://github.com/url-kaist)]

* **KAIST Cognitive Learning for Vision and Robotics CLVR lab** [[Homepage](https://github.com/clvrai)]
  
* **Japan National Institute of Advanced Industrial Science and Technology** [[Homepage](https://github.com/SMRT-AIST)]

* **Japan Nagoya** [[Homepage](https://github.com/MapIV)]

* **NTU ARIS** [[Homepage](https://github.com/ntu-aris)]

* **CUHK OpenMMLab** [[Homepage](https://github.com/open-mmlab)]

* **HKUST Aerial Robotics Group** [[Homepage](https://github.com/HKUST-Aerial-Robotics)]

* **HKU Mars Lab** [[Homepage](https://github.com/hku-mars)]

* **HKU CVMI Lab** [[Homepage](https://github.com/CVMI-Lab)]

* **CUHK Deep Vision Lab** [[Homepage](https://github.com/dvlab-research)]

* **City University of Hong Kong MetaSLAM** [[Homepage](https://github.com/MetaSLAM)]

* **HK PolyU Visual Learning and Reasoning Group** [[Homepage](https://github.com/vLAR-group)]

* **NTNU Autonomous Robots Lab** [[Homepage](https://github.com/ntnu-arl)]

* **Tsinghua IIIS MARS Lab** [[Homepage](https://github.com/Tsinghua-MARS-Lab)]

* **Tsinghua Institute for AI Industry Research** [[Homepage](https://github.com/AIR-THU)]

* **SJTU Vision and Intelligent System Group** [[Homepage](https://github.com/SJTU-ViSYS)]

* **SJTU Intelligent Robotics and Machine Vision Lab** [[Homepage](https://github.com/IRMVLab)]

* **SJTU Thinklab** [[Homepage](https://github.com/Thinklab-SJTU)]

* **ZJU Advanced-Perception-on-Robotics-and-Intelligent-Learning-Lab** [[Homepage](https://github.com/APRIL-ZJU)]

* **ZJU CAD CG** [[Homepage](https://github.com/zju3dv)]

* **ZJU Advanced Intelligent Machines AIM** [[Homepage](https://github.com/zju3dv)]

* **ZJU Robotics Lab** [[Homepage](https://github.com/aim-uofa)]

* **ZJU FAST Lab** [[Homepage](https://github.com/ZJU-FAST-Lab)]

* **Fudan Zhang Vision Group** [[Homepage](https://github.com/fudan-zvg)]

* **Tongji Intelligent Electric Vehicle Research Group** [[Homepage](https://github.com/tiev-tongji)]

* **Tongji Intelligent Sensing Perception and Computing Group** [[Homepage](https://github.com/ispc-lab/)]

* **NUDT NuBot** [[Homepage](https://github.com/nubot-nudt)]

* **HUST EIC Vision Lab** [[Homepage](https://github.com/hustvl)]

* **ShanghaiTech Vision and Intelligent Perception Lab** [[Homepage](https://github.com/svip-lab)]

* **HITSZ nROS-Lab** [[Homepage](https://github.com/HITSZ-NRSL)]

* **GAP-LAB-CUHK-SZ** [[Homepage](https://github.com/GAP-LAB-CUHK-SZ)]

* **Westlake University Audio Signal and Information Processing Lab** [[Homepage](https://github.com/Audio-WestlakeU)]

* **Wuhan University Urban Spatial Intelligence Research Group at LIESMARS** [[Homepage](https://github.com/WHU-USI3DV)]

* **Wuhan University Integrated and Intelligent Navigation Group** [[Homepage](https://github.com/i2Nav-WHU)]

* **WHU GREAT (GNSS+ Research, Application and Teaching) group** [[Homepage](https://github.com/GREAT-WHU)]

* **SYSU STAR Smart Aerial Robotics Group** [[Homepage](https://github.com/SYSU-STAR)]

* **SYSU RAPID Lab** [[Homepage](https://github.com/SYSU-RoboticsLab)]

* **SYSU Pengyu Team** [[Homepage](https://github.com/PengYu-Team)]

* **NKU Robot Autonomy and Human-AI Collaboration Group** [[Homepage](https://github.com/NKU-MobFly-Robotics)]

* **HKUSTGZ Research group of visual generative models** [[Homepage](https://github.com/EnVision-Research)]

* **HNU Neuromorphic Automation and Intelligence Lab** [[Homepage](https://github.com/NAIL-HNU)]

* **NEU REAL** [[Homepage](https://github.com/NEU-REAL)]

* **Israel Autonomous Navigation and Sensor Fusion Lab** [[Homepage](https://github.com/ansfl)]


### Australia

* **CSIRORobotics Brisbane, Queensland, Australia** [[Homepage](https://github.com/csiro-robotics)]

* **Robotics Institute, University of Technology Sydney** [[Homepage](https://github.com/UTS-RI)]



Find more on this link in case of u r looking for PhD positions[[I](https://zhuanlan.zhihu.com/p/682671294)] [[II](https://zhuanlan.zhihu.com/p/682692024)].

## Journals

* **Science Robotics** [[Registeration](https://www.science.org/journal/scirobotics)]

* **TRO IEEE Transactions on Robotics** [[Registeration](https://ras.papercept.net/journals/tro/scripts/login.pl)]

* **IJRR International Journal of Robotics Research** [[Registeration](https://mc.manuscriptcentral.com/ijrr)]

* **JFR Journal of Field Robotics** [[Registeration](https://onlinelibrary.wiley.com/page/journal/15564967/homepage/forauthors.html)]

* **Transportation Research Part C** [[Registeration](https://www.editorialmanager.com/trc/default.aspx)]

* **TII IEEE Transactions on Industrial Informatics** [[Registeration](https://mc.manuscriptcentral.com/tii)]

* **TITS IEEE Transactions on Intelligent Transportation Systems** [[Registeration](https://mc.manuscriptcentral.com/t-its)]

* **TASE IEEE Transactions on Automation Science and Engineering** [[Registeration](https://www.ieee-ras.org/publications/t-ase)]

* **TMech IEEE/ASME Transactions on Mechatronics** [[Registeration](http://www.ieee-asme-mechatronics.info/)]

* **TGRS IEEE Transactions on Geoscience and Remote Sensing** [[Registeration](https://mc.manuscriptcentral.com/tgrs)]

* **TCSVT IEEE Transactions on Circuits and Systems for Video Technology** [[Registeration](https://ieee.atyponrex.com/submission/dashboard?siteName=TCSVT)]

* **TIV IEEE Transactions on Intelligent Vehicles** [[Registeration](https://mc.manuscriptcentral.com/t-iv)]

* **TIM IEEE Transactions on Instrumentation & Measurement** [[Registeration](https://www.editorialmanager.com/tim/default.aspx)]

* **RA-L IEEE Robotics and Automation Letters** [[Registeration](https://ras.papercept.net/journals/ral/scripts/login.pl)]

* **TVT IEEE Transactions on Vehicular Technology** [[Registeration](https://mc.manuscriptcentral.com/tvt-ieee/)]

* **TTE IEEE Transactions on Transportation Electrification** [[Registeration](https://mc.manuscriptcentral.com/tte-ieee)]

* **TIE IEEE Transactions on Industrial Electronics** [[Registeration](https://mc.manuscriptcentral.com/tie-ieee)]

* **Communications in Transportation Research** [[Registeration](https://www.sciencedirect.com/journal/communications-in-transportation-research)]



## Conferences

* **Mar. RSS Robotics: Science and Systems** [[Registeration](https://roboticsconference.org/)]
  
* **Nov. CVPR IEEE Conference on Computer Vision and Pattern Recognition** [[Registeration](https://cvpr.thecvf.com/)]

* **Sept. ICRA IEEE International Conference on Robotics and Automation** [[Registeration](https://2024.ieee-icra.org/)]

* **Mar. IROS IEEE/RSJ lnternational Conference onlntelligent Robots and Systems** [[Registeration](https://ieee-iros.org/)]

* **Mar. ICCV International Conference on Computer Vision** [[Registeration](https://iccv2023.thecvf.com/)]

* **Mar. ECCV European Conference on Computer Vision** [[Registeration](https://www.ecva.net/index.php#conferences)]

* **Jan. SIGGRAPH Special Interest Group for Computer GRAPHICS** [[Registeration](https://s2023.siggraph.org/)]

* **Jul. CoRL Conference on Robot Learning** [[Registeration](https://www.corl2023.org/)]

* **Sept. ICLR International Conference on Learning Representations** [[Registeration](https://iclr.cc/)]


----

## Datasets
* **NYU MARS** [[Homepage](https://ai4ce.github.io/MARS/)]
* **KAIST Urban** [[Homepage](https://sites.google.com/view/complex-urban-dataset/)]
* **EuRoC** [[Homepage](https://github.com/topics/euroc-dataset/)]
* **MCD Dataset** [[Homepage](https://mcdviral.github.io/)]
* **MSU-4S** [[Homepage](https://egr.msu.edu/waves/msu4s/)]
* **HoloVIC** [[Homepage](https://holovic.net/)]
* **Oxford Robotcar** [[Homepage](https://robotcar-dataset.robots.ox.ac.uk/)]
* **NCLT** [[Homepage](http://robots.engin.umich.edu/nclt/)]
* **Newer College** [[Homepage](https://ori-drs.github.io/newer-college-dataset/)]
* **Mulran** [[Homepage](https://sites.google.com/view/mulran-pr/dataset)]
* **KITTI** [[Homepage](https://www.cvlibs.net/datasets/kitti/)]
* **Argoverse** [[Homepage](https://www.argoverse.org/)]
* **Onpenlanev2** [[Homepage](https://github.com/OpenDriveLab/OpenLane-V2)]
* **M2DGR** [[Homepage](https://github.com/SJTU-ViSYS/M2DGR)]
* **SubT** [[Homepage](https://www.openrobotics.org/blog/2022/2/3/open-robotics-and-the-darpa-subterranean-challenge)]
* **R3LIVE** [[Homepage](https://github.com/ziv-lin/r3live_dataset)]
* **Rubble** [[Homepage](https://meganerf.cmusatyalab.org/)]
* **TIERS** [[Homepage](https://github.com/TIERS/tiers-lidars-dataset)]
* **HeLiPR** [[Homepage](https://arxiv.org/pdf/2309.14590)]
* **GEODE** [[Homepage](https://arxiv.org/pdf/2409.04961v1)]
* **CVGlobal** [[Homepage](https://opendatalab.com/CVeRS/CVGlobal)]
* **mapillary** [[Homepage](https://github.com/mapillary)]
* **SLABIM** [[Homepage](https://github.com/HKUST-Aerial-Robotics/SLABIM)]
* **BorealTC** [[Homepage](https://github.com/norlab-ulaval/BorealTC)]
* **fusionportable** [[Homepage](https://fusionportable.github.io/dataset/fusionportable_v2/)]

find more in paperwithcode

## Competition

Practice makes perfect, though lack of innovation.

* **ICRA 2023 Robodepth** [[Link](https://robodepth.github.io/)]
* **ICRA 2024 RoboDrive** [[Link](https://robodrive-24.github.io/)]
* **ICRA 2023 Sim2Real** [[Link](https://air.tsinghua.edu.cn/robomaster/sim2real_icra23.html)]


----
## Tools

Libs:

* **g2o** [[Link](https://github.com/RainerKuemmerle/g2o)]
* **ceres** [[Link](https://github.com/ceres-solver/ceres-solver)]
* **iSAM** [[Link](https://github.com/ori-drs/isam)]
* **Sophus** [[Link](https://github.com/strasdat/Sophus)]
* **gsplines** [[Link](https://github.com/rafaelrojasmiliani/gsplines_cpp)]
* **VAE** [[Link](https://github.com/clementchadebec/benchmark_VAE)]



NeRF/3DGS:

* **COLMAP** [[Link](https://github.com/colmap/colmap/releases)]
* **LLFF** [[Link](https://github.com/Fyusion/LLFF)]

Calibration:

* **Kalibr** [[CamCam/CamIMU/IMUIMU/Intrinsic](https://github.com/ethz-asl/kalibr)]
* **SensorsCalibration** [[LiDARCam/LiDARIMU/LiDARLiDAR/RadarCam/RadarLiDAR/CamCam/Intrinsic](https://github.com/PJLab-ADG/SensorsCalibration)]
* **livox_camera_calib** [[LiDARCam](https://github.com/hku-mars/livox_camera_calib)]
* **direct_visual_lidar_calibration** [[LiDARCam](https://github.com/koide3/direct_visual_lidar_calibration)]
* **Calibration-Is-All-You-Need** [[LiDARCam/LiDARIMU/CamIMU/CamCam](https://github.com/linClubs/Calibration-Is-All-You-Need)]

Evaluation:

* **EVO** [[EVO](https://github.com/MichaelGrupp/evo)]
* **CloudMapEvaluation** [[CloudMapEvaluation](https://github.com/JokerJohn/Cloud_Map_Evaluation)]
* **SLAMHIVE** [[SLAMHIVE](https://github.com/STAR-Center/SLAM-Hive)]

Communication:

* **Message Queuing Telemetry Transport** [[Video](https://www.youtube.com/watch?v=EIxdz-2rhLs)]

* **Hypertext Transfer Protocol Secure** [[Video](https://www.bilibili.com/video/BV1ph41177rJ)]

* **Controller Area Network** [[Video](https://www.bilibili.com/video/BV1GP411A78u)]

Deep Learning Framework:

* **U-net** [[Video](https://www.youtube.com/watch?v=lBicvB4iyYU)]

* **ViT** [[Video](https://www.youtube.com/watch?v=G6_IA5vKXRI)]


----

## Sim (experienced software, not toys from paper)

* **AAI** [[Sim now](https://www.automotive-ai.com/)]
* **Airsim** [[Sim now](https://github.com/microsoft/AirSim)]
* **aiSim** [[Sim now](https://aimotive.com/aisim)]
* **Apollo** [[Sim now](https://github.com/ApolloAuto/apollo)]
* **CARLA** [[Sim now](https://github.com/carla-simulator/carla)]
* **CarMaker** [[Sim now](https://www.ipg-automotive.com/en/products-solutions/software/carmaker/)]
* **CarSim** [[Sim now](https://www.carsim.com/products/carsim/index.php)]
* **Cognata** [[Sim now](https://www.cognata.com/)]
* **ESI Pro-Sivic** [[Sim now](https://myesi.esi-group.com/downloads/software-downloads/pro-sivic-2021.0)]
* **Metamoto** [[Sim now](https://velodynelidar.com/automated-with-velodyne/metamoto/)]
* **NVDIA Drive Sim** [[Sim now](https://developer.nvidia.com/drive/simulation)]
* **NVDIA Isaac Sim** [[Sim now](https://developer.nvidia.cn/isaac/sim)]
* **OnSite** [[Sim now](https://www.onsite.com.cn/#/dist/home)]
* **PanoSim** [[Sim now](https://www.panosim.com/)]
* **ParallelDomain** [[Sim now](https://paralleldomain.com/)]
* **Pilot-D GaiA** [[Sim now](http://www.pd-automotive.com/pc/#/)]
* **PreScan** [[Sim now](https://plm.sw.siemens.com/en-US/simcenter/autonomous-vehicle-solutions/prescan/)]
* **rFpro** [[Sim now](https://ww2.mathworks.cn/products/connections/product_detail/rfpro.html)]
* **RightHook** [[Sim now](https://github.com/righthook)]
* **RoadRunner** [[Sim now](https://ww2.mathworks.cn/products/roadrunner.html)]
* **SCANeR** [[Sim now](https://www.avsimulation.com/scaner/)]
* **SILAB** [[Sim now](https://wivw.de/en/silab)]
* **TruckMaker** [[Sim now](https://www.ipg-automotive.com/en/products-solutions/software/truckmaker/)]
* **Vires VTD** [[Sim now](https://hexagon.com/products/virtual-test-drive?utm_easyredir=vires.mscsoftware.com)]
* **VI Worldsim** [[Sim now](https://www.vi-grade.com/en/products/vi-worldsim/)]
* **ZJL-VTS** [[Sim now](https://www.onsite.com.cn/#/dist/endToEndTestTool)]
* **51Sim-one** [[Sim now](https://www.51sim.com/products/simone)]
* **Mujoco** [[Sim now](https://github.com/google-deepmind/mujoco)]
* **Genesis** [[Sim now](https://github.com/Genesis-Embodied-AI/Genesis)]



----
## BuySensors

   Multi-sensors:RGB, RGBD, Infrared, Polarization, Event-based(Future), Motion, Capture(GT), 2D Spinning LiDAR, 3D Spinning LiDAR, 3D Solid-State LiDAR, 4D High Resolution Radar(Future), mmWave Radar, UWB, RTK(GT), Wheel Odom, IMU, Ultrasonic Sensor.
   
----

## FindIntern

   For bridging college and society, mainly base in Shanghai, Beijing, Hangzhou, Suzhou, Canton, Shenzhen, HongKong

   Shanghai: AI Lab, DiDi, Horizon, Li Auto, BOSCH, Huawei, SenseTime,

   Beijing: Xiaomi, Baidu, Pony.ai, 

   Hangzhou: Unitree, Damo,

   Suzhou: Momenta,

   Canton: ZhuoYu, WeRide, Xiaopeng,

   interview experience like [Find More](https://zhuanlan.zhihu.com/p/710108368)

   BTW, RA/TA Opportunities：

   CMU AirLab

   A STAR
   
   CityU MetaSLAM

   ETH-RSS

  
----

## WeMedia

   Follow [[XueLingFeiHua](https://www.zhihu.com/people/maxhnnl/answers)] for general understanding of sensors used by autonomous vehicle, I will do supplement when I have time.



----
## WhySDMap

   For autonomous driving vechile and outdoor robotics, use more light-weight map instead of no map, specifically more like maplite [[demo](https://github.com/awiesmulla/maplite)]

   Especially for indoor environments, the prior is BIM [[BIM](https://github.com/HKUST-Aerial-Robotics/SLABIM)]


* **RoadNet**: "Translating Images to Road Network: A Non-Autoregressive Sequence-to-Sequence Approach", *ICCV 2023 (Oral)*. [[Paper](https://arxiv.org/abs/2402.08207)] [[Code](https://github.com/fudan-zvg/RoadNet)]

* **OrienterNet**: "OrienterNet: Visual Localization in 2D Public Maps with Neural Matching", *CVPR 2023*. [[Paper](https://arxiv.org/pdf/2304.02009)] [[Code](https://github.com/facebookresearch/OrienterNet)]

* **OSMLoc**: "OSMLoc: Single Image-Based Visual Localization in OpenStreetMap with Semantic and Geometric Guidances", *arxiv 2024*. [[Paper](https://arxiv.org/pdf/2411.08665)] [[Code](https://github.com/WHU-USI3DV/OSMLoc/tree/main)]

* **svo-dt**: " Drift-free Visual SLAM using Digital Twins", *arxiv 2024*. [[Paper](https://arxiv.org/pdf/2412.08496)] [[Code](https://github.com/uzh-rpg/rpg_svo_pro_with_digital_twins)]

* **CityNav**: "CityNav: Language-Goal Aerial Navigation Dataset with Geographic Information", *arxiv 2024*.  [[Paper](https://arxiv.org/pdf/2406.14240)] 

* **DFVDT**: "Drift-free Visual SLAM using Digital Twins", *arxiv 2024*.  [[Paper](https://arxiv.org/pdf/2412.08496)] 


   Follow [[TUMFTM](https://github.com/TUMFTM)] for more information on HDMap & SDMap aided localization, I will do supplement when I have time.
   
   Follow [[Tong Qin](https://github.com/qintonguav)] for more information on SDMap aided localization, I will do supplement when I have time.

   Follow [[Yujiao Shi](https://github.com/YujiaoShi/)] and [[tudelft-iv](https://github.com/tudelft-iv)] for more information on Satellite images aided localization, I will do supplement when I have time.

----
## WhyTraversability

   For all terrain navigation, because We're tired of filtering ground points...
   

* **MonoForce**: "MonoForce: Self-supervised Learning of Physics-aware Model for Predicting Robot-terrain Interaction", *arXiv 2022*. [[Paper](https://arxiv.org/pdf/2309.09007)] [[Code](https://github.com/ctu-vras/traversability_estimation)]

* **TML**: "A Global Traversability Mapping Library easily integratable with any SLAM System". [[Code](https://github.com/suchetanrs/traversability_mapping)]

* **TMMP**: "Bayesian Generalized Kernel Inference for Terrain Traversability Mapping", *the 2nd Annual Conference on Robot Learning*. [[Code](https://github.com/TixiaoShan/traversability_mapping)]

* **STEPP**: "STEPP: Semantic Traversability Estimation using Pose Projected features", *not yet*. [[Code](https://github.com/RPL-CS-UCL/STEPP-Code)]

* **HFHWC**: "Autonomous Driving in Unstructured Environments: How Far Have We Come?", [[Code](https://github.com/chaytonmin/Survey-Autonomous-Driving-in-Unstructured-Environments)]

* **EcSemMap**: "Uncertainty-aware Evidential Bayesian Semantic Mapping (EBS)", [[Code](https://github.com/junwon-vision/EvSemMap)]

* **LoD**: "Learning-on-the-Drive: Self-supervised Adaptive Long-range Perception for High-speed Offroad Driving", [[Paper](https://arxiv.org/pdf/2306.15226)]

* **tapdo**: "tadpo", [[Code](https://github.com/tadpo-iclr/tadpo)]

* **ROLO-SLAM**: "ROLO-SLAM: Rotation-Optimized LiDAR-Only SLAM in Uneven Terrain with Ground Vehicle", [[Code](https://github.com/sdwyc/ROLO)]


----
## WhyLongTerm

   For multi-session mapping and updating (change detection), dynamic objects filter.

* **KISS-Matcher**: "KISS-Matcher: Fast and Robust Point Cloud Registration Revisited", *arxiv 2024*. [[Paper](https://arxiv.org/abs/2409.15615)] [[Code](https://github.com/MIT-SPARK/KISS-Matcher)]

* **G3Reg**: "Pyramid Semantic Graph-based Global Point Cloud Registration with Low Overlap", *IEEE/RSJ International Conference on Intelligent Robots and Systems 2023*. [[Code](https://github.com/HKUST-Aerial-Robotics/G3Reg)]

* **DeepPointMap**: "DeepPointMap: Advancing LiDAR SLAM with Unified Neural Descriptors", *AAAI Conference on Artificial Intelligence 2024*. [[Code](https://github.com/ZhangXiaze/DeepPointMap)]

* **GLoc3D**: "Global Localization in Large-scale Point Clouds via Roll-pitch-yaw Invariant Place Recognition and Low-overlap Global Registration", *IEEE Transactions on Circuits and Systems for Video Technology
*. [[Code](https://github.com/peterWon/GLoc3D)]

* **maplalb2.0**: "maplab 2.0 – A Modular and Multi-Modal Mapping Framework", *IEEE Robotics and Automation Letters*. [[Code](https://github.com/ethz-asl/maplab)]

* **MR-SLAM**: "Disco: Differentiable scan context with orientation", *IEEE Robotics and Automation Letters*. [[Code](https://github.com/MaverickPeter/MR_SLAM)]

* **MS-Mapping**: "MS-Mapping: An Uncertainty-Aware Large-Scale Multi-Session LiDAR Mapping System", *arxiv 2024*.  [[Paper](https://arxiv.org/abs/2408.03723/)] [[Code](https://github.com/JokerJohn/MS-Mapping)]

* **NF-Atlas**: "Multi-Volume Neural Feature Fields for Large Scale LiDAR Mapping", *IEEE Robotics and Automation Letters 2023*.  [[Paper](https://ieeexplore.ieee.org/document/10197558)] [[Code](https://github.com/yuxuan1206/NF-Atlas)]

* **DiSCo-SLAM**: "DiSCo-SLAM: Distributed Scan Context-Enabled Multi-Robot LiDAR SLAM With Two-Stage Global-Local Graph Optimization", *IEEE International Conference on Robotics and Automation 2022*.  [[Paper](https://ieeexplore./document/9662965)] [[Code](https://github.com/RobustFieldAutonomyLab/DiSCo-SLAM)]

* **imesa**: "{iMESA}: Incremental Distributed Optimization for Collaborative Simultaneous Localization and Mapping", *Robotics: Science and Systems 2024*.  [[Paper](https://arxiv.org/pdf/2406.07371)] [[Code](https://github.com/rpl-cmu/imesa)]

* **DCL-SLAM**: "Swarm-lio: Decentralized swarm lidar-inertial odometry", *IEEE International Conference on Robotics and Automation 2023*.  [[Paper](https://ieeexplore.ieee.org/document/10161355)] [[Code](https://github.com/PengYu-Team/DCL-SLAM)]

* **BlockMap**: "Block-Map-Based Localization in Large-Scale Environment", *IEEE International Conference on Robotics and Automation 2024*.  [[Paper](https://arxiv.org/pdf/2404.18192)] [[Code](https://github.com/YixFeng/Block-Map-Based-Localization)]

* **SLIM**: "SLIM: Scalable and Lightweight LiDAR Mapping in Urban Environments", *arxiv maybe tro 2024*.  [[Paper](https://arxiv.org/pdf/2409.08681)] [[Code](https://github.com/HKUST-Aerial-Robotics/SLIM)]
  
reduce z-axis drift

* **Norlab-icp**: "A 2-D/3-D mapping library relying on the "Iterative Closest Point" algorithm.", [[Code](https://github.com/norlab-ulaval/norlab_icp_mapper)]


   Follow [[Ji Zhang](https://github.com/jizhang-cmu)] for more information on Robust Navigation, I will do supplement when I have time.

   Follow [[Xiang Gao](https://github.com/gaoxiang12)] for more information on SLAM, I will do supplement when I have time.

   Follow [[Tong Qin](https://github.com/qintonguav)] for more information on V-SLAM, I will do supplement when I have time.
   
   Follow [[Gisbi Kim](https://github.com/gisbi-kim/lt-mapper)] for more information on Map Maintaining, I will do supplement when I have time.

   Follow [[Kin Zhang](https://github.com/Kin-Zhang)] for more information on Dynamic Objects Removing, I will do supplement when I have time.

   Follow [[Zhijian Qiao](https://github.com/qiaozhijian)] for more information on Multi-Session Mapping, I will do supplement when I have time.

   Follow [[koide](https://github.com/koide3)] for more information on Life-Long LiDAR Mapping, I will do supplement when I have time.

   Find more in [[SLAM-Handbook](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release)] about SLAM.



----

## Citation

If you find this repository useful, please consider citing this repo:

```
@misc{runjtu2024slamrepo,
    title = {awesome-and-novel-works-in-slam},
    author = {Runheng Zuo},
    howpublished = {\url{https://github.com/runjtu/awesome-and-novel-works-in-slam}},
    year = {2024},
    note = "[Online; accessed 04-October-2024]"
}
```


![Star History Chart](https://api.star-history.com/svg?repos=runjtu/awesome-and-novel-works-in-slam&type=Date)
