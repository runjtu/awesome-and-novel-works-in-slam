# awesome-and-novel-works-in-slam [![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

This repo contains a mostly cutting edge (new derives from old) list of **awesome and novel works in slam** <br>

If you find this repository useful, please consider STARing this list. Feel free to share this list with others! More comments will be found below, yet just opinions of mine. Let's make slamer great again. 
Please note that arxiv is a preprint platform that has not undergone peer review. Some of the content is of very low quality. It is necessary to pay attention to the latest version of formal journals or conferences.
There exists a significant disparity between engineering and academic pursuits, wherein numerous sophisticated concepts exhibit substantial gaps in practical implementation. This phenomenon is inherently normal and necessitates case-specific analysis to address the unique challenges encountered.

![graph](https://github.com/runjtu/awesome-and-novel-works-in-slam/blob/main/slam2.0.drawio.png)

---
## Overview

  - [3DGSNeRF](#3DGSNeRF)
  - [v2x](#v2x)
  - [Semantic](#semantic)
  - [novel](#novel)
  - [Largemodel](#largemodel)
  - [leaders/labs](#leaders/labs)
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

Follow [[Songyou Peng](https://pengsongyou.github.io/)] for more endeavors on 3DGS, I will do supplement when I have time.

Follow [[Jiheng Yang](https://github.com/yangjiheng/nerf_and_beyond_docs)] for more information on NeRF/3DGS/EVER, He will update news weekly, best Chinese community for NeRF/3DGS, I will do supplement when I have time.

Follow [[ai kwea](https://github.com/kwea123)] for more practical experience on NeRF, I will do supplement when I have time.

Follow [[Chongjie Ye](https://github.com/hugoycj)] for more practical experience on 3DGS, I will do supplement when I have time.


* **EVER**: "Exact Volumetric Ellipsoid Rendering for Real-time View Synthesis", *arxiv 2024*.  [[Code](https://github.com/half-potato/ever_training)] 

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

* **Normal-NeRF**: "Normal-NeRF: Ambiguity-Robust Normal Estimation for Highly Reflective Scenes", *arXiv 2025*. [[Code](https://github.com/sjj118/Normal-NeRF)]

* **GaussianProperty**: "Integrating Physical Properties to 3D Gaussians with LMMs", *arXiv 2025*. [[Project](https://gaussian-property.github.io/)]

* **InfiniCube**: "InfiniCube: Unbounded and Controllable Dynamic 3D Driving Scene Generation with World-Guided Video Models", *arXiv 2024*. [[Paper](https://arxiv.org/pdf/2412.03934)]

* **GS-LIVOM**: "Real-time Gaussian Splatting Assisted LiDAR-Inertial-Visual Odometry and Dense Mappings", *arXiv 2024*. [[Code](https://github.com/xieyuser/GS-LIVOM)]

* **OpenGS-SLAM**: "OpenGS-SLAM: Open-Set Dense Semantic SLAM with 3D Gaussian Splatting for Object-Level Scene Understanding", *arXiv 2024*. [[Code](https://github.com/YOUNG-bit/open_semantic_slam)]

* **AKF-LIO**: "AKF-LIO: LiDAR-Inertial Odometry with Gaussian Map by Adaptive Kalman Filter", *arXiv 2025*. [[Paper](https://arxiv.org/pdf/2503.06891)] [[Code](https://github.com/xpxie/AKF-LIO)]

* **GSPR**: "GSPR: Multimodal Place Recognition Using 3D Gaussian Splatting for Autonomous Driving", *arXiv 2025*. [[Paper](https://arxiv.org/abs/2410.00299)] 

* **MTGS**: "MTGS:Multi-Traversal Gaussian Splatting", *arXiv 2025*. [[Paper](https://arxiv.org/pdf/2503.12552)] 

* **Semantic Gaussians**: "Semantic Gaussians: Open-Vocabulary Scene Understanding with 3D Gaussian Splatting", *arXiv 2025*. [[Code](https://github.com/sharinka0715/semantic-gaussians)] 

* **diff-gaussian-rasterization**: "3D Gaussian Splatting for Real-Time Rendering of Radiance Fields", *ACM Transactions on Graphics 2023*. [[Code](https://github.com/JonathonLuiten/diff-gaussian-rasterization-w-depth)] 

* **WildGS-SLAM**: "WildGS-SLAM: Monocular Gaussian Splatting SLAM in Dynamic Environments", *CVPR 2025*. [[Code](https://github.com/GradientSpaces/WildGS-SLAM)] 

* **SiLVR**: "SiLVR: Scalable Lidar-Visual Reconstruction with Neural Radiance Fields for Robotic Inspection", *arxiv 2024*. [[Paper](https://arxiv.org/abs/2403.06877)] 

* **DyNFL**: "Dynamic LiDAR Re-simulation using Compositional Neural Fields", *CVPR 2024*. [[Paper](https://arxiv.org/pdf/2312.05247)] 

* **VPGS-SLAM**: "VPGS-SLAM: Voxel-based Progressive 3D Gaussian SLAM in Large-Scale Scenes", *arxiv 2025*. [[Paper](https://arxiv.org/pdf/2505.18992)] 

* **MAC-Ego3D**: "MAC-Ego3D: Multi-Agent Gaussian Consensus for Real-Time Collaborative Ego-Motion and Photorealistic 3D Reconstruction", *CVPR 2025*. [[Code](https://github.com/Xiaohao-Xu/MAC-Ego3D)]

* **LucidDreamer**: "LucidDreamer: Domain-free Generation of 3D Gaussian Splatting Scenes", *arxiv 2023*. [[Code](https://github.com/luciddreamer-cvlab/LucidDreamer)]

* **LODGE**: "LODGE:Level-of-Detail Large-Scale Gaussian Splatting with Efficient Rendering", *arxiv 2025*. [[Paper](https://arxiv.org/pdf/2505.23158)]

* **Omni-Scene**: "Omni-Scene: Omni-Gaussian Representation for Ego-Centric Sparse-View Scene Reconstruction", *CVPR 2025*. [[Paper](https://arxiv.org/pdf/2412.06273)]




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

* **FastMap**: "FastMap: Revisiting Dense and Scalable Structure from Motion", [[Code](https://github.com/pals-ttic/fastmap)]

* **VGGT**: "VGGT: Visual Geometry Grounded Transformer", *CVPR 2025*. [[Code](https://github.com/facebookresearch/vggt)]

* **MASt3R-SLAM**: "MASt3R-SLAM: Real-Time Dense SLAM with 3D Reconstruction Priors", [[Code](https://github.com/rmurai0610/MASt3R-SLAM)]

* **DRLSV**: "Dense reinforcement learning for safety validation of autonomous vehicles", *Nature 2023*. [[Paper](https://www.nature.com/articles/s41586-023-05732-2)]

* **Liquid AI**: "Closed-form continuous-time neural networks", *Nature 2023*. [[Paper](https://www.nature.com/articles/s42256-022-00556-7)]

* **UniDistill**: "A Universal Cross-Modality Knowledge Distillation Framework for 3D Object Detection in Bird's-Eye View", *CVPR 2023*. [[Code](https://github.com/megvii-research/CVPR2023-UniDistill)]

* **Let Occ Flow**: "Let Occ Flow: Self-Supervised 3D Occupancy Flow Prediction", *CoRL 2024*. [[Code](https://github.com/eliliu2233/occ-flow)]

* **UniLoc**: "UniLoc: Towards Universal Place Recognition Using Any Single Modality", *arxiv 2024*. [[Paper](https://arxiv.org/pdf/2412.12079)]

* **FAST-LIEO**: "FAST-LIEO", *github 2024*. [[Code](https://github.com/wsjpla/FAST-LIEO)]

* **Awesome-Robotics-Diffusion**: "Awesome-Robotics-Diffusion", *github 2024*. [[Code](https://github.com/showlab/Awesome-Robotics-Diffusion)]

* **Diffusion Planner**: "Diffusion-Based Planning for Autonomous Driving with Flexible Guidance", *2025 International Conference on Learning Representation (ICLR)*. [[Code](https://github.com/ZhengYinan-AIR/Diffusion-Planner)]

* **Learning More With Less**: "Learning More With Less: Sample Efficient Dynamics Learning and Model-Based RL for Loco-Manipulation", *arxiv 2025*. [[Paper](https://arxiv.org/pdf/2501.10499)]

* **SNN-VPR**: "Applications of Spiking Neural Networks in Visual Place Recognition", *IEEE Transactions on Robotics 2025*. [[Paper](https://arxiv.org/pdf/2311.13186)]

* **H3-Mapping**: "H3-Mapping: Quasi-Heterogeneous Feature Grids for Real-time Dense Mapping Using Hierarchical Hybrid Representation", *arxiv2024*. [[Code](https://github.com/SYSU-STAR/H3-Mapping)]

* **PE3R**: "PE3R: Perception-Efficient 3D Reconstruction", *arxiv 2025*. [[Code](https://github.com/hujiecpp/PE3R)]

* **SD-DefSLAM**: "SD-DefSLAM: Semi-Direct Monocular SLAM for Deformable and Intracorporeal Scenes", *2021 IEEE International Conference on Robotics and Automation (ICRA)*. [[Code](https://github.com/UZ-SLAMLab/SD-DefSLAM)]

* **robust-pose-estimator**: "Learning how to robustly estimate camera pose in endoscopic videos", *International Journal of Computer Assisted Radiology and Surgery 2023*. [[Code](https://github.com/aimi-lab/robust-pose-estimator)]

* **NR-SLAM**: "NR-SLAM: Non-Rigid Monocular SLAM", *2023 arxiv*. [[Code](https://github.com/endomapper/NR-SLAM)]

* **level-k**: "vehicle-interaction-decision-making", *2024 github*. [[Code](https://github.com/PuYuuu/vehicle-interaction-decision-making)]

* **Murre**: "Multi-view Reconstruction via SfM-guided Monocular Depth Estimation", *2025 arxiv*. [[Code](https://github.com/zju3dv/Murre)]

* **STDLoc**: "From Sparse to Dense: Camera Relocalization with Scene-Specific Detector from Feature Gaussian Splatting", *arxiv 2025*. [[Code](https://github.com/zju3dv/STDLoc)]

* **Scene Graph**: "Controllable 3D Outdoor Scene Generation via Scene Graphs", *arxiv 2025*. [[Paper](https://arxiv.org/pdf/2503.07152)]

* **SPR**: "SPR: Scene-agnostic Pose Regression for Visual Localization", *CVPR 2025*. [[Code](https://github.com/JunweiZheng93/SPR)]

* **BEV-LIO-LC**: "BEV Image Assisted LiDAR-Inertial Odometry with Loop Closure", *arxiv 2025*. [[Code](https://github.com/HxCa1/BEV-LIO-LC)]

* **DRO**: "DRO: Doppler-Aware Direct Radar Odometry", *RSS 2025*. [[Code](https://github.com/utiasASRL/dro)]

* **RIO**: "EKF-based Radar Inertial Odometry using 4D mmWave radar sensors", *github 2022*. [[Code](https://github.com/christopherdoer/rio)]

* **ApexNav**: "ApexNav: An Adaptive Exploration Strategy for Zero-Shot Object Navigation with Target-centric Semantic Fusion", *arxiv 2025*. [[Paper](https://arxiv.org/pdf/2504.14478)]

* **LP2**: "LP2: Language-based Probabilistic Long-term Prediction", *RAL 2024*. [[Code](https://github.com/MIT-SPARK/LP2)]

* **Matrix3D**: "Matrix3D: Large Photogrammetry Model All-in-One", *arxiv 2025*. [[Paper](https://arxiv.org/pdf/2502.07685)]

* **ViewCrafter**: "ViewCrafter: Taming Video Diffusion Models for High-fidelity Novel View Synthesis", *arxiv 2024*. [[Code](https://github.com/Drexubery/ViewCrafter)]

* **GelSLAM**: "GelSLAM: A Real-Time, High-Resolution, and Robust 3D Tactile SLAM System", *ICRA 2025*. [[Project](https://joehjhuang.github.io/gelslam/)]

* **MAC-VO**: "MAC-VO: Metrics-Aware Covariance for Learning-based Stereo Visual Odometry", *ICRA 2025*. [[Project](https://mac-vo.github.io/)]

* **GeoNav**: "GeoNav: Empowering MLLMs with Explicit Geospatial Reasoning Abilities for Language-Goal Aerial Navigation", *arxiv 2025*. [[Paper](https://arxiv.org/pdf/2504.09587)]

* **NavDP**: "NavDP: Learning Sim-to-Real Navigation Diffusion Policy with Privileged Information Guidance", *arxiv 2025*. [[Paper](https://arxiv.org/abs/2505.08712)]






---
## largemodel

* **DriveLikeAHuman**: "Rethinking Autonomous Driving with Large Language Models", *CVPR, 2023*. [[Code](https://github.com/PJLab-ADG/DriveLikeAHuman)]

* **ReAct: Synergizing Reasoning and Acting in Language Models**[[Code](https://github.com/ysymyth/ReAct)]

* **GraphRag**[[Code](https://github.com/microsoft/graphrag)]

* **DreamerV3**[[Code](https://github.com/danijar/dreamerv3)]

* **DinoV2**[[Code](https://github.com/facebookresearch/dinov2)]

* **CLIP**[[Code](https://github.com/openai/CLIP)]

* **llama**[[Code](https://github.com/meta-llama/llama)]

* **Gato**[[Paper](https://arxiv.org/pdf/2205.06175)]

* **Open the Black Box of Transformers**[[Paper](https://arxiv.org/abs/2311.13110)]

* **DeepSeek-V3**[[Code](https://github.com/deepseek-ai/DeepSeek-V3)]

* **DeepSeek-R1**[[Code](https://github.com/deepseek-ai/DeepSeek-R1)]
  
* **Lotus**[[Code](https://github.com/EnVision-Research/Lotus)]

* **VLN-CE-Isaac**[[Code](https://github.com/yang-zj1026/VLN-CE-Isaac)]

* **Orion**: "ORION: A Holistic End-to-End Autonomous Driving Framework by Vision-Language Instructed Action Generation", *arxiv 2025*. [[Project](https://xiaomi-mlab.github.io/Orion/)]

* **GASP**: "GASP: Unifying Geometric and Semantic Self-Supervised Pre-training for Autonomous Driving", *arxiv 2025*. [[Paper](https://arxiv.org/abs/2503.15672)]

* **SpatialLM**: "SpatialLM: Large Language Model for Spatial Understanding", *github 2025*. [[Code](https://github.com/manycore-research/SpatialLM)]

* **VL-Nav**: "VL-Nav: Real-time Vision-Language Navigation with Spatial Reasoning", *arxiv 2025*. [[Paper](https://arxiv.org/abs/2502.00931)]

* **MiLA**: "MiLA: Multi-view Intensive-fidelity Long-term Video Generation World Model for Autonomous Driving", *arxiv 2025*. [[Paper](https://arxiv.org/abs/2503.15875)]

* **GAIA-2**: "GAIA-2: Pushing the Boundaries of Video Generative Models for Safer Assisted and Automated Driving", *arxiv 2025*. [[Project](https://wayve.ai/thinking/gaia-2/)]

* **NeuPAN**: "NeuPAN: Direct Point Robot Navigation with End-to-End Model-based Learning", *IEEE Transactions on Robotics 2025*. [[Paper](https://arxiv.org/pdf/2403.06828)] [[Code](https://github.com/hanruihua/NeuPAN)]

* **OpenDriveVLA**: "OpenDriveVLA: Towards End-to-end Autonomous Driving with Large Vision Language Action Model", *arxiv 2025*. [[Code](https://github.com/DriveVLA/OpenDriveVLA)]

* **Distill Any Depth**: "Distill Any Depth: Distillation Creates a Stronger Monocular Depth Estimator", *arxiv 2025*. [[Code](https://github.com/Westlake-AGI-Lab/Distill-Any-Depth)]

* **TAP**: "Tracking Any Point (TAP)", *github 2025*. [[Code](https://github.com/google-deepmind/tapnet)]

* **ORION**: "ORION: A Holistic End-to-End Autonomous Driving Framework by Vision-Language Instructed Action Generation", *github 2025*. [[Code](https://github.com/xiaomi-mlab/Orion)]

* **FindAnything**: "FindAnything: Open-Vocabulary and Object-Centric Mapping for Robot Exploration in Any Environment", *arxiv 2025*. [[Paper](https://arxiv.org/pdf/2504.08603)]

* **Hier-SLAM**: "Hier-SLAM: Scaling-up Semantics in SLAM with a Hierarchically Categorical Gaussian Splatting", *ICRA 2025*. [[Code](https://github.com/LeeBY68/Hier-SLAM)]

* **SpatialLM**: "SpatialLM: Large Language Model for Spatial Understanding", *github 2025*. [[Code](https://github.com/manycore-research/SpatialLM)]

* **LightEMMA**: "LightEMMA: Lightweight End-to-End Multimodal Model for Autonomous Driving", *arxiv 2025*. [[Paper](https://arxiv.org/pdf/2505.00284)]

* **SOLVE**: "SOLVE: Synergy of Language-Vision and End-to-End Networks for Autonomous Driving", *arxiv 2025*. [[Paper](https://arxiv.org/pdf/2505.16805)]

* **VL-Nav**: "VL-Nav: Real-time Vision-Language Navigation with Spatial Reasoning", *arxiv 2025*. [[Paper](https://arxiv.org/pdf/2502.00931)]

* **VLM-3R**: "VLM-3R: Vision-Language Models Augmented with Instruction-Aligned 3D Reconstruction", *arxiv 2025*. [[Code](https://github.com/VITA-Group/VLM-3R)]

* **Impromptu-VLA**: "Impromptu VLA: Open Weights and Open Data for Driving Vision-Language-Action Models", *arxiv 2025*. [[Code](https://github.com/ahydchh/Impromptu-VLA)] [[Paper](https://arxiv.org/pdf/2505.23757v1)]

* **Cosmos-Drive-Dreams**: "Cosmos-Drive-Dreams: Scalable Synthetic Driving Data Generation with World Foundation Models", *arxiv 2025*. [[Project](https://research.nvidia.com/labs/toronto-ai/cosmos_drive_dreams/)] [[Paper](https://arxiv.org/pdf/2506.09042)]




----
## leaders/labs

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

* **Vector AI** [[Homepage](https://github.com/VectorInstitute)]

* **Motional** [[Homepage](https://github.com/nutonomy)]

* **valeo.ai** [[Homepage](https://github.com/valeoai)]

* **wayveai** [[Homepage](https://github.com/wayveai)]

* **Foxglove** [[Homepage](https://github.com/foxglove)]

* **rerun** [[Homepage](https://github.com/rerun-io/rerun/releases/tag/0.20.0)]

* **Udacity** [[Homepage](https://github.com/udacity)]
  
* **Nature Robots** [[Homepage](https://github.com/naturerobots)]

* **DeepSeek** [[Homepage](https://github.com/deepseek-ai)]

* **livox** [[Homepage](https://github.com/Livox-SDK)]

* **The International Digital Economy Academy** [[Homepage](https://github.com/IDEA-Research)]

* **ADG@PJLab** [[Homepage](https://github.com/PJLab-ADG)]

* **OpenDriveLab** [[Homepage](https://github.com/OpenDriveLab)]

* **OpenRobotLab** [[Homepage](https://github.com/OpenRobotLab)]

* **Alibaba AD Lab** [[Homepage](https://github.com/ADLab-AutoDrive)]

* **LiAutoAD** [[Homepage](https://github.com/LiAutoAD)]

* **Sense-GVT** [[Homepage](https://github.com/Sense-GVT)]

* **Megvii Robot** [[Homepage](https://github.com/MegviiRobot)]

* **Megvii Research** [[Homepage](https://github.com/megvii-research)]

* **Baidu Apollo Auto** [[Homepage](https://github.com/ApolloAuto)]

* **Alibaba TongYi Vision Intelligence Lab** [[Homepage](https://github.com/ali-vilab)]

* **qcraftai** [[Homepage](https://github.com/qcraftai)]

* **haomoai** [[Homepage](https://github.com/haomo-ai)]

* **HUAWEI Noah's Ark Lab** [[Homepage](https://github.com/huawei-noah)]

* **stepfun-ai** [[Homepage](https://github.com/stepfun-ai)]

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

* **ETHZ Photogrammetry and Remote Sensing Lab** [[Homepage](https://github.com/prs-eth)]

* **UZH Robotics and Perception Group** [[Homepage](https://github.com/uzh-rpg)]

* **EPFL CV LAB** [[Homepage](https://github.com/cvlab-epfl)]

* **EPFL Intelligent Maintenance and Operations Systems** [[Homepage](https://github.com/EPFL-IMOS)]

* **TUM-Institue-of-Auotomative-Technology** [[Homepage](https://github.com/TUMFTM)]

* **TUM-Computer-Vision-Group** [[Homepage](https://github.com/tum-vision)]

* **TUM Smart Robotics Lab** [[Homepage](https://github.com/smartroboticslab)]

* **Stuttgart Flight Robotics and Perception Group** [[Homepage](https://github.com/robot-perception-group)]

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

* **Istituto Italiano di Tecnologia Robotics** [[Homepage](https://www.iit.it/phd-positions-in-robotics)]

* **NMBU Robotics** [[Homepage](https://github.com/NMBURobotics)]

* **TU Delft 3D geoinformation research group** [[Homepage](https://github.com/tudelft3d)]

* **TU Delft Intelligent Vehicles** [[Homepage](https://github.com/tudelft-iv)]

* **TU Delft Autonomous Multi-Robots Lab** [[Homepage](https://github.com/tud-amr/)]

* **Poznan University of Technology Mobile Robots Lab** [[Homepage](https://github.com/LRMPUT)]

* **Sapienza University of Rome Robots Vision and Perception** [[Homepage](https://github.com/rvp-group)]

* **LTU Robotics & AI Engineering Group** [[Homepage](https://github.com/LTU-RAI)]

* **University of Luxembourg Automation and Robotics Research Group (ARG)** [[Homepage](https://github.com/snt-arg)]

* **Control of Networked Systems Group at University of Klagenfurt** [[Homepage](https://github.com/aau-cns/)]

* **Vision for Robotics and Autonomous Systems Czech Technical University in Prague (CTU)** [[Homepage](https://github.com/ctu-vras)]

* **Multi-robot Systems (MRS) group Czech Technical University in Prague (CTU)** [[Homepage](https://github.com/ctu-mrs)]
  
* **LARICS Lab** [[Homepage](https://github.com/larics)]




### Notrh America

* **Stanford Autonomous Systems Lab** [[Homepage](https://github.com/StanfordASL)]

* **Stanford Robotics Embodied Artificial Intelligence Lab REAL** [[Homepage](https://github.com/real-stanford)]

* **Stanford Vision and Learning Lab** [[Homepage](https://github.com/StanfordVL)]

* **Stanford Computational Vision and Geometry Lab** [[Homepage](https://github.com/cvgl)]

* **Stanford Center for Research on Foundation Models** [[Homepage](https://github.com/stanford-crfm)]

* **Stanford Open Virtual Assistant Lab** [[Homepage](https://github.com/stanford-oval)]

* **Stanford NAV Lab** [[Homepage](https://github.com/Stanford-NavLab)]

* **MIT-SPARKs** [[Homepage](https://github.com/MIT-SPARK)]

* **MIT CSAIL Computer Vision** [[Homepage](https://github.com/CSAILVision)]

* **MIT's Marine Robotics Group** [[Homepage](https://github.com/MarineRoboticsGroup)]

* **MIT HAN Lab** [[Homepage](https://github.com/mit-han-lab)]

* **MIT Aerospace Controls Laboratory** [[Homepage](https://github.com/mit-acl)]

* **MIT Robust Robotics Group** [[Homepage](https://github.com/robustrobotics)]

* **MIT Urban Mobility Lab + Transit Lab** [[Homepage](https://github.com/jtl-transit)]

* **MIT Driverless Perception Team** [[Homepage](https://github.com/cv-core)]

* **CMU Robotics Institute AirLab** [[Homepage](https://github.com/castacks)][[FRC](https://frc.ri.cmu.edu)]

* **CMU Robotic Exploration Lab** [[Homepage](https://github.com/RoboticExplorationLab)]

* **CMU Robot Perception Lab** [[Homepage](https://github.com/rpl-cmu)]

* **Learning and Control for Agile Robotics Lab at CMU** [[Homepage](https://github.com/LeCAR-Lab)]

* **Princeton Computational Imaging** [[Homepage](https://github.com/princeton-computational-imaging)]

* **Princeton Safe Robotics Lab** [[Homepage](https://github.com/SafeRoboticsLab)]

* **UPenn Perception Action Learning Group** [[Homepage](https://github.com/penn-pal-lab)]

* **UPenn Kumar Robotics** [[Homepage](https://github.com/KumarRobotics)]

* **UCB Model Predictive Control Laboratory** [[Homepage](https://github.com/MPC-Berkeley)]

* **Berkeley Automation Lab** [[Homepage](https://github.com/BerkeleyAutomation)]

* **JHU Laboratory for Computational Sensing and Robotics** [[Homepage](https://github.com/jhu-lcsr)]

* **UCLA Verifiable & Control-Theoretic Robotics Laboratory** [[Homepage](https://github.com/vectr-ucla)]

* **UCLA Mobility Lab** [[Homepage](https://github.com/ucla-mobility)]

* **Cornell Tech Kuleshov Group** [[Homepage](https://github.com/kuleshov-group)]

* **UCSD Existential Robotics Lab** [[Homepage](https://github.com/ExistentialRobotics)]

* **UCSD Autonomous Vehicle Laboratory** [[Homepage](https://github.com/AutonomousVehicleLaboratory)]

* **UCSD Hao Su's Lab** [[Homepage](https://github.com/haosulab)]

* **Umich Ford Center for Autonomous Vehicles FCAV** [[Homepage](https://github.com/umautobots)]

* **Umich The Autonomous Robotic Manipulation Lab studies motion planning manipulation human-robot collaboration** [[Homepage](https://github.com/UM-ARM-Lab)]

* **Umich Dynamic Legged Locomotion Robotics Lab** [[Homepage](https://github.com/UMich-BipedLab)]

* **Umich Computational Autonomy and Robotics Laboratory** [[Homepage](https://github.com/UMich-CURLY)]

* **Umich Mobility Transformation Lab** [[Homepage](https://traffic.engin.umich.edu/)]

* **NYU AI4CE Lab** [[Homepage](https://github.com/ai4ce)]

* **GaTech BORG Lab** [[Homepage](https://github.com/borglab)]

* **GaTech Intelligent Vision and Automation Laboratory IVA Lab** [[Homepage](https://github.com/ivalab)]

* **Special Interest Group for Robotics Enthusiasts at UIUC** [[Homepage](https://github.com/SIGRobotics-UIUC)]

* **UIUC-Robotics** [[Homepage](https://github.com/UIUC-Robotics)]

* **UIUC-iSE** [[Homepage](https://github.com/ise-uiuc)]

* **GAMMA UMD** [[Homepage](https://gamma.umd.edu/)]

* **Texas Austin Autonomous Mobile Robotics Laboratory** [[Homepage](https://github.com/ut-amrl)]

* **Texas Austin Visual Informatics Group** [[Homepage](https://github.com/VITA-Group)]

* **Texas Robot Perception and Learning Lab** [[Homepage](https://github.com/UT-Austin-RPL)]

* **University of Delaware Robot Perception Navigation Group RPNG** [[Homepage](https://github.com/rpng)]

* **Virginia Tech Transportation Institute** [[Homepage](https://github.com/VTTI)]

* **ASU Intelligent Robotics and Interactive Systems (IRIS) Lab** [[Homepage](https://github.com/asu-iris)]

* **SIT Robust Field Autonomy Lab** [[Homepage](https://github.com/RobustFieldAutonomyLab)]

* **University at Buffalo Spatial AI & Robotics Lab** [[Homepage](https://github.com/sair-lab)]

* **UCR Trustworthy Autonomous Systems Laboratory (TASL)** [[Homepage](https://github.com/tasl-lab)]

* **Toronto STARS Laboratory** [[Homepage](https://github.com/utiasSTARS)]

* **Toronto Autonomous Space Robotics Lab** [[Homepage](https://github.com/utiasASRL)]

* **Toronto TRAIL Lab** [[Homepage](https://www.trailab.utias.utoronto.ca/)]

* **UBC Computer Vision Group** [[Homepage](https://github.com/ubc-vision)]

* **UWaterloo CL2** [[Homepage](https://github.com/CL2-UWaterloo)]

* **IntRoLab - Intelligent / Interactive / Integrated / Interdisciplinary Robot Lab @ Université de Sherbrooke** [[Homepage](https://github.com/introlab)]

* **École Polytechnique de Montréal Making Innovative Space Technology Lab** [[Homepage](https://github.com/MISTLab)]

* **York-SDCNLab** [[Homepage](https://github.com/York-SDCNLab)]

* **Université Laval Northern Robotics Laboratory** [[Homepage](https://github.com/norlab-ulaval/)]

* **Queen's Estimation, Search, and Planning Research Group** [[Homepage](https://github.com/robotic-esp)]

* **Queen's offroad robotics** [[Homepage](https://www.queensu.ca/offroad-robotics/)]

  
### Asia 

* **KAIST Urban Robotics Lab** [[Homepage](https://github.com/url-kaist)]

* **KAIST Cognitive Learning for Vision and Robotics CLVR lab** [[Homepage](https://github.com/clvrai)]

* **Yonsei Computational Intelligence Laboratory** [[Homepage](https://github.com/yonsei-cilab)]

* **SNU RPM** [[Homepage](https://github.com/irapkaist)]

* **SNU Machine Perception and Reasoning Lab** [[Homepage](https://github.com/snumprlab)]

* **DGIST APRL** [[Homepage](https://sites.google.com/view/aprl-dgist)]

* **Japan National Institute of Advanced Industrial Science and Technology** [[Homepage](https://github.com/SMRT-AIST)]

* **Japan Nagoya** [[Homepage](https://github.com/MapIV)]

* **NUS showlab** [[Homepage](https://github.com/showlab)]

* **NTU ARIS** [[Homepage](https://github.com/ntu-aris)]

* **CUHK OpenMMLab** [[Homepage](https://github.com/open-mmlab)]

* **HKUST Aerial Robotics Group** [[Homepage](https://github.com/HKUST-Aerial-Robotics)]

* **HKUST XRIM-Lab** [[Homepage](https://github.com/XRIM-Lab)]

* **HKU Mars Lab** [[Homepage](https://github.com/hku-mars)]

* **HKU CVMI Lab** [[Homepage](https://github.com/CVMI-Lab)]

* **CUHK Deep Vision Lab** [[Homepage](https://github.com/dvlab-research)]

* **CUHK DeciForce** [[Homepage](https://github.com/decisionforce)]

* **City University of Hong Kong MetaSLAM** [[Homepage](https://github.com/MetaSLAM)]

* **HK PolyU Visual Learning and Reasoning Group** [[Homepage](https://github.com/vLAR-group)]

* **UMacau Intelligent Machine Research Lab (IMRL)** [[Homepage](https://github.com/IMRL)]

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

* **ZJU OM-AI Lab** [[Homepage](https://github.com/om-ai-lab)]

* **Fudan Zhang Vision Group** [[Homepage](https://github.com/fudan-zvg)]

* **Chongxuan Li's research group @ Renmin University of China** [[Homepage](https://github.com/ML-GSAI)]

* **Tongji Intelligent Electric Vehicle Research Group** [[Homepage](https://github.com/tiev-tongji)]

* **Tongji Intelligent Sensing Perception and Computing Group** [[Homepage](https://github.com/ispc-lab/)]

* **NUDT NuBot** [[Homepage](https://github.com/nubot-nudt)]

* **HUST EIC Vision Lab** [[Homepage](https://github.com/hustvl)]

* **ShanghaiTech Vision and Intelligent Perception Lab** [[Homepage](https://github.com/svip-lab)]

* **ShanghaiTech Automation and Robotics Center** [[Homepage](https://github.com/STAR-Center)]

* **HITSZ nROS-Lab** [[Homepage](https://github.com/HITSZ-NRSL)]

* **HITSZ CLASS-Lab** [[Homepage](https://github.com/CLASS-Lab)]

* **GAP-LAB-CUHK-SZ** [[Homepage](https://github.com/GAP-LAB-CUHK-SZ)]

* **Westlake University Audio Signal and Information Processing Lab** [[Homepage](https://github.com/Audio-WestlakeU)]

* **Wuhan University Urban Spatial Intelligence Research Group at LIESMARS** [[Homepage](https://github.com/WHU-USI3DV)]

* **Wuhan University Integrated and Intelligent Navigation Group** [[Homepage](https://github.com/i2Nav-WHU)]

* **WHU GREAT (GNSS+ Research, Application and Teaching) group** [[Homepage](https://github.com/GREAT-WHU)]

* **SYSU STAR Smart Aerial Robotics Group** [[Homepage](https://github.com/SYSU-STAR)]

* **SYSU RAPID Lab** [[Homepage](https://github.com/SYSU-RoboticsLab)]

* **SYSU Pengyu Team** [[Homepage](https://github.com/PengYu-Team)]

* **SYSU Human Cyber Physical (HCP) Intelligence Integration Lab** [[Homepage](https://github.com/HCPLab-SYSU)]

* **NKU Robot Autonomy and Human-AI Collaboration Group** [[Homepage](https://github.com/NKU-MobFly-Robotics)]

* **HKUSTGZ Research group of visual generative models** [[Homepage](https://github.com/EnVision-Research)]

* **HNU Neuromorphic Automation and Intelligence Lab** [[Homepage](https://github.com/NAIL-HNU)]

* **NEU REAL** [[Homepage](https://github.com/NEU-REAL)]

* **SZU College of Computer Science and Software Engineering** [[Homepage](https://github.com/SZU-AdvTech-2023)]

* **Israel Autonomous Navigation and Sensor Fusion Lab** [[Homepage](https://github.com/ansfl)]


### Australia

* **CSIRORobotics Brisbane, Queensland, Australia** [[Homepage](https://github.com/csiro-robotics)]

* **Robotics Institute, University of Technology Sydney, Sydney, Australia** [[Homepage](https://github.com/UTS-RI)]

* **Robotic Perception Group at the Australian Centre For Robotics, Sydney, Australia** [[Homepage](https://github.com/ACFR-RPG)]


Find more on this link in case of u r looking for PhD positions[[I](https://zhuanlan.zhihu.com/p/682671294)] [[II](https://zhuanlan.zhihu.com/p/682692024)].

## Journals

* **Science Robotics** [[Registeration](https://www.science.org/journal/scirobotics)]

* **TRO IEEE Transactions on Robotics** [[Registeration](https://ras.papercept.net/journals/tro/scripts/login.pl)]

* **IJRR International Journal of Robotics Research** [[Registeration](https://mc.manuscriptcentral.com/ijrr)]

* **Transportation Research Part C** [[Registeration](https://www.editorialmanager.com/trc/default.aspx)]

* **RCIM Robotics and Computer-Integrated Manufacturing** [[Registeration](https://www.editorialmanager.com/rcim/)]

* **TII IEEE Transactions on Industrial Informatics** [[Registeration](https://mc.manuscriptcentral.com/tii)]

* **JFR Journal of Field Robotics** [[Registeration](https://onlinelibrary.wiley.com/page/journal/15564967/homepage/forauthors.html)]

* **TASE IEEE Transactions on Automation Science and Engineering** [[Registeration](https://www.ieee-ras.org/publications/t-ase)]
  
* **TITS IEEE Transactions on Intelligent Transportation Systems** [[Registeration](https://mc.manuscriptcentral.com/t-its)]
  
* **TMech IEEE/ASME Transactions on Mechatronics** [[Registeration](http://www.ieee-asme-mechatronics.info/)]

* **TGRS IEEE Transactions on Geoscience and Remote Sensing** [[Registeration](https://mc.manuscriptcentral.com/tgrs)]

* **TCSVT IEEE Transactions on Circuits and Systems for Video Technology** [[Registeration](https://ieee.atyponrex.com/submission/dashboard?siteName=TCSVT)]

* **TIV IEEE Transactions on Intelligent Vehicles** [[Registeration](https://mc.manuscriptcentral.com/t-iv)]

* **TIM IEEE Transactions on Instrumentation & Measurement** [[Registeration](https://www.editorialmanager.com/tim/default.aspx)]

* **TTE IEEE Transactions on Transportation Electrification** [[Registeration](https://mc.manuscriptcentral.com/tte-ieee)]

* **TIE IEEE Transactions on Industrial Electronics** [[Registeration](https://mc.manuscriptcentral.com/tie-ieee)]

* **RA-L IEEE Robotics and Automation Letters** [[Registeration](https://ras.papercept.net/journals/ral/scripts/login.pl)]

* **TVT IEEE Transactions on Vehicular Technology** [[Registeration](https://mc.manuscriptcentral.com/tvt-ieee/)]

* **Communications in Transportation Research** [[Registeration](https://www.sciencedirect.com/journal/communications-in-transportation-research)]

* **IEEE Open Journal of Intelligent Transportation Systems** [[Registeration](https://ieee-itss.org/pub/oj-its/)]


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

* **EuRoC** [[Homepage](https://github.com/topics/euroc-dataset/)]
* **Oxford Robotcar** [[Homepage](https://robotcar-dataset.robots.ox.ac.uk/)]
* **NCLT** [[Homepage](http://robots.engin.umich.edu/nclt/)]
* **Newer College** [[Homepage](https://ori-drs.github.io/newer-college-dataset/)]
* **Mulran** [[Homepage](https://sites.google.com/view/mulran-pr/dataset)]
* **KITTI** [[Homepage](https://www.cvlibs.net/datasets/kitti/)]
* **Argoverse** [[Homepage](https://www.argoverse.org/)]
* **Onpenlanev2** [[Homepage](https://github.com/OpenDriveLab/OpenLane-V2)]
* **NYU MARS** [[Homepage](https://ai4ce.github.io/MARS/)]
* **KAIST Urban** [[Homepage](https://sites.google.com/view/complex-urban-dataset/)]
* **MCD Dataset** [[Homepage](https://mcdviral.github.io/)]
* **MSU-4S** [[Homepage](https://egr.msu.edu/waves/msu4s/)]
* **HoloVIC** [[Homepage](https://holovic.net/)]
* **Seeing in the Dark** [[Homepage](https://oxdan.active.vision/)]

* **BotanicGarden** [[Homepage](https://github.com/robot-pesg/BotanicGarden)]
* **M2DGR** [[Homepage](https://github.com/SJTU-ViSYS/M2DGR)]
* **GEODE** [[Homepage](https://github.com/PengYu-Team/GEODE_dataset)]
* **SubT** [[Homepage](https://www.openrobotics.org/blog/2022/2/3/open-robotics-and-the-darpa-subterranean-challenge)]
* **darpa_subt_worlds** [[Homepage](https://github.com/LTU-RAI/darpa_subt_worlds)]
* **R3LIVE** [[Homepage](https://github.com/ziv-lin/r3live_dataset)]
* **WildScenes** [[Homepage](https://github.com/csiro-robotics/WildScenes)]
* **RELLIS-3D** [[Homepage](https://github.com/unmannedlab/RELLIS-3D)]
* **Night-Voyager** [[Homepage](https://github.com/IMRL/Night-Voyager)]
* **ustc-flicar** [[Homepage](https://github.com/ustc-flicar/ustc-flicar.github.io)]
* **M2UDr** [[Homepage](https://github.com/Yaepiii/M2UD)]
* **GND** [[Homepage](https://cs.gmu.edu/~xiao/Research/GND/)]



* **Rubble** [[Homepage](https://meganerf.cmusatyalab.org/)]
* **TIERS** [[Homepage](https://github.com/TIERS/tiers-lidars-dataset)]
* **HeLiPR** [[Homepage](https://arxiv.org/pdf/2309.14590)]
* **GEODE** [[Homepage](https://arxiv.org/pdf/2409.04961v1)]
* **CVGlobal** [[Homepage](https://opendatalab.com/CVeRS/CVGlobal)]
* **mapillary** [[Homepage](https://github.com/mapillary)]
* **SLABIM** [[Homepage](https://github.com/HKUST-Aerial-Robotics/SLABIM)]
* **BIMNet** [[Homepage](https://github.com/LydJason/BIMNet)]
* **BorealTC** [[Homepage](https://github.com/norlab-ulaval/BorealTC)]
* **fusionportable** [[Homepage](https://fusionportable.github.io/dataset/fusionportable_v2)]
* **CVCities** [[Homepage](https://github.com/GaoShuang98/CVCities)]
* **GAMa** [[Homepage](https://github.com/svyas23/GAMa)]




find more in [paperwithcode](https://paperswithcode.com/), [awesome-slam-datasets](https://github.com/youngguncho/awesome-slam-datasets)

## Competition

Practice makes perfect, though lack of innovation.

* **ICRA 2025 FR** [[Link](https://norlab-ulaval.github.io/icra_workshop_field_robotics/)]
* **ICRA 2023 Robodepth** [[Link](https://robodepth.github.io/)]
* **ICRA 2024 RoboDrive** [[Link](https://robodrive-24.github.io/)]
* **ICRA 2023 Sim2Real** [[Link](https://air.tsinghua.edu.cn/robomaster/sim2real_icra23.html)]
* **nerfbaselines** [[Link](https://github.com/nerfbaselines/nerfbaselines)]


----
## Tools

Libs:

* **g2o** [[Link](https://github.com/RainerKuemmerle/g2o)]
* **ceres** [[Link](https://github.com/ceres-solver/ceres-solver)]
* **iSAM** [[Link](https://github.com/ori-drs/isam)]
* **Sophus** [[Link](https://github.com/strasdat/Sophus)]
* **gsplines** [[Link](https://github.com/rafaelrojasmiliani/gsplines_cpp)]
* **VAE** [[Link](https://github.com/clementchadebec/benchmark_VAE)]
* **pypose** [[Link](https://github.com/pypose/pypose)]
* **pyslam** [[Link](https://github.com/luigifreda/pyslam)]


NeRF/3DGS:

* **COLMAP** [[Link](https://github.com/colmap/colmap/releases)]
* **LLFF** [[Link](https://github.com/Fyusion/LLFF)]

Calibration:

* **Kalibr** [[CamCam/CamIMU/IMUIMU/Intrinsic](https://github.com/ethz-asl/kalibr)]
* **iKalibr** [[IMU/Cam/Radar/LiDAR/RGBD](https://github.com/Unsigned-Long/iKalibr)]
* **SensorsCalibration** [[LiDARCam/LiDARIMU/LiDARLiDAR/RadarCam/RadarLiDAR/CamCam/Intrinsic](https://github.com/PJLab-ADG/SensorsCalibration)]
* **livox_camera_calib** [[LiDARCam](https://github.com/hku-mars/livox_camera_calib)]
* **direct_visual_lidar_calibration** [[LiDARCam](https://github.com/koide3/direct_visual_lidar_calibration)]
* **Calibration-Is-All-You-Need** [[LiDARCam/LiDARIMU/CamIMU/CamCam](https://github.com/linClubs/Calibration-Is-All-You-Need)]
* **GRIL-Calib** [[LiDARIMU](https://github.com/Taeyoung96/GRIL-Calib)]



Evaluation:

* **EVO** [[EVO](https://github.com/MichaelGrupp/evo)]
* **CloudMapEvaluation** [[CloudMapEvaluation](https://github.com/JokerJohn/Cloud_Map_Evaluation)]
* **SLAMHIVE** [[SLAMHIVE](https://github.com/STAR-Center/SLAM-Hive)]
* **Robustness_Metric** [[SLAMHIVE](https://github.com/adrienzhh/Robustness_Metric)]


Communication:

* **Message Queuing Telemetry Transport** [[Video](https://www.youtube.com/watch?v=EIxdz-2rhLs)]

* **Hypertext Transfer Protocol Secure** [[Video](https://www.bilibili.com/video/BV1ph41177rJ)]

* **Controller Area Network** [[Video](https://www.bilibili.com/video/BV1GP411A78u)]

Deep Learning Framework:

* **U-net** [[Video](https://www.youtube.com/watch?v=lBicvB4iyYU)]

* **ViT** [[Video](https://www.youtube.com/watch?v=G6_IA5vKXRI)]

Writing:

* **NSFC-Latex** [[Video](https://github.com/MCG-NKU/NSFC-LaTex)]


Books and Reviews:

* **learnLLM**[[Code](https://github.com/Lordog/dive-into-llms)]

* **Foundations-of-LLMs**[[Code](https://github.com/ZJU-LLMs/Foundations-of-LLMs)]

* **MathRL**[[Code](https://github.com/MathFoundationRL/Book-Mathematical-Foundation-of-Reinforcement-Learning)]

* **SLAM-Handbook**[[Code](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release)]

* **slambook2**[[Code](https://github.com/gaoxiang12/slambook2)]

* **SLAM Course (2013)**[[Video](https://www.youtube.com/playlist?app=desktop&list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_)]

* **TransferLearning**[[Code](https://github.com/jindongwang/transferlearning)]

* **Present and Future of SLAM in Extreme Environments**: "Present and Future of SLAM in Extreme Environments: The DARPA SubT Challenge", *2022 IEEE Transactions on Robotics*.  [[Paper](https://ieeexplore.ieee.org/document/10286080)] 

* **General Place Recognition Survey**: "General Place Recognition Survey: Towards Real-World Autonomy", *2024 arxiv*.  [[Paper](https://arxiv.org/pdf/2405.04812)] [[Code](https://github.com/MetaSLAM/GPRS_Survey)]

* **NeRF in Robotics**: "NeRF in Robotics: A Survey", *2024 arxiv*.  [[Paper](https://arxiv.org/pdf/2405.01333)] 

* **Learning-based 3D Reconstruction in AD**: "Learning-based 3D Reconstruction in Autonomous Driving: A Comprehensive Survey", *2025 arxiv*.  [[Paper](https://arxiv.org/abs/2503.14537)] 

* **World Model**: "The Role of World Models in Shaping Autonomous Driving: A Comprehensive Survey", [[Code](https://github.com/LMD0311/Awesome-World-Model)]

* **HFHWC**: "Autonomous Driving in Unstructured Environments: How Far Have We Come?", [[Code](https://github.com/chaytonmin/Survey-Autonomous-Driving-in-Unstructured-Environments)]

* **FutureMapping2**: "FutureMapping 2: Gaussian Belief Propagation for Spatial AI", [[Paper](https://arxiv.org/abs/1910.14139)]

* **Vision-Language-ActionModels**: "Vision-Language-ActionModels: Concepts, Progress, Applications and Challenges", [[Paper](https://arxiv.org/pdf/2505.04769)]

* **Embodied Intelligence for 3D Understanding**: "Embodied Intelligence for 3D Understanding: A Survey on 3D Scene Question Answering", [[Paper](https://arxiv.org/pdf/2502.00342)]

* **Generative AI for AD: R**: "Generative AI for Autonomous Driving: A Review", [[Paper](https://arxiv.org/pdf/2505.15863)]

* **Generative AI for AD: FO**: "GENERATIVE AI FOR AUTONOMOUS DRIVING: FRONTIERS AND OPPORTUNITIES", [[Paper](https://arxiv.org/pdf/2505.08854)]

* **LLM4Drive**: "LLM4Drive: A Survey of Large Language Models for Autonomous Driving", *arxiv 2023*. [[Code](https://github.com/Thinklab-SJTU/Awesome-LLM4AD)]

* **MA-AD**: "Multi-Agent-Autonomous-Driving", *github 2025*. [[Code](https://github.com/dl-m9/Multi-Agent-Autonomous-Driving)]

* **4DRadar-tutorial**: "4D Radar Technology and Advanced Sensor Fusion AI", *ICRA 2025*. [[Project](https://sites.google.com/view/icra25-4dradar-tutorial)]

* **Toward Embodied AGI**: "Toward Embodied AGI: A Review of Embodied AI and the Road Ahead", *arxiv 2025*. [[Paper](https://arxiv.org/pdf/2505.14235)]




----

## Sim (experienced and promising software)

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
* **MineSim** [[Sim now](https://github.com/BUAA-TRANS-Mine-Group/MineSim-3DVisualTool)]
* **51Sim-one** [[Sim now](https://www.51sim.com/products/simone)]
* **Mujoco** [[Sim now](https://github.com/google-deepmind/mujoco)]
* **Genesis** [[Sim now](https://github.com/Genesis-Embodied-AI/Genesis)]
* **Cosmos** [[Sim now](https://github.com/NVIDIA/Cosmos)]
* **BeamNG** [[Sim now](https://www.beamng.com/game/)]
* **metadrive** [[Sim now](https://github.com/metadriverse/metadrive)]
* **cosmos-transfer** [[Sim now](https://github.com/nvidia-cosmos/cosmos-transfer1)]
* **LimSim** [[Sim now](https://github.com/PJLab-ADG/LimSim)]
* **robosuite** [[Sim now](https://github.com/ARISE-Initiative/robosuite)]







----
## BuySensors

   Multi-sensors:RGB, RGBD, Infrared, Polarization, Event-based(Future), Motion, Capture(GT), 2D Spinning LiDAR, 3D Spinning LiDAR, 3D Solid-State LiDAR, 4D High Resolution Radar(Future), mmWave Radar, UWB, RTK(GT), Wheel Odom, IMU, Ultrasonic Sensor.
   
----

## FindIntern

   For bridging college and society, mainly base in Shanghai, Beijing, Hangzhou, Suzhou, Canton, Shenzhen, HongKong

   Shanghai: AI Lab, DiDi, Horizon, Li Auto, BOSCH, Huawei, SenseTime, 

   Beijing: Xiaomi, Baidu, Pony.ai, MSRA, QCraft, 

   Hangzhou: Unitree, Damo, 

   Suzhou: Momenta, 

   Canton: ZhuoYu, WeRide, Xiaopeng, DeepRoute, 
   
   [Interview experience](https://zhuanlan.zhihu.com/p/710108368)

   BTW, RA/TA Opportunities：
   
   [Find More](https://github.com/StarCycle/Awesome-Embodied-AI-Job)

  
----

## WeMedia

   Follow [[XueLingFeiHua](https://www.zhihu.com/people/maxhnnl/answers)] for general understanding of sensors used by autonomous vehicle, I will do supplement when I have time.

   Follow [[MITRoboticsSeminar](https://www.youtube.com/c/MITRoboticsSeminar/videos)] for cutting edge seminar in robotics, I will do supplement when I have time.

   Follow [[IVDesign]([https://www.youtube.com/c/MITRoboticsSeminar/videos](https://mp.weixin.qq.com/s?__biz=Mzg5NDcxMDIxNA==&mid=2247516332&idx=1&sn=5d032b0138cd21a1ddb99bc782c0ef36&chksm=c19755d1da727e4d0d587f708cf0b55809dec06a24cf8750dfbf0adfdfa62ee8c68d0e1de83d&mpshare=1&scene=1&srcid=0602oIjWC7yhrpVLA8y1wEY3&sharer_shareinfo=6107cac786755ca405ea6c61abcec6b3&sharer_shareinfo_first=6107cac786755ca405ea6c61abcec6b3#rd))] [[ADCol]([https://mp.weixin.qq.com/s?__biz=Mzg2Mjg3MTc5OA==&mid=2247509070&idx=1&sn=06928562224f8925f4029024606c73ec&chksm=cfa060b85d43f23b20affd16d531baef6760207dc444d60404be7a01b2c5bb7dba2cf8f5f085&mpshare=1&scene=1&srcid=0602XGswFLqPDFyQR9GrrVoD&sharer_shareinfo=2dab53f5511ad80838f81c96cad4f399&sharer_shareinfo_first=2dab53f5511ad80838f81c96cad4f399#rd))] [[VLN]([https://mp.weixin.qq.com/s?__biz=MzkwMTgxNjk5OA==&mid=2247488168&idx=1&sn=c68f1ab4c1fc1bdeee6401503d2fb880&chksm=c10eb53663875cce034f79f5697d4878a0236dc328e13169e4d300eec111a820398f3b241b56&mpshare=1&scene=1&srcid=0602v2HW5yuGN3QT94iPwNXH&sharer_shareinfo=65b9bcf5f94e2e37b4032cdb20572d2b&sharer_shareinfo_first=65b9bcf5f94e2e37b4032cdb20572d2b#rd))] [[ADFeed]([https://mp.weixin.qq.com/s?__biz=MjM5NTI1NzgzMA==&mid=2447787620&idx=1&sn=9abb1bed1af25bdd016ee570264f6aad&chksm=b385150198ec29acc9b72cf0030f6013894a0085739b6926ad96cff02e21f28036403a48a4db&mpshare=1&scene=1&srcid=0602FttrTqDZmzic24Lk6PsL&sharer_shareinfo=457a044e076051cd771c5454f9dc1e7b&sharer_shareinfo_first=457a044e076051cd771c5454f9dc1e7b#rd))] for cutting edge paper and talks, scan QR code for more Wxtweet, I will do supplement when I have time.











----
## WhySDMap

   For autonomous driving vechile and outdoor robotics, use more light-weight map instead of no map, specifically more like maplite [[demo](https://github.com/awiesmulla/maplite)]

   Especially for indoor environments, the prior is BIM [[BIM](https://github.com/HKUST-Aerial-Robotics/SLABIM)]



* **AddressCLIP**: "AddressCLIP: Empowering Vision-Language Models for City-wide Image Address Localization", *arxiv 2024*. [[Paper](https://arxiv.org/abs/2407.08156)]

* **RoadNet**: "Translating Images to Road Network: A Non-Autoregressive Sequence-to-Sequence Approach", *ICCV 2023 (Oral)*. [[Paper](https://arxiv.org/abs/2402.08207)] [[Code](https://github.com/fudan-zvg/RoadNet)]

* **OrienterNet**: "OrienterNet: Visual Localization in 2D Public Maps with Neural Matching", *CVPR 2023*. [[Paper](https://arxiv.org/pdf/2304.02009)] [[Code](https://github.com/facebookresearch/OrienterNet)]

* **OSMLoc**: "OSMLoc: Single Image-Based Visual Localization in OpenStreetMap with Semantic and Geometric Guidances", *arxiv 2024*. [[Paper](https://arxiv.org/pdf/2411.08665)] [[Code](https://github.com/WHU-USI3DV/OSMLoc/tree/main)]

* **svo-dt**: " Drift-free Visual SLAM using Digital Twins", *arxiv 2024*. [[Paper](https://arxiv.org/pdf/2412.08496)] [[Code](https://github.com/uzh-rpg/rpg_svo_pro_with_digital_twins)]

* **CityNav**: "CityNav: Language-Goal Aerial Navigation Dataset with Geographic Information", *arxiv 2024*.  [[Paper](https://arxiv.org/pdf/2406.14240)] 

* **DFVDT**: "Drift-free Visual SLAM using Digital Twins", *arxiv 2024*.  [[Paper](https://arxiv.org/pdf/2412.08496)] 

* **BEVPlace2**: "BEVPlace++: Fast, Robust, and Lightweight LiDAR Global Localization for Unmanned Ground Vehicles", *arxiv 2024*.  [[Code](https://github.com/zjuluolun/BEVPlace2)]

* **TripletLoc**: "TripletLoc: One-Shot Global Localization using Semantic Triplet in Urban Environment", *arxiv2024*. [[Code](https://github.com/Weixin-Ma/tripletloc)]

* **Reliable-loc**: "Reliable LiDAR global localization using spatial verification and pose uncertainty", *arxiv 2024*. [[Code](https://github.com/zouxianghong/Reliable-loc)]

* **Render2Loc**: "Render-and-Compare: Cross-view 6-DoF Localization from Noisy Prior", *2023 IEEE International Conference on Multimedia and Expo (ICME)*. [[Code](https://github.com/Choyaa/Render2Loc)]

* **CityWalker**: "CityWalker: Learning Embodied Urban Navigation from Web-Scale Videos", *2025 CVPR*. [[Code](https://github.com/ai4ce/CityWalker)]

* **EI-Nav**: "OPEN: Openstreetmap-enhanced oPen-air sEmantic Navigation", *2025 arxiv*. [[Code](https://github.com/EI-Nav/light-map-navigation)]

* **DeepGPS**: "DeepGPS: deep learning enhanced GPS positioning in urban canyons", *2024 IEEE Transactions on Mobile Computing (TMC)*. [[Code](https://github.com/bducgroup/DeepGPS)]

* **TESM**: "Topological Exploration using Segmented Map with Keyframe Contribution in Subterranean Environments", *2023 arxiv*. [[Paper](https://arxiv.org/pdf/2309.08397)]

* **SD++**: "SD++: Enhancing Standard Definition Maps by Incorporating Road Knowledge using LLMs", *2025 arxiv*. [[Paper](https://arxiv.org/pdf/2502.02773)]

* **ERPoT**: "ERPoT: Effective and Reliable Pose Tracking for Mobile Robots Using Lightweight Polygon Maps", *2025 TRO*. [[Paper](https://ieeexplore.ieee.org/document/11024226)] [[Code](https://github.com/ghm0819/ERPoT)]


   Follow [[Gmberton](https://github.com/gmberton)] for more information on Visual Place Recognition, I will do supplement when I have time.

   Follow [[Sarlinpe](https://github.com/sarlinpe)] for more information on SfM & mapfusion, I will do supplement when I have time.
  
   Follow [[TUMFTM](https://github.com/TUMFTM)] for more information on HDMap & SDMap aided localization, I will do supplement when I have time.
   
   Follow [[Tong Qin](https://github.com/qintonguav)] for more information on SDMap aided localization, I will do supplement when I have time.

   Follow [[Yujiao Shi](https://github.com/YujiaoShi/)] and [[tudelft-iv](https://github.com/tudelft-iv)] for more information on Satellite images aided localization, I will do supplement when I have time.

----
## WhyTraversability

   For all terrain navigation, because We're tired of filtering ground points...
   


* **salon**: "Off-road driving by learning from interaction and demonstration". [[Code](https://github.com/castacks/salon_ws)] [[Project](https://theairlab.org/offroad)]

* **MonoForce**: "MonoForce: Self-supervised Learning of Physics-aware Model for Predicting Robot-terrain Interaction", *arXiv 2022*. [[Paper](https://arxiv.org/pdf/2309.09007)] [[Code](https://github.com/ctu-vras/traversability_estimation)]

* **TML**: "A Global Traversability Mapping Library easily integratable with any SLAM System". [[Code](https://github.com/suchetanrs/traversability_mapping)]

* **TMMP**: "Bayesian Generalized Kernel Inference for Terrain Traversability Mapping", *the 2nd Annual Conference on Robot Learning*. [[Code](https://github.com/TixiaoShan/traversability_mapping)]

* **STEPP**: "STEPP: Semantic Traversability Estimation using Pose Projected features", *not yet*. [[Code](https://github.com/RPL-CS-UCL/STEPP-Code)]

* **EcSemMap**: "Uncertainty-aware Evidential Bayesian Semantic Mapping (EBS)", [[Code](https://github.com/junwon-vision/EvSemMap)]

* **LoD**: "Learning-on-the-Drive: Self-supervised Adaptive Long-range Perception for High-speed Offroad Driving", [[Paper](https://arxiv.org/pdf/2306.15226)]

* **tapdo**: "tadpo", [[Code](https://github.com/tadpo-iclr/tadpo)]

* **ROLO-SLAM**: "ROLO-SLAM: Rotation-Optimized LiDAR-Only SLAM in Uneven Terrain with Ground Vehicle", [[Code](https://github.com/sdwyc/ROLO)]

* **MGGPlanner**: "Multi-robot Grid Graph Exploration Planner", [[Code](https://github.com/MISTLab/MGGPlanner)]

* **GroundGrid**: "GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation", [[Code](https://github.com/dcmlr/groundgrid)]

* **Holistic Fusion**: "Holistic Fusion: Task and Setup-agnostic Robot Localization and State Estimation with Factor Graphs", [[Code](https://github.com/leggedrobotics/holistic_fusion)]



   Follow [[Jianhao Jiao](https://github.com/gogojjh)] for more information on Intelligent Nav, I will do supplement when I have time.

----
## WhyLongTerm

   For multi-session mapping and updating (change detection), dynamic objects filter.

* **LT-Mapper**: "Lt-mapper: A modular framework for lidar-based lifelong mapping", *2022 International Conference on Robotics and Automation (ICRA)*.  [[Paper](https://github.com/gisbi-kim/lt-mapper/blob/main/doc/ltmapper.pdf)] [[Code](https://github.com/gisbi-kim/lt-mapper)]

* **RoLL**: "ROLL: Long-Term Robust LiDAR-based Localization With Temporary Mapping in Changing Environments", *arxiv 2025*. [[Code](https://github.com/HaisenbergPeng/ROLL)]

* **Elite**: "Ephemerality meets LiDAR-based Lifelong Mapping", *arxiv 2025*. [[Code](https://github.com/dongjae0107/ELite)]

* **SOLiD**: "SOLiD: Spatially Organized and Lightweight Global Descriptor for FOV-constrained LiDAR Place Recognition", *IEEE ROBOTICS AND AUTOMATION LETTERS 2024*. [[Code](https://github.com/sparolab/SOLiD)]

* **HeLiOS**: "HeLiOS: Heterogeneous LiDAR Place Recognition via Overlap-based Learning and Local Spherical Transformer", *2025 International Conference on Robotics and Automation (ICRA)*. [[Code](https://github.com/minwoo0611/HeLiOS)]

* **HHRM**: " Lifelong 3D Mapping Framework for Hand-held & Robot-mounted LiDAR Mapping Systems", *IEEE ROBOTICS AND AUTOMATION LETTERS 2024*. [[Paper](https://arxiv.org/pdf/2501.18110)]

* **LiLoc**: " LiLoc: Lifelong Localization using Adaptive Submap Joining and Egocentric Factor Graph", *2025 International Conference on Robotics and Automation (ICRA)*. [[Code](https://github.com/Yixin-F/LiLoc)]

* **KISS-Matcher**: "KISS-Matcher: Fast and Robust Point Cloud Registration Revisited", *arxiv 2024*. [[Paper](https://arxiv.org/abs/2409.15615)] [[Code](https://github.com/MIT-SPARK/KISS-Matcher)]

* **G3Reg**: "Pyramid Semantic Graph-based Global Point Cloud Registration with Low Overlap", *IEEE/RSJ International Conference on Intelligent Robots and Systems 2023*. [[Code](https://github.com/HKUST-Aerial-Robotics/G3Reg)]

* **SG-Reg**: "SG-Reg: Generalizable and Efficient Scene Graph Registration", *TRO under review*. [[Code](https://github.com/HKUST-Aerial-Robotics/SG-Reg)]

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

* **ROAM**: "Riemannian Optimization for Active Mapping with Robot Teams", *IEEE Transactions on Robotics 2025*. [[Project](https://existentialrobotics.org/ROAM_webpage/)]

* **BlockMap**: "Block-Map-Based Localization in Large-Scale Environment", *IEEE International Conference on Robotics and Automation 2024*.  [[Paper](https://arxiv.org/pdf/2404.18192)] [[Code](https://github.com/YixFeng/Block-Map-Based-Localization)]

* **SLIM**: "SLIM: Scalable and Lightweight LiDAR Mapping in Urban Environments", *arxiv maybe tro 2024*.  [[Paper](https://arxiv.org/pdf/2409.08681)] [[Code](https://github.com/HKUST-Aerial-Robotics/SLIM)]
  
reduce z-axis drift

* **Norlab-icp**: "A 2-D/3-D mapping library relying on the "Iterative Closest Point" algorithm", [[Code](https://github.com/norlab-ulaval/norlab_icp_mapper)]

* **SuperOdometry**: "SuperOdometry: Lightweight LiDAR-inertial Odometry and Mapping", [[Code](https://github.com/superxslam/SuperOdom)]

* **RING#**: "RING#: PR-by-PE Global Localization with Roto-translation Equivariant Gram Learning", [[Code](https://github.com/lus6-Jenny/RINGSharp)]






   Follow [[Ji Zhang](https://github.com/jizhang-cmu)] for more information on Robust Navigation, I will do supplement when I have time.

   Follow [[Xiang Gao](https://github.com/gaoxiang12)] for more information on SLAM, I will do supplement when I have time.

   Follow [[Tong Qin](https://github.com/qintonguav)] for more information on V-SLAM, I will do supplement when I have time.
   
   Follow [[Gisbi Kim](https://github.com/gisbi-kim/lt-mapper)] for more information on Map Maintaining, I will do supplement when I have time.

   Follow [[Qingwen Zhang](https://github.com/Kin-Zhang)] for more information on Dynamic Objects Removing, I will do supplement when I have time.

   Follow [[Zhijian Qiao](https://github.com/qiaozhijian)] for more information on Multi-Session Mapping, I will do supplement when I have time.




   Engineering:

   Follow [[koide](https://github.com/koide3)] for more information on Life-Long LiDAR Mapping, I will do supplement when I have time.

   Follow [[Zikang Yuan](https://github.com/ZikangYuan)] for more information on LIO, I will do supplement when I have time.

   Follow [[Xiangcheng Hu](https://github.com/JokerJohn)] for more information on Life-Long LiDAR Mapping, I will do supplement when I have time.

   Follow [[Chengwei Zhao](https://github.com/chengwei0427)] for more information on LiDAR SLAM, I will do supplement when I have time.

   Follow [[Heming Liang](https://github.com/liangheming)] for more information on LiDAR SLAM, I will do supplement when I have time.




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
