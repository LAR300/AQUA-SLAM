---
description: Linguagens, frameworks e libs do AQUA-SLAM com versões, e armadilhas de build. Abrir ao adicionar dependência, mexer no CMake/Docker ou depurar build.
sources: [CMakeLists.txt, package.xml, docker/Dockerfile, docker/docker-compose.yml, Thirdparty/]
---

# Stack

> Versões importam: o container é a fonte de verdade do que realmente compila.

## Runtime e linguagem
- **C++17** (`-std=c++17`, `CMakeLists.txt`).
- Build type `RelWithDebInfo`.

## Build
- **catkin / CMake ≥ 3.0.2**, ROS 1.
- Comando oficial (README, dentro do container): `catkin_make -DCMAKE_CXX_COMPILER=/usr/bin/clang++-10`.
- Compilador recomendado: **clang-10** (GCC também funciona).
- Pacote ROS: **`ORB_DVL2`** (não "AQUA-SLAM"). Executável compilado: **`stereo_dvl2`**.

## ROS
- **ROS 1 Noetic** (Ubuntu 20.04) no Docker. README diz testado em Ubuntu 20.04 e 22.04.

## Frameworks e bibliotecas principais
| Lib | Versão | Observação |
|---|---|---|
| OpenCV | CMake pede **3.2**; Docker compila **3.4.16 + contrib** | ⚠️ mismatch CMake×Docker — ver armadilhas |
| Eigen3 | **3.3** (do sistema, `libeigen3-dev`) | ver armadilha do `Eigen3_DIR` |
| Pangolin | commit fixo `bcf41ec8` | viewer |
| PCL | `libpcl-dev` (Noetic) | nuvem de pontos |
| fmt | **9.1.0** (`fmt::fmt-header-only`) | |
| Sophus | **1.22.10** | grupos de Lie |
| Boost | `log_setup, log, serialization, system, filesystem, iostreams` | |
| g2o | **vendorizado** em `Thirdparty/g2o` | back-end de otimização |
| DBoW2 | **vendorizado** em `Thirdparty/DBoW2` | loop detection / vocabulário |
| octomap | **vendorizado** em `Thirdparty/octomap` | mapa de ocupação |
| crypto | `-lcrypto` | |

**Não usados:** GTSAM, Ceres. Toda otimização é g2o.

## Dependência ROS externa
- **`waterlinked_a50_ros_driver`** — driver do DVL Water Linked A50; fornece as mensagens `DVL`/`DVLBeam`.
  Clonado no Docker de `github.com/SenseRoboticsLab/dvl_a50_ros_driver`. Sem ele, o nó não compila nem assina o DVL.

## Recursos externos (não versionados)
- **Vocabulário DBoW2**: `Vocabulary/ORBvoc.txt`, baixado via `gdown` (Google Drive) — ver README §3.
- **Datasets**: ROS bags do "Tank Dataset" (`Structure_Easy.bag` etc.), baixados de
  senseroboticslab.github.io/underwater-tank-dataset. `dataset/` vem vazio (só `Download_Data_Here`).

## Docker
- `docker/Dockerfile` (base `ubuntu:20.04`): ROS Noetic desktop-full, catkin-tools; compila do fonte OpenCV 3.4.16+contrib, Pangolin, fmt 9.1.0, Sophus 1.22.10; PCL/Eigen/Boost via apt; clona o driver do DVL. Cria `/root/catkin_ws`.
- `docker/docker-compose.yml`: serviço `orb_dvl2_ros`, container `orb_dvl2_ros_noetic`, `network_mode: host`, `privileged`, `/dev/dri` (OpenGL), X11 para RViz; monta o repo em `/root/catkin_ws/src/AQUA_SLAM`.

## Armadilhas de build (ler antes de mexer no CMake)
- **OpenCV**: `CMakeLists.txt` faz `find_package(OpenCV 3.2 REQUIRED)`, mas o Docker provê 3.4.16. Alinhe a versão pedida ao ambiente real ao portar.
- **Eigen**: o CMake seta `Eigen3_DIR=Thirdparty/eigen3.3.4`, mas **esse diretório não existe** no repo — o build cai no Eigen do sistema. Não confie no path vendorizado.
- **Paths hardcoded do autor**: há `/home/da/Matlab/...` no CMake e `/home/da/project/...` nos launch files. São resíduos do ambiente original; não quebram o build padrão, mas ajuste ao seu ambiente.
