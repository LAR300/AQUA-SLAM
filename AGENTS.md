# AGENTS.md

> Briefing para agentes de IA. Mantenha enxuto — carrega em toda sessão. Detalhe em `.ai/docs/`.

## Visão geral do projeto

AQUA-SLAM: SLAM subaquático **tightly-coupled** que funde câmeras estéreo + IMU + **DVL** (Doppler
Velocity Log). Derivado do **ORB-SLAM3**, com fusão própria DVL+giroscópio e calibração online do DVL.
**O pacote ROS chama-se `ORB_DVL2`** e o executável é `stereo_dvl2` — "AQUA-SLAM" é só o nome de publicação.

## Stack

Resumo aqui; detalhes e versões em `.ai/docs/stack.md`.

- Linguagem: C++17
- Build: catkin / CMake (ROS 1 **Noetic**), compilado com **clang-10**
- Otimização: **g2o** (vendorizado); DBoW2 e octomap também vendorizados. **Sem** GTSAM/Ceres.
- Dep. ROS externa: `waterlinked_a50_ros_driver` (mensagens do DVL A50)

## Comandos canônicos (não adivinhe)

```bash
# Ambiente (na raiz do repo)
cd docker && docker compose build && docker compose up -d && docker exec -it orb_dvl2_ros_noetic bash

# Build (dentro do container, em /root/catkin_ws)
catkin_make -DCMAKE_CXX_COMPILER=/usr/bin/clang++-10

# Vocabulário DBoW2 (baixar uma vez, ver README §3) e depois:
source devel/setup.bash

# Rodar (dentro do container)
roslaunch ORB_DVL2 blue_gx5_StructureEasy.launch
# Em outro terminal: rosbag play <sequencia>.bag
```

## Estrutura

- Código-fonte: `src/` + `include/`
- Entrypoint (nó ROS): `src/ros_stereo_DVL_tighly.cc` → executável `stereo_dvl2`
- Config/calibração: `data/*.yaml` (formato OpenCV FileStorage)
- Launch: `launch/blue_gx5_*.launch`

## Onde está o contexto detalhado

Leia sob demanda, conforme a tarefa:

- Visão geral: `.ai/docs/overview.md`
- Arquitetura e decisões: `.ai/docs/architecture.md`
- Stack completa e armadilhas de build: `.ai/docs/stack.md`
- Convenções de código: `.ai/docs/conventions.md`
- Vocabulário do domínio: `.ai/docs/glossary.md`
- **Fusão de sensores e integração de calibração (Kalibr):** `.ai/docs/domain/sensor-fusion.md`

## Regras inegociáveis

- Nunca commitar segredos. Datasets, vocabulário e bags ficam fora do versionamento.
- Distinga **herança ORB-SLAM3** de **diferencial AQUA-SLAM** antes de editar (ver `architecture.md`).
- **Calibração entra só pelo YAML de `data/`**, lida em `src/Tracking.cc` (`ParseIMUParamFile`). Não hardcode.
- Não "corrija" grafias de identificadores (`Oritenstion`, `tighly`, `GroPreIntegration`) — quebra o build.
- Vários fontes com `main` estão comentados no CMake de propósito; o alvo é `stereo_dvl2`.

## Diretrizes de PR / commit

- `NEEDS CLARIFICATION`: padrão de commit/branch não é imposto pelo repo. Confirmar com o dono.
