---
description: Decisões estruturais do AQUA-SLAM e o porquê delas. Abrir ao mexer em arquitetura, fronteiras de módulo, no factor graph ou ao avaliar uma mudança grande.
sources: [src/System.cc, src/Tracking.cc, src/LocalMapping.cc, src/Optimizer.cc, src/G2oTypes.cc, src/DVLGroPreIntegration.cpp, src/DvlGyroOptimizer.cpp, src/ros_stereo_DVL_tighly.cc, src/RosHandling.cpp, include/ImuTypes.h]
---

# Arquitetura

> Decisões estruturais e o **porquê** delas. O código mostra o "o quê"; aqui fica o "porquê".

## Visão geral

AQUA-SLAM herda a arquitetura de três threads do ORB-SLAM3 e acrescenta a fusão do DVL:

```
                nó ROS  stereo_dvl2  (src/ros_stereo_DVL_tighly.cc, main)
                 subscribe: câmera L/R, /imu/data, /dvl/data (hardcoded)
                 SyncWithImu2(): funde IMU+DVL numa sequência temporal de GyroDvlPoint
                                    │  TrackStereoGroDVL(imL, imR, t, vGyroDVLMeas, bDVL)
                                    ▼
   ┌──────────────────────  System (eSensor = DVL_STEREO = 5)  ──────────────────────┐
   │  Thread Tracking       Thread LocalMapping        Thread LoopClosing            │
   │  (Tracking.cc)         (LocalMapping.cc)          (LoopClosing.cc)              │
   │   - ORB / KLT           - BA local (g2o)           - DBoW2 place recognition    │
   │   - matching/PnP        - init IMU/DVL             - Sim3 / correção de loop    │
   │   - pré-integr. DVL     - inserção de KeyFrame     - fusão de mapas (Atlas)     │
   └───────────────────────────────────┬───────────────────────────────────────────┘
                Atlas (multi-mapa) · Optimizer (factor graph g2o) · DVLGroPreIntegration
                                        │
                                        ▼
                RosHandling  →  /AQUA_SLAM/*  (pose, odom, path, sparse_map, octomap)
```

## Decisões de arquitetura (ADRs leves)

### Base em ORB-SLAM3 (fork), não implementação do zero
- **Contexto:** precisavam de um SLAM visual maduro (multi-mapa Atlas, loop closing, relocalização) sobre o qual acoplar o DVL.
- **Decisão:** derivar do ORB-SLAM3 (namespace `ORB_SLAM3::`, cabeçalhos de licença preservados) e adicionar módulos DVL.
- **Por quê:** reaproveitar front-end/back-end/loop closing testados; concentrar o esforço no diferencial (fusão acústica).
- **Consequências:** o grosso do código é ORB-SLAM3 quase inalterado; o valor novo está em poucos arquivos (`DVLGroPreIntegration`, `DvlGyroOptimizer`, `G2oTypes` DVL, `dvl_model/dvl.cpp`, `ImuTypes` `Calib`). Ao navegar, distinga "herança" de "diferencial" — ver `glossary.md`.

### Fusão tightly-coupled num único factor graph g2o (sem GTSAM/Ceres)
- **Contexto:** acoplar visão + IMU + DVL de forma ótima.
- **Decisão:** um factor graph g2o com vértices/arestas customizados para DVL; **g2o vendorizado** em `Thirdparty/g2o`. GTSAM e Ceres **não** são usados.
- **Por quê:** o ORB-SLAM3 já usa g2o; manter um só otimizador reduz atrito.
- **Consequências:** todos os fatores DVL são tipos g2o próprios em `include/G2oTypes.h` (ver "Factor graph" abaixo).

### DVL como sensor de velocidade acústica, com pré-integração análoga à da IMU
- **Contexto:** o DVL mede velocidade em relação ao fundo por 4 feixes acústicos; ancora escala e reduz deriva onde a visão falha.
- **Decisão:** classe `DVLGroPreIntegration` pré-integra velocidade do DVL + giroscópio entre keyframes; `dvl_model/dvl.cpp` converte as 4 velocidades de feixe em velocidade 3D via mínimos quadrados `v = (EᵀE)⁻¹ Eᵀ v_beam`.
- **Por quê:** espelhar o padrão de pré-integração IMU do ORB-SLAM3 facilita encaixar o DVL no mesmo grafo.
- **Consequências:** a geometria dos feixes entra como `alpha`/`beta` (orientação dos 4 transdutores) no YAML.

### Calibração online da orientação dos feixes do DVL
- **Contexto:** a orientação real dos transdutores do DVL é difícil de medir com precisão e o título do paper é "…with Sensor Calibration".
- **Decisão:** estado de calibração `VertexDVLBeamOritenstion` (dim 8) refinado por `EdgeDVLBeamCalibration1/2`; a flag `Optimizer.calibrated` (YAML) escolhe entre rodar já-calibrado ou calibrando.
- **Por quê:** tirar erro sistemático dos feixes melhora escala/velocidade sem depender de calibração externa perfeita.
- **Consequências:** o DVL **não** precisa (nem é coberto por) uma ferramenta externa de calibração como o Kalibr — ele se auto-calibra. Extrínsecos câmera-IMU e ruído da IMU, sim, vêm de fora (Kalibr). Ver `domain/sensor-fusion.md`.

### Inicialização IMU/DVL disparada por movimento
- **Contexto:** escala e biases só são observáveis com excitação suficiente.
- **Decisão:** inicialização disparada quando o deslocamento passa de `IMUInitTranslation` ou a rotação passa de `IMUInitRotation` (`LocalMapping.cc`).
- **Consequências:** sequências curtas/pouco excitadas podem não inicializar; os dois YAMLs de exemplo diferem justamente nesse limiar (10.0 vs 3.0).

## Interface ROS (contrato externo)

O "contrato" do sistema são os tópicos ROS, não uma API web. Entrada (nomes vêm do YAML, exceto DVL):
`LeftImgTopic`/`RightImgTopic` (imagens, via `image_transport`, hint `compressed`), `ImuTopic` (`sensor_msgs/Imu`),
e **DVL hardcoded em `/dvl/data`** (`waterlinked_a50_ros_driver/DVL`) — o campo `DvlTopic` do YAML é lido mas
**não** usado no subscribe ativo. Saída sob o prefixo `/AQUA_SLAM/*` (`RosHandling.cpp`): `orb_pose`, `orb_odom`,
`orb_path`, `camera_pose`, `sparse_map` (PointCloud2), `octomap`, `markers`, imagens de debug.

## O que está "pela metade" de propósito

- **Crash em sequências longas** (README §7): problema de multithreading conhecido, ainda não resolvido. Não "conserte" reescrevendo a concorrência sem alinhar.
- **`Tbc` desativado:** o parsing de `Tbc` em `Tracking.cc` está **comentado**; os extrínsecos ativos são `T_gyro_c` e `T_dvl_c`. Não reative sem entender o efeito.
- **Tracking KLT** (`FrameKLT`, `LKTracker`): caminho alternativo desligado por padrão (`VisualIntegration: 0`).
- **DenseMapper:** reconstrução densa estéreo (SGBM) opcional, desligada por padrão (`DenseMapper.enable: 0`).
- **Fontes com `main` não compilados:** `ros_stereo_DVL_tighly_rovco.cc`, `stereo_dvl_ros.cpp`, `g2o_BA.cpp`, `src/test/*` estão comentados no CMake — variantes/experimentos, não o alvo de build.
- **Paths hardcoded do autor:** launch files têm `/home/da/...` em `out_path`/`traj_path`/`map_file`; ajustar por ambiente.

## Fronteiras de módulo

- O **único ponto de entrada de calibração** (intrínsecos, extrínsecos, ruído IMU, feixes DVL) é `Tracking::ParseIMUParamFile` (`src/Tracking.cc`, ~linhas 760–916). Mudanças de formato de calibração começam aqui.
- Extrínsecos ficam encapsulados na classe `IMU::Calib` (`include/ImuTypes.h`), acessível via `KeyFrame::mImuCalib`. Consumidores: `RosHandling.cpp`, `DenseMapper.cpp`, otimizadores DVL.
- Otimização concentrada em `Optimizer.cc` + `DvlGyroOptimizer.cpp`; tipos g2o em `G2oTypes.cc`. Nada fora daí deve montar arestas g2o.
- `RosHandling.cpp` é a única fronteira de I/O ROS de saída; o núcleo SLAM não conhece ROS diretamente.

## Factor graph (resumo do que é próprio do AQUA-SLAM)

Além dos tipos ORB-SLAM3 (`VertexPose`, `EdgeMono/Stereo/Inertial`, `VertexVelocity`, `VertexGyroBias`, `VertexAccBias`), há em `include/G2oTypes.h`:
- `VertexPoseDvlIMU` / `DvlImuCamPose` — pose acoplada DVL-IMU-câmera.
- `VertexDVLBeamOritenstion` (dim 8) — estado de calibração online dos feixes (alpha/beta).
- Arestas DVL: `EdgeSE3DVLPoseOnly`, `EdgeSE3DVLIMU`, `EdgeDvlGyroTrack`, `EdgeDvlGyroInit(2/3)`, `EdgeDvlIMU(2)`, `EdgeDvlVelocity`, `EdgeDVLRefine`, `EdgeDvlGyroBA`, `EdgeStereoBA_DvlGyros`, `EdgeMonoBA_DvlGyros`.
- `EdgeDVLBeamCalibration1/2` — refino online de alpha/beta.
