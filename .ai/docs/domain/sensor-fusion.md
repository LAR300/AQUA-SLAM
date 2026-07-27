---
description: Como o AQUA-SLAM funde câmera + IMU + DVL e como os parâmetros de calibração entram no sistema. Abrir ao mexer em fusão, calibração, extrínsecos, ruído da IMU, ou ao integrar com uma ferramenta de calibração (Kalibr).
sources: [src/Tracking.cc, src/ros_stereo_DVL_tighly.cc, src/DVLGroPreIntegration.cpp, src/dvl_model/dvl.cpp, src/G2oTypes.cc, include/ImuTypes.h, include/G2oTypes.h, data/underwater_orbslam3_blue_gx5_short.yaml]
---

# Fusão de sensores e integração de calibração

> Este é o doc mais importante para a integração futura com o Kalibr. Descreve **onde** e **como**
> os parâmetros de calibração entram no AQUA-SLAM.

## O ponto de entrada único: o YAML de `data/`

Toda a calibração é lida de **um único arquivo YAML** (formato OpenCV FileStorage, `%YAML:1.0`),
passado como 2º argumento do `stereo_dvl2`. O parsing acontece em **`src/Tracking.cc`**, no bloco
`ParseIMUParamFile` (aprox. linhas 760–916) e no parse de câmera (aprox. 226–299). Nenhum outro
lugar lê calibração. Arquivos de exemplo: `data/underwater_orbslam3_blue_gx5_short.yaml` e `_long.yaml`.

## Parâmetros consumidos

### 1. Intrínsecos da câmera (Tracking.cc ~226–299)
- `Camera.fx, Camera.fy, Camera.cx, Camera.cy` — foco e ponto principal.
- `Camera.k1, k2, p1, p2` (e `k3`) — distorção. `Camera.type`: `"PinHole"` ou KannalaBrandt (fisheye).
- `Camera.bf` — **baseline estéreo × fx** (não é a baseline em metros).
- `Camera.width/height/fps/RGB`, `ThDepth`, `ThFarDepth`, `ImageScale`.
- Para estéreo de 2 câmeras/fisheye há `Camera2.*` e `Tlr` (extrínseco left↔right).

### 2. Extrínsecos entre sensores (Tracking.cc ~764–803), matrizes 4×4 `!!opencv-matrix`
- **`T_gyro_c`** — transform **IMU(giroscópio) → câmera**. Comentário do código diz "only rotation,
  set translation 0 0 0", mas o YAML de exemplo traz translação não-nula. `NEEDS CLARIFICATION`:
  confirmar em `G2oTypes.cc`/`DVLGroPreIntegration.cpp` se a translação de `T_gyro_c` é efetivamente usada.
- **`T_dvl_c`** — transform **DVL → câmera**.
- **`Tbc`** — presente no YAML, mas o parsing está **comentado** em `Tracking.cc` → fica vazio. Extrínsecos
  ativos são `T_gyro_c` e `T_dvl_c`. Não reative `Tbc` sem entender o efeito.

### 3. Ruído / random walk da IMU (Tracking.cc ~843–916)
- `IMU.Frequency` (Hz, int), `IMU.NoiseGyro` (Ng), `IMU.NoiseAcc` (Na), `IMU.GyroWalk` (Ngw), `IMU.AccWalk` (Naw).
- Reescalados internamente por `sf = sqrt(freq)`: usa `Ng*sf, Na*sf, Ngw/sf, Naw/sf`. **Passe as densidades
  contínuas** (por √Hz), não valores já discretizados — o código faz a discretização.

### 4. Modelo de feixes do DVL (Tracking.cc ~807–839)
- `alpha`, `beta` — vetores 4×1 com a orientação dos 4 transdutores. Viram `Eigen::Vector4d` e vão para
  `SetBeamOrientation(alpha, beta)`. **Fora do escopo do Kalibr**: o DVL se auto-calibra online.

### Construção do objeto de calibração
```
mpImuCalib = new IMU::Calib(T_gyro_c, T_dvl_c, Ng*sf, Na*sf, Ngw/sf, Naw/sf);
```
A classe `IMU::Calib` (`include/ImuTypes.h`) calcula inversas e a composição `mT_gyro_dvl = mT_gyro_c * mT_c_dvl`.
Campos: `mT_gyro_c, mT_c_gyro, mT_dvl_c, mT_c_dvl, mT_gyro_dvl, Tbc, Tcb, Cov, CovWalk`. Fica acessível via
`KeyFrame::mImuCalib`.

## Como a fusão roda (runtime)

No nó (`src/ros_stereo_DVL_tighly.cc`, `SyncWithImu2`, ~425–674):
1. **IMU** (`/imu/data`) → buffer. Entre dois frames, as amostras (accel+gyro) viram `ImuPoint`/`GyroDvlPoint`.
2. **DVL** (`/dvl/data`, **hardcoded**) → filtragem: descarta se `!velocity_valid` ou `|v| > 1.0 m/s`. Guarda
   velocidade 3D + as 4 velocidades de feixe. Modelo geométrico em `src/dvl_model/dvl.cpp`:
   `v = (EᵀE)⁻¹ Eᵀ v_beam` (mínimos quadrados sobre a matriz de direções dos feixes E).
3. IMU e DVL são fundidos numa **sequência temporal unificada** de `GyroDvlPoint` ordenada por timestamp;
   a medida de DVL entra como "medida virtual" na sequência inercial.
4. Entregue ao núcleo via `TrackStereoGroDVL(imL, imR, t, vGyroDVLMeas, bDVL)`.

No back-end, os resíduos DVL entram no factor graph g2o como arestas próprias (ver `architecture.md` →
"Factor graph"), e a orientação dos feixes é refinada online por `EdgeDVLBeamCalibration1/2` quando
`Optimizer.calibrated = 0`.

## Ponto de integração com o Kalibr (mapa direcionado)

O Kalibr produz: intrínsecos de câmera, extrínseco câmera-IMU (`T_cam_imu`, 4×4) + `timeshift_cam_imu`,
e ruído/random-walk da IMU. Mapeando para o YAML do AQUA-SLAM:

| Saída do Kalibr | Campo no YAML do AQUA-SLAM | Cuidado |
|---|---|---|
| `intrinsics: [fx, fy, cx, cy]` | `Camera.fx/fy/cx/cy` | direto |
| `distortion_coeffs` (radtan) | `Camera.k1/k2/p1/p2` | conferir ordem dos coeficientes |
| baseline estéreo × fx | `Camera.bf` | Kalibr dá a baseline (via extrínseco entre câmeras); **derivar** `bf = baseline · fx` |
| `T_cam_imu` (4×4) | `T_gyro_c` (4×4, IMU→câmera) | **`NEEDS CLARIFICATION`: direção.** `T_gyro_c` é "gyro→câmera"; `T_cam_imu` do Kalibr é câmera↔IMU numa convenção específica — pode ser preciso **inverter**. E só o giroscópio é usado; ver dúvida da translação acima. |
| `accelerometer_noise_density` | `IMU.NoiseAcc` | densidade contínua (m/s²/√Hz) |
| `gyroscope_noise_density` | `IMU.NoiseGyro` | densidade contínua (rad/s/√Hz) |
| `accelerometer_random_walk` | `IMU.AccWalk` | m/s³/√Hz |
| `gyroscope_random_walk` | `IMU.GyroWalk` | rad/s²/√Hz |
| `update_rate` | `IMU.Frequency` | Hz |
| `timeshift_cam_imu` | — | **sem campo correspondente** no YAML atual; o AQUA-SLAM não consome timeshift explicitamente. Avaliar impacto. |
| — (Kalibr não calibra DVL) | `T_dvl_c`, `alpha`, `beta` | ficam com o AQUA-SLAM (calibração online) |

**Dúvidas de integração a resolver (`NEEDS CLARIFICATION`):**
1. Direção/convenção exata de `T_gyro_c` vs. `T_cam_imu` do Kalibr (source→target) — validar antes de plugar valores.
2. Se a translação de `T_gyro_c` é usada ou apenas a rotação (comentários divergem do YAML).
3. Onde entra o `timeshift_cam_imu` do Kalibr, já que não há campo no YAML — sincronização por hardware assumida?
4. Ordem/convenção dos coeficientes de distorção entre Kalibr (radtan) e o parse do AQUA-SLAM.
