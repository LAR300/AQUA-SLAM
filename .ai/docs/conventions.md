---
description: Estilo de código, organização de arquivos e convenções do AQUA-SLAM (herdadas do ORB-SLAM3). Abrir antes de escrever ou revisar código.
sources: [src/, include/, CMakeLists.txt, launch/, data/]
---

# Convenções

> Muita coisa é herdada do ORB-SLAM3. Ao editar, siga o padrão do arquivo que você está mexendo.

## Estilo de código
- **C++17**, estilo ORB-SLAM3: classes em `PascalCase`; membros com prefixo `m` (`mpSLAM`, `mImuCalib`, `mVelocityFromBeam`); ponteiros com prefixo `p` (`mpSLAM`), vetores/containers com `v`/`mv` (`vGyroDVLMeas`).
- Namespace `ORB_SLAM3::` em quase todo o núcleo (herança do fork).
- Extensões misturadas: código herdado do ORB-SLAM3 usa `.cc`; módulos novos (DVL/KLT/ROS) usam `.cpp`. Mantenha a extensão do módulo ao editar; não renomeie.
- Há erros de ortografia intencionais em identificadores públicos (ex: `VertexDVLBeamOritenstion`, `DVLGroPreIntegration`, `tighly`). **Não "corrija"** — quebra a compilação e o histórico.

## Organização de arquivos
- `src/` + `include/` espelham os nomes (`Foo.cc`/`Foo.cpp` ↔ `Foo.h`). Módulos de câmera em `*/CameraModels/`; modelo do DVL em `src/dvl_model/`.
- **Distinga herança de diferencial** ao navegar: arquivos ORB-SLAM3 (Tracking, LocalMapping, Optimizer, ORB*, Atlas, KeyFrame…) vs. módulos AQUA-SLAM (`DVLGroPreIntegration`, `DvlGyroOptimizer`, `dvl_model/dvl`, partes DVL de `G2oTypes`/`ImuTypes`, `RosHandling`). Ver `architecture.md`.
- **Config vive em YAML, não em código.** Novos parâmetros de calibração/tuning entram no YAML de `data/` e são lidos em `Tracking.cc` (`ParseIMUParamFile`), não hardcoded.

## CMake / build
- Um `add_executable` real hoje: `stereo_dvl2`. Vários fontes com `main` estão **comentados** no CMake de propósito (variantes/experimentos) — para reativar um, descomente conscientemente.
- Bibliotecas: `ORB_DVL2` (núcleo) e `ORB_DVL2_Optmization` (otimização). Coloque tipos g2o/otimização na segunda.

## Tópicos e YAML
- Nomes de tópicos de entrada vêm do YAML (`LeftImgTopic`, `RightImgTopic`, `ImuTopic`) — **exceto o DVL, hardcoded em `/dvl/data`** no nó. Saídas usam o prefixo `/AQUA_SLAM/`.
- YAML de calibração é formato **OpenCV FileStorage** (`%YAML:1.0`, matrizes `!!opencv-matrix`), não YAML "puro". Respeite esse formato ao editar `data/*.yaml`.

## Commits e Git
- Histórico atual é enxuto e informal (ex.: commit `update`). Não há convenção de commit imposta pelo repo.
- **Consulte o dono para o padrão de commit/branch** deste projeto — `NEEDS CLARIFICATION`.

## Princípios
- Preserve comportamento herdado do ORB-SLAM3 salvo motivo claro. YAGNI, DRY. Ao mudar comportamento, atualize o YAML de exemplo e a doc correspondente.
