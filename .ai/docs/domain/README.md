---
description: Índice das regras de domínio do AQUA-SLAM (uma por arquivo). Abrir ao mexer em comportamento de fusão de sensores, calibração ou inicialização.
sources: [src/Tracking.cc, src/DVLGroPreIntegration.cpp, src/dvl_model/dvl.cpp, include/ImuTypes.h, data/]
---

# Domínio — regras de negócio

> Regras detalhadas do comportamento do sistema, uma por arquivo.

- `sensor-fusion.md` — fusão câmera + IMU + DVL, modelo do DVL, e **como os parâmetros de calibração
  entram no sistema** (ponto de integração com o Kalibr).

Adicione um arquivo quando uma regra (ex: inicialização IMU/DVL, calibração online dos feixes)
ficar complexa o bastante para merecer detalhamento próprio.
