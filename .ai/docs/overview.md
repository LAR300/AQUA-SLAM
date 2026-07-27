---
description: Visão geral do AQUA-SLAM em uma página — o que é, para quem e como as peças se encaixam. Abrir primeiro, para situar-se no todo antes de mergulhar nos docs específicos.
sources: [README.md, package.xml, src/ros_stereo_DVL_tighly.cc, src/System.cc]
---

# Visão geral

> O "mapa" do projeto em uma página. É o primeiro doc que a IA lê pra se situar; o `AGENTS.md`
> aponta pra cá. O detalhe vive nos docs específicos.

## O que é

AQUA-SLAM é um sistema de **SLAM subaquático tightly-coupled acústico-visual-inercial**: funde
**câmeras estéreo**, **IMU** e **DVL (Doppler Velocity Log)** para estimar pose e reconstruir o
ambiente em cenários subaquáticos, onde câmera e IMU sozinhos degradam. É um derivado do
**ORB-SLAM3** com módulos próprios de fusão DVL + giroscópio e calibração online do DVL.

Publicação: Xu, Zhang, Wang — *"AQUA-SLAM: Tightly-Coupled Underwater Acoustic-Visual-Inertial SLAM
with Sensor Calibration"*, IEEE Transactions on Robotics, 2025.

> **Nome do pacote:** "AQUA-SLAM" é o nome de publicação. O pacote ROS real chama-se **`ORB_DVL2`**
> — todos os `roslaunch`, o `package.xml` e o CMake usam esse nome. O executável é `stereo_dvl2`.

## Problema que resolve

Debaixo d'água a estimativa de pose é difícil: pouca luz e turbidez degradam a visão; a IMU deriva
sem correção de escala; não há GPS. O DVL mede velocidade em relação ao fundo por acústica e ancora
a escala e a deriva. AQUA-SLAM acopla os três sensores num único factor graph e ainda **calibra
online** a orientação dos feixes do DVL, reduzindo a dependência de calibração manual perfeita.

## Como as peças se encaixam

```
ROS bag / sensores
  câmera estéreo (image_transport) ─┐
  IMU (/imu/data) ──────────────────┤→ nó stereo_dvl2 (ros_stereo_DVL_tighly.cc)
  DVL (/dvl/data) ──────────────────┘        │
                                             ▼
                        System (eSensor = DVL_STEREO)
                          ├─ Tracking      (front-end ORB + fusão)
                          ├─ LocalMapping  (BA local + init IMU/DVL)
                          ├─ LoopClosing   (DBoW2)
                          └─ Optimizer     (factor graph g2o: visual + IMU + DVL)
                                             │
                                             ▼
                        RosHandling → /AQUA_SLAM/* (pose, odom, path, mapa)
```

Toda a calibração (intrínsecos de câmera, extrínsecos câmera-IMU e câmera-DVL, ruído da IMU,
geometria dos feixes do DVL) é lida de **um único YAML** em `data/` — ver `architecture.md` e
`domain/sensor-fusion.md`. Detalhes de fluxo em `architecture.md`.

## Estado atual

Código de pesquisa que acompanha a publicação, rodável via Docker com o "Tank Dataset". Funciona
end-to-end nas sequências fornecidas. **Problema conhecido** (README §7): pode travar aleatoriamente
em sequências longas por questões de multithreading — reiniciar quando ocorrer. Há paths hardcoded
do ambiente original do autor nos launch files (`/home/da/...`).

## Por onde começar a ler

- Arquitetura e o porquê → `architecture.md`
- Stack e versões → `stack.md`
- Convenções → `conventions.md`
- Fusão de sensores e integração de calibração → `domain/sensor-fusion.md`
- Vocabulário → `glossary.md`
