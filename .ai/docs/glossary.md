---
description: Termos do domínio do AQUA-SLAM (SLAM, sensores, fusão subaquática). Abrir quando aparecer um termo cujo significado não é óbvio.
sources: [README.md, src/, include/, data/]
---

# Glossário do domínio

- **AQUA-SLAM / ORB_DVL2:** o mesmo sistema. "AQUA-SLAM" é o nome da publicação; `ORB_DVL2` é o nome do pacote ROS no código, launch e CMake.
- **DVL (Doppler Velocity Log):** sensor acústico que mede a velocidade do veículo em relação ao fundo por efeito Doppler em 4 feixes. Ancora escala e reduz deriva quando visão/IMU degradam. Modelo Water Linked A50.
- **Feixe (beam) / alpha, beta:** cada um dos 4 transdutores do DVL. `alpha`/`beta` (vetores 4×1 no YAML) descrevem a orientação de cada feixe; alpha ≈ inclinação em relação ao plano horizontal, beta ≈ rotação em torno de z. São **calibrados online** pelo próprio sistema.
- **IMU (Inertial Measurement Unit):** giroscópio + acelerômetro. Aqui, na prática, o **giroscópio** é o que entra na fusão DVL (`T_gyro_c`).
- **Tightly-coupled:** os dados brutos dos sensores entram juntos num único problema de otimização (factor graph), em vez de fundir estimativas já prontas (loosely-coupled).
- **Pré-integração:** somar medidas inerciais/DVL de alta taxa entre dois keyframes numa medida relativa única, para não colocar cada amostra no grafo. `DVLGroPreIntegration` faz isso para giroscópio+DVL.
- **Extrínseco (T_a_b):** transformação rígida 4×4 entre frames de sensores. `T_gyro_c` (IMU→câmera), `T_dvl_c` (DVL→câmera), `Tbc` (body↔câmera, desativado). Convenção de direção: **confirmar** ao integrar com Kalibr — ver `domain/sensor-fusion.md`.
- **Intrínseco:** parâmetros internos da câmera (`fx, fy, cx, cy`, distorção). `Camera.bf` = baseline estéreo × fx.
- **Factor graph / g2o:** grafo de otimização onde vértices são estados (poses, velocidades, biases, calibração de feixes) e arestas são resíduos (reprojeção, IMU, DVL). Resolvido com a lib g2o vendorizada.
- **Atlas:** estrutura multi-mapa do ORB-SLAM3 (vários submapas que podem ser fundidos após loop closing). Serializável (`.osa`, boost::serialization).
- **DBoW2 / ORBvoc.txt:** bag-of-words visual para reconhecimento de lugar (loop closing e relocalização). Vocabulário baixado à parte.
- **Keyframe (KF):** frame selecionado que entra no mapa e na otimização. `IMU::Calib` fica acessível via `KeyFrame::mImuCalib`.
- **Noise density / random walk (IMU):** parâmetros de ruído do sensor. `IMU.NoiseGyro`/`NoiseAcc` (densidade), `IMU.GyroWalk`/`AccWalk` (random walk). Vêm da calibração (ou do Kalibr).
- **Tank Dataset:** conjunto de ROS bags subaquáticos (câmera estéreo + IMU + DVL) usado para avaliar o sistema.
- **DenseMapper:** reconstrução densa estéreo (SGBM) opcional, desligada por padrão.
