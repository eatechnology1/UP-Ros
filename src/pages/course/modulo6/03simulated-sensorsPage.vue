<template>
  <q-page class="q-pa-lg column items-center">
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-green-4 text-weight-bold q-mb-sm">
          MÓDULO 6.3: PERCEPCIÓN SINTÉTICA
        </div>
        <h1 class="hero-title">
          Sensores Simulados: <span class="text-white">Ojos Digitales</span>
        </h1>
        <TextBlock>
          Un robot en Gazebo es sordo y ciego por defecto. Para que pueda navegar, inyectamos
          <strong>Plugins</strong>. Son pequeños programas que leen la "Matrix" (la geometría
          perfecta de Gazebo) y generan mensajes ROS con errores calculados, engañando a tu nodo de
          navegación para que crea que está en el mundo real.
        </TextBlock>
      </div>
    </section>

    <div class="section-group self-stretch">
      <SectionTitle>1. El Puente: De la Física al Tópico</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Gazebo (Física) y ROS 2 (Lógica) hablan idiomas distintos.
            <br /><br />
            El <strong>Plugin</strong> actúa como traductor en tiempo real: 1.
            <strong>Lee:</strong> Intersecta rayos en el mundo 3D. 2.
            <strong>Convierte:</strong> Transforma distancias a formato <code>sensor_msgs</code>. 3.
            <strong>Publica:</strong> Envía los datos por la red DDS.
          </TextBlock>
          <AlertBlock type="info" title="Librerías Estándar">
            Usaremos <code>gazebo_ros_pkgs</code>. Ya trae drivers virtuales para Lidars, Cámaras,
            IMUs y GPS.
          </AlertBlock>
        </template>

        <template #right>
          <div
            class="tool-card bg-slate-900 flex flex-center relative-position overflow-hidden h-full shadow-green"
          >
            <div class="row items-center justify-between full-width q-px-lg z-top">
              <div class="column items-center">
                <div class="cube-box bg-orange-5 shadow-orange-glow">
                  <q-icon name="public" color="black" size="md" />
                </div>
                <div class="text-xxs text-orange-4 q-mt-sm font-mono">PHYSICS</div>
              </div>

              <div class="pipeline-track relative-position col q-mx-md">
                <div class="track-line bg-grey-8"></div>

                <div
                  class="plugin-chip absolute-center bg-slate-800 border-green text-green-4 text-xxs font-mono q-px-sm rounded z-top"
                >
                  libgazebo_ros...
                </div>

                <div class="packet raw p1 bg-orange-5"></div>
                <div class="packet processed p2 bg-blue-5"></div>
              </div>

              <div class="column items-center">
                <div class="cube-box bg-blue-6 shadow-blue-glow">
                  <q-icon name="hub" color="white" size="md" />
                </div>
                <div class="text-xxs text-blue-4 q-mt-sm font-mono">/SCAN</div>
              </div>
            </div>
            <div class="matrix-bg absolute-full opacity-10"></div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>2. Lidar: El Escáner Láser</SectionTitle>

      [Image of lidar sensor diagram]

      <div class="row q-col-gutter-lg q-mt-md">
        <div class="col-12 col-md-5">
          <TextBlock>El sensor rey de la navegación 2D. Lanza rayos en abanico.</TextBlock>
          <ul class="q-pl-md text-grey-4 tool-list text-sm">
            <li><strong>Update Rate:</strong> 10Hz (Estándar).</li>
            <li><strong>Samples:</strong> 360 rayos (1 por grado).</li>
            <li><strong>Range:</strong> 0.1m (Min) - 12m (Max).</li>
          </ul>
        </div>
        <div class="col-12 col-md-7">
          <div
            class="tool-card bg-black relative-position overflow-hidden border-red shadow-red"
            style="height: 240px"
          >
            <div class="absolute-top-right q-pa-sm text-red-5 font-mono text-xxs">
              LASER_SCANNER_V2
            </div>

            <div class="absolute-center">
              <div class="lidar-unit bg-grey-8 border-red z-top"></div>
              <div class="scan-fan"></div>
            </div>

            <div class="obstacle obs-1 bg-white absolute"></div>
            <div class="obstacle obs-2 bg-white absolute"></div>

            <div class="hit-point hp-1 absolute bg-red-5"></div>
            <div class="hit-point hp-2 absolute bg-red-5"></div>

            <div
              class="absolute-bottom bg-slate-900 q-py-xs row justify-center font-mono text-xxs text-green-4 border-t-light"
            >
              [inf, 2.1, 2.1, inf, 5.4, inf]
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <div class="code-label xml">📄 lidar_plugin.xml (Fragmento)</div>
      <CodeBlock
        lang="xml"
        :copyable="true"
        code='<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <min_angle>-3.14</min_angle> <max_angle>3.14</max_angle>
      </horizontal>
    </scan>
    <range> <min>0.3</min> <max>12.0</max> </range>
  </ray>
  <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros> <argument>~/out:=scan</argument> </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>'
      />
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>3. Cámara y el "Frame Óptico"</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            En Robótica (ROS), <strong>X es adelante</strong>.
            <br />
            En Visión (OpenCV), <strong>Z es profundidad</strong>. <br /><br />
            Si montas una cámara sin corregir esto, tu robot mirará al suelo. Necesitamos un
            <code>optical_link</code> rotado.
          </TextBlock>
          <AlertBlock type="warning" title="La Rotación Mágica">
            Debes rotar el link óptico: Roll=-90°, Yaw=-90°.
          </AlertBlock>
        </template>
        <template #right>
          <div class="tool-card bg-grid relative-position overflow-hidden" style="height: 280px">
            <div class="absolute-top-left q-pa-sm text-cyan-4 font-mono text-xxs">
              FRAME CONVERSION
            </div>

            <div class="absolute-center transform-scene">
              <div class="frame standard-frame">
                <div class="axis x bg-red-5"></div>
                <div class="text-xxs text-red-5 axis-label x-label">X (Front)</div>
                <div class="axis z bg-blue-5"></div>
                <div class="text-xxs text-blue-5 axis-label z-label">Z (Up)</div>
                <div class="origin bg-white"></div>
                <div class="text-caption text-grey-5 absolute" style="top: 20px">ROS Body</div>
              </div>

              <div class="arrow-arc"></div>

              <div class="frame optical-frame">
                <div class="axis z bg-blue-5"></div>
                <div class="text-xxs text-blue-5 axis-label z-label-opt">Z (Depth)</div>
                <div class="axis y bg-green-5"></div>
                <div class="text-xxs text-green-5 axis-label y-label-opt">Y (Down)</div>
                <div class="origin bg-cyan-4"></div>
                <div class="text-caption text-cyan-4 absolute" style="top: 20px">
                  Camera Optical
                </div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>4. IMU: El Sentido del Equilibrio</SectionTitle>
      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-6">
          <TextBlock>Mide Aceleración y Giro. Esencial para saber "dónde es abajo".</TextBlock>
          <div class="row q-gutter-sm q-mt-sm">
            <q-chip color="purple-9" text-color="white" icon="explore">Gyro</q-chip>
            <q-chip color="purple-9" text-color="white" icon="speed">Accel</q-chip>
          </div>
        </div>
        <div class="col-12 col-md-6">
          <div class="tool-card bg-slate-900 q-pa-md flex flex-center border-purple">
            <div class="imu-chip relative-position">
              <div class="chip-body bg-grey-9 border-purple shadow-purple">
                <div class="chip-text font-mono text-xxs text-purple-3">MPU6050</div>
              </div>
              <div class="axis-line x-axis bg-red-5"></div>
              <div class="axis-line y-axis bg-green-5"></div>
              <div class="axis-line z-axis bg-blue-5"></div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>5. La Importancia del Ruido (Noise)</SectionTitle>
      <TextBlock>
        El mundo real es sucio. Si simulas sensores perfectos, tu código fallará en la realidad.
        Añadimos <strong>Ruido Gaussiano</strong> para robustez.
      </TextBlock>

      <div
        class="tool-card bg-black q-mt-md border-grey relative-position overflow-hidden"
        style="height: 200px"
      >
        <div class="absolute-top-left q-pa-sm row q-gutter-md">
          <div class="text-xxs font-mono text-green-4">● IDEAL SIGNAL</div>
          <div class="text-xxs font-mono text-red-4">● WITH GAUSSIAN NOISE</div>
        </div>

        <svg
          class="full-width full-height absolute"
          viewBox="0 0 100 50"
          preserveAspectRatio="none"
        >
          <path
            d="M0,25 Q12.5,5 25,25 T50,25 T75,25 T100,25"
            fill="none"
            stroke="#4ade80"
            stroke-width="0.5"
            class="sine-wave"
          />
          <path
            d="M0,25 Q12.5,5 25,25 T50,25 T75,25 T100,25"
            fill="none"
            stroke="#ef4444"
            stroke-width="0.5"
            class="noisy-wave"
          />
        </svg>
      </div>

      <div class="q-mt-md">
        <CodeBlock
          lang="xml"
          code="<noise> <type>gaussian</type> <mean>0.0</mean> <stddev>0.01</stddev> </noise>"
        />
      </div>
    </div>

    <div class="section-group self-stretch q-mb-xl">
      <SectionTitle>Troubleshooting: ¿Por qué no veo nada?</SectionTitle>
      <div class="q-gutter-y-sm">
        <q-expansion-item
          icon="visibility_off"
          label="Cámara en negro/gris"
          header-class="bg-slate-800 text-white rounded-borders"
        >
          <div class="q-pa-md bg-slate-900 text-grey-4">
            1. Falta luz: Añade
            <code>&lt;include&gt;&lt;uri&gt;model://sun&lt;/uri&gt;&lt;/include&gt;</code> al
            mundo.<br />
            2. Clip incorrecto: Revisa <code>&lt;near&gt;0.05&lt;/near&gt;</code>.
          </div>
        </q-expansion-item>
        <q-expansion-item
          icon="blur_on"
          label="Lidar detecta al propio robot"
          header-class="bg-slate-800 text-white rounded-borders"
        >
          <div class="q-pa-md bg-slate-900 text-grey-4">
            Los rayos chocan con el chasis. Sube el Lidar en Z o usa la etiqueta
            <code>&lt;ignore_rays&gt;</code>.
          </div>
        </q-expansion-item>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
</script>

<style scoped>
/* GENERAL */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}
.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(34, 197, 94, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
}
.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  color: #f8fafc;
}

/* UTILS */
.bg-slate-900 {
  background: #0f172a;
}
.bg-slate-800 {
  background: #1e293b;
}
.bg-grid {
  background-image:
    linear-gradient(rgba(255, 255, 255, 0.05) 1px, transparent 1px),
    linear-gradient(90deg, rgba(255, 255, 255, 0.05) 1px, transparent 1px);
  background-size: 20px 20px;
  background-color: #0f172a;
}
.border-green {
  border: 1px solid #22c55e;
}
.border-red {
  border: 1px solid #ef4444;
}
.border-cyan {
  border: 1px solid #06b6d4;
}
.border-purple {
  border: 1px solid #a855f7;
}
.shadow-green {
  box-shadow: 0 0 20px rgba(34, 197, 94, 0.2);
}
.shadow-orange-glow {
  box-shadow: 0 0 15px #f97316;
}
.shadow-blue-glow {
  box-shadow: 0 0 15px #3b82f6;
}
.shadow-red {
  box-shadow: 0 0 15px rgba(239, 68, 68, 0.3);
}
.shadow-purple {
  box-shadow: 0 0 15px #a855f7;
}
.text-xxs {
  font-size: 0.7rem;
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.z-top {
  z-index: 10;
}
.h-full {
  height: 100%;
}

/* 1. DATA PIPELINE */
.cube-box {
  width: 60px;
  height: 60px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 8px;
  z-index: 2;
}
.pipeline-track {
  height: 40px;
  display: flex;
  align-items: center;
}
.track-line {
  width: 100%;
  height: 2px;
}
.packet {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  position: absolute;
  top: 15px;
  opacity: 0;
}
.raw {
  animation: moveRaw 3s infinite linear;
}
.processed {
  animation: moveProcessed 3s infinite linear 1.5s;
} /* Starts after plugin */
@keyframes moveRaw {
  0% {
    left: 0;
    opacity: 1;
  }
  45% {
    left: 50%;
    opacity: 1;
  }
  50% {
    left: 50%;
    opacity: 0;
    transform: scale(2);
  }
  100% {
    opacity: 0;
  }
}
@keyframes moveProcessed {
  0% {
    left: 50%;
    opacity: 0;
  }
  5% {
    left: 50%;
    opacity: 1;
    transform: scale(1);
  }
  100% {
    left: 100%;
    opacity: 1;
  }
}

/* 2. LIDAR SCAN */
.lidar-unit {
  width: 16px;
  height: 16px;
  border-radius: 50%;
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
}
.scan-fan {
  width: 300px;
  height: 300px;
  background: conic-gradient(from 0deg, rgba(239, 68, 68, 0.5) 0deg, transparent 30deg);
  border-radius: 50%;
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  animation: spin 1s infinite linear;
}
.obstacle {
  width: 40px;
  height: 40px;
}
.obs-1 {
  top: 20%;
  left: 20%;
}
.obs-2 {
  bottom: 30%;
  right: 20%;
  width: 10px;
  height: 80px;
}
.hit-point {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  box-shadow: 0 0 10px #ef4444;
  animation: hitPulse 1s infinite;
}
.hp-1 {
  top: 28%;
  left: 28%;
  animation-delay: 0.1s;
}
.hp-2 {
  bottom: 50%;
  right: 24%;
  animation-delay: 0.6s;
}
@keyframes spin {
  from {
    transform: translate(-50%, -50%) rotate(0deg);
  }
  to {
    transform: translate(-50%, -50%) rotate(360deg);
  }
}
@keyframes hitPulse {
  0%,
  80% {
    opacity: 0;
  }
  85% {
    opacity: 1;
    transform: scale(1.5);
  }
  100% {
    opacity: 0;
  }
}

/* 3. OPTICAL FRAME */
.transform-scene {
  width: 200px;
  height: 100px;
}
.frame {
  position: absolute;
  width: 0;
  height: 0;
}
.axis {
  position: absolute;
  border-radius: 2px;
}
.x {
  width: 40px;
  height: 2px;
}
.z {
  width: 2px;
  height: 40px;
  top: -40px;
}
.y {
  width: 2px;
  height: 40px;
  top: 0;
}
.origin {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  position: absolute;
  top: -2px;
  left: -2px;
}
.axis-label {
  position: absolute;
  white-space: nowrap;
}
.x-label {
  left: 45px;
  top: -5px;
}
.z-label {
  top: -55px;
  left: -5px;
}
.standard-frame {
  left: -50px;
}
.optical-frame {
  left: 50px;
  animation: frameAppear 4s infinite;
}
.arrow-arc {
  position: absolute;
  width: 60px;
  height: 40px;
  top: -20px;
  left: -30px;
  border-top: 2px dashed #475569;
  border-radius: 50%;
}
@keyframes frameAppear {
  0%,
  20% {
    opacity: 0;
    transform: rotate(-90deg);
  }
  40%,
  80% {
    opacity: 1;
    transform: rotate(0deg);
  }
  100% {
    opacity: 0;
  }
}

/* 4. IMU CHIP */
.imu-chip {
  width: 80px;
  height: 80px;
  animation: floatChip 4s infinite ease-in-out;
}
.chip-body {
  width: 60px;
  height: 60px;
  position: absolute;
  top: 10px;
  left: 10px;
  border-radius: 8px;
  display: flex;
  align-items: center;
  justify-content: center;
}
.axis-line {
  position: absolute;
  border-radius: 2px;
}
.x-axis {
  width: 100px;
  height: 2px;
  top: 40px;
  left: -10px;
}
.y-axis {
  width: 2px;
  height: 100px;
  top: -10px;
  left: 40px;
}
.z-axis {
  width: 2px;
  height: 40px;
  top: 20px;
  left: 40px;
  background: #3b82f6;
  transform: rotate(45deg);
}
@keyframes floatChip {
  0%,
  100% {
    transform: translateY(0) rotateX(10deg);
  }
  50% {
    transform: translateY(-10px) rotateX(-10deg);
  }
}

/* 5. OSCILLOSCOPE */
.sine-wave {
  animation: dash 3s linear infinite;
  stroke-dasharray: 200;
}
.noisy-wave {
  animation: dashNoisy 3s linear infinite;
  stroke-dasharray: 200;
  filter: drop-shadow(0 0 2px #ef4444);
}
@keyframes dash {
  from {
    stroke-dashoffset: 200;
  }
  to {
    stroke-dashoffset: 0;
  }
}
/* A simpler dash animation for the noisy one, creates visual jitter */
@keyframes dashNoisy {
  from {
    stroke-dashoffset: 200;
    transform: translateX(0);
  }
  to {
    stroke-dashoffset: 0;
    transform: translateX(1px);
  }
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .tool-card {
    height: auto !important;
    min-height: 250px;
  }
}
</style>
