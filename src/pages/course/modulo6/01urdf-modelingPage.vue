<template>
  <q-page class="q-pa-lg column items-center">
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-pink-4 text-weight-bold q-mb-sm">
          MÓDULO 6.1: MODELADO FÍSICO
        </div>
        <h1 class="hero-title">URDF: El <span class="text-white">ADN Digital</span></h1>
        <TextBlock>
          Antes de que un robot pueda moverse en una simulación, necesita un cuerpo.
          <strong>URDF (Unified Robot Description Format)</strong> es el estándar XML que define la
          geometría, la física y la cinemática. Sin él, ROS 2 no sabe si controlas un dron o una
          tostadora.
        </TextBlock>
      </div>
    </section>

    <div class="section-group self-stretch">
      <SectionTitle>1. Huesos y Articulaciones</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Un robot es un árbol genealógico estricto. El movimiento se propaga de Padres a Hijos.
          </TextBlock>

          <div class="row q-gutter-sm q-my-md">
            <div class="col-12 bg-slate-800 q-pa-sm rounded border-l-blue">
              <div class="text-blue-3 text-weight-bold">🦴 Link (Eslabón)</div>
              <div class="text-xs text-grey-4">La parte rígida. Tiene masa y forma visual.</div>
            </div>
            <div class="col-12 bg-slate-800 q-pa-sm rounded border-l-orange">
              <div class="text-orange-3 text-weight-bold">⚙️ Joint (Articulación)</div>
              <div class="text-xs text-grey-4">
                La bisagra. Define CÓMO se mueve el hijo respecto al padre.
              </div>
            </div>
          </div>

          <AlertBlock type="info" title="Jerarquía">
            Si mueves el "Hombro" (Padre), el "Codo" y la "Mano" (Hijos) se mueven con él
            automáticamente.
          </AlertBlock>
        </template>

        <template #right>
          <div
            class="tool-card bg-slate-900 relative-position overflow-hidden flex flex-center shadow-pink"
            style="height: 320px"
          >
            <div class="absolute-top-right q-ma-sm font-mono text-xxs text-pink-4">
              Kinematic Chain Viz
            </div>

            <div class="base-plate bg-grey-8"></div>

            <div class="link-1 bg-blue-9 border-light flex flex-center">
              <span class="text-xxs text-white rotate-text">Base Link</span>
            </div>

            <div class="joint-1 bg-orange-5 shadow-glow-orange"></div>

            <div class="link-2-container">
              <div class="link-2 bg-purple-9 border-light flex flex-center">
                <span class="text-xxs text-white">Shoulder</span>
              </div>

              <div class="joint-2 bg-orange-5 shadow-glow-orange"></div>

              <div class="link-3-container">
                <div class="link-3 bg-cyan-9 border-light flex flex-center">
                  <span class="text-xxs text-black">Forearm</span>
                </div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <div class="section-group self-stretch">
      <SectionTitle>2. El Origen: XYZ y RPY</SectionTitle>
      <TextBlock>
        El concepto que rompe cabezas: Debes definir dónde está el hijo
        <strong>relativo al sistema de coordenadas del padre</strong>.
      </TextBlock>

      <div
        class="tool-card bg-grid relative-position overflow-hidden q-mt-md"
        style="height: 250px"
      >
        <div class="absolute-top-left q-pa-sm text-xxs font-mono text-grey-5 z-top">
          ORIGIN TRANSFORM: <span class="text-white">PARENT → CHILD</span>
        </div>

        <div class="absolute-center transform-scene">
          <div class="frame parent-frame">
            <div class="axis x bg-red-5"></div>
            <div class="axis y bg-green-5"></div>
            <div class="origin-dot bg-white"></div>
            <div class="text-xs text-grey-5 absolute" style="top: -20px; left: -10px">
              Parent (0,0,0)
            </div>
          </div>

          <div class="frame ghost-frame">
            <div class="axis x bg-red-9 opacity-30"></div>
            <div class="axis y bg-green-9 opacity-30"></div>
            <div class="dashed-line"></div>
            <div class="label-measure text-xxs text-yellow-4 bg-black q-px-xs rounded">
              xyz="0.8 0 0"
            </div>
          </div>

          <div class="frame child-frame">
            <div class="axis x bg-red-4"></div>
            <div class="axis y bg-green-4"></div>
            <div class="origin-dot bg-pink-4"></div>
            <div class="label-rot text-xxs text-pink-4 bg-black q-px-xs rounded">
              rpy="0 0 0.78" (45°)
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. La Doble Realidad: Visual vs Colisión</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            <strong>Visual:</strong> Lo que ves en RViz. Mallas complejas (.stl/.dae). <br />
            <strong>Colisión:</strong> Lo que usa el motor físico. Formas primitivas
            (cajas/cilindros).
          </TextBlock>
          <AlertBlock type="danger" title="CPU Warning">
            Usar la malla visual para colisiones hará que tu simulación vaya a 1 FPS. ¡Simplifica
            siempre!
          </AlertBlock>
        </div>

        <div class="col-12 col-md-7">
          <div
            class="scanner-card bg-slate-900 border-grey shadow-2 relative-position overflow-hidden rounded-borders"
            style="height: 220px"
          >
            <div class="absolute-center row q-gutter-xl text-center" style="width: 100%">
              <div class="col">
                <q-icon name="model_training" size="5rem" color="cyan-4" class="visual-icon" />
                <div class="text-caption text-cyan-4 q-mt-sm font-bold">
                  Visual Mesh<br /><span class="text-xxs text-grey-5 font-mono"
                    >15,000 Polygons</span
                  >
                </div>
              </div>

              <div class="col relative-position">
                <div class="collision-box bg-red-9-dim border-red absolute-center"></div>
                <div class="text-caption text-red-4 q-mt-xl relative-position" style="top: 40px">
                  Collision Shape<br /><span class="text-xxs text-grey-5 font-mono"
                    >1 Box Primitive</span
                  >
                </div>
              </div>
            </div>

            <div class="scan-bar"></div>
            <div class="scan-overlay"></div>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch q-mt-lg">
      <div class="code-label xml">📄 robot.urdf (Fragmento Crítico)</div>
      <CodeBlock
        lang="xml"
        code='<link name="base_link">
  <visual>
    <geometry> <mesh filename="package://my_bot/meshes/chassis.stl"/> </geometry>
    <material name="blue"/>
  </visual>

  <collision>
    <geometry> <box size="0.5 0.3 0.1"/> </geometry>
  </collision>

  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>'
        :copyable="true"
      />
    </div>

    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>4. Xacro: Programando el XML</SectionTitle>
      <TextBlock>
        ¿Copiar y pegar código para 4 ruedas? Jamás. <strong>Xacro</strong> nos permite usar
        variables y macros (funciones).
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-md">
        <div class="col-12 col-md-6">
          <div class="tool-card bg-slate-800 q-pa-md border-l-green h-full">
            <div class="text-green-4 text-weight-bold q-mb-xs">Definición (Macro)</div>
            <CodeBlock
              lang="xml"
              :copyable="false"
              code='<xacro:property name="radius" value="0.2" />

<xacro:macro name="make_wheel" params="side">
  <link name="${side}_wheel">
    <cylinder radius="${radius}" ... />
  </link>
</xacro:macro>'
            />
          </div>
        </div>

        <div class="col-12 col-md-6">
          <div class="tool-card bg-slate-800 q-pa-md border-l-purple h-full">
            <div class="text-purple-3 text-weight-bold q-mb-xs">Uso (Llamada)</div>
            <CodeBlock
              lang="xml"
              :copyable="false"
              code='<xacro:make_wheel side="left" />
<xacro:make_wheel side="right" />'
            />
            <div class="q-mt-sm text-xs text-grey-5 italic">
              Al compilar, ROS reemplazará ${side} por "left" y "right".
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <SectionTitle>Tipos de Articulaciones Comunes</SectionTitle>
      <div class="row q-col-gutter-sm text-center">
        <div class="col-6 col-md-3">
          <div class="joint-card bg-slate-900 q-pa-md hover-scale">
            <q-icon name="sync" size="md" color="pink-4" class="spin-anim" />
            <div class="text-subtitle2 text-white q-mt-sm">Continuous</div>
            <div class="text-caption text-grey-5 text-xxs">Ruedas (∞)</div>
          </div>
        </div>
        <div class="col-6 col-md-3">
          <div class="joint-card bg-slate-900 q-pa-md hover-scale">
            <q-icon name="rotate_90_degrees_ccw" size="md" color="blue-4" class="rock-anim" />
            <div class="text-subtitle2 text-white q-mt-sm">Revolute</div>
            <div class="text-caption text-grey-5 text-xxs">Codo (Limitado)</div>
          </div>
        </div>
        <div class="col-6 col-md-3">
          <div class="joint-card bg-slate-900 q-pa-md hover-scale">
            <q-icon name="expand" size="md" color="green-4" class="slide-anim" />
            <div class="text-subtitle2 text-white q-mt-sm">Prismatic</div>
            <div class="text-caption text-grey-5 text-xxs">Pistón (Lineal)</div>
          </div>
        </div>
        <div class="col-6 col-md-3">
          <div class="joint-card bg-slate-900 q-pa-md hover-scale">
            <q-icon name="lock" size="md" color="grey-4" />
            <div class="text-subtitle2 text-white q-mt-sm">Fixed</div>
            <div class="text-caption text-grey-5 text-xxs">Sensor (Fijo)</div>
          </div>
        </div>
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
/* --- ESTILOS MAESTROS --- */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3.5rem auto;
}
.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at center, rgba(236, 72, 153, 0.15), transparent 60%),
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
.text-xxs {
  font-size: 0.7rem;
}
.text-xs {
  font-size: 0.8rem;
}
.font-mono {
  font-family: 'Fira Code', monospace;
}
.bg-slate-900 {
  background: #0f172a;
}
.bg-slate-800 {
  background: #1e293b;
}
.border-light {
  border: 1px solid rgba(255, 255, 255, 0.1);
}
.border-l-blue {
  border-left: 4px solid #3b82f6;
}
.border-l-orange {
  border-left: 4px solid #f97316;
}
.border-l-green {
  border-left: 4px solid #4ade80;
}
.border-l-purple {
  border-left: 4px solid #a855f7;
}
.shadow-pink {
  box-shadow: 0 0 20px rgba(236, 72, 153, 0.15);
}
.shadow-glow-orange {
  box-shadow: 0 0 10px #f97316;
}
.h-full {
  height: 100%;
}

/* --- 1. KINEMATIC CHAIN ANIMATION --- */
.base-plate {
  width: 60px;
  height: 10px;
  position: absolute;
  bottom: 20px;
  border-radius: 4px;
  z-index: 1;
}
.link-1 {
  width: 20px;
  height: 80px;
  position: absolute;
  bottom: 30px;
  border-radius: 4px;
  z-index: 2;
}
.joint-1 {
  width: 14px;
  height: 14px;
  border-radius: 50%;
  border: 2px solid white;
  position: absolute;
  bottom: 95px;
  z-index: 5;
}

/* Parent Container for rotation */
.link-2-container {
  position: absolute;
  bottom: 102px;
  left: 0;
  width: 0;
  height: 0;
  z-index: 3;
  animation: shoulderMove 4s infinite ease-in-out;
}
.link-2 {
  width: 80px;
  height: 20px;
  position: absolute;
  top: -10px;
  left: 0;
  border-radius: 4px;
  transform-origin: 0 50%;
}
.joint-2 {
  width: 12px;
  height: 12px;
  border-radius: 50%;
  border: 2px solid white;
  position: absolute;
  top: -6px;
  left: 70px;
  z-index: 5;
}

/* Grandchild Container */
.link-3-container {
  position: absolute;
  top: 0;
  left: 76px;
  width: 0;
  height: 0;
  z-index: 4;
  animation: elbowMove 4s infinite ease-in-out;
}
.link-3 {
  width: 60px;
  height: 16px;
  position: absolute;
  top: -8px;
  left: 0;
  border-radius: 4px;
  transform-origin: 0 50%;
}

@keyframes shoulderMove {
  0%,
  100% {
    transform: rotate(0deg);
  }
  50% {
    transform: rotate(-45deg);
  }
}
@keyframes elbowMove {
  0%,
  100% {
    transform: rotate(0deg);
  }
  50% {
    transform: rotate(90deg);
  }
}
.rotate-text {
  display: block;
  transform: rotate(-90deg);
  white-space: nowrap;
}

/* --- 2. TRANSFORM SCENE --- */
.bg-grid {
  background-color: #0f172a;
  background-image:
    linear-gradient(rgba(255, 255, 255, 0.05) 1px, transparent 1px),
    linear-gradient(90deg, rgba(255, 255, 255, 0.05) 1px, transparent 1px);
  background-size: 20px 20px;
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
  top: 0;
  left: 0;
}
.y {
  width: 2px;
  height: 40px;
  top: -40px;
  left: 0;
}
.origin-dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  position: absolute;
  top: -2px;
  left: -2px;
}

.parent-frame {
  top: 0;
  left: -80px;
}
.ghost-frame {
  top: 0;
  left: -80px;
  animation: translateFrame 5s infinite ease-in-out;
}
.child-frame {
  top: 0;
  left: -80px;
  animation: rotateFrame 5s infinite ease-in-out;
}

.dashed-line {
  position: absolute;
  top: 1px;
  left: 0;
  height: 0;
  border-top: 1px dashed #facc15;
  animation: growDash 5s infinite ease-in-out;
}

@keyframes translateFrame {
  0% {
    transform: translate(0, 0);
    opacity: 0;
  }
  20% {
    opacity: 1;
  }
  40%,
  100% {
    transform: translate(80px, -30px);
    opacity: 0.3;
  }
}
@keyframes growDash {
  0%,
  20% {
    width: 0;
    transform: rotate(0deg);
  }
  40%,
  100% {
    width: 85px;
    transform: rotate(-20deg);
    transform-origin: 0 0;
  }
}
@keyframes rotateFrame {
  0%,
  40% {
    transform: translate(0, 0) rotate(0deg);
    opacity: 0;
  }
  50% {
    transform: translate(80px, -30px) rotate(0deg);
    opacity: 1;
  }
  80%,
  100% {
    transform: translate(80px, -30px) rotate(45deg);
    opacity: 1;
  }
}

.label-measure {
  position: absolute;
  top: -20px;
  left: 20px;
  white-space: nowrap;
  opacity: 0;
  animation: labelFade 5s infinite;
}
.label-rot {
  position: absolute;
  top: 10px;
  left: 40px;
  white-space: nowrap;
  opacity: 0;
  animation: labelFadeRot 5s infinite;
}

@keyframes labelFade {
  0%,
  20% {
    opacity: 0;
  }
  40%,
  80% {
    opacity: 1;
  }
  100% {
    opacity: 0;
  }
}
@keyframes labelFadeRot {
  0%,
  50% {
    opacity: 0;
  }
  80%,
  100% {
    opacity: 1;
  }
}

/* --- 3. SCANNER ANIMATION --- */
.collision-box {
  width: 60px;
  height: 60px;
  border: 2px solid;
}
.border-red {
  border-color: #ef4444;
}
.bg-red-9-dim {
  background: rgba(185, 28, 28, 0.3);
}
.scan-bar {
  position: absolute;
  top: 0;
  left: -10%;
  width: 4px;
  height: 100%;
  background: #22d3ee;
  box-shadow: 0 0 15px #22d3ee;
  animation: scannerMove 4s infinite linear;
  z-index: 10;
}
.scan-overlay {
  position: absolute;
  top: 0;
  left: 0;
  height: 100%;
  width: 0%;
  background: rgba(15, 23, 42, 0.85); /* Hides complex mesh */
  animation: scannerReveal 4s infinite linear;
  z-index: 5;
}
@keyframes scannerMove {
  0% {
    left: -10%;
  }
  100% {
    left: 110%;
  }
}
@keyframes scannerReveal {
  0% {
    width: 0%;
  }
  100% {
    width: 100%;
  }
}

/* --- JOINTS ICONS --- */
.joint-card {
  border-radius: 12px;
  border: 1px solid rgba(255, 255, 255, 0.05);
  transition: transform 0.2s;
}
.hover-scale:hover {
  transform: translateY(-5px);
  border-color: #ec4899;
}
.spin-anim {
  animation: spin 2s infinite linear;
}
.rock-anim {
  animation: rock 2s infinite ease-in-out;
}
.slide-anim {
  animation: slide 2s infinite ease-in-out;
}

@keyframes spin {
  100% {
    transform: rotate(360deg);
  }
}
@keyframes rock {
  0%,
  100% {
    transform: rotate(0deg);
  }
  50% {
    transform: rotate(90deg);
  }
}
@keyframes slide {
  0%,
  100% {
    transform: translateY(5px);
  }
  50% {
    transform: translateY(-5px);
  }
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .tool-card,
  .scanner-card {
    height: auto;
    min-height: 250px;
  }
}
</style>
