<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-orange-4 text-weight-bold q-mb-sm">
          MÓDULO 4.2: EL TALLER DIGITAL
        </div>

        <h1 class="hero-title">Workspaces y <span class="text-white">Paquetes</span></h1>

        <TextBlock>
          En ROS 2, no puedes simplemente tirar archivos donde quieras. Necesitas una estructura
          rígida para que el sistema de compilación (Colcon) sepa qué hacer. Aprende a montar tu
          "Taller" (Workspace) y a fabricar tu primera "Caja de Herramientas" (Paquete).
        </TextBlock>
      </div>
    </section>

    <!-- 2. ANATOMÍA DEL WORKSPACE -->
    <div class="section-group self-stretch">
      <SectionTitle>1. El Taller (Workspace)</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            Un <strong>Workspace</strong> (usualmente llamado <code>ros2_ws</code>) es simplemente
            una carpeta con 4 subcarpetas sagradas. <br /><br />
            <strong>El Ciclo de Vida:</strong>
          </TextBlock>
          <ol class="q-pl-md q-mt-md text-grey-4 tool-list">
            <li>Escribes código en <code>src</code>.</li>
            <li>Compilas con <code>colcon build</code>.</li>
            <li>ROS genera <code>build</code>, <code>install</code> y <code>log</code>.</li>
            <li>Activas el taller con <code>source install/setup.bash</code>.</li>
          </ol>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL WORKSPACE -->
          <div
            class="tool-card workspace-viz q-pa-lg bg-slate-900 relative-position shadow-2 overflow-hidden"
          >
            <div
              class="text-center text-h6 text-white q-mb-lg font-mono bg-black inline-block q-px-md rounded-borders border-light relative-position z-top"
            >
              ~/ros2_ws
            </div>

            <div class="row q-col-gutter-md relative-position z-top">
              <!-- SRC (USER ZONE) -->
              <div class="col-12">
                <div
                  class="folder-zone user-zone row items-center justify-center cursor-pointer transition-hover"
                >
                  <q-icon name="folder_open" color="green-4" size="3rem" class="q-mr-md" />
                  <div class="text-left">
                    <div class="text-green-4 text-h6 font-mono">src/</div>
                    <div class="text-caption text-grey-3 text-weight-bold">¡AQUÍ TRABAJAS TÚ!</div>
                  </div>
                  <div class="absolute-right q-pa-sm">
                    <q-icon name="edit" color="green-8" />
                  </div>
                </div>
              </div>

              <!-- BUILD SYSTEM ZONES (AUTO) -->
              <div class="col-4">
                <div class="folder-zone auto-zone opacity-60 column flex-center">
                  <q-icon name="handyman" color="orange-4" size="2rem" />
                  <div class="text-caption text-orange-4 q-mt-sm font-mono">build/</div>
                  <div class="text-xs text-grey-5">Temp</div>
                </div>
              </div>
              <div class="col-4">
                <div class="folder-zone auto-zone border-glow column flex-center">
                  <q-icon name="system_update_alt" color="blue-4" size="2rem" />
                  <div class="text-caption text-blue-4 q-mt-sm font-mono">install/</div>
                  <div class="text-xs text-grey-5">Final</div>
                </div>
              </div>
              <div class="col-4">
                <div class="folder-zone auto-zone opacity-60 column flex-center">
                  <q-icon name="history" color="grey-4" size="2rem" />
                  <div class="text-caption text-grey-4 q-mt-sm font-mono">log/</div>
                  <div class="text-xs text-grey-5">Debug</div>
                </div>
              </div>
            </div>

            <!-- ANIMATED ARROWS BEHIND -->
            <div class="absolute-full flex flex-center" style="z-index: 1; opacity: 0.1">
              <q-icon
                name="keyboard_double_arrow_down"
                size="10rem"
                color="white"
                class="flow-anim"
              />
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. EL PAQUETE (LA UNIDAD ATÓMICA) -->
    <div class="section-group self-stretch">
      <SectionTitle>2. La Caja de Herramientas (Package)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            El <strong>Paquete</strong> es la unidad mínima de software en ROS 2. Si quieres
            compartir tu código con un amigo, le envías la carpeta del paquete, no un archivo
            suelto.
          </TextBlock>
          <div class="q-mt-md">
            <AlertBlock type="warning" title="Python vs C++">
              Al crear un paquete, debes decidir su "idioma nativo". La estructura de carpetas
              cambia totalmente.
            </AlertBlock>
          </div>
        </template>

        <template #right>
          <div class="row q-col-gutter-md">
            <!-- PYTHON PACKAGE -->
            <div class="col-12 col-md-6">
              <div class="tool-card pkg-card python-style bg-slate-800 transition-hover shadow-2">
                <div class="row items-center q-mb-md border-bottom-dark q-pb-sm">
                  <q-icon
                    name="img:https://cdn.jsdelivr.net/gh/devicons/devicon/icons/python/python-original.svg"
                    size="1.8rem"
                  />
                  <span class="text-subtitle1 text-weight-bold q-ml-sm text-yellow-4"
                    >ament_python</span
                  >
                </div>
                <ul class="file-list text-xs">
                  <li>
                    <q-icon name="description" color="red-4" />
                    <span class="text-red-4">package.xml</span>
                  </li>
                  <li>
                    <q-icon name="settings" color="orange-4" />
                    <span class="text-orange-4">setup.py</span>
                  </li>
                  <li>
                    <q-icon name="folder" color="green-4" />
                    <span class="text-green-4">mi_paquete/</span>
                    <ul class="sub-list">
                      <li>__init__.py</li>
                      <li>nodo.py</li>
                    </ul>
                  </li>
                </ul>
              </div>
            </div>

            <!-- C++ PACKAGE -->
            <div class="col-12 col-md-6">
              <div class="tool-card pkg-card cpp-style bg-slate-800 transition-hover shadow-2">
                <div class="row items-center q-mb-md border-bottom-dark q-pb-sm">
                  <q-icon
                    name="img:https://cdn.jsdelivr.net/gh/devicons/devicon/icons/cplusplus/cplusplus-original.svg"
                    size="1.8rem"
                  />
                  <span class="text-subtitle1 text-weight-bold q-ml-sm text-blue-4"
                    >ament_cmake</span
                  >
                </div>
                <ul class="file-list text-xs">
                  <li>
                    <q-icon name="description" color="red-4" />
                    <span class="text-red-4">package.xml</span>
                  </li>
                  <li>
                    <q-icon name="settings" color="blue-4" />
                    <span class="text-blue-4">CMakeLists.txt</span>
                  </li>
                  <li>
                    <q-icon name="folder" color="purple-4" />
                    <span class="text-purple-4">include/</span>
                  </li>
                  <li>
                    <q-icon name="folder" color="green-4" /> <span class="text-green-4">src/</span>
                  </li>
                </ul>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. COMANDOS DE CREACIÓN -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Creación: El Conjuro Mágico</SectionTitle>
      <TextBlock>
        Nunca crees las carpetas a mano. Usa el comando generador para asegurar que tienes todos los
        archivos de configuración (package.xml, setup.py) correctamente enlazados.
      </TextBlock>

      <div
        class="tool-card cmd-display q-mt-lg bg-black q-pa-none shadow-2 overflow-hidden border-grey"
      >
        <!-- PESTAÑAS FALSAS -->
        <div class="row bg-slate-900 q-px-sm q-py-xs border-bottom-dark">
          <div class="tab-pill bg-yellow-9 text-black q-mr-sm">Python</div>
          <div class="tab-pill bg-slate-800 text-grey-6 border-light">C++</div>
        </div>

        <div class="font-mono text-sm q-pa-lg">
          <div class="text-grey-5 text-caption q-mb-xs">
            # 1. Ve a la carpeta src (¡IMPORTANTE!)
          </div>
          <div class="cmd-line q-mb-md">
            <span class="text-green-4">cd</span> <span class="text-white">~/ros2_ws/src</span>
          </div>

          <div class="text-grey-5 text-caption q-mb-xs"># 2. Crea el paquete</div>
          <div class="cmd-line q-mb-md">
            <span class="text-blue-4">ros2 pkg create</span>
            <span class="text-grey-4"> --build-type </span>
            <span class="text-yellow-4">ament_python</span>
            <span class="text-purple-4"> mi_robot_control</span>
          </div>

          <div class="text-grey-5 text-caption q-mb-xs"># 3. Añade dependencias (Opcional)</div>
          <div class="cmd-line">
            <span class="text-grey-4"> --dependencies </span>
            <span class="text-orange-4">rclpy std_msgs geometry_msgs</span>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. COLCON BUILD (LA FÁBRICA) -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <SectionTitle>4. Compilar: Colcon</SectionTitle>
      <div class="row q-col-gutter-xl items-center">
        <div class="col-12 col-md-8">
          <TextBlock>
            Una vez creado el paquete, el sistema no sabe que existe hasta que lo procesas.
            <code>colcon build</code> es el capataz que recorre tu carpeta <code>src</code>, busca
            paquetes y los prepara en <code>install</code>.
          </TextBlock>
          <div class="q-mt-lg">
            <AlertBlock type="danger" title="¡Vuelve a la Raíz!">
              Nunca ejecutes <code>colcon build</code> dentro de <code>src</code>. Siempre debes
              estar en la raíz del workspace (<code>~/ros2_ws</code>).
            </AlertBlock>
          </div>
        </div>
        <div class="col-12 col-md-4 text-center">
          <div
            class="tool-card build-btn bg-gradient-orange cursor-pointer shadow-glow-orange group"
          >
            <div class="icon-wrapper q-mb-md">
              <q-icon name="build" size="4rem" color="white" class="spin-hover" />
            </div>
            <div class="text-h5 text-white text-weight-bold font-mono">colcon build</div>
            <div
              class="text-caption text-white opacity-80 q-mt-sm bg-black q-px-sm rounded-borders font-mono"
            >
              --symlink-install
            </div>
          </div>
          <div class="text-caption text-grey-5 q-mt-md font-italic">
            (Haz clic mentalmente para compilar)
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
    radial-gradient(circle at center, rgba(251, 146, 60, 0.15), transparent 60%),
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
  line-height: 1.1;
  color: #f8fafc;
}

/* TOOL CARDS */
.tool-card {
  height: 100%;
  border-radius: 16px;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.05);
}

/* WORKSPACE VIZ */
.workspace-viz {
  border-top: 4px solid #f97316;
}

.folder-zone {
  border-radius: 12px;
  padding: 20px;
  text-align: center;
  border: 1px solid rgba(255, 255, 255, 0.1);
  background: #1e293b;
  position: relative;
  height: 100%;
}
.user-zone {
  background: rgba(74, 222, 128, 0.1);
  border: 2px dashed #4ade80;
}
.auto-zone {
  background: #0f172a;
}

.border-glow {
  border: 1px solid #60a5fa;
  box-shadow: 0 0 15px rgba(96, 165, 250, 0.2);
}
.opacity-60 {
  opacity: 0.6;
}
.z-top {
  z-index: 2;
}
.transition-hover {
  transition: transform 0.2s;
}
.transition-hover:hover {
  transform: translateY(-5px);
}

.flow-anim {
  animation: flowDown 3s infinite linear;
}
@keyframes flowDown {
  0% {
    transform: translateY(-20px);
    opacity: 0;
  }
  50% {
    opacity: 0.2;
  }
  100% {
    transform: translateY(20px);
    opacity: 0;
  }
}

/* PACKAGE CARDS */
.pkg-card {
  padding: 24px;
}
.python-style {
  border-left: 4px solid #facc15;
}
.cpp-style {
  border-left: 4px solid #3b82f6;
}

.file-list {
  list-style: none;
  padding: 0;
  margin-top: 15px;
  font-family: 'Fira Code', monospace;
  color: #cbd5e1;
}
.file-list li {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 8px;
}
.sub-list {
  list-style: none;
  padding-left: 24px;
  margin-top: 4px;
  border-left: 1px solid #475569;
}

/* CMD DISPLAY */
.cmd-display {
  border: 1px solid #333;
}
.tab-pill {
  padding: 6px 16px;
  border-radius: 6px 6px 0 0;
  font-size: 0.8rem;
  font-weight: bold;
  cursor: pointer;
}
.cmd-line {
  background: rgba(255, 255, 255, 0.05);
  padding: 8px;
  border-radius: 4px;
}

/* BUILD BUTTON */
.bg-gradient-orange {
  background: linear-gradient(135deg, #ea580c 0%, #c2410c 100%);
}
.build-btn {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 40px;
  transition: all 0.2s;
  border: 4px solid rgba(255, 255, 255, 0.1);
}
.build-btn:active {
  transform: scale(0.95);
}
.shadow-glow-orange {
  box-shadow: 0 0 30px rgba(249, 115, 22, 0.6);
}

.group:hover .spin-hover {
  animation: spin 0.8s infinite linear;
}
@keyframes spin {
  100% {
    transform: rotate(360deg);
  }
}

/* UTILS */
.font-mono {
  font-family: 'Fira Code', monospace;
}
.text-xs {
  font-size: 0.8rem;
}
.text-sm {
  font-size: 0.9rem;
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
.border-bottom-dark {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.border-grey {
  border-color: #334155;
}
.tool-list {
  list-style: none;
  padding: 0;
}
.tool-list li {
  margin-bottom: 12px;
  font-size: 1rem;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
