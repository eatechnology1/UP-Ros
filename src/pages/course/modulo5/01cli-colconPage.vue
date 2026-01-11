<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-pink-4 text-weight-bold q-mb-sm">
          MÓDULO 5.1: HERRAMIENTAS DE DESARROLLO
        </div>

        <h1 class="hero-title">CLI Avanzada & <span class="text-white">Colcon</span></h1>

        <TextBlock>
          Hasta ahora has usado comandos básicos. Pero, ¿qué pasa si el sistema va lento? ¿O si
          necesitas compilar solo un paquete entre 50? En este módulo aprenderás las herramientas de
          línea de comandos (CLI) que distinguen a un usuario de un ingeniero de ROS 2. Dominarás el
          diagnóstico de red y la compilación selectiva.
        </TextBlock>
      </div>
    </section>

    <!-- 2. COLCON: TRUCOS DE COMPILACIÓN -->
    <div class="section-group self-stretch">
      <SectionTitle>1. Colcon: Compilación Inteligente</SectionTitle>

      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-5">
          <TextBlock>
            <code>colcon build</code> es lento si compilas todo el workspace cada vez que cambias
            una línea de código. <br /><br />
            <strong>Optimizaciones Vitales:</strong>
          </TextBlock>
          <ul class="q-pl-md q-mt-md text-grey-4 tool-list">
            <li>
              ⚡ <strong>Symlink Install:</strong> Crea enlaces simbólicos. Si cambias un script de
              Python o un archivo Launch, ¡no necesitas recompilar! Solo guardas y ejecutas.
            </li>
            <li>
              🎯 <strong>Packages Select:</strong> Compila solo el paquete que estás modificando,
              ignorando el resto.
            </li>
          </ul>
        </div>

        <div class="col-12 col-md-7">
          <!-- VISUAL COLCON -->
          <div
            class="tool-card colcon-viz q-pa-lg bg-black relative-position overflow-hidden shadow-2"
          >
            <div
              class="text-center text-subtitle1 text-white q-mb-lg font-mono bg-slate-800 inline-block q-px-md rounded-borders border-light"
            >
              ~/ros2_ws $
            </div>

            <div class="row q-col-gutter-md">
              <!-- OPCIÓN 1: SYMLINK -->
              <div class="col-12">
                <div
                  class="cmd-box bg-slate-900 q-pa-md border-pink transition-hover shadow-1 cursor-pointer"
                >
                  <div class="row items-center q-mb-sm">
                    <q-icon name="link" color="pink-4" size="sm" class="q-mr-sm" />
                    <div class="text-pink-4 text-weight-bold">
                      Desarrollo Rápido (Python/Launch)
                    </div>
                  </div>
                  <div class="font-mono text-xs text-white bg-black q-pa-sm rounded border-light">
                    colcon build <span class="text-green-4">--symlink-install</span>
                  </div>
                  <div class="text-caption text-grey-5 q-mt-sm font-italic">
                    Edita el código y ejecuta. Sin esperas.
                  </div>
                </div>
              </div>

              <!-- OPCIÓN 2: SELECT -->
              <div class="col-12">
                <div
                  class="cmd-box bg-slate-900 q-pa-md border-blue transition-hover shadow-1 cursor-pointer"
                >
                  <div class="row items-center q-mb-sm">
                    <q-icon name="filter_alt" color="blue-4" size="sm" class="q-mr-sm" />
                    <div class="text-blue-4 text-weight-bold">Compilación Selectiva</div>
                  </div>
                  <div class="font-mono text-xs text-white bg-black q-pa-sm rounded border-light">
                    colcon build <span class="text-yellow-4">--packages-select</span> my_robot_nav
                  </div>
                  <div class="text-caption text-grey-5 q-mt-sm font-italic">
                    Ignora los otros 99 paquetes del workspace.
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 3. DIAGNÓSTICO DE TÓPICOS -->
    <div class="section-group self-stretch">
      <SectionTitle>2. Diagnóstico de Tópicos (HZ & BW)</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            A veces el robot "funciona", pero va a tirones. ¿Es culpa del código o de la red? ROS 2
            tiene herramientas integradas para medir la salud de los datos.
            <br /><br />
            <strong>Las dos métricas clave:</strong>
          </TextBlock>
          <ul class="q-pl-md q-mt-md text-grey-4 tool-list">
            <li>
              ⏱️ <strong>HZ (Frecuencia):</strong> ¿Están llegando los mensajes a la velocidad
              prometida (ej: 30 FPS)?
            </li>
            <li>
              📦 <strong>BW (Ancho de Banda):</strong> ¿Estás saturando el WiFi enviando nubes de
              puntos gigantes?
            </li>
          </ul>
        </template>

        <template #right>
          <div
            class="tool-card cli-monitor bg-black q-pa-none shadow-2 overflow-hidden border-grey"
          >
            <!-- HEADER -->
            <div
              class="window-header row items-center q-px-md q-py-xs bg-slate-800 border-bottom-dark"
            >
              <div class="window-dots row q-mr-md">
                <div class="dot bg-red-5 q-mr-xs"></div>
                <div class="dot bg-yellow-5 q-mr-xs"></div>
                <div class="dot bg-green-5"></div>
              </div>
              <span class="text-grey-5 font-mono text-xs">Terminal - Diagnóstico</span>
            </div>

            <!-- CONTENT -->
            <div class="q-pa-md font-mono text-xs">
              <!-- HZ COMMAND -->
              <div class="q-mb-md">
                <div class="text-green-4 border-bottom-dark q-pb-xs">
                  $ ros2 topic hz /camera/image_raw
                </div>
                <div class="text-grey-4 q-pl-md q-mt-xs border-l-green bg-green-9-soft q-py-xs">
                  average rate: <span class="text-red-4 text-weight-bold">12.512</span>
                  <span class="text-grey-5">(esperado: 30.0)</span><br />
                  min: 0.078s max: 0.085s std dev: 0.002s window: 15
                </div>
                <div class="row items-center q-mt-xs text-red-3 text-caption q-pl-sm">
                  <q-icon name="warning" size="xs" class="q-mr-xs" /> ¡Alerta! La cámara va lenta.
                </div>
              </div>

              <!-- BW COMMAND -->
              <div class="q-mt-lg">
                <div class="text-blue-4 border-bottom-dark q-pb-xs">
                  $ ros2 topic bw /lidar/points
                </div>
                <div class="text-grey-4 q-pl-md q-mt-xs border-l-blue bg-blue-9-soft q-py-xs">
                  average: <span class="text-yellow-4 text-weight-bold">8.45 MB/s</span><br />
                  mean: 2.10 MB min: 2.05 MB max: 2.15 MB window: 10
                </div>
                <div class="row items-center q-mt-xs text-yellow-3 text-caption q-pl-sm">
                  <q-icon name="signal_wifi_bad" size="xs" class="q-mr-xs" /> Alto consumo de red
                  detectado.
                </div>
              </div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. INTROSPECCIÓN DE INTERFACES -->
    <div class="section-group self-stretch q-mt-xl">
      <SectionTitle>3. Introspección de Interfaces</SectionTitle>
      <TextBlock>
        ¿Olvidaste qué campos tiene un mensaje <code>LaserScan</code>? No necesitas ir a la
        documentación web. Usa <code>ros2 interface</code> para ver la estructura de cualquier
        mensaje, servicio o acción desde la terminal.
      </TextBlock>

      <div class="row q-col-gutter-md q-mt-md">
        <div class="col-12 col-md-12">
          <div
            class="tool-card interface-card bg-slate-900 q-pa-lg border-purple shadow-2 overflow-hidden relative-position"
          >
            <!-- DECORATIVE BG -->
            <div
              class="absolute-right full-height bg-purple-9 opacity-10"
              style="width: 100px; clip-path: polygon(20% 0%, 100% 0, 100% 100%, 0% 100%)"
            ></div>

            <div class="row items-center justify-between q-mb-md relative-position z-top">
              <div class="row items-center">
                <div class="icon-box bg-purple-9 q-mr-md shadow-1">
                  <q-icon name="visibility" color="white" size="sm" />
                </div>
                <div class="text-purple-4 text-h6">Ver Estructura</div>
              </div>
              <div class="text-caption text-grey-5 font-italic">Ejemplo Real</div>
            </div>

            <div
              class="cmd-line bg-black q-pa-sm rounded-borders font-mono text-sm q-mb-md border-light relative-position z-top"
            >
              <span class="text-green-4">$ ros2 interface show</span>
              <span class="text-white">sensor_msgs/msg/LaserScan</span>
            </div>

            <div
              class="font-mono text-xs text-grey-4 columns-2 code-preview relative-position z-top"
            >
              <div class="q-mb-xs"><span class="text-blue-4">std_msgs/Header</span> header</div>
              <div class="q-mb-xs"><span class="text-purple-4">float32</span> angle_min</div>
              <div class="q-mb-xs"><span class="text-purple-4">float32</span> angle_max</div>
              <div class="q-mb-xs"><span class="text-purple-4">float32</span> angle_increment</div>
              <div class="q-mb-xs"><span class="text-purple-4">float32</span> time_increment</div>
              <div class="q-mb-xs"><span class="text-purple-4">float32</span> scan_time</div>
              <div class="q-mb-xs"><span class="text-purple-4">float32</span> range_min</div>
              <div class="q-mb-xs"><span class="text-purple-4">float32</span> range_max</div>
              <div class="q-mb-xs"><span class="text-purple-4">float32[]</span> ranges</div>
              <div class="q-mb-xs"><span class="text-purple-4">float32[]</span> intensities</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 5. CHEATSHEET RESUMEN -->
    <div class="section-group self-stretch q-mt-xl q-mb-xl">
      <SectionTitle>4. Resumen de Comandos Pro</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12">
          <div class="custom-card border-grey bg-slate-900 shadow-2">
            <div class="card-header row items-center q-pa-md border-bottom-dark bg-slate-800">
              <q-icon name="terminal" color="pink-4" size="sm" class="q-mr-md" />
              <div class="text-h6 text-white">Tu Navaja Suiza</div>
            </div>

            <div class="q-pa-md">
              <table class="cmd-table full-width text-left font-mono text-xs text-grey-4">
                <thead>
                  <tr class="text-pink-4">
                    <th class="q-pb-sm">Comando</th>
                    <th class="q-pb-sm">Descripción</th>
                  </tr>
                </thead>
                <tbody>
                  <tr class="hover-row">
                    <td class="q-py-sm text-green-4 font-weight-bold">
                      colcon build --symlink-install
                    </td>
                    <td class="q-py-sm">Compila creando enlaces (hot-reload para Python).</td>
                  </tr>
                  <tr class="hover-row">
                    <td class="q-py-sm text-green-4 font-weight-bold">colcon test</td>
                    <td class="q-py-sm">Ejecuta los tests automáticos del paquete.</td>
                  </tr>
                  <tr class="hover-row">
                    <td class="q-py-sm text-green-4 font-weight-bold">
                      ros2 topic hz &lt;topic&gt;
                    </td>
                    <td class="q-py-sm">Mide la frecuencia de publicación (Hz).</td>
                  </tr>
                  <tr class="hover-row">
                    <td class="q-py-sm text-green-4 font-weight-bold">
                      ros2 topic bw &lt;topic&gt;
                    </td>
                    <td class="q-py-sm">Mide el ancho de banda consumido (Bytes/s).</td>
                  </tr>
                  <tr class="hover-row">
                    <td class="q-py-sm text-green-4 font-weight-bold">
                      ros2 interface show &lt;type&gt;
                    </td>
                    <td class="q-py-sm">Muestra los campos internos de un mensaje.</td>
                  </tr>
                  <tr class="hover-row">
                    <td class="q-py-sm text-green-4 font-weight-bold">ros2 doctor</td>
                    <td class="q-py-sm">Diagnostica problemas del sistema ROS.</td>
                  </tr>
                </tbody>
              </table>
            </div>
          </div>
        </div>
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
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

/* COLCON VIZ */
.colcon-viz {
  border-top: 4px solid #ec4899;
}
.cmd-box {
  border-left: 4px solid;
  border-radius: 8px;
}
.border-pink {
  border-left-color: #ec4899;
}
.border-blue {
  border-left-color: #3b82f6;
}

/* CLI MONITOR */
.window-dots .dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
}
.border-l-green {
  border-left: 3px solid #4ade80;
}
.border-l-blue {
  border-left: 3px solid #60a5fa;
}
.border-bottom-dark {
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}
.bg-green-9-soft {
  background: rgba(74, 222, 128, 0.1);
}
.bg-blue-9-soft {
  background: rgba(96, 165, 250, 0.1);
}

/* INTERFACE CARD */
.border-purple {
  border-left: 4px solid #a855f7;
}
.columns-2 {
  column-count: 2;
  column-gap: 2rem;
}
.icon-box {
  width: 40px;
  height: 40px;
  border-radius: 8px;
  display: flex;
  align-items: center;
  justify-content: center;
}
.z-top {
  z-index: 2;
}
.opacity-10 {
  opacity: 0.1;
}

/* TABLE */
.cmd-table {
  border-collapse: collapse;
}
.cmd-table td {
  border-bottom: 1px solid rgba(255, 255, 255, 0.05);
}
.cmd-table tr:last-child td {
  border-bottom: none;
}
.hover-row:hover {
  background: rgba(255, 255, 255, 0.03);
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
.border-grey {
  border-color: #334155;
}
.transition-hover {
  transition:
    transform 0.2s,
    box-shadow 0.2s;
}
.transition-hover:hover {
  transform: translateY(-3px);
  box-shadow: 0 5px 15px rgba(0, 0, 0, 0.3);
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
  .columns-2 {
    column-count: 1;
  }
}
</style>
