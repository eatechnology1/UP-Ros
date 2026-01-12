<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      Las <strong>herramientas de desarrollo</strong> de ROS 2 permiten diagnosticar, optimizar y
      debuggear sistemas robóticos complejos. Dominar colcon, CLI tools, y profiling es esencial
      para desarrollo profesional y deployment en producción.
    </TextBlock>

    <AlertBlock type="info" title="Herramientas Críticas">
      <strong>colcon:</strong> Build system paralelo con optimización incremental
      <br />
      <strong>ros2 CLI:</strong> Introspección de topics, services, actions, nodes
      <br />
      <strong>rosbag:</strong> Grabación y replay de datos para testing
      <br />
      <strong>Profiling:</strong> ros2_tracing, perf, valgrind para performance analysis
    </AlertBlock>

    <!-- BUILD SYSTEM -->
    <div class="section-group">
      <SectionTitle>1. colcon Build System Deep Dive</SectionTitle>
      <TextBlock>
        <strong>colcon</strong> (collective construction) orquesta CMake, ament_cmake, y setuptools
        para compilar workspaces multi-paquete con dependencias complejas. Implementa compilación
        paralela, caching, y optimización incremental.
      </TextBlock>

      <div class="build-optimization q-mt-md">
        <div class="opt-card symlink">
          <div class="opt-header">
            <q-icon name="link" size="2rem" />
            <span>Symlink Install</span>
          </div>
          <div class="opt-desc">
            Crea enlaces simbólicos en lugar de copiar archivos. Cambios en Python/Launch se
            reflejan inmediatamente sin recompilar.
          </div>
          <CodeBlock
            lang="bash"
            content="colcon build --symlink-install

# Beneficios:
# - Python: Sin recompilación (editar y ejecutar)
# - Launch files: Cambios instantáneos
# - Config files: Sin rebuild
# - Speedup: 10x para iteración rápida"
            :copyable="true"
          />
          <div class="opt-metric">⚡ 10x más rápido para desarrollo iterativo</div>
        </div>

        <div class="opt-card parallel">
          <div class="opt-header">
            <q-icon name="memory" size="2rem" />
            <span>Parallel Workers</span>
          </div>
          <div class="opt-desc">
            Compila paquetes independientes simultáneamente. Speedup lineal con número de cores.
          </div>
          <CodeBlock
            lang="bash"
            content="# Auto-detect cores (default)
colcon build

# Especificar workers
colcon build --parallel-workers 8

# Limitar por memoria
colcon build --executor sequential"
            :copyable="true"
          />
          <div class="opt-metric">⚡ 4x speedup en CPU de 8 cores</div>
        </div>

        <div class="opt-card ccache">
          <div class="opt-header">
            <q-icon name="cached" size="2rem" />
            <span>ccache Integration</span>
          </div>
          <div class="opt-desc">
            Cachea resultados de compilación. Rebuilds son 10-15x más rápidos.
          </div>
          <CodeBlock
            lang="bash"
            content="# Instalar ccache
sudo apt install ccache

# Configurar
export CC='ccache gcc'
export CXX='ccache g++'

# Compilar
colcon build

# Ver estadísticas
ccache -s"
            :copyable="true"
          />
          <div class="opt-metric">⚡ 15x speedup en rebuilds</div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Build Types</SectionTitle>
        <div class="build-types q-mt-md">
          <div class="type-card debug">
            <div class="type-name">Debug</div>
            <div class="type-flags">-O0 -g</div>
            <div class="type-desc">Sin optimización, debug symbols completos</div>
            <div class="type-use">Desarrollo, debugging con gdb</div>
          </div>

          <div class="type-card release">
            <div class="type-name">Release</div>
            <div class="type-flags">-O3 -DNDEBUG</div>
            <div class="type-desc">Optimización máxima, sin debug symbols</div>
            <div class="type-use">Producción, benchmarks</div>
            <div class="type-perf">3-5x más rápido que Debug</div>
          </div>

          <div class="type-card relwithdebinfo">
            <div class="type-name">RelWithDebInfo</div>
            <div class="type-flags">-O2 -g</div>
            <div class="type-desc">Optimizado + debug symbols</div>
            <div class="type-use">Profiling, debugging de performance</div>
          </div>
        </div>

        <CodeBlock
          title="Especificar build type"
          lang="bash"
          content="# Release (producción)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Debug (desarrollo)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# RelWithDebInfo (profiling)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
          :copyable="true"
        />
      </div>
    </div>

    <!-- CLI TOOLS -->
    <div class="section-group">
      <SectionTitle>2. Advanced CLI Tools</SectionTitle>

      <div class="cli-tools q-mt-md">
        <div class="tool-section">
          <div class="tool-title">
            <q-icon name="topic" />
            <span>Topic Introspection</span>
          </div>
          <div class="tool-commands">
            <div class="cmd-item">
              <code>ros2 topic hz /scan</code>
              <span>Frecuencia de publicación (Hz)</span>
            </div>
            <div class="cmd-item">
              <code>ros2 topic bw /camera/image</code>
              <span>Bandwidth (MB/s)</span>
            </div>
            <div class="cmd-item">
              <code>ros2 topic delay /odom</code>
              <span>Latencia end-to-end</span>
            </div>
            <div class="cmd-item">
              <code>ros2 topic echo /cmd_vel --no-arr</code>
              <span>Ver mensajes (sin arrays)</span>
            </div>
          </div>
          <CodeBlock
            title="Ejemplo: Diagnóstico de cámara"
            lang="bash"
            content="# Verificar frecuencia (debe ser 30 Hz)
ros2 topic hz /camera/image_raw
# average rate: 29.8 Hz

# Verificar bandwidth
ros2 topic bw /camera/image_raw
# average: 25.3 MB/s

# Diagnóstico: Frecuencia OK, bandwidth alto (comprimir?)"
            :copyable="true"
          />
        </div>

        <div class="tool-section">
          <div class="tool-title">
            <q-icon name="album" />
            <span>rosbag Advanced</span>
          </div>
          <div class="tool-commands">
            <div class="cmd-item">
              <code>ros2 bag record -a</code>
              <span>Grabar todos los topics</span>
            </div>
            <div class="cmd-item">
              <code>ros2 bag record /scan /odom -o data</code>
              <span>Grabar topics específicos</span>
            </div>
            <div class="cmd-item">
              <code>ros2 bag play data.db3 --rate 0.5</code>
              <span>Replay a mitad de velocidad</span>
            </div>
            <div class="cmd-item">
              <code>ros2 bag info data.db3</code>
              <span>Ver metadata (duración, topics, tamaño)</span>
            </div>
          </div>
          <CodeBlock
            title="Filtering y compression"
            lang="bash"
            content="# Grabar con regex filter
ros2 bag record -e '/camera/.*/compressed'

# Comprimir con zstd
ros2 bag record -a --compression-mode file \
  --compression-format zstd

# Split por tamaño (1GB)
ros2 bag record -a --max-bag-size 1000000000"
            :copyable="true"
          />
        </div>

        <div class="tool-section">
          <div class="tool-title">
            <q-icon name="healing" />
            <span>ros2 doctor</span>
          </div>
          <div class="tool-desc">
            Diagnóstico automático de problemas comunes: network, DDS, QoS mismatches
          </div>
          <CodeBlock
            lang="bash"
            content="# Diagnóstico completo
ros2 doctor

# Ver solo warnings
ros2 doctor --report

# Incluir network info
ros2 doctor --include-network"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- PROFILING -->
    <div class="section-group">
      <SectionTitle>3. Profiling & Performance Analysis</SectionTitle>
      <TextBlock>
        El profiling permite identificar bottlenecks de CPU, memoria, y latencia. ROS 2 provee
        herramientas específicas para tracing de eventos y análisis de performance.
      </TextBlock>

      <div class="profiling-tools q-mt-md">
        <div class="prof-card">
          <div class="prof-header">
            <q-icon name="speed" color="blue-4" />
            <span>ros2_tracing</span>
          </div>
          <div class="prof-desc">
            Tracing de bajo overhead basado en LTTng. Captura eventos de ROS 2 (callbacks,
            publishers, timers) con timestamps precisos.
          </div>
          <CodeBlock
            lang="bash"
            content="# Instalar
sudo apt install ros-humble-ros2trace

# Grabar trace
ros2 trace

# Ejecutar nodo (en otra terminal)
ros2 run my_pkg my_node

# Analizar con babeltrace
babeltrace2 ~/.ros/tracing/session-*/ust/uid/*/*"
            :copyable="true"
          />
        </div>

        <div class="prof-card">
          <div class="prof-header">
            <q-icon name="memory" color="green-4" />
            <span>perf (CPU Profiling)</span>
          </div>
          <div class="prof-desc">
            Profiler de CPU del kernel Linux. Identifica funciones que consumen más tiempo.
          </div>
          <CodeBlock
            lang="bash"
            content="# Instalar
sudo apt install linux-tools-common

# Profile nodo
perf record -g ros2 run my_pkg my_node

# Analizar
perf report

# Flamegraph
perf script | stackcollapse-perf.pl | flamegraph.pl > flame.svg"
            :copyable="true"
          />
        </div>

        <div class="prof-card">
          <div class="prof-header">
            <q-icon name="bug_report" color="red-4" />
            <span>valgrind (Memory Leaks)</span>
          </div>
          <div class="prof-desc">Detecta memory leaks, use-after-free, buffer overflows.</div>
          <CodeBlock
            lang="bash"
            content="# Instalar
sudo apt install valgrind

# Detectar leaks
valgrind --leak-check=full ros2 run my_pkg my_node

# Profiling de memoria
valgrind --tool=massif ros2 run my_pkg my_node"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- BENCHMARKS -->
    <div class="section-group">
      <SectionTitle>4. Performance Benchmarks</SectionTitle>

      <div class="benchmark-table q-mt-md">
        <div class="bench-header">
          <div class="bench-title">Build Performance (10 paquetes C++)</div>
        </div>
        <div class="bench-rows">
          <div class="bench-row header">
            <div class="bench-cell">Método</div>
            <div class="bench-cell">Tiempo</div>
            <div class="bench-cell">Speedup</div>
          </div>
          <div class="bench-row">
            <div class="bench-cell">colcon build</div>
            <div class="bench-cell">180s</div>
            <div class="bench-cell baseline">1x</div>
          </div>
          <div class="bench-row">
            <div class="bench-cell">--symlink-install</div>
            <div class="bench-cell">175s</div>
            <div class="bench-cell">1.03x</div>
          </div>
          <div class="bench-row">
            <div class="bench-cell">--parallel-workers 8</div>
            <div class="bench-cell">45s</div>
            <div class="bench-cell good">4x</div>
          </div>
          <div class="bench-row">
            <div class="bench-cell">+ ccache (rebuild)</div>
            <div class="bench-cell">12s</div>
            <div class="bench-cell excellent">15x</div>
          </div>
        </div>
      </div>

      <div class="benchmark-table q-mt-md">
        <div class="bench-header">
          <div class="bench-title">Topic Latency (localhost)</div>
        </div>
        <div class="bench-rows">
          <div class="bench-row header">
            <div class="bench-cell">Message Size</div>
            <div class="bench-cell">Intra-Process</div>
            <div class="bench-cell">Inter-Process</div>
          </div>
          <div class="bench-row">
            <div class="bench-cell">1 KB</div>
            <div class="bench-cell">8 μs</div>
            <div class="bench-cell">120 μs</div>
          </div>
          <div class="bench-row">
            <div class="bench-cell">100 KB</div>
            <div class="bench-cell">12 μs</div>
            <div class="bench-cell">850 μs</div>
          </div>
          <div class="bench-row">
            <div class="bench-cell">1 MB</div>
            <div class="bench-cell">45 μs</div>
            <div class="bench-cell">8.5 ms</div>
          </div>
        </div>
      </div>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://youtu.be/Romc22GgusU"
            title="ROS 2 Development Tools"
            frameborder="0"
            allow="
              accelerometer;
              autoplay;
              clipboard-write;
              encrypted-media;
              gyroscope;
              picture-in-picture;
            "
            allowfullscreen
          ></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" color="blue-4" size="sm" />
          Reemplaza con video técnico sobre herramientas de desarrollo
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>colcon build</code>
          <span>Build system paralelo</span>
        </div>
        <div class="summary-item">
          <code>--symlink-install</code>
          <span>10x speedup desarrollo</span>
        </div>
        <div class="summary-item">
          <code>ccache</code>
          <span>15x speedup rebuilds</span>
        </div>
        <div class="summary-item">
          <code>ros2 topic hz/bw</code>
          <span>Diagnóstico de topics</span>
        </div>
        <div class="summary-item">
          <code>ros2 bag</code>
          <span>Record/replay datos</span>
        </div>
        <div class="summary-item">
          <code>ros2_tracing</code>
          <span>Performance profiling</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices" class="q-mt-lg">
        ✅ Usar --symlink-install para desarrollo Python/Launch
        <br />
        ✅ Compilar en Release para producción (3-5x speedup)
        <br />
        ✅ Instalar ccache para rebuilds frecuentes (15x speedup)
        <br />
        ✅ Monitorear hz/bw de topics críticos
        <br />
        ✅ Grabar rosbags para debugging y testing
        <br />
        ✅ Usar ros2_tracing para identificar bottlenecks
      </AlertBlock>
    </div>
  </LessonContainer>
</template>

<script setup lang="ts">
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
</script>

<style scoped>
.section-group {
  margin-bottom: 3.5rem;
}

/* BUILD OPTIMIZATION */
.build-optimization {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.opt-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.opt-card.symlink {
  border-color: #14b8a6;
}

.opt-card.parallel {
  border-color: #06b6d4;
}

.opt-card.ccache {
  border-color: #0ea5e9;
}

.opt-header {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
}

.opt-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.opt-metric {
  padding: 1rem;
  background: rgba(20, 184, 166, 0.1);
  border: 1px solid #14b8a6;
  border-radius: 8px;
  color: #5eead4;
  font-weight: 700;
  text-align: center;
}

/* BUILD TYPES */
.build-types {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1.5rem;
}

.type-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.type-name {
  font-weight: 700;
  color: #14b8a6;
  font-size: 1.1rem;
}

.type-flags {
  font-family: 'Fira Code', monospace;
  color: #94a3b8;
  font-size: 0.85rem;
}

.type-desc,
.type-use {
  color: #cbd5e1;
  font-size: 0.9rem;
}

.type-perf {
  padding: 0.75rem;
  background: rgba(16, 185, 129, 0.1);
  border: 1px solid #10b981;
  border-radius: 6px;
  color: #6ee7b7;
  font-weight: 700;
  text-align: center;
  font-size: 0.9rem;
}

/* CLI TOOLS */
.cli-tools {
  display: flex;
  flex-direction: column;
  gap: 2rem;
}

.tool-section {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.tool-title {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-size: 1.2rem;
  font-weight: 700;
  color: #14b8a6;
  margin-bottom: 1.5rem;
}

.tool-commands {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  margin-bottom: 1.5rem;
}

.cmd-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
}

.cmd-item code {
  font-family: 'Fira Code', monospace;
  color: #5eead4;
  font-weight: 700;
}

.cmd-item span {
  color: #94a3b8;
  font-size: 0.85rem;
}

.tool-desc {
  color: #cbd5e1;
  margin-bottom: 1rem;
}

/* PROFILING */
.profiling-tools {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.prof-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.prof-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
}

.prof-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

/* BENCHMARKS */
.benchmark-table {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  overflow: hidden;
}

.bench-header {
  padding: 1rem 1.5rem;
  background: rgba(20, 184, 166, 0.1);
  border-bottom: 1px solid rgba(20, 184, 166, 0.3);
}

.bench-title {
  font-weight: 700;
  color: #14b8a6;
  font-size: 1.1rem;
}

.bench-rows {
  padding: 1.5rem;
}

.bench-row {
  display: grid;
  grid-template-columns: 2fr 1fr 1fr;
  gap: 1rem;
  padding: 1rem;
  border-bottom: 1px solid rgba(148, 163, 184, 0.1);
}

.bench-row.header {
  font-weight: 700;
  color: #f1f5f9;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
}

.bench-row:last-child {
  border-bottom: none;
}

.bench-cell {
  color: #cbd5e1;
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

.bench-cell.baseline {
  color: #94a3b8;
}

.bench-cell.good {
  color: #5eead4;
  font-weight: 700;
}

.bench-cell.excellent {
  color: #6ee7b7;
  font-weight: 700;
}

/* VIDEO */
.video-container {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 1.5rem;
}

.video-wrapper {
  position: relative;
  padding-bottom: 56.25%;
  height: 0;
  overflow: hidden;
  border-radius: 12px;
  background: #000;
}

.video-wrapper iframe {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}

.video-caption {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-top: 1rem;
  padding: 0.75rem;
  background: rgba(59, 130, 246, 0.1);
  border-radius: 8px;
  color: #94a3b8;
  font-size: 0.85rem;
}

/* SUMMARY */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.summary-item code {
  font-family: 'Fira Code', monospace;
  color: #14b8a6;
  font-size: 0.95rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

@media (max-width: 1024px) {
  .build-types {
    grid-template-columns: 1fr;
  }
}
</style>
