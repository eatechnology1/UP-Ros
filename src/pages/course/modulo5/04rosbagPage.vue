<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      <strong>rosbag2</strong> es el sistema de grabación y replay de ROS 2. Permite capturar topics
      en archivos SQLite3/MCAP para testing, debugging, y análisis offline. Soporta compression,
      filtering, y playback con rate control.
    </TextBlock>

    <AlertBlock type="info" title="Capacidades Clave">
      <strong>Recording:</strong> Grabar topics con filtering y compression
      <br />
      <strong>Playback:</strong> Replay con rate control, loop, remapping
      <br />
      <strong>Storage:</strong> SQLite3 (default), MCAP (high-performance)
      <br />
      <strong>Compression:</strong> zstd, lz4 para reducir tamaño
    </AlertBlock>

    <!-- RECORDING -->
    <div class="section-group">
      <SectionTitle>1. Advanced Recording</SectionTitle>

      <div class="recording-options q-mt-md">
        <div class="rec-card basic">
          <div class="rec-header">
            <q-icon name="fiber_manual_record" />
            <span>Basic Recording</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Grabar todos los topics
ros2 bag record -a

# Grabar topics específicos
ros2 bag record /scan /odom /camera/image

# Output file custom
ros2 bag record -a -o my_data"
            :copyable="true"
          />
        </div>

        <div class="rec-card filtering">
          <div class="rec-header">
            <q-icon name="filter_alt" />
            <span>Filtering</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Regex filtering
ros2 bag record -e '/camera/.*/compressed'

# Excluir topics
ros2 bag record -a -x '/rosout|/diagnostics'

# Solo topics de un namespace
ros2 bag record -e '/robot1/.*'"
            :copyable="true"
          />
        </div>

        <div class="rec-card compression">
          <div class="rec-header">
            <q-icon name="compress" />
            <span>Compression</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Comprimir con zstd (mejor ratio)
ros2 bag record -a \
  --compression-mode file \
  --compression-format zstd

# Comprimir con lz4 (más rápido)
ros2 bag record -a \
  --compression-mode message \
  --compression-format lz4"
            :copyable="true"
          />
          <div class="rec-metric">⚡ zstd: 3-5x compression ratio</div>
        </div>

        <div class="rec-card splitting">
          <div class="rec-header">
            <q-icon name="call_split" />
            <span>Splitting</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Split por tamaño (1GB)
ros2 bag record -a --max-bag-size 1000000000

# Split por duración (300s = 5min)
ros2 bag record -a --max-bag-duration 300

# Útil para:
# - Evitar archivos gigantes
# - Facilitar transferencia
# - Procesamiento paralelo"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- PLAYBACK -->
    <div class="section-group">
      <SectionTitle>2. Playback Strategies</SectionTitle>

      <div class="playback-options q-mt-md">
        <div class="play-card">
          <div class="play-header">
            <q-icon name="play_arrow" color="green-4" />
            <span>Rate Control</span>
          </div>
          <div class="play-desc">Controlar velocidad de replay</div>
          <CodeBlock
            lang="bash"
            content="# Replay a velocidad normal
ros2 bag play my_data.db3

# Mitad de velocidad (slow motion)
ros2 bag play my_data.db3 --rate 0.5

# Doble velocidad
ros2 bag play my_data.db3 --rate 2.0

# Útil para:
# - Debugging (slow motion)
# - Testing rápido (fast forward)"
            :copyable="true"
          />
        </div>

        <div class="play-card">
          <div class="play-header">
            <q-icon name="loop" color="blue-4" />
            <span>Loop Playback</span>
          </div>
          <div class="play-desc">Replay continuo para testing</div>
          <CodeBlock
            lang="bash"
            content="# Loop infinito
ros2 bag play my_data.db3 --loop

# Útil para:
# - Testing de algoritmos
# - Demostración continua
# - Stress testing"
            :copyable="true"
          />
        </div>

        <div class="play-card">
          <div class="play-header">
            <q-icon name="swap_horiz" color="purple-4" />
            <span>Topic Remapping</span>
          </div>
          <div class="play-desc">Redirigir topics durante replay</div>
          <CodeBlock
            lang="bash"
            content="# Remap topic
ros2 bag play my_data.db3 \
  --remap /old_topic:=/new_topic

# Útil para:
# - Testing con diferentes nombres
# - Evitar conflictos
# - Multi-robot scenarios"
            :copyable="true"
          />
        </div>

        <div class="play-card">
          <div class="play-header">
            <q-icon name="schedule" color="orange-4" />
            <span>Clock Simulation</span>
          </div>
          <div class="play-desc">Publicar /clock para simulación de tiempo</div>
          <CodeBlock
            lang="bash"
            content="# Publicar /clock
ros2 bag play my_data.db3 --clock

# Nodes deben usar use_sim_time:=true
ros2 run my_pkg my_node --ros-args \
  -p use_sim_time:=true"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- ANALYSIS -->
    <div class="section-group">
      <SectionTitle>3. Analysis Tools</SectionTitle>

      <div class="analysis-tools q-mt-md">
        <div class="analysis-card">
          <div class="analysis-header">
            <q-icon name="info" />
            <span>Bag Info</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Ver metadata
ros2 bag info my_data.db3

# Output:
# Files:             my_data_0.db3
# Bag size:          1.2 GB
# Storage id:        sqlite3
# Duration:          120.5s
# Start:             Jan 11 2026 04:00:00
# End:               Jan 11 2026 04:02:00
# Messages:          45230
# Topic information:
#   /scan: 1205 msgs (sensor_msgs/LaserScan)
#   /odom: 12050 msgs (nav_msgs/Odometry)
#   /camera/image: 3600 msgs (sensor_msgs/Image)"
            :copyable="true"
          />
        </div>

        <div class="analysis-card">
          <div class="analysis-header">
            <q-icon name="search" />
            <span>Message Extraction</span>
          </div>
          <CodeBlock
            lang="bash"
            content="# Exportar a CSV (requiere plugin)
ros2 bag export my_data.db3 --output-format csv

# Procesar con Python
import rosbag2_py
from rclpy.serialization import deserialize_message

reader = rosbag2_py.SequentialReader()
reader.open('my_data.db3')

while reader.has_next():
    topic, data, timestamp = reader.read_next()
    # Procesar mensaje"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- STORAGE PLUGINS -->
    <div class="section-group">
      <SectionTitle>4. Storage Plugins</SectionTitle>

      <div class="storage-comparison q-mt-md">
        <div class="storage-card sqlite">
          <div class="storage-name">SQLite3</div>
          <div class="storage-desc">Default storage plugin</div>
          <div class="storage-pros">
            <div class="pro-item">✅ Amplio soporte</div>
            <div class="pro-item">✅ Fácil de usar</div>
            <div class="pro-item">✅ SQL queries</div>
          </div>
          <div class="storage-cons">
            <div class="con-item">❌ Performance limitado</div>
            <div class="con-item">❌ No streaming</div>
          </div>
        </div>

        <div class="storage-card mcap">
          <div class="storage-name">MCAP</div>
          <div class="storage-desc">High-performance storage</div>
          <div class="storage-pros">
            <div class="pro-item">✅ 10x más rápido</div>
            <div class="pro-item">✅ Streaming support</div>
            <div class="pro-item">✅ Mejor compression</div>
          </div>
          <div class="storage-cons">
            <div class="con-item">❌ Requiere instalación</div>
          </div>
          <CodeBlock
            lang="bash"
            content="# Instalar MCAP plugin
sudo apt install ros-humble-rosbag2-storage-mcap

# Usar MCAP
ros2 bag record -a -s mcap"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- BENCHMARKS -->
    <div class="section-group">
      <SectionTitle>5. Performance Benchmarks</SectionTitle>

      <div class="benchmark-table q-mt-md">
        <div class="bench-header">
          <div class="bench-title">Compression Comparison (10 min recording)</div>
        </div>
        <div class="bench-rows">
          <div class="bench-row header">
            <div class="bench-cell">Method</div>
            <div class="bench-cell">Size</div>
            <div class="bench-cell">Ratio</div>
          </div>
          <div class="bench-row">
            <div class="bench-cell">Uncompressed</div>
            <div class="bench-cell">5.2 GB</div>
            <div class="bench-cell baseline">1x</div>
          </div>
          <div class="bench-row">
            <div class="bench-cell">lz4</div>
            <div class="bench-cell">2.8 GB</div>
            <div class="bench-cell good">1.86x</div>
          </div>
          <div class="bench-row">
            <div class="bench-cell">zstd</div>
            <div class="bench-cell">1.1 GB</div>
            <div class="bench-cell excellent">4.73x</div>
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
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="rosbag2 Advanced Usage"
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
          Reemplaza con video técnico sobre rosbag2
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>ros2 bag record</code>
          <span>Grabar topics</span>
        </div>
        <div class="summary-item">
          <code>-e (regex)</code>
          <span>Filtering avanzado</span>
        </div>
        <div class="summary-item">
          <code>--compression zstd</code>
          <span>4.7x compression</span>
        </div>
        <div class="summary-item">
          <code>ros2 bag play</code>
          <span>Replay con rate control</span>
        </div>
        <div class="summary-item">
          <code>MCAP storage</code>
          <span>10x performance</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices" class="q-mt-lg">
        ✅ Usar compression zstd para ahorrar espacio (4.7x)
        <br />
        ✅ Split bags por tamaño para facilitar manejo
        <br />
        ✅ Filtering con regex para grabar solo lo necesario
        <br />
        ✅ MCAP storage para high-performance recording
        <br />
        ✅ Loop playback para testing continuo
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

/* RECORDING OPTIONS */
.recording-options {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.rec-card {
  background: var(--bg-surface);
  border: 2px solid #14b8a6;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.rec-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-weight: 700;
  color: var(--text-primary);
  font-size: 1.1rem;
}

.rec-metric {
  padding: 1rem;
  background: rgba(20, 184, 166, 0.1);
  border: 1px solid #14b8a6;
  border-radius: 8px;
  color: #5eead4;
  font-weight: 700;
  text-align: center;
}

/* PLAYBACK OPTIONS */
.playback-options {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.play-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.play-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: var(--text-primary);
  font-size: 1.05rem;
}

.play-desc {
  color: var(--text-muted);
  font-size: 0.9rem;
}

/* ANALYSIS TOOLS */
.analysis-tools {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.analysis-card {
  background: var(--bg-surface);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.analysis-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #14b8a6;
  font-size: 1.05rem;
  margin-bottom: 1rem;
}

/* STORAGE COMPARISON */
.storage-comparison {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.storage-card {
  background: var(--bg-surface);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.storage-card.sqlite {
  border-color: #06b6d4;
}

.storage-card.mcap {
  border-color: #10b981;
}

.storage-name {
  font-weight: 700;
  color: var(--text-primary);
  font-size: 1.3rem;
}

.storage-desc {
  color: var(--text-muted);
  font-size: 0.9rem;
}

.storage-pros,
.storage-cons {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.pro-item,
.con-item {
  color: var(--text-secondary);
  font-size: 0.9rem;
}

/* BENCHMARKS */
.benchmark-table {
  background: var(--bg-surface);
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
  color: var(--text-primary);
  background: var(--bg-surface-hover);
  border-radius: 8px;
}

.bench-row:last-child {
  border-bottom: none;
}

.bench-cell {
  color: var(--text-secondary);
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
}

.bench-cell.baseline {
  color: var(--text-muted);
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
  background: var(--bg-surface);
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
  color: var(--text-muted);
  font-size: 0.85rem;
}

/* SUMMARY */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: var(--bg-surface);
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
  color: var(--text-secondary);
  font-size: 0.85rem;
}

@media (max-width: 1024px) {
  .recording-options,
  .playback-options,
  .analysis-tools,
  .storage-comparison {
    grid-template-columns: 1fr;
  }
}
</style>
