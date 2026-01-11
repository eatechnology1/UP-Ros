<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      <strong>TF2</strong> (Transform Framework 2) gestiona el árbol de coordenadas del robot.
      Permite transformar puntos entre frames (base_link → map, camera → laser) con interpolación
      temporal. Esencial para fusión de sensores, navegación, y manipulación.
    </TextBlock>

    <AlertBlock type="info" title="Conceptos Clave">
      <strong>Transform Tree:</strong> Grafo acíclico dirigido de frames
      <br />
      <strong>Buffer:</strong> Cache de transforms con interpolación temporal
      <br />
      <strong>Static vs Dynamic:</strong> Transforms fijos vs variables en tiempo
      <br />
      <strong>Latency:</strong> Extrapolación limitada (default 10s)
    </AlertBlock>

    <!-- TF2 ARCHITECTURE -->
    <div class="section-group">
      <SectionTitle>1. TF2 Architecture</SectionTitle>

      <div class="architecture-diagram q-mt-md">
        <div class="arch-layer broadcaster">
          <div class="layer-title">Transform Broadcasters</div>
          <div class="layer-desc">Publican transforms a /tf y /tf_static</div>
          <div class="layer-examples">
            <div class="example-item">robot_state_publisher (URDF)</div>
            <div class="example-item">SLAM (map → odom)</div>
            <div class="example-item">Odometry (odom → base_link)</div>
          </div>
        </div>

        <div class="arch-arrow">↓ publish</div>

        <div class="arch-layer topics">
          <div class="layer-title">Topics</div>
          <div class="topic-grid">
            <div class="topic-item">
              <code>/tf</code>
              <span>Dynamic transforms (30-100 Hz)</span>
            </div>
            <div class="topic-item">
              <code>/tf_static</code>
              <span>Static transforms (latched)</span>
            </div>
          </div>
        </div>

        <div class="arch-arrow">↓ subscribe</div>

        <div class="arch-layer buffer">
          <div class="layer-title">TF2 Buffer</div>
          <div class="layer-desc">
            Cache de transforms con interpolación temporal (default 10s history)
          </div>
          <div class="buffer-features">
            <div class="feature">✅ Interpolación linear</div>
            <div class="feature">✅ Extrapolación limitada</div>
            <div class="feature">✅ Thread-safe</div>
          </div>
        </div>

        <div class="arch-arrow">↓ query</div>

        <div class="arch-layer listener">
          <div class="layer-title">Transform Listeners</div>
          <div class="layer-desc">Consultan transforms del buffer</div>
          <CodeBlock
            lang="cpp"
            content='// Lookup transform
auto transform = tf_buffer->lookupTransform(
  "map", "base_link", tf2::TimePointZero);'
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- STATIC VS DYNAMIC -->
    <div class="section-group">
      <SectionTitle>2. Static vs Dynamic Transforms</SectionTitle>

      <div class="transform-comparison q-mt-md">
        <div class="tf-card static">
          <div class="tf-header">
            <q-icon name="lock" />
            <span>Static Transforms</span>
          </div>
          <div class="tf-desc">
            Transforms que no cambian en el tiempo. Publicados en /tf_static (latched topic).
          </div>
          <div class="tf-examples">
            <div class="tf-example">base_link → laser</div>
            <div class="tf-example">base_link → camera</div>
            <div class="tf-example">camera → camera_optical</div>
          </div>
          <CodeBlock
            title="Publicar static transform"
            lang="bash"
            content="# CLI
ros2 run tf2_ros static_transform_publisher \
  0.1 0 0.2 0 0 0 base_link laser

# Launch file
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0.1', '0', '0.2', '0', '0', '0',
               'base_link', 'laser']
)"
            :copyable="true"
          />
        </div>

        <div class="tf-card dynamic">
          <div class="tf-header">
            <q-icon name="update" />
            <span>Dynamic Transforms</span>
          </div>
          <div class="tf-desc">
            Transforms que cambian continuamente. Publicados en /tf a alta frecuencia.
          </div>
          <div class="tf-examples">
            <div class="tf-example">map → odom (SLAM)</div>
            <div class="tf-example">odom → base_link (Odometry)</div>
            <div class="tf-example">base_link → gripper (Manipulator)</div>
          </div>
          <CodeBlock
            title="Publicar dynamic transform (C++)"
            lang="cpp"
            content='#include <tf2_ros/transform_broadcaster.h>

auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

geometry_msgs::msg::TransformStamped t;
t.header.stamp = node->now();
t.header.frame_id = "odom";
t.child_frame_id = "base_link";
t.transform.translation.x = odom_x;
t.transform.translation.y = odom_y;
t.transform.rotation = odom_quat;

tf_broadcaster->sendTransform(t);'
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- DEBUGGING TOOLS -->
    <div class="section-group">
      <SectionTitle>3. Debugging Tools</SectionTitle>

      <div class="debug-tools q-mt-md">
        <div class="debug-card">
          <div class="debug-header">
            <q-icon name="account_tree" color="green-4" />
            <span>view_frames</span>
          </div>
          <div class="debug-desc">Genera PDF con el TF tree completo</div>
          <CodeBlock
            lang="bash"
            content="# Generar PDF
ros2 run tf2_tools view_frames

# Output: frames_YYYY-MM-DD_HH-MM-SS.pdf
# Muestra:
# - Todos los frames
# - Parent/child relationships
# - Frecuencia de publicación
# - Latencia promedio"
            :copyable="true"
          />
        </div>

        <div class="debug-card">
          <div class="debug-header">
            <q-icon name="swap_horiz" color="blue-4" />
            <span>tf2_echo</span>
          </div>
          <div class="debug-desc">Monitorea transform entre dos frames en tiempo real</div>
          <CodeBlock
            lang="bash"
            content="# Ver transform
ros2 run tf2_ros tf2_echo map base_link

# Output (30 Hz):
# At time 1234.567
# - Translation: [1.23, 0.45, 0.00]
# - Rotation: in Quaternion [0.00, 0.00, 0.38, 0.92]
#             in RPY (radian) [0.00, 0.00, 0.78]
#             in RPY (degree) [0.00, 0.00, 44.70]"
            :copyable="true"
          />
        </div>

        <div class="debug-card">
          <div class="debug-header">
            <q-icon name="monitor_heart" color="purple-4" />
            <span>tf2_monitor</span>
          </div>
          <div class="debug-desc">Detecta problemas: missing transforms, delays, loops</div>
          <CodeBlock
            lang="bash"
            content="# Monitorear TF tree
ros2 run tf2_ros tf2_monitor

# Detecta:
# ❌ Missing transforms
# ❌ Circular dependencies
# ❌ High latency (>100ms)
# ❌ Low frequency (<10Hz)"
            :copyable="true"
          />
        </div>

        <div class="debug-card">
          <div class="debug-header">
            <q-icon name="visibility" color="orange-4" />
            <span>RViz TF Display</span>
          </div>
          <div class="debug-desc">Visualización 3D del TF tree</div>
          <CodeBlock
            lang="yaml"
            content="# RViz config
- Class: rviz_default_plugins/TF
  Show Names: true
  Show Axes: true
  Show Arrows: true
  Marker Scale: 1.0
  Update Interval: 0.0"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- COMMON ISSUES -->
    <div class="section-group">
      <SectionTitle>4. Common Issues & Solutions</SectionTitle>

      <div class="issues-grid q-mt-md">
        <div class="issue-card">
          <div class="issue-title">❌ Missing Transform</div>
          <div class="issue-symptom">
            <strong>Error:</strong> "Could not find transform from X to Y"
          </div>
          <div class="issue-causes">
            <strong>Causas:</strong>
            <ul>
              <li>Broadcaster no está corriendo</li>
              <li>Nombre de frame incorrecto</li>
              <li>Timestamp futuro (extrapolación)</li>
            </ul>
          </div>
          <div class="issue-solution">
            <strong>Solución:</strong>
            <CodeBlock
              lang="bash"
              content="# Verificar frames disponibles
ros2 topic echo /tf --once

# Ver TF tree
ros2 run tf2_tools view_frames"
              :copyable="true"
            />
          </div>
        </div>

        <div class="issue-card">
          <div class="issue-title">❌ Circular Dependency</div>
          <div class="issue-symptom"><strong>Error:</strong> "Detected loop in TF tree"</div>
          <div class="issue-causes">
            <strong>Causas:</strong>
            <ul>
              <li>Dos broadcasters publican mismo transform</li>
              <li>Parent/child invertidos</li>
            </ul>
          </div>
          <div class="issue-solution">
            <strong>Solución:</strong>
            <CodeBlock
              lang="bash"
              content="# Identificar loop
ros2 run tf2_ros tf2_monitor

# Deshabilitar broadcaster duplicado"
              :copyable="true"
            />
          </div>
        </div>

        <div class="issue-card">
          <div class="issue-title">⚠️ High Latency</div>
          <div class="issue-symptom"><strong>Warning:</strong> "Transform is X seconds old"</div>
          <div class="issue-causes">
            <strong>Causas:</strong>
            <ul>
              <li>Broadcaster con frecuencia baja</li>
              <li>Network delay</li>
              <li>CPU overload</li>
            </ul>
          </div>
          <div class="issue-solution">
            <strong>Solución:</strong>
            <CodeBlock
              lang="bash"
              content="# Verificar frecuencia
ros2 topic hz /tf

# Debe ser >10 Hz para transforms críticos"
              :copyable="true"
            />
          </div>
        </div>

        <div class="issue-card">
          <div class="issue-title">⚠️ Timestamp Issues</div>
          <div class="issue-symptom"><strong>Error:</strong> "Extrapolation into the future"</div>
          <div class="issue-causes">
            <strong>Causas:</strong>
            <ul>
              <li>Timestamp futuro en mensaje</li>
              <li>Clocks desincronizados</li>
            </ul>
          </div>
          <div class="issue-solution">
            <strong>Solución:</strong>
            <CodeBlock
              lang="cpp"
              content='// Usar timestamp actual
t.header.stamp = node->now();

// O usar tf2::TimePointZero para latest
auto transform = tf_buffer->lookupTransform(
  "map", "base_link", tf2::TimePointZero);'
              :copyable="true"
            />
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
            src="https://www.youtube.com/embed/dQw4w9WgXcQ"
            title="TF2 Debugging Guide"
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
          Reemplaza con video técnico sobre TF2
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>TF2 Buffer</code>
          <span>Cache con interpolación temporal</span>
        </div>
        <div class="summary-item">
          <code>/tf vs /tf_static</code>
          <span>Dynamic vs static transforms</span>
        </div>
        <div class="summary-item">
          <code>view_frames</code>
          <span>Generar PDF del TF tree</span>
        </div>
        <div class="summary-item">
          <code>tf2_echo</code>
          <span>Monitor transform en tiempo real</span>
        </div>
        <div class="summary-item">
          <code>tf2_monitor</code>
          <span>Detectar problemas</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices" class="q-mt-lg">
        ✅ Usar /tf_static para transforms fijos (reduce bandwidth)
        <br />
        ✅ Publicar transforms dinámicos a >10 Hz
        <br />
        ✅ Usar view_frames para debugging de arquitectura
        <br />
        ✅ Evitar circular dependencies (un solo parent por frame)
        <br />
        ✅ Usar tf2::TimePointZero para latest transform
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

/* ARCHITECTURE DIAGRAM */
.architecture-diagram {
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.arch-layer {
  background: rgba(0, 0, 0, 0.3);
  border: 2px solid;
  border-radius: 12px;
  padding: 1.5rem;
}

.arch-layer.broadcaster {
  border-color: #14b8a6;
}

.arch-layer.topics {
  border-color: #06b6d4;
}

.arch-layer.buffer {
  border-color: #8b5cf6;
}

.arch-layer.listener {
  border-color: #10b981;
}

.layer-title {
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
  margin-bottom: 0.75rem;
}

.layer-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
  margin-bottom: 1rem;
}

.layer-examples,
.buffer-features {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.example-item,
.feature {
  color: #94a3b8;
  font-size: 0.85rem;
}

.topic-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1rem;
}

.topic-item {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
}

.topic-item code {
  font-family: 'Fira Code', monospace;
  color: #5eead4;
  font-weight: 700;
}

.topic-item span {
  color: #94a3b8;
  font-size: 0.85rem;
}

.arch-arrow {
  text-align: center;
  color: #14b8a6;
  font-weight: 700;
  font-size: 1.5rem;
}

/* TRANSFORM COMPARISON */
.transform-comparison {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.tf-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.tf-card.static {
  border-color: #06b6d4;
}

.tf-card.dynamic {
  border-color: #10b981;
}

.tf-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
}

.tf-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.tf-examples {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.tf-example {
  padding: 0.5rem 1rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  color: #5eead4;
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
}

/* DEBUG TOOLS */
.debug-tools {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.debug-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.debug-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.05rem;
}

.debug-desc {
  color: #94a3b8;
  font-size: 0.9rem;
}

/* ISSUES GRID */
.issues-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.issue-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid rgba(239, 68, 68, 0.3);
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.issue-title {
  font-weight: 700;
  color: #fca5a5;
  font-size: 1.1rem;
}

.issue-symptom,
.issue-causes,
.issue-solution {
  color: #cbd5e1;
  font-size: 0.9rem;
}

.issue-symptom strong,
.issue-causes strong,
.issue-solution strong {
  color: #f1f5f9;
}

.issue-causes ul,
.issue-solution ul {
  margin: 0.5rem 0 0 1.5rem;
  padding: 0;
}

.issue-causes li {
  color: #94a3b8;
  font-size: 0.85rem;
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
  .transform-comparison,
  .debug-tools,
  .issues-grid {
    grid-template-columns: 1fr;
  }

  .topic-grid {
    grid-template-columns: 1fr;
  }
}
</style>
