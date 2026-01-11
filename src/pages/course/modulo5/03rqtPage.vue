<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      <strong>rqt</strong> es el framework de GUI modular de ROS 2 basado en Qt5. Provee plugins
      para introspección (rqt_graph), plotting (rqt_plot), logging (rqt_console), y dynamic
      reconfigure. Extensible mediante plugins custom en Python o C++.
    </TextBlock>

    <AlertBlock type="info" title="Plugins Esenciales">
      <strong>rqt_graph:</strong> Visualización de computation graph (nodes/topics)
      <br />
      <strong>rqt_plot:</strong> Real-time plotting de datos numéricos
      <br />
      <strong>rqt_console:</strong> Log viewer con filtering por severity
      <br />
      <strong>rqt_reconfigure:</strong> Dynamic parameter tuning
    </AlertBlock>

    <!-- CORE PLUGINS -->
    <div class="section-group">
      <SectionTitle>1. Core Plugins Deep Dive</SectionTitle>

      <div class="plugin-grid q-mt-md">
        <div class="plugin-card graph">
          <div class="plugin-header">
            <q-icon name="account_tree" />
            <span>rqt_graph</span>
          </div>
          <div class="plugin-desc">
            Visualiza el computation graph: nodes, topics, services, actions. Útil para debugging de
            arquitectura y detección de conexiones faltantes.
          </div>
          <div class="plugin-features">
            <div class="feature-item">
              <q-icon name="check" size="sm" />
              <span>Filtrado por namespace</span>
            </div>
            <div class="feature-item">
              <q-icon name="check" size="sm" />
              <span>Agrupación de nodes</span>
            </div>
            <div class="feature-item">
              <q-icon name="check" size="sm" />
              <span>Refresh automático</span>
            </div>
          </div>
          <CodeBlock
            lang="bash"
            content="# Lanzar rqt_graph
rqt_graph

# Opciones:
# - Nodes only: Ver solo nodes
# - Nodes/Topics (active): Solo conexiones activas
# - Nodes/Topics (all): Todas las conexiones"
            :copyable="true"
          />
        </div>

        <div class="plugin-card plot">
          <div class="plugin-header">
            <q-icon name="show_chart" />
            <span>rqt_plot</span>
          </div>
          <div class="plugin-desc">
            Plotting en tiempo real de datos numéricos. Soporta múltiples topics, zoom, y export a
            CSV.
          </div>
          <div class="plugin-features">
            <div class="feature-item">
              <q-icon name="check" size="sm" />
              <span>Multi-topic plotting</span>
            </div>
            <div class="feature-item">
              <q-icon name="check" size="sm" />
              <span>Zoom y pan</span>
            </div>
            <div class="feature-item">
              <q-icon name="check" size="sm" />
              <span>Export a CSV/PNG</span>
            </div>
          </div>
          <CodeBlock
            lang="bash"
            content="# Lanzar rqt_plot
rqt_plot

# Plot topic específico
rqt_plot /odom/pose/pose/position/x

# Plot múltiples fields
rqt_plot /odom/pose/pose/position/x:y:z"
            :copyable="true"
          />
        </div>

        <div class="plugin-card console">
          <div class="plugin-header">
            <q-icon name="terminal" />
            <span>rqt_console</span>
          </div>
          <div class="plugin-desc">
            Visualizador de logs con filtering por severity (DEBUG, INFO, WARN, ERROR, FATAL) y node
            name.
          </div>
          <div class="plugin-features">
            <div class="feature-item">
              <q-icon name="check" size="sm" />
              <span>Severity filtering</span>
            </div>
            <div class="feature-item">
              <q-icon name="check" size="sm" />
              <span>Regex search</span>
            </div>
            <div class="feature-item">
              <q-icon name="check" size="sm" />
              <span>Pause/Resume</span>
            </div>
          </div>
          <CodeBlock
            lang="bash"
            content="# Lanzar rqt_console
rqt_console

# Severity levels:
# DEBUG: Información detallada
# INFO: Eventos normales
# WARN: Advertencias
# ERROR: Errores recuperables
# FATAL: Errores críticos"
            :copyable="true"
          />
        </div>

        <div class="plugin-card reconfigure">
          <div class="plugin-header">
            <q-icon name="tune" />
            <span>rqt_reconfigure</span>
          </div>
          <div class="plugin-desc">
            Ajuste dinámico de parámetros sin reiniciar nodes. Útil para tuning de PID, thresholds,
            y configuración en runtime.
          </div>
          <div class="plugin-features">
            <div class="feature-item">
              <q-icon name="check" size="sm" />
              <span>Sliders interactivos</span>
            </div>
            <div class="feature-item">
              <q-icon name="check" size="sm" />
              <span>Dropdowns para enums</span>
            </div>
            <div class="feature-item">
              <q-icon name="check" size="sm" />
              <span>Checkboxes para bools</span>
            </div>
          </div>
          <CodeBlock
            lang="bash"
            content="# Lanzar rqt_reconfigure
rqt_reconfigure

# Ejemplo: Tuning de PID
# - Ajustar Kp, Ki, Kd con sliders
# - Ver efecto en tiempo real
# - Guardar configuración"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- ADDITIONAL PLUGINS -->
    <div class="section-group">
      <SectionTitle>2. Additional Useful Plugins</SectionTitle>

      <div class="additional-plugins q-mt-md">
        <div class="add-plugin">
          <div class="add-header">
            <q-icon name="image" />
            <span>rqt_image_view</span>
          </div>
          <div class="add-desc">Visualizador de imágenes (RGB, depth, compressed)</div>
        </div>

        <div class="add-plugin">
          <div class="add-header">
            <q-icon name="list" />
            <span>rqt_topic</span>
          </div>
          <div class="add-desc">Monitor de topics (hz, bw, echo)</div>
        </div>

        <div class="add-plugin">
          <div class="add-header">
            <q-icon name="settings_remote" />
            <span>rqt_service_caller</span>
          </div>
          <div class="add-desc">Llamar services con GUI</div>
        </div>

        <div class="add-plugin">
          <div class="add-header">
            <q-icon name="flag" />
            <span>rqt_action_type_browser</span>
          </div>
          <div class="add-desc">Browser de action types</div>
        </div>

        <div class="add-plugin">
          <div class="add-header">
            <q-icon name="info" />
            <span>rqt_msg_type_browser</span>
          </div>
          <div class="add-desc">Browser de message types</div>
        </div>

        <div class="add-plugin">
          <div class="add-header">
            <q-icon name="dashboard" />
            <span>rqt_robot_monitor</span>
          </div>
          <div class="add-desc">Monitor de diagnósticos del robot</div>
        </div>
      </div>
    </div>

    <!-- CUSTOM PLUGIN -->
    <div class="section-group">
      <SectionTitle>3. Custom Plugin Development</SectionTitle>
      <TextBlock>
        rqt permite crear plugins custom en Python o C++. Útil para interfaces especializadas,
        dashboards, y herramientas de debugging específicas del proyecto.
      </TextBlock>

      <div class="custom-dev q-mt-md">
        <CodeBlock
          title="Plugin skeleton (Python)"
          lang="python"
          content="from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton

class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        self.setObjectName('MyPlugin')

        # Create widget
        self._widget = QWidget()
        layout = QVBoxLayout()

        # Add UI elements
        button = QPushButton('Click Me')
        button.clicked.connect(self.on_button_click)
        layout.addWidget(button)

        self._widget.setLayout(layout)
        context.add_widget(self._widget)

        # Create ROS 2 subscriber
        self._node = context.node
        self._sub = self._node.create_subscription(
            String, '/topic', self.callback, 10)

    def callback(self, msg):
        print(f'Received: {msg.data}')

    def on_button_click(self):
        print('Button clicked!')

    def shutdown_plugin(self):
        self._sub.destroy()
        self._node.destroy_node()"
          :copyable="true"
        />

        <div class="q-mt-md">
          <CodeBlock
            title="plugin.xml"
            lang="xml"
            content='<library path="src">
  <class name="MyPlugin" type="my_rqt_plugin.my_plugin.MyPlugin" base_class_type="rqt_gui_py.plugin.Plugin">
    <description>
      My custom rqt plugin
    </description>
    <qtgui>
      <label>My Plugin</label>
      <icon type="theme">system-help</icon>
      <statustip>My custom plugin tooltip</statustip>
    </qtgui>
  </class>
</library>'
            :copyable="true"
          />
        </div>

        <div class="q-mt-md">
          <CodeBlock
            title="package.xml"
            lang="xml"
            content='<export>
  <rqt_gui plugin="${prefix}/plugin.xml"/>
</export>'
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- PERSPECTIVES -->
    <div class="section-group">
      <SectionTitle>4. Perspectives & Layouts</SectionTitle>
      <TextBlock>
        rqt permite guardar <strong>perspectives</strong>: layouts de múltiples plugins. Útil para
        configuraciones recurrentes (debugging, teleoperation, monitoring).
      </TextBlock>

      <CodeBlock
        lang="bash"
        content="# Guardar perspective
# Perspectives -> Save Perspective -> my_debug_layout

# Cargar perspective
rqt --perspective-file my_debug_layout.perspective

# Perspective desde launch
Node(
    package='rqt_gui',
    executable='rqt_gui',
    arguments=['--perspective-file', 'my_layout.perspective']
)"
        :copyable="true"
      />
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/dQw4w9WgXcQ"
            title="rqt Plugins Tutorial"
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
          Reemplaza con video técnico sobre rqt
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>rqt_graph</code>
          <span>Computation graph visualization</span>
        </div>
        <div class="summary-item">
          <code>rqt_plot</code>
          <span>Real-time data plotting</span>
        </div>
        <div class="summary-item">
          <code>rqt_console</code>
          <span>Log viewer con filtering</span>
        </div>
        <div class="summary-item">
          <code>rqt_reconfigure</code>
          <span>Dynamic parameter tuning</span>
        </div>
        <div class="summary-item">
          <code>Custom Plugins</code>
          <span>Python/C++ extensibility</span>
        </div>
        <div class="summary-item">
          <code>Perspectives</code>
          <span>Saved layouts</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices" class="q-mt-lg">
        ✅ Usar rqt_graph para debugging de arquitectura
        <br />
        ✅ rqt_plot para tuning de controladores
        <br />
        ✅ rqt_console con severity filtering (WARN+)
        <br />
        ✅ Guardar perspectives para workflows recurrentes
        <br />
        ✅ Crear custom plugins para interfaces especializadas
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

/* PLUGIN GRID */
.plugin-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.plugin-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.plugin-card.graph {
  border-color: #14b8a6;
}

.plugin-card.plot {
  border-color: #06b6d4;
}

.plugin-card.console {
  border-color: #0ea5e9;
}

.plugin-card.reconfigure {
  border-color: #8b5cf6;
}

.plugin-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.2rem;
}

.plugin-desc {
  color: #cbd5e1;
  font-size: 0.95rem;
}

.plugin-features {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.feature-item {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  color: #5eead4;
  font-size: 0.9rem;
}

/* ADDITIONAL PLUGINS */
.additional-plugins {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1rem;
}

.add-plugin {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.add-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-weight: 700;
  color: #14b8a6;
  font-size: 1rem;
}

.add-desc {
  color: #94a3b8;
  font-size: 0.85rem;
}

/* CUSTOM DEV */
.custom-dev {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
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
  .plugin-grid {
    grid-template-columns: 1fr;
  }

  .additional-plugins {
    grid-template-columns: repeat(2, 1fr);
  }
}
</style>
