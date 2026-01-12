<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      <strong>RViz2</strong> es el visualizador 3D estándar de ROS 2. Permite renderizar datos de
      sensores (PointCloud2, LaserScan, Image), TF trees, markers, y modelos URDF en tiempo real.
      Esencial para debugging, teleoperation, y demostración de sistemas robóticos.
    </TextBlock>

    <AlertBlock type="info" title="Capacidades Clave">
      <strong>3D Rendering:</strong> PointClouds, meshes, markers con OpenGL
      <br />
      <strong>Sensor Data:</strong> Camera, Lidar, depth, IMU visualization
      <br />
      <strong>TF Visualization:</strong> Transform tree con frames interactivos
      <br />
      <strong>Interactive Markers:</strong> Teleop, goal setting, manipulation
    </AlertBlock>

    <!-- DISPLAY TYPES -->
    <div class="section-group">
      <SectionTitle>1. Display Types Exhaustivo</SectionTitle>
      <TextBlock>
        RViz2 organiza visualizaciones en <strong>Displays</strong>. Cada display subscribe a un
        topic y renderiza datos según su tipo.
      </TextBlock>

      <div class="display-grid q-mt-md">
        <div class="display-card sensor">
          <div class="display-header">
            <q-icon name="sensors" />
            <span>Sensor Displays</span>
          </div>
          <div class="display-list">
            <div class="display-item">
              <strong>PointCloud2:</strong> Nubes de puntos 3D (Lidar, depth camera)
            </div>
            <div class="display-item"><strong>LaserScan:</strong> Scans 2D de Lidar planar</div>
            <div class="display-item"><strong>Image:</strong> Imágenes RGB/depth/compressed</div>
            <div class="display-item">
              <strong>Camera:</strong> Frustum de cámara con imagen proyectada
            </div>
          </div>
          <CodeBlock
            lang="yaml"
            content="# PointCloud2 con color mapping
- Class: rviz_default_plugins/PointCloud2
  Topic: /velodyne_points
  Size: 0.03
  Color Transformer: Intensity
  Min Intensity: 0
  Max Intensity: 4096"
            :copyable="true"
          />
        </div>

        <div class="display-card robot">
          <div class="display-header">
            <q-icon name="smart_toy" />
            <span>Robot Model</span>
          </div>
          <div class="display-list">
            <div class="display-item"><strong>RobotModel:</strong> Renderiza URDF con texturas</div>
            <div class="display-item"><strong>TF:</strong> Visualiza transform tree con ejes</div>
            <div class="display-item">
              <strong>Axes:</strong> Ejes de referencia (X=red, Y=green, Z=blue)
            </div>
          </div>
          <CodeBlock
            lang="yaml"
            content="# Robot Model
- Class: rviz_default_plugins/RobotModel
  Description Topic: /robot_description
  Visual Enabled: true
  Collision Enabled: false
  Alpha: 1.0"
            :copyable="true"
          />
        </div>

        <div class="display-card nav">
          <div class="display-header">
            <q-icon name="navigation" />
            <span>Navigation</span>
          </div>
          <div class="display-list">
            <div class="display-item"><strong>Map:</strong> Occupancy grid (SLAM, cost maps)</div>
            <div class="display-item"><strong>Path:</strong> Trayectorias planificadas</div>
            <div class="display-item"><strong>Odometry:</strong> Pose history con covariance</div>
            <div class="display-item"><strong>Polygon:</strong> Footprint del robot</div>
          </div>
          <CodeBlock
            lang="yaml"
            content="# Map display
- Class: rviz_default_plugins/Map
  Topic: /map
  Color Scheme: map
  Alpha: 0.7
  Draw Behind: true"
            :copyable="true"
          />
        </div>

        <div class="display-card markers">
          <div class="display-header">
            <q-icon name="place" />
            <span>Markers</span>
          </div>
          <div class="display-list">
            <div class="display-item">
              <strong>Marker:</strong> Primitivas 3D (sphere, cube, arrow, text)
            </div>
            <div class="display-item"><strong>MarkerArray:</strong> Múltiples markers con IDs</div>
            <div class="display-item">
              <strong>InteractiveMarker:</strong> Markers con drag & drop
            </div>
          </div>
          <CodeBlock
            lang="cpp"
            content='// Publicar marker
auto marker = visualization_msgs::msg::Marker();
marker.header.frame_id = "map";
marker.type = Marker::SPHERE;
marker.action = Marker::ADD;
marker.pose.position.x = 1.0;
marker.scale.x = 0.5;
marker.color.r = 1.0;
marker.color.a = 1.0;
marker_pub->publish(marker);'
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- CONFIGURATION -->
    <div class="section-group">
      <SectionTitle>2. Configuration Management</SectionTitle>
      <TextBlock>
        Las configuraciones de RViz2 se guardan en archivos <code>.rviz</code> (YAML). Permiten
        reproducir layouts, displays, y viewpoints.
      </TextBlock>

      <div class="config-structure q-mt-md">
        <div class="config-section">
          <div class="config-title">Estructura .rviz</div>
          <CodeBlock
            lang="yaml"
            content="Panels:
  - Class: rviz_common/Displays
  - Class: rviz_common/Views
  - Class: rviz_common/Time

Visualization Manager:
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30

  Displays:
    - Class: rviz_default_plugins/Grid
      Cell Size: 1.0
      Color: 160; 160; 164

    - Class: rviz_default_plugins/RobotModel
      Description Topic: /robot_description

    - Class: rviz_default_plugins/TF
      Show Names: true
      Show Axes: true

Views:
  Current:
    Class: rviz_default_plugins/Orbit
    Distance: 10.0
    Focal Point:
      X: 0
      Y: 0
      Z: 0"
            :copyable="true"
          />
        </div>

        <div class="config-section">
          <div class="config-title">Cargar configuración</div>
          <CodeBlock
            lang="bash"
            content="# Desde línea de comandos
rviz2 -d /path/to/config.rviz

# Desde launch file
Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', '/path/to/config.rviz']
)"
            :copyable="true"
          />
        </div>

        <div class="config-section">
          <div class="config-title">Programmatic Configuration</div>
          <CodeBlock
            lang="python"
            content="# Generar .rviz desde Python
import yaml

config = {
    'Visualization Manager': {
        'Global Options': {
            'Fixed Frame': 'map',
            'Frame Rate': 30
        },
        'Displays': [
            {
                'Class': 'rviz_default_plugins/Grid',
                'Cell Size': 1.0
            }
        ]
    }
}

with open('config.rviz', 'w') as f:
    yaml.dump(config, f)"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- INTERACTIVE MARKERS -->
    <div class="section-group">
      <SectionTitle>3. Interactive Markers</SectionTitle>
      <TextBlock>
        Los <strong>Interactive Markers</strong> permiten manipular objetos 3D con el mouse. Útiles
        para teleop, goal setting, y ajuste de parámetros visuales.
      </TextBlock>

      <div class="interactive-example q-mt-md">
        <CodeBlock
          title="Crear Interactive Marker (C++)"
          lang="cpp"
          content='#include <interactive_markers/interactive_marker_server.hpp>

auto server = std::make_shared<InteractiveMarkerServer>(
  "simple_marker", node);

// Crear marker
InteractiveMarker int_marker;
int_marker.header.frame_id = "map";
int_marker.name = "goal_pose";
int_marker.description = "Set Goal";

// Control de 6-DOF
InteractiveMarkerControl control;
control.orientation.w = 1;
control.orientation.x = 1;
control.orientation.y = 0;
control.orientation.z = 0;
control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
int_marker.controls.push_back(control);

// Callback de feedback
auto feedback_cb = [](const auto& feedback) {
  RCLCPP_INFO(node->get_logger(),
    "Marker at: %.2f, %.2f, %.2f",
    feedback->pose.position.x,
    feedback->pose.position.y,
    feedback->pose.position.z);
};

server->insert(int_marker, feedback_cb);
server->applyChanges();'
          :copyable="true"
        />
      </div>
    </div>

    <!-- PERFORMANCE -->
    <div class="section-group">
      <SectionTitle>4. Performance Optimization</SectionTitle>

      <div class="perf-tips q-mt-md">
        <div class="perf-card">
          <div class="perf-header">
            <q-icon name="speed" color="green-4" />
            <span>PointCloud Optimization</span>
          </div>
          <div class="perf-list">
            <div class="perf-item">
              <strong>Decimation:</strong> Reducir puntos (1/10 = 10x speedup)
            </div>
            <div class="perf-item">
              <strong>Style:</strong> Usar "Flat Squares" en lugar de "Spheres"
            </div>
            <div class="perf-item">
              <strong>Size:</strong> Reducir tamaño de punto (0.01 → 0.03)
            </div>
          </div>
          <CodeBlock
            lang="yaml"
            content="# PointCloud optimizado
- Class: rviz_default_plugins/PointCloud2
  Topic: /velodyne_points
  Size: 0.01
  Style: Flat Squares
  Selectable: false
  Use Fixed Frame: true"
            :copyable="true"
          />
        </div>

        <div class="perf-card">
          <div class="perf-header">
            <q-icon name="visibility_off" color="orange-4" />
            <span>Culling & LOD</span>
          </div>
          <div class="perf-list">
            <div class="perf-item">
              <strong>Frustum Culling:</strong> No renderizar objetos fuera de vista
            </div>
            <div class="perf-item"><strong>Distance Culling:</strong> Ocultar objetos lejanos</div>
            <div class="perf-item"><strong>LOD:</strong> Reducir detalle con distancia</div>
          </div>
        </div>

        <div class="perf-card">
          <div class="perf-header">
            <q-icon name="settings" color="blue-4" />
            <span>Frame Rate Control</span>
          </div>
          <div class="perf-list">
            <div class="perf-item">
              <strong>Target FPS:</strong> 30 Hz para visualización fluida
            </div>
            <div class="perf-item">
              <strong>Update Rate:</strong> Limitar updates de displays pesados
            </div>
          </div>
          <CodeBlock
            lang="yaml"
            content="# Global Options
Global Options:
  Frame Rate: 30  # Target 30 FPS"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- PLUGINS -->
    <div class="section-group">
      <SectionTitle>5. Custom Plugins</SectionTitle>
      <TextBlock>
        RViz2 permite crear displays custom mediante plugins. Útil para visualizaciones
        especializadas no cubiertas por displays estándar.
      </TextBlock>

      <CodeBlock
        title="Plugin skeleton (C++)"
        lang="cpp"
        content='#include <rviz_common/display.hpp>

class MyCustomDisplay : public rviz_common::Display {
public:
  MyCustomDisplay() {
    // Constructor
  }

  void onInitialize() override {
    // Setup: crear subscribers, publishers
    subscription_ = node_->create_subscription<MyMsg>(
      "/my_topic", 10,
      std::bind(&MyCustomDisplay::processMessage, this, _1));
  }

  void update(float wall_dt, float ros_dt) override {
    // Llamado cada frame para renderizar
  }

private:
  void processMessage(const MyMsg::SharedPtr msg) {
    // Procesar mensaje y actualizar visualización
  }

  rclcpp::Subscription<MyMsg>::SharedPtr subscription_;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MyCustomDisplay, rviz_common::Display)'
        :copyable="true"
      />
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://www.youtube.com/embed/Romc22GgusU"
            title="RViz2 Advanced Features"
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
          Reemplaza con video técnico sobre RViz2
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>Display Types</code>
          <span>PointCloud2, LaserScan, Image, TF, Markers</span>
        </div>
        <div class="summary-item">
          <code>.rviz Config</code>
          <span>YAML configuration files</span>
        </div>
        <div class="summary-item">
          <code>Interactive Markers</code>
          <span>6-DOF manipulation</span>
        </div>
        <div class="summary-item">
          <code>Performance</code>
          <span>Decimation, culling, LOD</span>
        </div>
        <div class="summary-item">
          <code>Custom Plugins</code>
          <span>Extend con C++</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices" class="q-mt-lg">
        ✅ Guardar configuraciones .rviz para reproducibilidad
        <br />
        ✅ Usar Flat Squares para PointClouds (mejor performance)
        <br />
        ✅ Limitar frame rate a 30 Hz para visualización fluida
        <br />
        ✅ Deshabilitar displays no usados para ahorrar CPU
        <br />
        ✅ Usar Interactive Markers para teleop y goal setting
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

/* DISPLAY GRID */
.display-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
}

.display-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid;
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.display-card.sensor {
  border-color: #14b8a6;
}

.display-card.robot {
  border-color: #06b6d4;
}

.display-card.nav {
  border-color: #0ea5e9;
}

.display-card.markers {
  border-color: #8b5cf6;
}

.display-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
}

.display-list {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.display-item {
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  color: #cbd5e1;
  font-size: 0.9rem;
}

.display-item strong {
  color: #5eead4;
}

/* CONFIG STRUCTURE */
.config-structure {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.config-section {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.config-title {
  font-weight: 700;
  color: #14b8a6;
  font-size: 1.1rem;
  margin-bottom: 1rem;
}

/* INTERACTIVE EXAMPLE */
.interactive-example {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

/* PERFORMANCE */
.perf-tips {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.perf-card {
  background: rgba(15, 23, 42, 0.8);
  border: 2px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.perf-header {
  display: flex;
  align-items: center;
  gap: 1rem;
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
}

.perf-list {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.perf-item {
  padding: 0.75rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 6px;
  color: #cbd5e1;
  font-size: 0.9rem;
}

.perf-item strong {
  color: #5eead4;
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
  .display-grid {
    grid-template-columns: 1fr;
  }
}
</style>
