<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      <strong>Nav2 (Navigation 2)</strong> es el sucesor espiritual del Navigation Stack de ROS 1,
      pero reescrito desde cero. Es un sistema modular basado en <strong>Behavior Trees</strong> y
      <strong>Action Servers</strong>. <br /><br />
      Ya no es solo "ir de A a B". Es "ir de A a B, pero si hay un obstáculo, espera 5s, si no se
      mueve, pita, y si sigue ahí, recalculando ruta". Esto es <strong>coreografía</strong>.
    </TextBlock>

    <AlertBlock type="info" title="Arquitectura Desacoplada">
      Nav2 separa la inteligencia en tres cerebros independientes:
      <ul>
        <li><strong>Planner Server:</strong> Calcula la ruta global (GPS).</li>
        <li>
          <strong>Controller Server:</strong> Sigue la ruta esquivando obstáculos locales (Chofer).
        </li>
        <li><strong>Behavior Server:</strong> Ejecuta recuperaciones (Recovery Behaviors).</li>
      </ul>
    </AlertBlock>

    <!-- ARCHITECTURE VIZ -->
    <div class="section-group">
      <SectionTitle>1. Nav2 Architecture & Lifecycle</SectionTitle>
      <div class="arch-diagram q-mt-md">
        <div class="arch-box bt-navigator">
          <div class="box-title">BT Navigator</div>
          <div class="box-desc">Orquestador (XML)</div>
        </div>

        <div class="arch-arrows">
          <div class="arrow">⬇ Action: NavigateToPose</div>
        </div>

        <div class="server-grid">
          <div class="arch-box server planner">
            <div class="box-title">Planner Server</div>
            <div class="box-sub">Plugins: NavFn, Smac</div>
          </div>
          <div class="arch-box server controller">
            <div class="box-title">Controller Server</div>
            <div class="box-sub">Plugins: DWB, MPPI, RPP</div>
          </div>
          <div class="arch-box server recoveries">
            <div class="box-title">Behavior Server</div>
            <div class="box-sub">Plugins: Spin, Wait, BackUp</div>
          </div>
        </div>
      </div>
    </div>

    <!-- BEHAVIOR TREES INTERACTIVE -->
    <div class="section-group">
      <SectionTitle>2. Behavior Trees (Laboratorio Interactivo)</SectionTitle>
      <TextBlock>
        Un Behavior Tree es como un diagrama de flujo, pero mejor. Se ejecuta en cada "tick"
        (100Hz).
        <br />
        <strong>Regla de Oro:</strong> Los nodos devuelven SUCCESS, FAILURE o RUNNING.
      </TextBlock>

      <div class="bt-lab q-mt-md">
        <div class="lab-controls">
          <div class="control-title">Paleta de Nodos</div>
          <div class="node-palette">
            <div class="draggable-node sequence"><q-icon name="arrow_forward" /> Sequence</div>
            <div class="draggable-node fallback"><q-icon name="help_outline" /> Fallback</div>
            <div class="draggable-node action"><q-icon name="directions_run" /> Action</div>
            <div class="draggable-node condition"><q-icon name="rule" /> Condition</div>
          </div>
        </div>

        <div class="lab-canvas">
          <div class="canvas-title">Tu Diseño: "Ir a la Cocina"</div>
          <div class="tree-viz">
            <!-- Root -->
            <div class="tree-node fallback-root">
              <span class="node-label">Fallback (Recoveries)</span>
              <div class="node-connectors">
                <!-- Child 1: Main Task -->
                <div class="connector-line"></div>
                <div class="tree-node sequence">
                  <span class="node-label">Sequence (Navigate)</span>
                  <div class="child-nodes">
                    <div class="leaf action">ComputePath</div>
                    <div class="leaf action">FollowPath</div>
                  </div>
                </div>

                <!-- Child 2: Recovery -->
                <div class="connector-line"></div>
                <div class="tree-node sequence recovery">
                  <span class="node-label">Sequence (Unstuck)</span>
                  <div class="child-nodes">
                    <div class="leaf action">BackUp</div>
                    <div class="leaf action">Spin</div>
                  </div>
                </div>
              </div>
            </div>
          </div>
          <div class="simulation-feedback">
            <q-icon name="play_circle" size="md" color="green" />
            <span>Simulación: Si 'Navigate' falla, 'Unstuck' se activa automáticamente.</span>
          </div>
        </div>
      </div>

      <div class="bt-code q-mt-md">
        <CodeBlock
          title="Generated XML"
          lang="xml"
          content='<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6">
      <PipelineSequence name="Navigate">
        <ComputePathToPose/>
        <FollowPath/>
      </PipelineSequence>
      <Sequence name="Unstuck">
        <BackUp/>
        <Spin/>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>'
          :copyable="true"
        />
      </div>
    </div>

    <!-- PLANNERS & CONTROLLERS CONFIG -->
    <div class="section-group">
      <SectionTitle>3. Planners & Controllers (Plugins)</SectionTitle>
      <div class="plugins-grid q-mt-md">
        <!-- SMAC PLANNER -->
        <div class="plugin-card planner-card">
          <div class="card-header"><q-icon name="map" /> Smac Planner (Global)</div>
          <div class="card-content">
            <p><strong>Algoritmo:</strong> Hybrid-A* o State Lattice.</p>
            <p>
              <strong>Uso:</strong> Robots tipo coche (Ackermann) o industriales que no pueden girar
              en sitio.
            </p>
            <p><strong>Ventaja:</strong> Genera rutas cinemáticamente factibles (suaves).</p>
          </div>
          <CodeBlock
            lang="yaml"
            content="planner_server:
  ros__parameters:
    planner_plugins: ['GridBased']
    GridBased:
      plugin: 'nav2_smac_planner/SmacPlannerHybrid'
      downsample_costmap: false
      allow_unknown: true
      max_iterations: 1000000"
            :copyable="true"
          />
        </div>

        <!-- DWB CONTROLLER -->
        <div class="plugin-card controller-card">
          <div class="card-header"><q-icon name="gamepad" /> DWB Controller (Local)</div>
          <div class="card-content">
            <p><strong>Algoritmo:</strong> Dynamic Window Approach (DWA upgrade).</p>
            <p><strong>Uso:</strong> Robots diferenciales (Turtlebot) o omnidireccionales.</p>
            <p>
              <strong>Ventaja:</strong> Altamente configurable mediante "Critics" (cost functions).
            </p>
          </div>
          <CodeBlock
            lang="yaml"
            content="controller_server:
  ros__parameters:
    FollowPath:
      plugin: 'dwb_core::DWBLocalPlanner'
      critics: ['RotateOnGoal', 'Oscillation', 'BaseObstacle']
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0"
            :copyable="true"
          />
        </div>

        <!-- MPPI CONTROLLER -->
        <div class="plugin-card mppi-card">
          <div class="card-header"><q-icon name="speed" /> MPPI Controller (Pro)</div>
          <div class="card-content">
            <p><strong>Algoritmo:</strong> Model Predictive Path Integral.</p>
            <p><strong>Uso:</strong> Robots rápidos, drones, terrenos difíciles.</p>
            <p>
              <strong>Ventaja:</strong> Simula miles de trayectorias en paralelo (GPU/CPU-heavy).
              Predictivo y suave.
            </p>
          </div>
        </div>
      </div>
    </div>

    <!-- COSTMAP LAYERS -->
    <div class="section-group">
      <SectionTitle>4. Costmap Layers: La Visión del Mundo</SectionTitle>
      <TextBlock> El robot navega sobre una superposición de "capas de coste". </TextBlock>

      <div class="layers-viz-container q-mt-md">
        <div class="layer-block voxel">
          <div class="layer-title">Voxel Layer (3D)</div>
          <div class="layer-desc">
            Procesa nubes de puntos 3D (Depth Cam/Lidar 3D). Detecta obstáculos flotantes (mesas) o
            agujeros.
          </div>
        </div>
        <div class="layer-block obstacle">
          <div class="layer-title">Obstacle Layer (2D)</div>
          <div class="layer-desc">Scan 2D plano. Rápido y barato.</div>
        </div>
        <div class="layer-block inflation">
          <div class="layer-title">Inflation Layer</div>
          <div class="layer-desc">Añade "padding" de seguridad matemático.</div>
        </div>
        <div class="layer-block static">
          <div class="layer-title">Static Layer</div>
          <div class="layer-desc">El mapa base (paredes fijas) del SLAM.</div>
        </div>
      </div>

      <div class="voxel-config q-mt-md">
        <CodeBlock
          title="Voxel Layer Config (YAML)"
          lang="yaml"
          content="voxel_layer:
  plugin: 'nav2_costmap_2d::VoxelLayer'
  enabled: true
  footprint_clearing_enabled: true
  max_obstacle_height: 2.0
  z_resolution: 0.05
  z_voxels: 10
  origin_z: 0.0
  mark_threshold: 0
  publish_voxel_map: true"
          :copyable="true"
        />
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>Behavior Trees</code>
          <span>Lógica de navegación en XML (Sequence, Selector)</span>
        </div>
        <div class="summary-item">
          <code>Smac Planner</code>
          <span>Planificador global cinemático (Ackermann-friendly)</span>
        </div>
        <div class="summary-item">
          <code>MPPI</code>
          <span>Controlador predictivo de alto rendimiento</span>
        </div>
        <div class="summary-item">
          <code>Voxel Layer</code>
          <span>Manejo de obstáculos 3D en costmap</span>
        </div>
        <div class="summary-item">
          <code>Recovery Behaviors</code>
          <span>Acciones de emergencia (Spin, BackUp)</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices" class="q-mt-lg">
        ✅ Usa <strong>MPPI</strong> si tienes una buena GPU/CPU, es superior a DWB.
        <br />
        ✅ Ajusta <code>inflation_radius</code> mayor al radio de tu robot.
        <br />
        ✅ Para robots 3D, la <strong>Voxel Layer</strong> es obligatoria (evita chocar con mesas).
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

/* ARCHIGRAM */
.arch-diagram {
  background: rgba(15, 23, 42, 0.8);
  border-radius: 12px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1.5rem;
}

.arch-box {
  background: rgba(30, 41, 59, 0.8);
  border: 2px solid;
  border-radius: 8px;
  padding: 1rem;
  text-align: center;
  min-width: 200px;
}

.bt-navigator {
  border-color: #f59e0b;
}
.server.planner {
  border-color: #3b82f6;
}
.server.controller {
  border-color: #10b981;
}
.server.recoveries {
  border-color: #ef4444;
}

.box-title {
  font-weight: 700;
  color: #f1f5f9;
}
.box-sub {
  font-size: 0.8rem;
  color: #cbd5e1;
}

.arch-arrows {
  color: #94a3b8;
  font-family: 'Fira Code', monospace;
}

.server-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 1.5rem;
  width: 100%;
}

/* BT VISUAL */
.bt-explained {
  display: grid;
  grid-template-columns: 1fr 1.5fr;
  gap: 2rem;
}

.bt-visual {
  background: rgba(0, 0, 0, 0.3);
  border-radius: 12px;
  padding: 2rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
  position: relative;
}

.bt-node {
  padding: 0.5rem 1rem;
  border-radius: 6px;
  color: #fff;
  font-family: 'Fira Code', monospace;
  font-size: 0.8rem;
  display: inline-flex;
  align-items: center;
  gap: 0.5rem;
  position: relative;
  cursor: help;
}

.bt-node.sequence {
  background: #f59e0b;
  align-self: center;
}
.bt-node.selector {
  background: #8b5cf6;
}
.bt-node.action {
  background: #10b981;
}
.bt-node.recovery {
  background: #ef4444;
}

.tooltip {
  visibility: hidden;
  position: absolute;
  top: 100%;
  left: 50%;
  transform: translateX(-50%);
  background: #333;
  color: #fff;
  padding: 0.5rem;
  border-radius: 4px;
  font-size: 0.7rem;
  width: 200px;
  z-index: 10;
  margin-top: 5px;
}

.bt-node:hover .tooltip {
  visibility: visible;
}

.bt-branch,
.bt-branch-2 {
  display: flex;
  gap: 1rem;
  justify-content: center;
  padding-top: 1rem;
  border-top: 2px solid #555;
  margin-top: 0.5rem;
}

/* PLUGIN CARDS */
.plugins-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.plugin-card {
  background: rgba(30, 41, 59, 0.5);
  border: 1px solid rgba(255, 255, 255, 0.05);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.planner-card {
  border-top: 4px solid #3b82f6;
}
.controller-card {
  border-top: 4px solid #10b981;
}
.mppi-card {
  border-top: 4px solid #a855f7;
}

.card-header {
  font-weight: 700;
  color: #f1f5f9;
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-size: 1.1rem;
}

.card-content p {
  font-size: 0.9rem;
  color: #cbd5e1;
  margin-bottom: 0.5rem;
}

/* LAYERS VIZ */
.layers-viz-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  perspective: 1000px;
}

.layer-block {
  width: 80%;
  max-width: 400px;
  padding: 1rem;
  text-align: center;
  background: rgba(15, 23, 42, 0.9);
  border: 1px solid rgba(255, 255, 255, 0.2);
  border-radius: 8px;
  transform: rotateX(20deg);
  box-shadow: 0 10px 20px rgba(0, 0, 0, 0.5);
  transition: transform 0.3s;
}

.layer-block:hover {
  transform: rotateX(0deg) scale(1.05);
  z-index: 10;
}

.layer-block.voxel {
  background: rgba(168, 85, 247, 0.2);
  border-color: #a855f7;
}
.layer-block.obstacle {
  background: rgba(239, 68, 68, 0.2);
  border-color: #ef4444;
}
.layer-block.inflation {
  background: rgba(59, 130, 246, 0.2);
  border-color: #3b82f6;
}
.layer-block.static {
  background: rgba(148, 163, 184, 0.2);
  border-color: #94a3b8;
}

.layer-title {
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.25rem;
}
.layer-desc {
  font-size: 0.8rem;
  color: #cbd5e1;
}

.voxel-config {
  width: 100%;
  max-width: 800px;
  margin: 2rem auto 0;
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
  color: #6ee7b7;
  font-family: 'Fira Code', monospace;
  font-size: 0.95rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

/* RESPONSIVE */
@media (max-width: 1024px) {
  .bt-explained {
    grid-template-columns: 1fr;
  }

  .server-grid {
    grid-template-columns: 1fr;
  }

  .plugins-grid {
    grid-template-columns: 1fr;
  }
}
</style>
