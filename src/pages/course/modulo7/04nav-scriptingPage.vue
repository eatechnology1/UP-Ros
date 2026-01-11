<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      Usar `Nav2` desde RViz está bien para demos, pero en producción el robot navega solo. La API
      <strong>Nav2 Simple Commander</strong> (Python) es la interfaz principal para ordenar tareas:
      ir a pose, seguir waypoints, o cancelar misiones.
    </TextBlock>

    <AlertBlock type="info" title="Action Clients Under the Hood">
      Cuando llamas a <code>goToPose()</code>, en realidad estás enviando una meta al Action Server
      <code>/navigate_to_pose</code>. El script actúa como cliente, esperando feedback (progreso) y
      resultado (éxito/fallo).
    </AlertBlock>

    <!-- BASIC API & LIFECYCLE -->
    <div class="section-group">
      <SectionTitle>1. Nav2 Lifecycle Management</SectionTitle>
      <TextBlock>
        Nav2 usa <strong>Lifecycle Nodes</strong> (Managed Nodes). No arrancan solos, deben
        transicionar: Unconfigured → Inactive → Active.
      </TextBlock>

      <div class="lifecycle-viz q-mt-md">
        <div class="state unconfigured">
          <div class="state-dot"></div>
          Unconfigured
          <div class="state-sub">Carga params</div>
        </div>
        <div class="arrow">⬇ configure()</div>
        <div class="state inactive">
          <div class="state-dot"></div>
          Inactive
          <div class="state-sub">Configura topics (pausa)</div>
        </div>
        <div class="arrow">⬇ activate()</div>
        <div class="state active">
          <div class="state-dot"></div>
          Active
          <div class="state-sub">Processing Data</div>
        </div>

        <div class="lifecycle-code">
          <CodeBlock
            title="Startup Script"
            lang="python"
            content="from nav2_simple_commander.robot_navigator import BasicNavigator

navigator = BasicNavigator()

# Espera a que Nav2 esté totalmente activo
navigator.waitUntilNav2Active(localizer='amcl')

# ... ahora es seguro enviar comandos"
            :copyable="true"
          />
        </div>
      </div>
    </div>

    <!-- ADVANCED TASK EXECUTION -->
    <div class="section-group">
      <SectionTitle>2. Advanced Task Execution</SectionTitle>

      <div class="task-grid q-mt-md">
        <!-- SINGLE GOAL -->
        <div class="task-card single">
          <div class="task-header"><q-icon name="flag" /> Go To Pose</div>
          <div class="task-desc">Navegación punto a punto con evitación de obstáculos.</div>
          <CodeBlock
            lang="python"
            content="goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 2.5
goal_pose.pose.orientation.w = 1.0

navigator.goToPose(goal_pose)

while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    print(f'Distance: {feedback.distance_remaining:.2f}m')"
            :copyable="true"
          />
        </div>

        <!-- WAYPOINTS -->
        <div class="task-card multi">
          <div class="task-header"><q-icon name="timeline" /> Follow Waypoints</div>
          <div class="task-desc">Ruta compleja pasando por puntos intermedios (inspección).</div>
          <CodeBlock
            lang="python"
            content="waypoints = [pose1, pose2, pose3, pose4]

# Envía la lista completa
navigator.followWaypoints(waypoints)

# Monitorea índice actual
i = navigator.getFeedback().current_waypoint
print(f'Executing Waypoint {i+1}/{len(waypoints)}')"
            :copyable="true"
          />
        </div>

        <!-- KEEPOUT ZONES -->
        <div class="task-card zones">
          <div class="task-header"><q-icon name="block" /> Keepout Zones</div>
          <div class="task-desc">Usar filtros de costmap para vetar áreas dinámicamente.</div>
          <div class="viz-mini-map">
            <div class="map-area"></div>
            <div class="keepout-zone">NO GO</div>
          </div>
        </div>
      </div>
    </div>

    <!-- FEEDBACK HANDLING & RECOVERY -->
    <div class="section-group">
      <SectionTitle>3. Handling Failures & Recovery</SectionTitle>
      <TextBlock>
        En el mundo real, los robots fallan. Quedan atrapados, pierden localización... Tu script
        debe manejar estos casos.
      </TextBlock>

      <CodeBlock
        title="Robust Navigation Loop"
        lang="python"
        content="result = navigator.getResult()
if result == TaskResult.SUCCEEDED:
    print('Goal reached!')
elif result == TaskResult.CANCELED:
    print('Task was canceled')
elif result == TaskResult.FAILED:
    print('Task failed!')
    # Custom Recovery Strategy
    navigator.backup(backup_dist=0.5, backup_speed=0.05)
    navigator.spin(spin_dist=3.14)"
        :copyable="true"
      />
    </div>

    <!-- HYBRID NAV -->
    <div class="section-group">
      <SectionTitle>4. Hybrid Navigation (GPS -> SLAM)</SectionTitle>
      <TextBlock>
        Robots de exteriores usan GPS para acercarse al edificio, y cambian a SLAM (Lidar) al
        entrar. Esto requiere cambiar el mapa y el localizador al vuelo.
      </TextBlock>

      <div class="hybrid-viz q-mt-md">
        <div class="mode gps">
          <q-icon name="satellite" size="lg" />
          <span>GPS Navigation</span>
          <div class="tech">NavSatFix -> EKF -> Odometry</div>
        </div>
        <div class="transition-arrow">
          <q-icon name="sync_alt" />
          <span>Handover</span>
        </div>
        <div class="mode slam">
          <q-icon name="grid_view" size="lg" />
          <span>Lidar SLAM</span>
          <div class="tech">AMCL -> Map Server</div>
        </div>
      </div>
    </div>

    <!-- DIDACTIC FLEET MANAGEMENT -->
    <div class="section-group">
      <SectionTitle>5. Mini-Proyecto: Fleet Coordinator</SectionTitle>

      <div class="challenge-card">
        <div class="challenge-title">🕵️ Misión: Flota de Patrulla</div>
        <div class="challenge-desc">
          Tienes 3 robots (Robot_A, Robot_B, Robot_C). Deben patrullar 3 pasillos diferentes sin
          chocar.
          <br />
          Completa el script del <strong>FleetManager</strong>.
        </div>

        <CodeBlock
          title="fleet_manager.py (Incompleto)"
          lang="python"
          content="class FleetManager:
    def __init__(self):
        self.robots = {
            'waffle_1': BasicNavigator(namespace='robot1'),
            'waffle_2': BasicNavigator(namespace='robot2')
        }

    def dispatch_patrol(self):
        # TODO: Asigna targets diferentes para evitar colisiones
        # Pista: Usa 'navigator.goToPose()' en loop no bloqueante

        target_A = self.get_pose(x=5.0, y=0.0)
        target_B = self.get_pose(x=0.0, y=5.0)

        # 1. Enviar órdenes (Async)
        self.robots['waffle_1'].goToPose(target_A)
        self.robots['waffle_2'].goToPose(target_B)

        # 2. Monitor Loop
        while not self.all_idle():
            # Completa lógica de chequeo...
            pass"
          :copyable="true"
        />
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>BasicNavigator</code>
          <span>Clase Python helper para Nav2 Action Clients</span>
        </div>
        <div class="summary-item">
          <code>LifecycleNode</code>
          <span>Nodos gestionados (Configure -> Activate)</span>
        </div>
        <div class="summary-item">
          <code>TaskResult</code>
          <span>Enum de resultados (SUCCEEDED, FAILED, CANCELED)</span>
        </div>
        <div class="summary-item">
          <code>followWaypoints</code>
          <span>Ejecutar secuencia de poses</span>
        </div>
        <div class="summary-item">
          <code>KeepoutFilter</code>
          <span>Costmap layer para zonas prohibidas virtuales</span>
        </div>
      </div>

      <AlertBlock type="success" title="Best Practices" class="q-mt-lg">
        ✅ Siempre implementa un timeout en tus bucles <code>while/wait</code>.
        <br />
        ✅ Usa <code>navigator.lifecycleStartup()</code> si quieres que tu script arranque Nav2.
        <br />
        ✅ Monitorea <code>distance_remaining</code> para decelarar o activar actuadores antes de
        llegar.
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

/* LIFECYCLE VIZ */
.lifecycle-viz {
  display: grid;
  grid-template-columns: 1fr 2fr;
  gap: 2rem;
  background: rgba(15, 23, 42, 0.8);
  border-radius: 12px;
  padding: 2rem;
  align-items: center;
}

.state {
  display: flex;
  flex-direction: column;
  padding: 0.5rem 1rem;
  background: rgba(30, 41, 59, 0.8);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 8px;
  position: relative;
  width: 150px;
}

.state.active {
  border-color: #22c55e;
}
.state.inactive {
  border-color: #f59e0b;
}
.state.unconfigured {
  border-color: #94a3b8;
}

.state-dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  position: absolute;
  top: 10px;
  right: 10px;
}

.active .state-dot {
  background: #22c55e;
  box-shadow: 0 0 10px #22c55e;
}
.inactive .state-dot {
  background: #f59e0b;
}
.unconfigured .state-dot {
  background: #94a3b8;
}

.state-sub {
  font-size: 0.7rem;
  color: #94a3b8;
  margin-top: 0.25rem;
}

.arrow {
  text-align: center;
  color: #64748b;
  font-size: 0.8rem;
  font-family: 'Fira Code', monospace;
  margin: 0.5rem 0;
}

/* TASK GRID */
.task-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1.5rem;
}

.task-card {
  background: rgba(30, 41, 59, 0.5);
  border: 1px solid rgba(255, 255, 255, 0.05);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.task-card.single {
  border-top: 4px solid #3b82f6;
}
.task-card.multi {
  border-top: 4px solid #a855f7;
}
.task-card.zones {
  border-top: 4px solid #ef4444;
}

.task-header {
  font-weight: 700;
  color: #f1f5f9;
  font-size: 1.1rem;
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.task-desc {
  font-size: 0.9rem;
  color: #cbd5e1;
}

.viz-mini-map {
  height: 100px;
  background: #0f172a;
  border-radius: 8px;
  position: relative;
  display: flex;
  align-items: center;
  justify-content: center;
}

.keepout-zone {
  width: 60px;
  height: 60px;
  background: repeating-linear-gradient(
    45deg,
    rgba(239, 68, 68, 0.2),
    rgba(239, 68, 68, 0.2) 10px,
    rgba(239, 68, 68, 0.4) 10px,
    rgba(239, 68, 68, 0.4) 20px
  );
  border: 2px solid #ef4444;
  display: flex;
  align-items: center;
  justify-content: center;
  color: #ef4444;
  font-weight: 900;
  font-size: 0.8rem;
}

/* HYBRID VIZ */
.hybrid-viz {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 3rem;
  background: linear-gradient(90deg, rgba(15, 23, 42, 0.6) 0%, rgba(30, 41, 59, 0.6) 100%);
  padding: 2rem;
  border-radius: 12px;
}

.mode {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  color: #f1f5f9;
}

.mode span {
  font-weight: 700;
}
.mode .tech {
  font-size: 0.75rem;
  color: #94a3b8;
  background: rgba(0, 0, 0, 0.3);
  padding: 2px 6px;
  border-radius: 4px;
}

.transition-arrow {
  display: flex;
  flex-direction: column;
  align-items: center;
  color: #eab308;
}

.transition-arrow span {
  font-size: 0.7rem;
  font-weight: 700;
  margin-top: 5px;
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
  .lifecycle-viz {
    grid-template-columns: 1fr;
  }

  .task-grid {
    grid-template-columns: 1fr;
  }

  .hybrid-viz {
    flex-direction: column;
    gap: 2rem;
  }

  .transition-arrow {
    transform: rotate(90deg);
  }
}
</style>
