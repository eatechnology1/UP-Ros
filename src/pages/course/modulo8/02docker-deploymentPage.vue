<template>
  <LessonContainer>
    <!-- HERO INTRO -->
    <TextBlock>
      Tu robot funciona en tu laptop ("Works on my machine"), pero falla en el campo.
      <strong>Docker</strong> soluciona esto empaquetando todo el OS y dependencias en un "Tupper"
      digital inmutable. <br /><br />
      En producción, no instalas ROS, despliegas contenedores. Y <strong>Kubernetes</strong> es el
      refrigerador inteligente que gestiona esos Tuppers.
    </TextBlock>

    <!-- DIDACTIC ANALOGY -->
    <div class="section-group">
      <SectionTitle>1. Concepto: El Contenedor</SectionTitle>

      <div class="container-viz q-mt-md">
        <div class="system traditional">
          <div class="sys-header">Traditional System</div>
          <div class="layer app">App (V1.2)</div>
          <div class="layer lib">Lib (V3.0)</div>
          <div class="layer lib-conflict">Lib (V2.0) 🔥 CONFLICT</div>
          <div class="layer os">Host OS (Updates breaks libs)</div>
        </div>

        <div class="arrow">vs</div>

        <div class="system docker">
          <div class="sys-header">Containerized (Docker)</div>
          <div class="container-box">
            <div class="layer app">App & Libs Isolated</div>
            <div class="layer os">Binary Compatible OS</div>
          </div>
          <div class="layer kernel">Generic Linux Kernel (Shared)</div>
        </div>
      </div>
    </div>

    <!-- MULTI-STAGE BUILDS -->
    <div class="section-group">
      <SectionTitle>2. Ingeniería de Imagen: Multi-Stage Build</SectionTitle>
      <TextBlock>
        Para un robot comercial, no puedes enviar una imagen de 3GB con compiladores (GCC) y código
        fuente. Usamos <strong>Multi-Stage Builds</strong> para copiar solo los binarios compilados.
      </TextBlock>

      <div class="code-implementation q-mt-md">
        <CodeBlock
          title="Dockerfile.production"
          lang="dockerfile"
          content="# STAGE 1: Build & Compile
FROM ros:humble-ros-base as builder

# Instalar dependencias de compilación
RUN apt-get update && apt-get install -y build-essential git

# Copiar código fuente
WORKDIR /app_ws
COPY src ./src

# Compilar release optimizada
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# STAGE 2: Runtime (Slim)
FROM ros:humble-ros-core

# Copiar solo los artifacts instalados desde 'builder'
COPY --from=builder /app_ws/install /opt/my_robot

# Setup entrypoint
COPY ros_entrypoint.sh /
ENTRYPOINT ['/ros_entrypoint.sh']
CMD ['ros2', 'launch', 'my_robot', 'main.launch.py']"
          :copyable="true"
        />
        <div class="results-stats">
          <div class="stat">
            <span class="label">Standard Image</span>
            <span class="val bad">~2.4 GB</span>
          </div>
          <div class="stat">
            <span class="label">Multi-Stage</span>
            <span class="val good">~450 MB</span>
          </div>
        </div>
      </div>
    </div>

    <!-- NETWORKING CHALLENGE (DOCTORAL) -->
    <div class="section-group">
      <SectionTitle>3. El Problema de Red DDS</SectionTitle>
      <AlertBlock type="warning" title="DDS vs Docker Bridge">
        Por defecto, Docker aísla la red en un "Bridge" virtual. ROS 2 (DDS) usa Multicast para
        descubrimiento. <strong>El Multicast no atraviesa el Bridge por defecto.</strong>
      </AlertBlock>

      <div class="network-viz q-mt-md">
        <div class="net-mode bridge">
          <q-icon name="router" size="md" />
          <div class="net-title">Bridge Mode (Default)</div>
          <div class="net-desc">
            Aislamiento total. Requiere mapear puertos UDP masivamente o configurar
            <code>CycloneDDS XML</code> complejo. 🔴 Difícil para ROS.
          </div>
        </div>
        <div class="net-mode host">
          <q-icon name="lan" size="md" />
          <div class="net-title">Host Mode (--net=host)</div>
          <div class="net-desc">
            El contenedor comparte la IP del host. Multicast fluye libremente. El robot "ve" toda la
            red. 🟢 Estándar en robótica.
          </div>
        </div>
      </div>

      <CodeBlock
        title="Run Command (Host Mode)"
        lang="bash"
        content="docker run -it --net=host --privileged my_robot:latest"
        :copyable="true"
        class="q-mt-sm"
      />
    </div>

    <!-- ORCHESTRATION (K8S) -->
    <div class="section-group">
      <SectionTitle>4. Orchestration: Kubernetes (K8s) para Flotas</SectionTitle>
      <TextBlock>
        Cuando tienes 50 robots en un almacén, no haces `docker run` en cada uno manualmente. Usas
        Kubernetes o K3s (versión ligera para Edge/Robots).
      </TextBlock>

      <div class="k8s-viz">
        <div class="k8s-master">
          <div class="role-title">Control Plane</div>
          <div class="role-desc">Programa actualizaciones y monitorea salud.</div>
        </div>
        <div class="k8s-arrows">➜ Wi-Fi / 5G ➜</div>
        <div class="robot-fleet">
          <div class="robot-node">
            <q-icon name="smart_toy" /> Robot 1
            <span class="status healthy">Running V2.0</span>
          </div>
          <div class="robot-node">
            <q-icon name="smart_toy" /> Robot 2
            <span class="status updating">Pulling V2.1...</span>
          </div>
          <div class="robot-node">
            <q-icon name="smart_toy" /> Robot 3
            <span class="status error">CrashLoopBackOff</span>
          </div>
        </div>
      </div>
    </div>

    <!-- DOCTORAL CHALLENGE -->
    <div class="section-group">
      <SectionTitle>5. Doctor''s Challenge: Networking Black Hole</SectionTitle>

      <div class="challenge-card">
        <div class="challenge-title">🕳️ El Agujero Negro de Tópicos</div>
        <div class="challenge-desc">
          Lanzas tus nodos en contenedores Docker diferentes en la misma laptop. Usas
          <code>--net=host</code>. Sin embargo, <code>ros2 topic list</code> ve los tópicos, pero
          <code>ros2 topic echo</code> NO recibe datos.
          <br />
          ¿Qué está pasando con la Shared Memory?
        </div>

        <div class="solution-spoiler">
          <q-expansion-item
            icon="quiz"
            label="Ver Respuesta de Ingeniería"
            header-class="text-primary"
            style="background: rgba(0, 0, 0, 0.2); border-radius: 8px"
          >
            <div class="q-pa-md text-grey-4">
              <p>
                <strong>Causa:</strong> ROS 2 usa <strong>Shared Memory (Iceoryx)</strong> para
                optimizar procesos locales. Docker aísla la memoria compartida (`/dev/shm`) por
                defecto.
              </p>
              <p>
                Los nodos negocian usar Shared Memory, intentan escribir, pero fallan
                silenciosamente o escriben en segmentos de memoria aislados que el otro contenedor
                no ve.
              </p>
              <p>
                <strong>Solución:</strong> Montar el volumen de memoria compartida:
                <code>-v /dev/shm:/dev/shm</code> al correr los contenedores.
              </p>
            </div>
          </q-expansion-item>
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen Técnico</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>Multi-Stage</code>
          <span>Técnica para eliminar build tools de la imagen final</span>
        </div>
        <div class="summary-item">
          <code>--net=host</code>
          <span>Comparte stack de red para permitir DDS Multicast</span>
        </div>
        <div class="summary-item">
          <code>/dev/shm</code>
          <span>Volumen crítico para Zero-Copy entre contenedores</span>
        </div>
        <div class="summary-item">
          <code>K3s</code>
          <span>Kubernetes ligero optimizado para IoT/Robótica</span>
        </div>
      </div>
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

/* CONTAINER VIZ */
.container-viz {
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 2rem;
  background: var(--bg-surface);
  padding: 2rem;
  border-radius: 12px;
}

.system {
  display: flex;
  flex-direction: column;
  width: 200px;
}

.sys-header {
  text-align: center;
  font-weight: 700;
  color: var(--text-muted);
  margin-bottom: 0.5rem;
  font-size: 0.9rem;
}

.layer {
  padding: 0.5rem;
  text-align: center;
  font-size: 0.8rem;
  border: 1px solid rgba(0, 0, 0, 0.2);
  color: var(--text-primary);
}

.traditional .app {
  background: #3b82f6;
  border-radius: 8px 8px 0 0;
}
.traditional .lib {
  background: #60a5fa;
}
.traditional .lib-conflict {
  background: #ef4444;
}
.traditional .os {
  background: var(--bg-surface-solid);
  border-radius: 0 0 8px 8px;
  margin-top: 2px;
}

.docker .container-box {
  border: 2px dashed #3b82f6;
  padding: 5px;
  border-radius: 8px;
  margin-bottom: 5px;
}
.docker .app {
  background: #3b82f6;
  border-radius: 4px;
  margin-bottom: 2px;
}
.docker .os {
  background: var(--bg-surface-hover);
  color: var(--text-secondary);
  border-radius: 4px;
  font-size: 0.75rem;
}
.docker .kernel {
  background: var(--bg-surface-solid);
  border-radius: 8px;
}

.arrow {
  font-weight: 700;
  color: #64748b;
  font-size: 1.2rem;
}

/* RESULTS STATS */
.results-stats {
  display: flex;
  gap: 2rem;
  margin-top: 1rem;
  background: var(--bg-surface-hover);
  padding: 1rem;
  border-radius: 8px;
  justify-content: center;
}

.stat {
  display: flex;
  flex-direction: column;
  align-items: center;
}
.stat .label {
  font-size: 0.8rem;
  color: var(--text-muted);
}
.stat .val {
  font-size: 1.2rem;
  font-weight: 700;
  font-family: monospace;
}
.stat .val.bad {
  color: #ef4444;
}
.stat .val.good {
  color: var(--text-code);
}

/* NETWORK VIZ */
.network-viz {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1.5rem;
}

.net-mode {
  background: var(--bg-surface-solid);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  gap: 0.5rem;
}

.net-mode.bridge {
  border-top: 4px solid #ef4444;
  color: var(--text-danger, #dc2626);
}
.net-mode.host {
  border-top: 4px solid #22c55e;
  color: var(--text-code);
}

.net-title {
  font-weight: 700;
  font-size: 1.1rem;
  color: var(--text-primary);
}
.net-desc {
  font-size: 0.85rem;
  color: var(--text-secondary);
}

/* K8S VIZ */
.k8s-viz {
  background: var(--bg-surface);
  padding: 2rem;
  border-radius: 12px;
  display: flex;
  align-items: center;
  justify-content: space-between;
}

.k8s-master {
  background: #3b82f6;
  padding: 1rem;
  border-radius: 8px;
  width: 150px;
  text-align: center;
}
.role-title {
  font-weight: 700;
  color: var(--text-primary);
}
.role-desc {
  font-size: 0.75rem;
  color: #dbeafe;
}

.k8s-arrows {
  color: #64748b;
  font-family: monospace;
  font-weight: 700;
}

.robot-fleet {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.robot-node {
  background: var(--bg-surface-hover);
  padding: 0.5rem 1rem;
  border-radius: 6px;
  display: flex;
  align-items: center;
  gap: 1rem;
  border: 1px solid #334155;
  color: var(--text-primary);
}

.status {
  font-size: 0.75rem;
  padding: 2px 6px;
  border-radius: 4px;
  font-weight: 700;
}
.status.healthy {
  background: rgba(34, 197, 94, 0.2);
  color: var(--text-code);
}
.status.updating {
  background: rgba(234, 179, 8, 0.2);
  color: #eab308;
}
.status.error {
  background: rgba(239, 68, 68, 0.2);
  color: #ef4444;
}

/* CHALLENGE */
.challenge-card {
  background: var(--bg-surface-solid);
  border-radius: 16px;
  padding: 2rem;
  border-left: 5px solid #2563eb;
}

.challenge-title {
  font-size: 1.2rem;
  font-weight: 700;
  color: #93c5fd;
  margin-bottom: 0.5rem;
}
.challenge-desc {
  color: var(--text-secondary);
  margin-bottom: 1.5rem;
}

/* SUMMARY */
.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
}

.summary-item {
  background: var(--bg-surface);
  padding: 1rem;
  border-radius: 8px;
  display: flex;
  flex-direction: column;
}

.summary-item code {
  color: var(--text-info, #2563eb);
  font-family: monospace;
}
.summary-item span {
  font-size: 0.85rem;
  color: var(--text-muted);
}

@media (max-width: 768px) {
  .container-viz,
  .k8s-viz {
    flex-direction: column;
    gap: 1rem;
  }
}
</style>
