<template>
  <LessonContainer>
    <!-- INTRO -->
    <TextBlock>
      ¿Por qué tu terminal sabe dónde está <code>python3</code> pero no dónde está
      <code>ros2</code>? La respuesta está en las <strong>Variables de Entorno</strong>: la memoria
      invisible de tu sistema que determina qué comandos funcionan y cuáles no. <br /><br />
      Esta lección te enseña a manipular el "ADN" de tu sesión para que ROS 2 funcione
      automáticamente en cada terminal que abras.
    </TextBlock>

    <!-- QUÉ SON -->
    <div class="section-group">
      <SectionTitle>1. ¿Qué son las Variables de Entorno?</SectionTitle>
      <TextBlock>
        Son <strong>pares clave-valor</strong> que almacenan información sobre tu sesión actual.
        Cada terminal que abres nace con una "mochila" llena de estas variables.
      </TextBlock>

      <div class="var-anatomy q-mt-md">
        <div class="anatomy-title">Anatomía de una Variable</div>
        <div class="anatomy-formula">
          <span class="formula-key">NOMBRE</span>
          <span class="formula-equals">=</span>
          <span class="formula-value">"valor"</span>
        </div>
        <div class="anatomy-examples">
          <div class="example-row">
            <code class="ex-key">HOME</code>
            <span class="ex-desc">Tu carpeta personal (<code>/home/alexander</code>)</span>
          </div>
          <div class="example-row">
            <code class="ex-key">USER</code>
            <span class="ex-desc">Tu nombre de usuario (<code>alexander</code>)</span>
          </div>
          <div class="example-row">
            <code class="ex-key">PATH</code>
            <span class="ex-desc">Dónde buscar comandos ejecutables</span>
          </div>
          <div class="example-row">
            <code class="ex-key">ROS_DISTRO</code>
            <span class="ex-desc"
              >Versión de ROS 2 activa (<code>humble</code>, <code>jazzy</code>)</span
            >
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <SectionTitle>Ver Tus Variables Actuales</SectionTitle>
        <div class="row q-col-gutter-md">
          <div class="col-12 col-md-6">
            <CodeBlock
              title="Ver todas las variables"
              lang="bash"
              content="printenv"
              :copyable="true"
            />
          </div>
          <div class="col-12 col-md-6">
            <CodeBlock
              title="Ver solo variables de ROS"
              lang="bash"
              content="printenv | grep ROS"
              :copyable="true"
            />
          </div>
          <div class="col-12 col-md-6">
            <CodeBlock
              title="Ver una variable específica"
              lang="bash"
              content="echo $HOME
echo $PATH"
              :copyable="true"
            />
          </div>
          <div class="col-12 col-md-6">
            <CodeBlock
              title="Crear variable temporal"
              lang="bash"
              content='export MI_ROBOT="TurtleBot3"
echo $MI_ROBOT'
              :copyable="true"
            />
          </div>
        </div>
      </div>
    </div>

    <!-- PATH -->
    <div class="section-group">
      <SectionTitle>2. La Variable Reina: $PATH</SectionTitle>
      <TextBlock>
        Cuando escribes <code>python3</code> o <code>ros2</code>, Linux NO busca en todo el disco.
        Solo busca en las carpetas listadas en <code>$PATH</code>.
      </TextBlock>

      <div class="path-demo q-mt-md">
        <div class="path-header">
          <q-icon name="search" color="blue-4" size="md" />
          <span>Cómo Funciona PATH</span>
        </div>
        <div class="path-flow">
          <div class="flow-step">
            <div class="step-num">1</div>
            <div class="step-desc">Escribes: <code>ros2 topic list</code></div>
          </div>
          <div class="flow-arrow">→</div>
          <div class="flow-step">
            <div class="step-num">2</div>
            <div class="step-desc">Linux busca <code>ros2</code> en cada carpeta de PATH</div>
          </div>
          <div class="flow-arrow">→</div>
          <div class="flow-step">
            <div class="step-num">3</div>
            <div class="step-desc">
              Si lo encuentra: ✅ Ejecuta
              <br />
              Si no: ❌ "Command not found"
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <CodeBlock
          title="Ver tu PATH actual"
          lang="bash"
          content="echo $PATH
# Output típico (separado por :):
# /usr/local/bin:/usr/bin:/bin:/opt/ros/humble/bin"
          :copyable="true"
        />
      </div>

      <AlertBlock type="warning" title="El Problema de ROS 2" class="q-mt-md">
        ROS 2 se instala en <code>/opt/ros/humble</code> (o <code>jazzy</code>), una carpeta que
        <strong>NO está en PATH por defecto</strong>. Por eso obtienes "Command 'ros2' not found"
        hasta que lo agregues manualmente.
      </AlertBlock>
    </div>

    <!-- SOURCE -->
    <div class="section-group">
      <SectionTitle>3. El Ritual del "source"</SectionTitle>
      <TextBlock>
        El comando <code>source</code> ejecuta un script que modifica tus variables de entorno. En
        ROS 2, el archivo <code>setup.bash</code> agrega las rutas necesarias a PATH.
      </TextBlock>

      <div class="source-demo q-mt-md">
        <div class="demo-before">
          <div class="demo-label">❌ Antes de source</div>
          <CodeBlock
            :hide-header="true"
            lang="bash"
            content="ros2 topic list
# bash: ros2: command not found"
          />
        </div>
        <div class="demo-action">
          <q-icon name="bolt" color="yellow-6" size="2rem" />
          <CodeBlock
            :hide-header="true"
            lang="bash"
            content="source /opt/ros/humble/setup.bash"
            :copyable="true"
          />
        </div>
        <div class="demo-after">
          <div class="demo-label">✅ Después de source</div>
          <CodeBlock
            :hide-header="true"
            lang="bash"
            content="ros2 topic list
# /parameter_events
# /rosout"
          />
        </div>
      </div>

      <div class="q-mt-lg">
        <AlertBlock type="info" title="¿Qué hace setup.bash exactamente?">
          <ol class="q-pl-md">
            <li>Agrega <code>/opt/ros/humble/bin</code> a PATH</li>
            <li>Define variables como <code>ROS_DISTRO=humble</code></li>
            <li>Configura rutas de Python para encontrar paquetes ROS</li>
            <li>Establece configuraciones de DDS (middleware)</li>
          </ol>
        </AlertBlock>
      </div>

      <div class="q-mt-md">
        <div class="amnesia-card">
          <q-icon name="warning" color="orange-4" size="xl" />
          <div class="amnesia-content">
            <div class="amnesia-title">⚠️ Amnesia de Terminal</div>
            <div class="amnesia-desc">
              Las variables son <strong>temporales</strong>. Si cierras la terminal, desaparecen.
              Tendrías que hacer <code>source</code> en cada nueva ventana... a menos que lo
              automatices.
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- BASHRC -->
    <div class="section-group">
      <SectionTitle>4. La Solución Permanente: .bashrc</SectionTitle>
      <TextBlock>
        El archivo <code>~/.bashrc</code> es un script que se ejecuta
        <strong>automáticamente</strong> cada vez que abres una terminal. Es el lugar perfecto para
        "tatuar" tus configuraciones.
      </TextBlock>

      <div class="bashrc-workflow q-mt-md">
        <div class="workflow-card">
          <div class="workflow-step">
            <div class="workflow-num">1</div>
            <div class="workflow-content">
              <div class="workflow-title">Abrir .bashrc</div>
              <CodeBlock lang="bash" content="nano ~/.bashrc" :copyable="true" />
            </div>
          </div>
        </div>

        <div class="workflow-card">
          <div class="workflow-step">
            <div class="workflow-num">2</div>
            <div class="workflow-content">
              <div class="workflow-title">Agregar al final del archivo</div>
              <CodeBlock
                lang="bash"
                content="# ROS 2 Humble Setup
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42

# Alias útiles
alias cb='colcon build --symlink-install'
alias sb='source install/setup.bash'"
                :copyable="true"
              />
            </div>
          </div>
        </div>

        <div class="workflow-card">
          <div class="workflow-step">
            <div class="workflow-num">3</div>
            <div class="workflow-content">
              <div class="workflow-title">Guardar y aplicar cambios</div>
              <CodeBlock
                lang="bash"
                content="# Ctrl+O para guardar, Ctrl+X para salir

# Recargar .bashrc sin cerrar terminal
source ~/.bashrc"
                :copyable="true"
              />
            </div>
          </div>
        </div>

        <div class="workflow-card">
          <div class="workflow-step">
            <div class="workflow-num">4</div>
            <div class="workflow-content">
              <div class="workflow-title">Verificar</div>
              <CodeBlock
                lang="bash"
                content="echo $ROS_DISTRO
# humble

ros2 --version
# ros2 cli version 0.x.x"
                :copyable="true"
              />
            </div>
          </div>
        </div>
      </div>

      <AlertBlock type="success" title="¡Ahora es Permanente!" class="q-mt-lg">
        Cada nueva terminal que abras tendrá ROS 2 configurado automáticamente. No más
        <code>source</code> manual.
      </AlertBlock>
    </div>

    <!-- ROS_DOMAIN_ID -->
    <div class="section-group">
      <SectionTitle>5. Variable Crítica: ROS_DOMAIN_ID</SectionTitle>
      <TextBlock>
        ROS 2 usa DDS (Data Distribution Service) para comunicación. Por defecto, todos los robots
        en la misma red se ven entre sí. <code>ROS_DOMAIN_ID</code> crea "canales privados".
      </TextBlock>

      <div class="domain-scenario q-mt-md">
        <div class="scenario-bad">
          <div class="scenario-header bad">
            <q-icon name="error" size="md" />
            <span>Sin ROS_DOMAIN_ID (Default: 0)</span>
          </div>
          <div class="scenario-content">
            <div class="scenario-desc">
              20 estudiantes en la misma WiFi → Todos en dominio 0 → Caos total → Los comandos de un
              estudiante afectan el robot de otro
            </div>
            <div class="scenario-visual">
              <div class="robot-icon">🤖</div>
              <div class="robot-icon">🤖</div>
              <div class="robot-icon">🤖</div>
              <div class="chaos-label">Todos escuchan a todos</div>
            </div>
          </div>
        </div>

        <div class="scenario-good">
          <div class="scenario-header good">
            <q-icon name="check_circle" size="md" />
            <span>Con ROS_DOMAIN_ID Único</span>
          </div>
          <div class="scenario-content">
            <div class="scenario-desc">
              Cada estudiante tiene su propio dominio (0-101) → Aislamiento total → Solo tu laptop
              controla tu robot
            </div>
            <div class="scenario-visual">
              <div class="domain-box">
                <div class="domain-label">Dominio 42</div>
                <div class="robot-icon">🤖</div>
              </div>
              <div class="domain-box">
                <div class="domain-label">Dominio 43</div>
                <div class="robot-icon">🤖</div>
              </div>
              <div class="domain-box">
                <div class="domain-label">Dominio 44</div>
                <div class="robot-icon">🤖</div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <CodeBlock
          title="Configurar tu dominio único"
          lang="bash"
          content="# En ~/.bashrc
export ROS_DOMAIN_ID=42  # Elige un número entre 0 y 101

# Verificar
echo $ROS_DOMAIN_ID"
          :copyable="true"
        />
      </div>
    </div>

    <!-- WORKSPACE OVERLAY -->
    <div class="section-group">
      <SectionTitle>6. Overlay de Workspace</SectionTitle>
      <TextBlock>
        Cuando compilas tu propio código con <code>colcon build</code>, se genera un
        <code>install/setup.bash</code> en tu workspace. Debes hacer source de AMBOS: ROS base + tu
        workspace.
      </TextBlock>

      <div class="overlay-diagram q-mt-md">
        <div class="overlay-layer base">
          <div class="layer-title">Capa Base (Underlay)</div>
          <div class="layer-content">
            <code>source /opt/ros/humble/setup.bash</code>
            <div class="layer-desc">Paquetes oficiales de ROS 2</div>
          </div>
        </div>
        <div class="overlay-arrow">↓ Overlay</div>
        <div class="overlay-layer workspace">
          <div class="layer-title">Capa de Workspace (Overlay)</div>
          <div class="layer-content">
            <code>source ~/ros2_ws/install/setup.bash</code>
            <div class="layer-desc">Tus paquetes personalizados</div>
          </div>
        </div>
      </div>

      <div class="q-mt-lg">
        <CodeBlock
          title="Orden correcto en .bashrc"
          lang="bash"
          content="# 1. Primero el ROS base
source /opt/ros/humble/setup.bash

# 2. Luego tu workspace (si existe)
if [ -f ~/ros2_ws/install/setup.bash ]; then
  source ~/ros2_ws/install/setup.bash
fi"
          :copyable="true"
        />
      </div>
    </div>

    <!-- ERRORES COMUNES -->
    <div class="section-group">
      <SectionTitle>Errores Comunes y Soluciones</SectionTitle>

      <q-expansion-item
        icon="error"
        label="bash: ros2: command not found"
        header-class="error-header"
      >
        <div class="error-content">
          <strong>Causa:</strong> No has hecho <code>source</code> del setup.bash de ROS 2.
          <br /><br />
          <strong>Solución:</strong>
          <CodeBlock
            lang="bash"
            content="source /opt/ros/humble/setup.bash
# O agrega esa línea a ~/.bashrc"
            :copyable="true"
          />
        </div>
      </q-expansion-item>

      <q-expansion-item
        icon="error"
        label="Package 'mi_paquete' not found"
        header-class="error-header"
        class="q-mt-sm"
      >
        <div class="error-content">
          <strong>Causa:</strong> Compilaste tu workspace pero no hiciste source del
          <code>install/setup.bash</code>. <br /><br />
          <strong>Solución:</strong>
          <CodeBlock
            lang="bash"
            content="cd ~/ros2_ws
source install/setup.bash"
            :copyable="true"
          />
        </div>
      </q-expansion-item>

      <q-expansion-item
        icon="error"
        label="Veo topics de otros robots en mi red"
        header-class="error-header"
        class="q-mt-sm"
      >
        <div class="error-content">
          <strong>Causa:</strong> Estás usando el <code>ROS_DOMAIN_ID</code> por defecto (0).
          <br /><br />
          <strong>Solución:</strong>
          <CodeBlock
            lang="bash"
            content="export ROS_DOMAIN_ID=42  # Elige un número único
# Agrega esto a ~/.bashrc para hacerlo permanente"
            :copyable="true"
          />
        </div>
      </q-expansion-item>
    </div>

    <!-- VIDEO -->
    <div class="section-group">
      <SectionTitle>📹 Video Complementario</SectionTitle>
      <div class="video-container">
        <div class="video-wrapper">
          <iframe
            src="https://youtu.be/Romc22GgusU"
            title="Variables de Entorno en Linux"
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
          Video En progreso
        </div>
      </div>
    </div>

    <!-- RESUMEN -->
    <div class="section-group q-mb-xl">
      <SectionTitle>📝 Resumen de Comandos Esenciales</SectionTitle>
      <div class="summary-grid">
        <div class="summary-item">
          <code>printenv</code>
          <span>Ver todas las variables de entorno</span>
        </div>
        <div class="summary-item">
          <code>echo $PATH</code>
          <span>Ver dónde busca Linux los comandos</span>
        </div>
        <div class="summary-item">
          <code>source setup.bash</code>
          <span>Cargar configuración de ROS 2</span>
        </div>
        <div class="summary-item">
          <code>nano ~/.bashrc</code>
          <span>Editar configuración permanente</span>
        </div>
        <div class="summary-item">
          <code>export VAR=valor</code>
          <span>Crear variable temporal</span>
        </div>
        <div class="summary-item">
          <code>source ~/.bashrc</code>
          <span>Recargar configuración</span>
        </div>
      </div>

      <AlertBlock type="success" title="Tu .bashrc Ideal para ROS 2" class="q-mt-lg">
        <CodeBlock
          lang="bash"
          content="# ROS 2 Setup
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42

# Workspace (si existe)
if [ -f ~/ros2_ws/install/setup.bash ]; then
  source ~/ros2_ws/install/setup.bash
fi

# Alias útiles
alias cb='colcon build --symlink-install'
alias sb='source install/setup.bash'
alias rt='ros2 topic list'
alias rn='ros2 node list'"
          :copyable="true"
        />
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

.var-anatomy {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.9), rgba(30, 41, 59, 0.9));
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.anatomy-title {
  text-align: center;
  font-size: 1.25rem;
  font-weight: 700;
  color: #60a5fa;
  margin-bottom: 1.5rem;
}

.anatomy-formula {
  text-align: center;
  font-family: 'Fira Code', monospace;
  font-size: 2rem;
  margin-bottom: 2rem;
}

.formula-key {
  color: #a855f7;
  font-weight: 700;
}

.formula-equals {
  color: #94a3b8;
  margin: 0 1rem;
}

.formula-value {
  color: #22c55e;
}

.anatomy-examples {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.example-row {
  display: flex;
  align-items: center;
  gap: 1rem;
  background: rgba(0, 0, 0, 0.3);
  padding: 0.75rem 1rem;
  border-radius: 8px;
}

.ex-key {
  font-family: 'Fira Code', monospace;
  color: #fbbf24;
  font-weight: 700;
  min-width: 120px;
}

.ex-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
}

.path-demo {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 16px;
  padding: 2rem;
}

.path-header {
  display: flex;
  align-items: center;
  gap: 12px;
  font-size: 1.25rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1.5rem;
}

.path-flow {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 1rem;
}

.flow-step {
  flex: 1;
  background: rgba(0, 0, 0, 0.3);
  border: 1px solid rgba(59, 130, 246, 0.3);
  border-radius: 12px;
  padding: 1.5rem;
  text-align: center;
}

.step-num {
  width: 2rem;
  height: 2rem;
  background: #3b82f6;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
  color: white;
  margin: 0 auto 0.75rem;
}

.step-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
}

.flow-arrow {
  font-size: 2rem;
  color: #3b82f6;
}

.source-demo {
  display: grid;
  grid-template-columns: 1fr auto 1fr;
  gap: 1.5rem;
  align-items: center;
}

.demo-before,
.demo-after {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.demo-label {
  font-weight: 700;
  margin-bottom: 1rem;
  text-align: center;
}

.demo-action {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 1rem;
}

.amnesia-card {
  background: linear-gradient(135deg, rgba(120, 53, 15, 0.2), rgba(146, 64, 14, 0.2));
  border: 2px solid rgba(251, 146, 60, 0.5);
  border-radius: 16px;
  padding: 2rem;
  display: flex;
  align-items: center;
  gap: 1.5rem;
}

.amnesia-content {
  flex-grow: 1;
}

.amnesia-title {
  font-size: 1.25rem;
  font-weight: 700;
  color: #fbbf24;
  margin-bottom: 0.5rem;
}

.amnesia-desc {
  color: #fde68a;
  line-height: 1.6;
}

.bashrc-workflow {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.workflow-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  padding: 1.5rem;
}

.workflow-step {
  display: flex;
  gap: 1.5rem;
  align-items: flex-start;
}

.workflow-num {
  min-width: 3rem;
  height: 3rem;
  background: linear-gradient(135deg, #8b5cf6, #7c3aed);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.5rem;
  font-weight: 700;
  color: white;
  flex-shrink: 0;
}

.workflow-content {
  flex-grow: 1;
}

.workflow-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 0.75rem;
}

.domain-scenario {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.scenario-bad,
.scenario-good {
  border-radius: 16px;
  overflow: hidden;
}

.scenario-header {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 1rem 1.5rem;
  font-weight: 700;
  font-size: 1.1rem;
}

.scenario-header.bad {
  background: rgba(127, 29, 29, 0.3);
  border-bottom: 2px solid #ef4444;
  color: #fca5a5;
}

.scenario-header.good {
  background: rgba(20, 83, 45, 0.3);
  border-bottom: 2px solid #22c55e;
  color: #86efac;
}

.scenario-content {
  padding: 1.5rem;
  background: rgba(15, 23, 42, 0.6);
}

.scenario-desc {
  color: #cbd5e1;
  margin-bottom: 1.5rem;
  line-height: 1.6;
}

.scenario-visual {
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 1rem;
  padding: 1rem;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 8px;
}

.robot-icon {
  font-size: 2rem;
}

.chaos-label {
  color: #fca5a5;
  font-size: 0.85rem;
  font-style: italic;
}

.domain-box {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  padding: 1rem;
  background: rgba(59, 130, 246, 0.1);
  border: 1px solid rgba(59, 130, 246, 0.3);
  border-radius: 8px;
}

.domain-label {
  font-size: 0.75rem;
  color: #60a5fa;
  font-family: 'Fira Code', monospace;
}

.overlay-diagram {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  align-items: center;
}

.overlay-layer {
  width: 100%;
  max-width: 600px;
  border-radius: 12px;
  padding: 1.5rem;
  text-align: center;
}

.overlay-layer.base {
  background: rgba(59, 130, 246, 0.1);
  border: 2px solid #3b82f6;
}

.overlay-layer.workspace {
  background: rgba(168, 85, 247, 0.1);
  border: 2px solid #a855f7;
}

.layer-title {
  font-weight: 700;
  color: #f1f5f9;
  margin-bottom: 1rem;
}

.layer-content code {
  display: block;
  margin-bottom: 0.5rem;
}

.layer-desc {
  font-size: 0.85rem;
  color: #94a3b8;
}

.overlay-arrow {
  font-size: 1.5rem;
  font-weight: 700;
  color: #a855f7;
}

:deep(.error-header) {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid rgba(239, 68, 68, 0.3);
  border-radius: 8px;
  color: #fca5a5;
}

.error-content {
  background: rgba(15, 23, 42, 0.6);
  padding: 1.5rem;
  color: #cbd5e1;
}

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

.summary-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
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
  color: #22c55e;
  font-size: 1rem;
}

.summary-item span {
  color: #cbd5e1;
  font-size: 0.85rem;
}

@media (max-width: 768px) {
  .path-flow {
    flex-direction: column;
  }

  .flow-arrow {
    transform: rotate(90deg);
  }

  .source-demo {
    grid-template-columns: 1fr;
  }

  .demo-action {
    order: 2;
  }
}
</style>
