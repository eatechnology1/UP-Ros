<template>
  <LessonContainer>

    <!-- INTRO + FACT PILLS -->
    <div class="section-group">
      <TextBlock>
        Un <strong>workspace</strong> en ROS 2 es el entorno de desarrollo donde organizas, compilas
        y ejecutas paquetes. Implementa <strong>overlay/underlay</strong>: puedes extender la
        instalación base de ROS 2 sin modificarla — fundamental para trabajo en equipo y testing
        aislado.
      </TextBlock>
      <div class="fact-pills q-mt-lg">
        <div v-for="f in facts" :key="f.label" class="fact-pill">
          <span class="fp-icon">{{ f.icon }}</span>
          <span class="fp-text">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══ 01 ANATOMÍA DEL WORKSPACE ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        Anatomía del Workspace
      </SectionTitle>
      <TextBlock>
        Un workspace ROS 2 tiene cuatro directorios con roles bien definidos. Solo
        <code>src/</code> contiene tu código — los demás son generados por colcon y pueden
        borrarse sin perder trabajo:
      </TextBlock>

      <!-- Animated workspace tree -->
      <div class="ws-tree q-mt-xl">
        <div class="wst-root">
          <q-icon name="folder" size="20px" :style="{ color: '#60a5fa' }"/>
          <code class="wst-root-name">~/ros2_ws/</code>
          <span class="wst-root-tag">workspace raíz</span>
        </div>
        <div class="wst-folders">
          <div v-for="(folder, idx) in wsFolders" :key="folder.name" class="wst-folder"
            :style="{ '--wsf-color': folder.color, animationDelay: idx * 0.12 + 's' }">
            <div class="wsf-icon">
              <q-icon :name="folder.icon" size="22px" :style="{ color: folder.color }"/>
            </div>
            <div class="wsf-info">
              <div class="wsf-name">
                <code>{{ folder.name }}</code>
                <div class="wsf-tags">
                  <span v-for="tag in folder.tags" :key="tag" class="wsf-tag">{{ tag }}</span>
                </div>
              </div>
              <div class="wsf-desc">{{ folder.desc }}</div>
            </div>
            <div class="wsf-files">
              <div v-for="file in folder.files" :key="file" class="wsf-file">
                <q-icon name="insert_drive_file" size="12px" style="color:var(--text-muted)"/>
                <code>{{ file }}</code>
              </div>
            </div>
          </div>
        </div>
      </div>

      <CodeBlock title="Crear workspace desde cero" lang="bash"
        :content="wsCreateCode" :copyable="true" class="q-mt-xl"/>

      <div class="bashrc-tip q-mt-lg">
        <div class="bt-header">
          <q-icon name="terminal" size="18px" style="color:#4ade80"/>
          <strong>Automatizar en .bashrc — hazlo una vez, funciona siempre</strong>
        </div>
        <CodeBlock :hide-header="true" lang="bash" :content="bashrcCode" :copyable="true"/>
      </div>
    </div>

    <!-- ══ 02 CREAR PAQUETES ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        Crear Paquetes con ros2 pkg create
      </SectionTitle>
      <TextBlock>
        <code>ros2 pkg create</code> es la forma oficial de crear paquetes en ROS 2. Genera
        toda la estructura de archivos y configura el build type correcto automáticamente:
      </TextBlock>

      <div class="pkg-create-grid q-mt-lg">
        <div v-for="pt in pkgTypes" :key="pt.type" class="pkgt-card"
          :style="{ '--pkgt-color': pt.color }">
          <div class="pkgtc-header">
            <div class="pkgtc-icon" :style="{ background: pt.color + '18' }">
              <q-icon :name="pt.icon" size="24px" :style="{ color: pt.color }"/>
            </div>
            <div>
              <div class="pkgtc-type">{{ pt.type }}</div>
              <div class="pkgtc-build">{{ pt.buildType }}</div>
            </div>
          </div>
          <CodeBlock :hide-header="true" lang="bash" :content="pt.cmd" :copyable="true"/>
          <div class="pkgtc-files">
            <div class="pkgtcf-label">Archivos generados:</div>
            <div class="pkgtcf-list">
              <div v-for="file in pt.files" :key="file.name" class="pkgtcf-item"
                :style="{ '--pf-color': file.color }">
                <q-icon :name="file.icon" size="13px" :style="{ color: file.color }"/>
                <code class="pf-name">{{ file.name }}</code>
                <span class="pf-role">{{ file.role }}</span>
              </div>
            </div>
          </div>
        </div>
      </div>

      <div class="pkg-flags q-mt-xl">
        <div class="pkgf-title">Flags útiles de ros2 pkg create:</div>
        <div class="pkgf-grid">
          <div v-for="flag in pkgFlags" :key="flag.flag" class="pkgf-item"
            :style="{ '--pkgf-color': flag.color }">
            <code class="pkgfi-flag">{{ flag.flag }}</code>
            <div class="pkgfi-desc">{{ flag.desc }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══ 03 COLCON BUILD ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        colcon build — El Sistema de Build de ROS 2
      </SectionTitle>
      <TextBlock>
        <strong>colcon</strong> (collective construction) orquesta CMake para C++ y setuptools
        para Python. Analiza las dependencias entre paquetes y los compila en el orden correcto,
        en paralelo:
      </TextBlock>

      <!-- Build pipeline animated -->
      <div class="build-pipeline q-mt-xl">
        <div v-for="(stage, idx) in buildStages" :key="stage.title" class="bps-wrapper">
          <div class="build-stage" :style="{ '--bs-color': stage.color, '--bs-delay': idx * 0.15 + 's' }">
            <div class="bss-number" :style="{ background: stage.color, color: '#0d1117' }">{{ stage.num }}</div>
            <div class="bss-icon">
              <q-icon :name="stage.icon" size="22px" :style="{ color: stage.color }"/>
            </div>
            <div class="bss-content">
              <div class="bss-title" :style="{ color: stage.color }">{{ stage.title }}</div>
              <div class="bss-desc">{{ stage.desc }}</div>
              <CodeBlock :hide-header="true" lang="bash" :content="stage.code" :copyable="true"/>
            </div>
          </div>
          <div v-if="idx < buildStages.length - 1" class="bps-arrow">
            <div class="bpsa-track">
              <div class="bpsa-packet" :style="{ '--bpa-color': stage.color, animationDelay: idx * 0.4 + 's' }"></div>
            </div>
            <q-icon name="arrow_downward" size="14px" style="color:var(--text-muted)"/>
          </div>
        </div>
      </div>

      <!-- Colcon advanced options -->
      <SectionTitle class="q-mt-xl">Opciones Avanzadas de colcon</SectionTitle>
      <div class="colcon-opts q-mt-lg">
        <div v-for="opt in colconOpts" :key="opt.title" class="colcon-opt-card"
          :style="{ '--coc-color': opt.color }">
          <div class="cocc-header">
            <q-icon :name="opt.icon" size="18px" :style="{ color: opt.color }"/>
            <span class="cocc-title">{{ opt.title }}</span>
            <div class="cocc-badge" :style="{ background: opt.color + '15', color: opt.color }">
              {{ opt.badge }}
            </div>
          </div>
          <CodeBlock :hide-header="true" lang="bash" :content="opt.code" :copyable="true"/>
        </div>
      </div>
    </div>

    <!-- ══ 04 ESTRUCTURA DE PAQUETES ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        Estructura Interna de Paquetes
      </SectionTitle>
      <TextBlock>
        Cada tipo de paquete tiene archivos con roles específicos. Conocerlos de memoria
        acelera el desarrollo — entenderás qué modificar cuando algo falla:
      </TextBlock>

      <div class="pkg-structs q-mt-lg">
        <div v-for="ps in pkgStructures" :key="ps.lang" class="pks-card"
          :style="{ '--pks-color': ps.color }">
          <div class="pksc-header">
            <q-icon :name="ps.icon" size="22px" :style="{ color: ps.color }"/>
            <span class="pksc-lang">{{ ps.lang }}</span>
            <code class="pksc-build">{{ ps.buildType }}</code>
          </div>
          <div class="pksc-files">
            <div v-for="file in ps.files" :key="file.name" class="pkscf-item"
              :style="{ '--pksf-color': file.color }">
              <div class="pkscfi-left">
                <q-icon :name="file.type === 'dir' ? 'folder' : 'description'" size="14px" :style="{ color: file.color }"/>
                <code class="pkscfi-name">{{ file.name }}</code>
              </div>
              <div class="pkscfi-right">
                <span class="pkscfi-role">{{ file.role }}</span>
                <q-icon v-if="file.required" name="star" size="12px" style="color:#fbbf24" title="Requerido"/>
              </div>
            </div>
          </div>
          <CodeBlock :hide-header="true" :lang="ps.lang === 'C++' ? 'cmake' : 'python'"
            :content="ps.mainFileCode" :copyable="true"/>
        </div>
      </div>
    </div>

    <!-- ══ 05 DEPENDENCIAS ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Dependencias — package.xml y rosdep
      </SectionTitle>
      <TextBlock>
        El archivo <code>package.xml</code> (formato REP-149) es la fuente de verdad de las
        dependencias de tu paquete. Define qué necesitas en compilación, en runtime y en testing
        — y rosdep los instala automáticamente:
      </TextBlock>

      <div class="dep-types q-mt-lg">
        <div v-for="dt in depTypes" :key="dt.tag" class="dep-card"
          :style="{ '--dep-color': dt.color }">
          <div class="depc-header">
            <q-icon :name="dt.icon" size="18px" :style="{ color: dt.color }"/>
            <code class="depc-tag">{{ dt.tag }}</code>
          </div>
          <div class="depc-desc">{{ dt.desc }}</div>
          <div class="depc-example" :style="{ color: dt.color, background: dt.color + '10', borderColor: dt.color + '25' }">
            {{ dt.example }}
          </div>
        </div>
      </div>

      <CodeBlock title="package.xml completo (ROS 2 Jazzy)" lang="xml"
        :content="packageXmlCode" :copyable="true" class="q-mt-xl"/>

      <div class="q-my-xl">
        <DependencySolver />
      </div>

      <div class="rosdep-box q-mt-lg">
        <div class="rb-header">
          <q-icon name="download" size="18px" style="color:#22d3ee"/>
          <strong>rosdep — Resolución Automática de Dependencias del SO</strong>
        </div>
        <p class="rb-desc">
          rosdep mapea nombres de paquetes ROS a paquetes del sistema operativo (apt, pip, etc.).
          Solo necesitas correrlo una vez al clonar un workspace nuevo:
        </p>
        <CodeBlock :hide-header="true" lang="bash" :content="rosdepCode" :copyable="true"/>
      </div>
    </div>

    <!-- ══ 06 OVERLAY/UNDERLAY ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">06</span>
        Overlay/Underlay — Workspaces en Capas
      </SectionTitle>
      <TextBlock>
        ROS 2 permite <strong>encadenar workspaces</strong>: el último que se sourcea tiene
        prioridad. Esto permite modificar o extender paquetes de la instalación base sin
        tocarla — invaluable para probar cambios antes de mergearlos:
      </TextBlock>

      <!-- Animated overlay stack -->
      <div class="overlay-stack q-mt-xl">
        <div class="ovs-label">Orden de búsqueda de paquetes (arriba = mayor prioridad)</div>
        <div class="ovs-layers">
          <div v-for="(layer, idx) in overlayLayers" :key="layer.path" class="ovs-layer"
            :style="{ '--ol-color': layer.color, '--ol-delay': idx * 0.15 + 's' }">
            <div class="ovsl-left">
              <div class="ovsl-icon">
                <q-icon name="layers" size="18px" :style="{ color: layer.color }"/>
              </div>
              <div class="ovsl-info">
                <div class="ovsl-label" :style="{ color: layer.color }">{{ layer.label }}</div>
                <code class="ovsl-path">{{ layer.path }}</code>
              </div>
            </div>
            <div class="ovsl-right">
              <div class="ovsl-priority" :style="{ background: layer.color + '15', color: layer.color, borderColor: layer.color + '30' }">
                {{ layer.priority }}
              </div>
            </div>
          </div>
          <div class="ovs-arrow-col">
            <div v-for="i in overlayLayers.length - 1" :key="i" class="ovsa-item">
              <div class="ovsa-packet" :style="{ '--op-color': overlayLayers[i - 1]!.color, animationDelay: (i - 1) * 0.6 + 's' }"></div>
              <code class="ovsa-text">source install/setup.bash</code>
            </div>
          </div>
        </div>
      </div>

      <CodeBlock title="Crear y usar overlay" lang="bash"
        :content="overlayCode" :copyable="true" class="q-mt-xl"/>

      <div class="sourcing-order q-mt-lg">
        <div class="so-title">
          <q-icon name="warning" size="18px" color="warning"/>
          Orden de sourcing — El último gana
        </div>
        <div class="so-sequence">
          <div v-for="(step, idx) in sourcingOrder" :key="step.cmd" class="so-step"
            :style="{ '--ss-color': step.color }">
            <div class="sss-num" :style="{ background: step.color + '20', color: step.color }">{{ idx + 1 }}</div>
            <code class="sss-cmd">{{ step.cmd }}</code>
            <div class="sss-desc">{{ step.desc }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══ 07 FLUJO DE TRABAJO ══ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">07</span>
        Flujo de Trabajo Diario — De la Idea al Nodo
      </SectionTitle>
      <TextBlock>
        El ciclo de desarrollo en ROS 2 sigue siempre la misma secuencia. Memorizarla
        convierte el proceso en algo automático:
      </TextBlock>

      <div class="workflow-steps q-mt-xl">
        <div v-for="(step, idx) in workflowSteps" :key="step.title" class="wfs-item">
          <div class="wfsi-left">
            <div class="wfsi-num" :style="{ background: step.color }">{{ idx + 1 }}</div>
            <div v-if="idx < workflowSteps.length - 1" class="wfsi-line"></div>
          </div>
          <div class="wfsi-content" :style="{ '--wfs-color': step.color }">
            <div class="wfsic-header">
              <q-icon :name="step.icon" size="18px" :style="{ color: step.color }"/>
              <span class="wfsic-title">{{ step.title }}</span>
            </div>
            <div class="wfsic-desc">{{ step.desc }}</div>
            <CodeBlock :hide-header="true" lang="bash" :content="step.code" :copyable="true"/>
          </div>
        </div>
      </div>

      <!-- Testing section -->
      <SectionTitle class="q-mt-xl">Testing con colcon test</SectionTitle>
      <TextBlock>
        ROS 2 integra testing directamente en el pipeline de colcon. Siempre deberías compilar
        y testear antes de un commit:
      </TextBlock>

      <div class="test-cmds q-mt-lg">
        <div v-for="tc in testCmds" :key="tc.cmd" class="tc-item"
          :style="{ '--tc-color': tc.color }">
          <code class="tci-cmd">{{ tc.cmd }}</code>
          <div class="tci-desc">{{ tc.desc }}</div>
        </div>
      </div>

      <CodeBlock title="Agregar test en CMakeLists.txt" lang="cmake"
        :content="testCmakeCode" :copyable="true" class="q-mt-lg"/>
    </div>

    <!-- ══ ERRORES COMUNES ══ -->
    <div class="section-group">
      <SectionTitle>Errores Comunes de Workspace</SectionTitle>
      <div class="error-list q-mt-lg">
        <div v-for="(err, i) in commonErrors" :key="i" class="error-item"
          :style="{ '--err-color': err.color }">
          <div class="err-header" @click="err.open = !err.open">
            <div class="err-left">
              <div class="err-num" :style="{ background: err.color + '18', color: err.color }">{{ i + 1 }}</div>
              <div>
                <div class="err-type" :style="{ color: err.color }">{{ err.type }}</div>
                <div class="err-summary">{{ err.summary }}</div>
              </div>
            </div>
            <q-icon :name="err.open ? 'expand_less' : 'expand_more'"
              size="20px" style="color:var(--text-muted);flex-shrink:0"/>
          </div>
          <div v-show="err.open" class="err-body">
            <div class="err-cause">
              <q-icon name="search" size="14px" class="q-mr-xs"/>
              <strong>Situación:</strong> {{ err.cause }}
            </div>
            <CodeBlock :hide-header="true" lang="bash" :content="err.code"/>
            <div class="err-fix">
              <q-icon name="build" size="14px" class="q-mr-xs" color="positive"/>
              <strong>Solución:</strong> {{ err.fix }}
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══ RETO ══ -->
    <div class="section-group">
      <SectionTitle>Reto — Tu Primer Paquete ROS 2</SectionTitle>
      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning"/>
          </div>
          <div>
            <div class="challenge-title">Crea un workspace con un paquete C++ y uno Python</div>
            <div class="challenge-subtitle">Desde cero hasta ros2 run funcionando en ambos</div>
          </div>
          <div class="challenge-badge">60 min</div>
        </div>
        <div class="challenge-steps q-mt-md">
          <div class="cs-title">Pasos:</div>
          <div class="cs-list">
            <div v-for="step in challengeSteps" :key="step.num" class="cs-item">
              <div class="cs-num" :style="{ background: step.color }">{{ step.num }}</div>
              <div class="cs-text">{{ step.text }}</div>
            </div>
          </div>
        </div>
        <CodeBlock title="Comandos del reto" lang="bash"
          :content="challengeCode" :copyable="true" class="q-mt-md"/>
        <q-expansion-item icon="lightbulb" label="Ver pistas"
          header-class="answer-header" class="q-mt-md">
          <div class="answer-body">
            <div v-for="h in challengeHints" :key="h" class="answer-row">
              <q-icon name="chevron_right" size="14px" style="color:#4ade80"/>
              {{ h }}
            </div>
          </div>
        </q-expansion-item>
      </div>
    </div>

    <!-- ══ VIDEO ══ -->
    <div class="section-group">
      <SectionTitle>Video Complementario</SectionTitle>
      <TextBlock>Workspaces, paquetes y colcon build en ROS 2:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="ROS 2 Workspace and Packages" frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
            allowfullscreen></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" size="16px" color="info" class="q-mr-sm"/>
          Video en revisión — será reemplazado con contenido del curso.
        </div>
      </div>
    </div>

    <!-- ══ RESUMEN ══ -->
    <div class="section-group q-mb-xl">
      <SectionTitle>Resumen — Workspace y Paquetes</SectionTitle>
      <div class="summary-grid q-mt-lg">
        <div v-for="s in summaryItems" :key="s.term" class="summary-card"
          :style="{ '--sc-color': s.color }">
          <code class="sc-term">{{ s.term }}</code>
          <div class="sc-desc">{{ s.desc }}</div>
          <div class="sc-note">{{ s.note }}</div>
        </div>
      </div>
    </div>

    <!-- ══ CTA ══ -->
    <div class="section-group q-mt-xl">
      <div class="final-cta">
        <div class="fca-icon">
          <q-icon name="topic" size="40px" color="primary"/>
        </div>
        <h2 class="fca-title">¡Workspace dominado!</h2>
        <p class="fca-sub">
          Sabes crear workspaces, paquetes C++ y Python, compilar con colcon y gestionar
          dependencias. El siguiente paso: Topics — la comunicación publish-subscribe de ROS 2.
        </p>
        <div class="fca-actions">
          <q-btn color="primary" unelevated rounded size="lg" padding="14px 40px"
            to="/modulo-4/03topicsPage"
            icon="arrow_forward" label="Topics y Mensajes"
            class="text-weight-bold"/>
        </div>
      </div>
    </div>

  </LessonContainer>
</template>

<script setup lang="ts">
import { reactive } from 'vue';
import LessonContainer from 'components/content/LessonContainer.vue';
import TextBlock from 'components/content/TextBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import DependencySolver from 'components/content/interactive/DependencySolver.vue';

// ═══════════════════════════════════════════════════
// CODE CONSTANTS
// ═══════════════════════════════════════════════════

const wsCreateCode = [
  '# 1. Crear estructura del workspace',
  'mkdir -p ~/ros2_ws/src',
  'cd ~/ros2_ws',
  '',
  '# 2. Instalar dependencias del sistema',
  'rosdep install --from-paths src --ignore-src -r -y',
  '',
  '# 3. Compilar (primera vez)',
  'colcon build --symlink-install',
  '',
  '# 4. Activar el workspace',
  'source install/setup.bash',
  '',
  '# 5. Verificar que funciona',
  'ros2 pkg list | head -20',
].join('\n');

const bashrcCode = [
  '# Agregar al final de ~/.bashrc (una sola vez)',
  'echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc',
  'echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc',
  'echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc',
  '',
  '# Recargar sin cerrar la terminal',
  'source ~/.bashrc',
  '',
  '# Verificar',
  'echo $ROS_DOMAIN_ID   # debe mostrar 42',
  'ros2 --version        # debe funcionar en cualquier terminal nueva',
].join('\n');

const packageXmlCode = [
  '<?xml version="1.0"?>',
  '<package format="3">',
  '  <name>mi_robot_pkg</name>',
  '  <version>1.0.0</version>',
  '  <description>Control de motores para TurtleBot3</description>',
  '  <maintainer email="dev@robot.com">Alexander</maintainer>',
  '  <license>Apache License 2.0</license>',
  '',
  '  <!-- Build tool (siempre requerido) -->',
  '  <buildtool_depend>ament_cmake</buildtool_depend>',
  '',
  '  <!-- Dependencias build + exec (shorthand) -->',
  '  <depend>rclcpp</depend>',
  '  <depend>std_msgs</depend>',
  '  <depend>sensor_msgs</depend>',
  '',
  '  <!-- Solo compilación (no en runtime) -->',
  '  <build_depend>geometry_msgs</build_depend>',
  '',
  '  <!-- Solo runtime (no necesaria para compilar) -->',
  '  <exec_depend>nav2_msgs</exec_depend>',
  '',
  '  <!-- Solo para tests -->',
  '  <test_depend>ament_cmake_gtest</test_depend>',
  '  <test_depend>ament_lint_auto</test_depend>',
  '',
  '  <export>',
  '    <build_type>ament_cmake</build_type>',
  '  </export>',
  '</package>',
].join('\n');

const rosdepCode = [
  '# Inicializar rosdep (solo la primera vez en el sistema)',
  'sudo rosdep init',
  'rosdep update',
  '',
  '# Instalar todas las dependencias del workspace',
  'cd ~/ros2_ws',
  'rosdep install --from-paths src --ignore-src -r -y',
  '',
  '# Dry-run: ver qué instalaría sin hacerlo',
  'rosdep install --from-paths src --ignore-src -r -y --simulate',
  '',
  '# Verificar dependencias de un paquete específico',
  'rosdep check --from-paths src/mi_paquete',
].join('\n');

const overlayCode = [
  '# Situación: quieres modificar el paquete nav2_bringup de ROS 2',
  '',
  '# 1. Source el underlay (instalación base)',
  'source /opt/ros/jazzy/setup.bash',
  '',
  '# 2. Crear overlay workspace',
  'mkdir -p ~/overlay_ws/src && cd ~/overlay_ws',
  '',
  '# 3. Clonar el paquete a modificar',
  'git clone https://github.com/ros-navigation/navigation2 src/navigation2',
  '',
  '# 4. Hacer solo el paquete que necesitas',
  'colcon build --packages-select nav2_bringup --symlink-install',
  '',
  '# 5. Source el overlay (tiene prioridad sobre el underlay)',
  'source install/setup.bash',
  '',
  '# 6. Verificar: ahora usa tu versión modificada',
  'ros2 pkg prefix nav2_bringup',
  '# /home/user/overlay_ws/install/nav2_bringup  ← tu versión',
].join('\n');

const testCmakeCode = [
  'if(BUILD_TESTING)',
  '  find_package(ament_cmake_gtest REQUIRED)',
  '',
  '  ament_add_gtest(test_motor_controller',
  '    test/test_motor_controller.cpp',
  '  )',
  '  ament_target_dependencies(test_motor_controller',
  '    rclcpp std_msgs',
  '  )',
  'endif()',
].join('\n');

const challengeCode = [
  '# Paso 1: Crear workspace',
  'mkdir -p ~/mi_primera_ws/src && cd ~/mi_primera_ws',
  '',
  '# Paso 2: Crear paquete C++',
  'cd src',
  'ros2 pkg create --build-type ament_cmake motor_controller \\',
  '  --dependencies rclcpp std_msgs geometry_msgs',
  '',
  '# Paso 3: Crear paquete Python',
  'ros2 pkg create --build-type ament_python sensor_dashboard \\',
  '  --dependencies rclpy sensor_msgs',
  '',
  '# Paso 4: Compilar todo',
  'cd ~/mi_primera_ws',
  'colcon build --symlink-install',
  '',
  '# Paso 5: Activar y verificar',
  'source install/setup.bash',
  'ros2 pkg list | grep -E "motor_controller|sensor_dashboard"',
  '',
  '# Paso 6: Ver el árbol de archivos generados',
  'tree src/motor_controller',
  'tree src/sensor_dashboard',
].join('\n');

// ═══════════════════════════════════════════════════
// DATA ARRAYS
// ═══════════════════════════════════════════════════

const facts = [
  { icon: '⚡', label: 'colcon --symlink-install: cambia un .py y ejecutas inmediatamente sin recompilar' },
  { icon: '🔗', label: 'Overlay/Underlay: puedes tener 3+ workspaces encadenados — el último sourced gana siempre' },
  { icon: '📦', label: 'ros2 pkg create genera toda la estructura en segundos — nunca crees archivos a mano' },
];

const wsFolders = [
  {
    name: 'src/',
    icon: 'folder_open',
    color: '#4ade80',
    tags: ['Tu código', 'Git'],
    desc: 'El único directorio que debes editar. Cada subdirectorio es un paquete ROS 2.',
    files: ['mi_paquete/', 'nav2_custom/', 'robot_msgs/'],
  },
  {
    name: 'build/',
    icon: 'handyman',
    color: '#f97316',
    tags: ['Auto-generado', 'No editar'],
    desc: 'Artefactos temporales: CMake cache, archivos .o, Makefiles. Puedes borrarlo sin perder código.',
    files: ['CMakeCache.txt', 'Makefile', '*.o'],
  },
  {
    name: 'install/',
    icon: 'system_update_alt',
    color: '#60a5fa',
    tags: ['Ejecutable', 'setup.bash'],
    desc: 'Resultado final de colcon build. Contiene binarios y el script setup.bash que debes sourcear.',
    files: ['setup.bash', 'local_setup.bash', 'lib/'],
  },
  {
    name: 'log/',
    icon: 'history',
    color: '#94a3b8',
    tags: ['Debugging', 'Diagnóstico'],
    desc: 'Logs de compilación y errores de colcon. Aquí buscas cuando un build falla.',
    files: ['latest_build/', 'build_*.log', 'test_*.log'],
  },
];

const pkgTypes = [
  {
    type: 'Paquete C++',
    buildType: 'ament_cmake',
    icon: 'memory',
    color: '#60a5fa',
    cmd: [
      '# Crear paquete C++ con dependencias',
      'ros2 pkg create --build-type ament_cmake mi_nodo_cpp \\',
      '  --dependencies rclcpp std_msgs geometry_msgs',
    ].join('\n'),
    files: [
      { name: 'package.xml',     role: 'Metadatos y dependencias', color: '#fbbf24', icon: 'description' },
      { name: 'CMakeLists.txt',  role: 'Configuración de build',    color: '#60a5fa', icon: 'settings' },
      { name: 'include/',        role: 'Headers públicos (.hpp)',    color: '#4ade80', icon: 'folder' },
      { name: 'src/',            role: 'Implementación (.cpp)',      color: '#4ade80', icon: 'folder' },
    ],
  },
  {
    type: 'Paquete Python',
    buildType: 'ament_python',
    icon: 'code',
    color: '#fbbf24',
    cmd: [
      '# Crear paquete Python con dependencias',
      'ros2 pkg create --build-type ament_python mi_nodo_py \\',
      '  --dependencies rclpy std_msgs sensor_msgs',
    ].join('\n'),
    files: [
      { name: 'package.xml', role: 'Metadatos y dependencias',   color: '#fbbf24', icon: 'description' },
      { name: 'setup.py',    role: 'Setuptools + entry points',  color: '#f97316', icon: 'description' },
      { name: 'setup.cfg',   role: 'Entry points de ros2 run',   color: '#f87171', icon: 'description' },
      { name: 'mi_nodo_py/', role: 'Módulos Python (.py)',        color: '#4ade80', icon: 'folder' },
    ],
  },
];

const pkgFlags = [
  { flag: '--build-type',     color: '#60a5fa', desc: 'ament_cmake (C++), ament_python (Python) o ament_cmake_python (ambos)' },
  { flag: '--dependencies',   color: '#4ade80', desc: 'Lista de dependencias → se añaden automáticamente a package.xml y CMakeLists.txt' },
  { flag: '--node-name',      color: '#fbbf24', desc: 'Crea un nodo de ejemplo con ese nombre en src/ (no necesitas escribirlo desde cero)' },
  { flag: '--library-name',   color: '#c084fc', desc: 'Crea una librería compartida además del ejecutable (patrón library + node)' },
  { flag: '--maintainer-name',color: '#f97316', desc: 'Tu nombre en los metadatos de package.xml' },
  { flag: '--license',        color: '#94a3b8', desc: 'Licencia del paquete: Apache-2.0, MIT, BSD... (default: Apache-2.0)' },
];

const buildStages = [
  {
    num: 1, title: 'Dependency Resolution', icon: 'account_tree', color: '#4ade80',
    desc: 'Lee package.xml de todos los paquetes y construye un grafo de dependencias topológicamente ordenado.',
    code: [
      '# Ver el grafo de dependencias',
      'colcon graph',
      '# Muestra: paquete A → paquete B → paquete C',
    ].join('\n'),
  },
  {
    num: 2, title: 'CMake Configuration', icon: 'settings', color: '#fbbf24',
    desc: 'Para paquetes C++: ejecuta cmake con ament_cmake macros. Genera Makefiles en build/.',
    code: [
      '# Equivalente a lo que colcon hace internamente:',
      'cmake -DCMAKE_INSTALL_PREFIX=install/mi_pkg \\',
      '      -DCMAKE_BUILD_TYPE=Release \\',
      '      src/mi_pkg',
    ].join('\n'),
  },
  {
    num: 3, title: 'Compilación Paralela', icon: 'bolt', color: '#60a5fa',
    desc: 'Compila paquetes independientes en paralelo. Paquetes con dependencias esperan a sus dependencias.',
    code: [
      '# Controlar paralelismo (default: nproc)',
      'colcon build --parallel-workers 4',
      '',
      '# Ver progreso en tiempo real',
      'colcon build --event-handlers console_direct+',
    ].join('\n'),
  },
  {
    num: 4, title: 'Installation', icon: 'system_update_alt', color: '#c084fc',
    desc: 'Copia binarios, bibliotecas y recursos a install/. Genera setup.bash para sourcing.',
    code: [
      '# Symlink install: no copia, usa enlaces simbólicos',
      '# Cambios en Python/launch files = efecto inmediato',
      'colcon build --symlink-install',
    ].join('\n'),
  },
];

const colconOpts = [
  {
    title: 'Compilación Selectiva',
    icon: 'filter_alt',
    color: '#4ade80',
    badge: 'Más usado',
    code: [
      '# Solo un paquete',
      'colcon build --packages-select mi_pkg',
      '',
      '# Paquete + sus dependencias',
      'colcon build --packages-up-to mi_pkg',
      '',
      '# Excluir paquetes pesados',
      'colcon build --packages-skip simulation_pkg',
    ].join('\n'),
  },
  {
    title: 'Build Types',
    icon: 'speed',
    color: '#fbbf24',
    badge: 'Performance',
    code: [
      '# Release: optimizado (producción)',
      'colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release',
      '',
      '# Debug: sin optimización + debug symbols',
      'colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug',
      '',
      '# RelWithDebInfo: optimizado + debug (recomendado)',
      'colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo',
    ].join('\n'),
  },
  {
    title: 'Limpieza y Diagnóstico',
    icon: 'cleaning_services',
    color: '#f87171',
    badge: 'Troubleshoot',
    code: [
      '# Limpiar todo (rebuild completo)',
      'rm -rf build/ install/ log/',
      '',
      '# Limpiar solo un paquete',
      'colcon build --packages-select mi_pkg --cmake-clean-cache',
      '',
      '# Verbose + continuar en errores',
      'colcon build --event-handlers console_cohesion+ --continue-on-error',
    ].join('\n'),
  },
];

const pkgStructures = [
  {
    lang: 'C++',
    icon: 'memory',
    color: '#60a5fa',
    buildType: 'ament_cmake',
    files: [
      { name: 'package.xml',     role: 'Metadatos',           color: '#fbbf24', type: 'file', required: true  },
      { name: 'CMakeLists.txt',  role: 'Build config',        color: '#60a5fa', type: 'file', required: true  },
      { name: 'include/pkg/',    role: 'Headers públicos',     color: '#4ade80', type: 'dir',  required: false },
      { name: 'src/',            role: 'Implementación .cpp',  color: '#4ade80', type: 'dir',  required: true  },
      { name: 'launch/',         role: 'Launch files (.py)',   color: '#c084fc', type: 'dir',  required: false },
      { name: 'config/',         role: 'Parámetros YAML',      color: '#f97316', type: 'dir',  required: false },
      { name: 'msg/ / srv/',     role: 'Definición mensajes',  color: '#f87171', type: 'dir',  required: false },
      { name: 'test/',           role: 'Unit tests (gtest)',   color: '#94a3b8', type: 'dir',  required: false },
    ],
    mainFileCode: [
      'cmake_minimum_required(VERSION 3.8)',
      'project(mi_robot_pkg)',
      '',
      'find_package(ament_cmake REQUIRED)',
      'find_package(rclcpp REQUIRED)',
      'find_package(std_msgs REQUIRED)',
      '',
      'add_executable(mi_nodo src/mi_nodo.cpp)',
      'ament_target_dependencies(mi_nodo rclcpp std_msgs)',
      '',
      'install(TARGETS mi_nodo',
      '  DESTINATION lib/${PROJECT_NAME})',
      '',
      'ament_package()',
    ].join('\n'),
  },
  {
    lang: 'Python',
    icon: 'code',
    color: '#fbbf24',
    buildType: 'ament_python',
    files: [
      { name: 'package.xml',    role: 'Metadatos',           color: '#fbbf24', type: 'file', required: true  },
      { name: 'setup.py',       role: 'Setuptools config',   color: '#f97316', type: 'file', required: true  },
      { name: 'setup.cfg',      role: 'Nombre del paquete',  color: '#f87171', type: 'file', required: true  },
      { name: 'pkg_name/',      role: 'Módulos Python',      color: '#4ade80', type: 'dir',  required: true  },
      { name: '__init__.py',    role: 'Módulo Python',       color: '#4ade80', type: 'file', required: true  },
      { name: 'launch/',        role: 'Launch files (.py)',  color: '#c084fc', type: 'dir',  required: false },
      { name: 'resource/',      role: 'Ament resource index',color: '#94a3b8', type: 'dir',  required: true  },
      { name: 'test/',          role: 'Tests (pytest)',      color: '#94a3b8', type: 'dir',  required: false },
    ],
    mainFileCode: [
      'from setuptools import setup',
      '',
      'package_name = "mi_robot_pkg_py"',
      '',
      'setup(',
      '    name=package_name,',
      '    version="1.0.0",',
      '    packages=[package_name],',
      '    install_requires=["setuptools"],',
      '    zip_safe=True,',
      '    maintainer="Alexander",',
      '    maintainer_email="dev@robot.com",',
      '    description="Sensor dashboard en Python",',
      '    license="Apache-2.0",',
      '    entry_points={',
      '        "console_scripts": [',
      '            "mi_nodo = mi_robot_pkg_py.mi_nodo:main",',
      '        ],',
      '    },',
      ')',
    ].join('\n'),
  },
];

const depTypes = [
  { tag: '<depend>',          icon: 'all_inclusive',  color: '#4ade80', desc: 'Build + exec. El más común. Úsalo cuando necesitas el paquete para compilar y para ejecutar.', example: '<depend>rclcpp</depend>' },
  { tag: '<build_depend>',    icon: 'handyman',        color: '#60a5fa', desc: 'Solo compilación (headers, libs de desarrollo). No se instala en el nodo destino si no tiene código fuente.', example: '<build_depend>geometry_msgs</build_depend>' },
  { tag: '<exec_depend>',     icon: 'play_circle',    color: '#fbbf24', desc: 'Solo runtime. Paquetes que necesitas cuando el programa corre pero no cuando compilas.', example: '<exec_depend>nav2_msgs</exec_depend>' },
  { tag: '<test_depend>',     icon: 'science',        color: '#c084fc', desc: 'Solo para tests. No se incluye en instalaciones de producción. Frameworks de test y mocks.', example: '<test_depend>ament_cmake_gtest</test_depend>' },
];

const overlayLayers = [
  { label: 'Overlay 2 — test_ws (más prioridad)', path: '~/test_ws', color: '#4ade80', priority: 'Prioridad 1 (alta)' },
  { label: 'Overlay 1 — ros2_ws (desarrollo)',    path: '~/ros2_ws', color: '#60a5fa', priority: 'Prioridad 2' },
  { label: 'Underlay — instalación base ROS 2',  path: '/opt/ros/jazzy', color: '#94a3b8', priority: 'Prioridad 3 (base)' },
];

const sourcingOrder = [
  { cmd: 'source /opt/ros/jazzy/setup.bash',         color: '#94a3b8', desc: 'Base ROS 2 — siempre primero' },
  { cmd: 'source ~/ros2_ws/install/setup.bash',      color: '#60a5fa', desc: 'Tu workspace principal' },
  { cmd: 'source ~/test_ws/install/setup.bash',      color: '#4ade80', desc: 'Workspace de testing — gana a todos' },
];

const workflowSteps = [
  {
    title: 'Editar código en src/',
    icon: 'edit',
    color: '#4ade80',
    desc: 'Modifica archivos en ~/ros2_ws/src/. Con --symlink-install, los .py tienen efecto inmediato.',
    code: '# Editar en tu IDE (VS Code recomendado)\ncode ~/ros2_ws/src/mi_paquete',
  },
  {
    title: 'Compilar paquete modificado',
    icon: 'handyman',
    color: '#fbbf24',
    desc: 'Solo recompila el paquete que cambiaste, no el workspace entero.',
    code: [
      'cd ~/ros2_ws',
      'colcon build --packages-select mi_paquete --symlink-install',
    ].join('\n'),
  },
  {
    title: 'Sourcear el workspace',
    icon: 'refresh',
    color: '#60a5fa',
    desc: 'Siempre sourcear después de compilar para que el nuevo binario sea visible.',
    code: 'source install/setup.bash',
  },
  {
    title: 'Ejecutar y probar',
    icon: 'play_circle',
    color: '#c084fc',
    desc: 'Ejecutar el nodo y verificar comportamiento con CLI o rqt.',
    code: [
      '# Ejecutar el nodo',
      'ros2 run mi_paquete mi_nodo',
      '',
      '# En otra terminal: inspeccionar',
      'ros2 topic echo /mi_topic',
      'ros2 node info /mi_nodo',
    ].join('\n'),
  },
  {
    title: 'Testear antes del commit',
    icon: 'fact_check',
    color: '#f97316',
    desc: 'Siempre correr los tests antes de commitear. colcon test integra gtest y pytest.',
    code: [
      'colcon test --packages-select mi_paquete',
      'colcon test-result --verbose',
    ].join('\n'),
  },
];

const testCmds = [
  { cmd: 'colcon test',                              color: '#4ade80', desc: 'Ejecutar todos los tests de todos los paquetes del workspace' },
  { cmd: 'colcon test --packages-select mi_pkg',    color: '#fbbf24', desc: 'Solo los tests de un paquete específico (mucho más rápido)' },
  { cmd: 'colcon test-result --verbose',             color: '#60a5fa', desc: 'Ver resultados detallados: qué tests pasaron y qué falló' },
  { cmd: 'colcon test-result --all',                 color: '#c084fc', desc: 'Ver todos los resultados incluyendo los tests que pasaron' },
];

const commonErrors = reactive([
  {
    type: 'package not found después de colcon build',
    summary: 'ros2 run mi_paquete mi_nodo falla con "Package not found"',
    color: '#f87171',
    cause: 'El workspace no está sourced, o se olvidó sourcear después de compilar. Muy común al abrir una terminal nueva.',
    code: [
      '# Verificar si el workspace está activo',
      'echo $AMENT_PREFIX_PATH',
      '# Si está vacío, el workspace no está sourced',
      '',
      '# Solución',
      'source ~/ros2_ws/install/setup.bash',
      '',
      '# Para que sea automático, agregar al .bashrc:',
      'echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc',
    ].join('\n'),
    fix: 'Sourcear install/setup.bash en la terminal actual. Para que sea permanente, agrégalo al ~/.bashrc.',
    open: false,
  },
  {
    type: 'colcon build falla con "dependency not found"',
    summary: 'CMake error: "Could not find a package configuration file for rclcpp"',
    color: '#fbbf24',
    cause: 'La dependencia no está declarada en package.xml, o no está instalada en el sistema (rosdep no se ejecutó).',
    code: [
      '# Verificar qué está instalado',
      'ros2 pkg list | grep rclcpp',
      '',
      '# Si no aparece: instalar con rosdep',
      'rosdep install --from-paths src --ignore-src -r -y',
      '',
      '# Si rosdep falla: instalar manualmente',
      'sudo apt install ros-jazzy-rclcpp',
      '',
      '# Verificar que package.xml tiene la dependencia:',
      'grep rclcpp src/mi_paquete/package.xml',
    ].join('\n'),
    fix: 'Ejecutar rosdep install desde el workspace. Si la dependencia no está en rosdep, instala el paquete apt manualmente (ros-jazzy-NOMBRE).',
    open: false,
  },
  {
    type: 'Cambios en Python no tienen efecto al ejecutar',
    summary: 'Edité mi_nodo.py pero ros2 run sigue mostrando el comportamiento anterior',
    color: '#c084fc',
    cause: 'Se compiló sin --symlink-install. colcon copió el .py a install/ y editar src/ no actualiza install/.',
    code: [
      '# Verificar si usa symlinks',
      'ls -la ~/ros2_ws/install/mi_paquete_py/lib/',
      '# Si ves "-> .../src/mi_paquete_py/...", está bien',
      '# Si ves archivos normales (no flechas), necesitas:',
      '',
      '# Solución: recompilar CON symlink-install',
      'colcon build --packages-select mi_paquete_py --symlink-install',
      '',
      '# Ahora los cambios en .py tienen efecto inmediato',
      'ros2 run mi_paquete_py mi_nodo',
    ].join('\n'),
    fix: 'Siempre usa colcon build --symlink-install durante desarrollo Python. Para C++, siempre necesitas recompilar después de cambios.',
    open: false,
  },
  {
    type: 'colcon build tarda demasiado — compila todo siempre',
    summary: 'Cada colcon build recompila todos los paquetes aunque no cambiaron',
    color: '#f97316',
    cause: 'Se está usando colcon build sin filtrar paquetes, o se borró build/ que contiene la cache de CMake.',
    code: [
      '# Compilar SOLO el paquete modificado',
      'colcon build --packages-select mi_pkg --symlink-install',
      '',
      '# Compilar paquete + sus dependencias (si cambió una lib)',
      'colcon build --packages-up-to mi_pkg',
      '',
      '# Ver qué paquetes existen en el workspace',
      'colcon list',
      '',
      '# Si borré build/: la primera compilación será lenta (normal)',
      '# Las siguientes serán rápidas por la cache CMake',
    ].join('\n'),
    fix: 'Usa --packages-select para compilar solo lo que cambiaste. Solo borra build/ cuando tengas errores de cache corrupta.',
    open: false,
  },
  {
    type: 'Overlay sourced en orden incorrecto — paquete equivocado',
    summary: 'ros2 run usa la versión del sistema, no mi versión modificada en el workspace',
    color: '#60a5fa',
    cause: 'El workspace personal se sourced antes que el workspace del sistema, o el orden en .bashrc está al revés.',
    code: [
      '# Verificar qué versión está activa',
      'ros2 pkg prefix nav2_bringup',
      '# /opt/ros/jazzy/... ← versión del sistema (malo)',
      '# /home/user/ros2_ws/... ← tu versión (bien)',
      '',
      '# El orden correcto en .bashrc:',
      'source /opt/ros/jazzy/setup.bash    # 1. Sistema (base)',
      'source ~/ros2_ws/install/setup.bash  # 2. Tu overlay (gana)',
      '',
      '# INCORRECTO (workspace sourced antes del sistema):',
      '# source ~/ros2_ws/install/setup.bash  # ← PRIMERO (mal)',
      '# source /opt/ros/jazzy/setup.bash      # ← ÚLTIMO (pisa el overlay)',
    ].join('\n'),
    fix: 'Sourcear siempre el underlay PRIMERO y el overlay DESPUÉS. El último gana. Verifica con ros2 pkg prefix PAQUETE.',
    open: false,
  },
]);

const challengeSteps = [
  { num: 1, color: '#4ade80', text: 'Crea un workspace ~/mi_primera_ws con la estructura correcta' },
  { num: 2, color: '#fbbf24', text: 'Crea un paquete C++ "motor_controller" con dependencias rclcpp, std_msgs, geometry_msgs' },
  { num: 3, color: '#60a5fa', text: 'Crea un paquete Python "sensor_dashboard" con rclpy y sensor_msgs' },
  { num: 4, color: '#c084fc', text: 'Compila ambos paquetes con --symlink-install y sourcear install/setup.bash' },
  { num: 5, color: '#f87171', text: 'Verifica con ros2 pkg list | grep y ros2 pkg prefix que ambos están activos' },
  { num: 6, color: '#f97316', text: '(Bonus) Configura el workspace en tu .bashrc para que se active automáticamente' },
];

const challengeHints = [
  'ros2 pkg create --node-name crea también un archivo de ejemplo — léelo para entender la estructura',
  'colcon list muestra todos los paquetes que colcon encuentra en src/',
  'Si el paquete no aparece en ros2 pkg list después de compilar, revisa que sourced el workspace',
  'El archivo setup.cfg en paquetes Python define el console_script — sin él, ros2 run no funciona',
  'Para ver los symlinks: ls -la install/mi_pkg/lib/ mostrará → apuntando a src/',
];

const summaryItems = [
  { term: '~/ros2_ws/src/',         desc: 'Tu código. Único directorio editable. Versionado en Git.', note: 'mkdir -p ~/ros2_ws/src', color: '#4ade80' },
  { term: 'colcon build',           desc: 'Compila todos los paquetes en orden topológico', note: '--packages-select para uno solo', color: '#fbbf24' },
  { term: '--symlink-install',      desc: 'Desarrollo Python sin recompilar. Cambios = efecto inmediato.', note: 'Solo Python y archivos estáticos', color: '#60a5fa' },
  { term: 'ros2 pkg create',        desc: 'Genera estructura de paquete C++ o Python en segundos', note: '--build-type ament_cmake / ament_python', color: '#c084fc' },
  { term: 'package.xml',            desc: 'Metadatos: versión, autor, dependencias build/exec/test', note: 'rosdep lee esto para instalar deps', color: '#f87171' },
  { term: 'rosdep install',         desc: 'Instala todas las dependencias del workspace automáticamente', note: '--from-paths src --ignore-src -r -y', color: '#f97316' },
  { term: 'Overlay/Underlay',       desc: 'El último workspace sourced tiene prioridad sobre los anteriores', note: 'source underlay ANTES del overlay', color: '#4ade80' },
  { term: 'CMAKE_BUILD_TYPE',       desc: 'Release (producción), Debug (desarrollo), RelWithDebInfo', note: 'Release es 3-5× más rápido que Debug', color: '#60a5fa' },
  { term: 'colcon test',            desc: 'Ejecuta gtest (C++) y pytest (Python) integrados en colcon', note: 'colcon test-result --verbose', color: '#c084fc' },
];
</script>

<style scoped>
/* ══════════════════════════════════════════
   BASE
══════════════════════════════════════════ */
.section-group { margin-bottom: 3.5rem; }

code {
  background: var(--bg-code); color: var(--text-code);
  padding: 2px 7px; border-radius: 5px;
  font-family: 'Fira Code', monospace; font-size: .9em;
}
.cmd-badge {
  display: inline-flex; align-items: center; justify-content: center;
  width: 28px; height: 28px; border-radius: 8px;
  font-size: .75rem; font-weight: 800; margin-right: 8px; vertical-align: middle;
}
.cmd-badge.green  { background: rgba( 74,222,128,.15); color: #4ade80; }
.cmd-badge.amber  { background: rgba(251,191, 36,.15); color: #fbbf24; }
.cmd-badge.cyan   { background: rgba( 34,211,238,.15); color: #22d3ee; }
.cmd-badge.purple { background: rgba(192,132,252,.15); color: #c084fc; }
.cmd-badge.red    { background: rgba(248,113,113,.15); color: #f87171; }

.fact-pills { display: flex; gap: 10px; flex-wrap: wrap; }
.fact-pill {
  display: flex; align-items: center; gap: 8px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 999px; padding: 7px 16px; font-size: .84rem; color: var(--text-secondary);
  transition: transform .2s;
}
.fact-pill:hover { transform: translateY(-2px); }
.fp-icon { font-size: 1rem; }

/* ══════════════════════════════════════════
   WORKSPACE TREE
══════════════════════════════════════════ */
.ws-tree { background: var(--bg-deep, #0d1117); border: 1px solid var(--border-subtle); border-radius: 16px; overflow: hidden; }
.wst-root { display: flex; align-items: center; gap: 10px; padding: 14px 20px; background: var(--bg-surface-solid); border-bottom: 1px solid var(--border-subtle); }
.wst-root-name { font-size: .95rem; font-weight: 800; background: none; padding: 0; }
.wst-root-tag  { font-size: .72rem; color: var(--text-muted); background: var(--bg-surface-hover); padding: 2px 8px; border-radius: 5px; }
.wst-folders { display: flex; flex-direction: column; gap: 0; padding: 0; }
.wst-folder {
  display: grid; grid-template-columns: 48px 1fr 200px; gap: 0; align-items: stretch;
  border-bottom: 1px solid var(--border-subtle);
  border-left: 3px solid var(--wsf-color);
  animation: wsfSlide .4s ease backwards;
  animation-delay: inherit;
  transition: background .2s;
}
.wst-folder:last-child { border-bottom: none; }
.wst-folder:hover { background: var(--bg-surface-hover); }
@keyframes wsfSlide {
  from { opacity: 0; transform: translateX(-12px); }
  to   { opacity: 1; transform: translateX(0); }
}
.wsf-icon { display: flex; align-items: center; justify-content: center; padding: 14px; border-right: 1px solid var(--border-subtle); }
.wsf-info { padding: 14px 16px; border-right: 1px solid var(--border-subtle); display: flex; flex-direction: column; gap: 6px; }
.wsf-name { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
.wsf-name code { font-size: .95rem; font-weight: 800; color: var(--wsf-color); background: none; padding: 0; }
.wsf-tags { display: flex; gap: 5px; }
.wsf-tag  { font-size: .68rem; font-weight: 700; padding: 1px 6px; border-radius: 4px; background: color-mix(in srgb, var(--wsf-color) 12%, transparent); color: var(--wsf-color); }
.wsf-desc { font-size: .8rem; color: var(--text-secondary); line-height: 1.4; }
.wsf-files { padding: 14px 16px; display: flex; flex-direction: column; gap: 4px; }
.wsf-file  { display: flex; align-items: center; gap: 6px; font-family: 'Fira Code', monospace; font-size: .74rem; color: var(--text-muted); }
.wsf-file code { background: none; padding: 0; font-size: .74rem; }

.bashrc-tip { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 3px solid #4ade80; border-radius: 12px; padding: 1.1rem 1.25rem; }
.bt-header  { display: flex; align-items: center; gap: 8px; margin-bottom: 10px; font-size: .88rem; color: var(--text-primary); }

/* ══════════════════════════════════════════
   PKG CREATE
══════════════════════════════════════════ */
.pkg-create-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 16px; }
.pkgt-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--pkgt-color); border-radius: 16px;
  padding: 1.25rem; display: flex; flex-direction: column; gap: 12px; min-width: 0;
}
.pkgtc-header { display: flex; align-items: center; gap: 12px; }
.pkgtc-icon   { width: 46px; height: 46px; border-radius: 12px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.pkgtc-type   { font-size: .92rem; font-weight: 800; color: var(--text-primary); }
.pkgtc-build  { font-family: 'Fira Code', monospace; font-size: .76rem; color: var(--pkgt-color); }
.pkgtc-files  { background: var(--bg-surface-hover); border-radius: 10px; padding: 10px 12px; display: flex; flex-direction: column; gap: 4px; }
.pkgtcf-label { font-size: .74rem; font-weight: 700; color: var(--text-muted); margin-bottom: 4px; }
.pkgtcf-item  { display: flex; align-items: center; gap: 7px; }
.pf-name { font-size: .78rem; font-weight: 600; color: var(--pf-color); background: none; padding: 0; }
.pf-role { font-size: .72rem; color: var(--text-muted); margin-left: auto; }

.pkg-flags { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.25rem; }
.pkgf-title { font-size: .88rem; font-weight: 700; color: var(--text-secondary); margin-bottom: 12px; }
.pkgf-grid  { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
.pkgf-item  { background: var(--bg-surface-hover); border-left: 3px solid var(--pkgf-color); border-radius: 8px; padding: 8px 12px; min-width: 0; }
.pkgfi-flag { display: block; font-size: .78rem; font-weight: 700; color: var(--pkgf-color); background: none; padding: 0; margin-bottom: 4px; word-break: break-all; }
.pkgfi-desc { font-size: .74rem; color: var(--text-muted); line-height: 1.4; }

/* ══════════════════════════════════════════
   BUILD PIPELINE
══════════════════════════════════════════ */
.build-pipeline { display: flex; flex-direction: column; gap: 0; }
.bps-wrapper { display: flex; flex-direction: column; align-items: stretch; gap: 0; }
.build-stage {
  display: grid; grid-template-columns: 36px 44px 1fr; gap: 14px; align-items: start;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--bs-color); border-radius: 14px;
  padding: 1.25rem; min-width: 0;
  animation: bsSlide .4s ease backwards;
  animation-delay: var(--bs-delay);
}
@keyframes bsSlide {
  from { opacity: 0; transform: translateY(16px); }
  to   { opacity: 1; transform: translateY(0); }
}
.bss-number { min-width: 36px; height: 36px; border-radius: 50%; display: flex; align-items: center; justify-content: center; font-size: .9rem; font-weight: 900; flex-shrink: 0; }
.bss-icon   { display: flex; align-items: flex-start; padding-top: 2px; }
.bss-title  { font-size: .9rem; font-weight: 700; margin-bottom: 6px; }
.bss-desc   { font-size: .82rem; color: var(--text-secondary); margin-bottom: 10px; line-height: 1.5; }
.bps-arrow  { display: flex; flex-direction: column; align-items: center; padding: 4px 0; gap: 2px; position: relative; }
.bpsa-track { width: 3px; height: 28px; background: var(--border-subtle); border-radius: 2px; position: relative; overflow: hidden; }
.bpsa-packet {
  width: 100%; height: 10px; border-radius: 2px;
  background: var(--bpa-color);
  animation: packetDown 1.5s ease infinite;
  animation-delay: inherit;
}
@keyframes packetDown {
  0%   { transform: translateY(-10px); opacity: 0; }
  20%  { opacity: 1; }
  80%  { opacity: 1; }
  100% { transform: translateY(28px); opacity: 0; }
}

.colcon-opts { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; }
.colcon-opt-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--coc-color); border-radius: 12px;
  padding: 1.1rem; display: flex; flex-direction: column; gap: 10px; min-width: 0;
}
.cocc-header { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.cocc-title  { font-size: .88rem; font-weight: 700; color: var(--text-primary); flex: 1; }
.cocc-badge  { font-size: .7rem; font-weight: 800; padding: 2px 8px; border-radius: 6px; white-space: nowrap; }

/* ══════════════════════════════════════════
   PKG STRUCTURES
══════════════════════════════════════════ */
.pkg-structs { display: grid; grid-template-columns: repeat(2, 1fr); gap: 16px; }
.pks-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--pks-color); border-radius: 16px;
  padding: 1.25rem; display: flex; flex-direction: column; gap: 12px; min-width: 0;
}
.pksc-header { display: flex; align-items: center; gap: 10px; }
.pksc-lang   { font-size: 1rem; font-weight: 800; color: var(--text-primary); flex: 1; }
.pksc-build  { font-family: 'Fira Code', monospace; font-size: .76rem; color: var(--pks-color); background: color-mix(in srgb, var(--pks-color) 10%, transparent); padding: 2px 8px; border-radius: 6px; }
.pksc-files  { display: flex; flex-direction: column; gap: 3px; background: var(--bg-surface-hover); border-radius: 10px; padding: 10px 12px; }
.pkscf-item  { display: flex; align-items: center; gap: 7px; padding: 3px 0; }
.pkscfi-left  { display: flex; align-items: center; gap: 6px; flex: 1; min-width: 0; }
.pkscfi-name  { font-size: .78rem; font-weight: 600; color: var(--pksf-color); background: none; padding: 0; }
.pkscfi-right { display: flex; align-items: center; gap: 5px; flex-shrink: 0; }
.pkscfi-role  { font-size: .7rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   DEPENDENCIES
══════════════════════════════════════════ */
.dep-types { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
.dep-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--dep-color); border-radius: 12px;
  padding: 1.1rem; display: flex; flex-direction: column; gap: 8px; min-width: 0;
}
.depc-header { display: flex; align-items: center; gap: 8px; }
.depc-tag  { font-size: .86rem; font-weight: 800; color: var(--dep-color); background: none; padding: 0; }
.depc-desc { font-size: .82rem; color: var(--text-secondary); line-height: 1.45; }
.depc-example {
  font-family: 'Fira Code', monospace; font-size: .76rem;
  padding: 6px 10px; border-radius: 6px; border: 1px solid;
}
.rosdep-box { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 3px solid #22d3ee; border-radius: 12px; padding: 1.1rem 1.25rem; display: flex; flex-direction: column; gap: 10px; }
.rb-header  { display: flex; align-items: center; gap: 8px; font-size: .88rem; color: var(--text-primary); }
.rb-desc    { font-size: .84rem; color: var(--text-secondary); line-height: 1.5; margin: 0; }

/* ══════════════════════════════════════════
   OVERLAY STACK
══════════════════════════════════════════ */
.overlay-stack { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 16px; padding: 1.5rem; }
.ovs-label { font-size: .82rem; color: var(--text-muted); text-align: center; margin-bottom: 16px; }
.ovs-layers { display: flex; flex-direction: column; gap: 6px; position: relative; }
.ovs-layer {
  display: flex; align-items: center; justify-content: space-between; gap: 12px;
  background: color-mix(in srgb, var(--ol-color) 8%, var(--bg-surface-hover));
  border: 1px solid color-mix(in srgb, var(--ol-color) 25%, transparent);
  border-left: 3px solid var(--ol-color);
  border-radius: 10px; padding: 12px 14px;
  animation: layerUp .4s ease backwards;
  animation-delay: var(--ol-delay);
}
@keyframes layerUp {
  from { opacity: 0; transform: translateY(12px); }
  to   { opacity: 1; transform: translateY(0); }
}
.ovsl-left { display: flex; align-items: center; gap: 10px; }
.ovsl-icon { width: 34px; height: 34px; border-radius: 8px; display: flex; align-items: center; justify-content: center; background: color-mix(in srgb, var(--ol-color) 15%, transparent); flex-shrink: 0; }
.ovsl-label { font-size: .84rem; font-weight: 700; }
.ovsl-path  { font-family: 'Fira Code', monospace; font-size: .76rem; color: var(--text-muted); display: block; background: none; padding: 0; }
.ovsl-priority { font-size: .72rem; font-weight: 800; padding: 3px 10px; border-radius: 999px; border: 1px solid; white-space: nowrap; }
.ovs-arrow-col { display: none; } /* visual only — represented by the ordered stack */
.ovsa-item { display: flex; align-items: center; gap: 6px; }
.ovsa-packet { width: 8px; height: 8px; border-radius: 50%; background: var(--op-color); animation: opDrop 2s ease infinite; animation-delay: inherit; }
@keyframes opDrop {
  0%  { opacity: 0; transform: translateY(-4px); }
  20% { opacity: 1; }
  80% { opacity: 1; }
  100%{ opacity: 0; transform: translateY(20px); }
}
.ovsa-text { font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--text-muted); background: none; padding: 0; }

.sourcing-order { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; padding: 1.25rem; }
.so-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 700; color: var(--text-primary); margin-bottom: 14px; }
.so-sequence { display: flex; flex-direction: column; gap: 8px; }
.so-step { display: flex; align-items: center; gap: 12px; padding: 8px 12px; background: var(--bg-surface-hover); border-radius: 8px; }
.sss-num  { width: 24px; height: 24px; border-radius: 50%; font-size: .78rem; font-weight: 800; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.sss-cmd  { font-size: .8rem; font-weight: 700; flex: 1; background: none; padding: 0; word-break: break-all; }
.sss-desc { font-size: .74rem; color: var(--text-muted); flex-shrink: 0; }

/* ══════════════════════════════════════════
   WORKFLOW
══════════════════════════════════════════ */
.workflow-steps { display: flex; flex-direction: column; gap: 0; }
.wfs-item { display: flex; gap: 14px; align-items: stretch; }
.wfsi-left { display: flex; flex-direction: column; align-items: center; gap: 0; flex-shrink: 0; width: 36px; }
.wfsi-num  { width: 36px; height: 36px; border-radius: 50%; font-size: .9rem; font-weight: 800; color: #0d1117; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.wfsi-line { flex: 1; width: 2px; background: var(--border-medium); margin: 4px 0; min-height: 24px; }
.wfsi-content {
  flex: 1; min-width: 0;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--wfs-color); border-radius: 12px;
  padding: 1rem 1.25rem; margin-bottom: 8px;
  display: flex; flex-direction: column; gap: 8px;
}
.wfsic-header { display: flex; align-items: center; gap: 8px; }
.wfsic-title  { font-size: .9rem; font-weight: 700; color: var(--text-primary); }
.wfsic-desc   { font-size: .83rem; color: var(--text-secondary); line-height: 1.5; }

.test-cmds { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
.tc-item { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 3px solid var(--tc-color); border-radius: 10px; padding: 10px 14px; display: flex; flex-direction: column; gap: 5px; }
.tci-cmd  { font-family: 'Fira Code', monospace; font-size: .78rem; font-weight: 700; color: var(--tc-color); background: none; padding: 0; }
.tci-desc { font-size: .78rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   ERROR ACCORDION
══════════════════════════════════════════ */
.error-list { display: flex; flex-direction: column; gap: 10px; }
.error-item { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-left: 3px solid var(--err-color); border-radius: 12px; overflow: hidden; }
.err-header { display: flex; align-items: center; justify-content: space-between; padding: .9rem 1.25rem; cursor: pointer; gap: 12px; transition: background .2s; }
.err-header:hover { background: var(--bg-surface-hover); }
.err-left   { display: flex; align-items: flex-start; gap: 10px; min-width: 0; }
.err-num    { min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0; font-size: .8rem; font-weight: 800; display: flex; align-items: center; justify-content: center; }
.err-type    { font-size: .82rem; font-weight: 700; color: var(--text-primary); margin-bottom: 2px; }
.err-summary { font-size: .78rem; color: var(--text-muted); }
.err-body  { padding: .9rem 1.4rem 1.1rem; border-top: 1px solid var(--border-subtle); display: flex; flex-direction: column; gap: 10px; }
.err-cause { font-size: .86rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px; }
.err-fix   { font-size: .85rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px; }

/* ══════════════════════════════════════════
   CHALLENGE
══════════════════════════════════════════ */
.challenge-box { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 20px; padding: 1.75rem; border-top: 3px solid #f59e0b; }
.challenge-header { display: flex; align-items: flex-start; gap: 1rem; flex-wrap: wrap; }
.challenge-icon { width: 52px; height: 52px; background: rgba(245,158,11,.15); border-radius: 14px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.challenge-title { font-size: 1.05rem; font-weight: 700; color: var(--text-primary); margin-bottom: 4px; }
.challenge-subtitle { font-size: .9rem; color: var(--text-secondary); }
.challenge-badge { margin-left: auto; font-size: .72rem; font-weight: 800; padding: 4px 12px; border-radius: 999px; white-space: nowrap; background: rgba(96,165,250,.12); color: #60a5fa; border: 1px solid rgba(96,165,250,.3); }
.challenge-steps { background: var(--bg-surface-hover); border-radius: 12px; padding: 1rem 1.25rem; }
.cs-title { font-size: .88rem; font-weight: 700; color: var(--text-secondary); margin-bottom: 10px; }
.cs-list  { display: flex; flex-direction: column; gap: 8px; }
.cs-item  { display: flex; align-items: flex-start; gap: 10px; }
.cs-num   { min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0; font-size: .82rem; font-weight: 800; color: #0d1117; display: flex; align-items: center; justify-content: center; }
.cs-text  { font-size: .87rem; color: var(--text-secondary); padding-top: 3px; }
:deep(.answer-header) { background: rgba(34,197,94,.08); border: 1px solid rgba(34,197,94,.25); border-radius: 10px; color: #22c55e; }
.answer-body { background: var(--bg-surface-hover); padding: 1.1rem 1.25rem; border-radius: 0 0 10px 10px; display: flex; flex-direction: column; gap: 8px; }
.answer-row  { display: flex; align-items: baseline; gap: 8px; font-size: .88rem; color: var(--text-secondary); }

/* ══════════════════════════════════════════
   VIDEO
══════════════════════════════════════════ */
.video-card { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 16px; padding: 1.25rem; overflow: hidden; }
.video-wrapper { position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; border-radius: 10px; background: #000; }
.video-wrapper iframe { position: absolute; top: 0; left: 0; width: 100%; height: 100%; }
.video-caption { display: flex; align-items: center; margin-top: 12px; font-size: .82rem; color: var(--text-muted); padding: 8px 12px; background: var(--bg-surface-hover); border-radius: 8px; }

/* ══════════════════════════════════════════
   SUMMARY
══════════════════════════════════════════ */
.summary-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; }
.summary-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid var(--sc-color); border-radius: 12px;
  padding: 1rem 1.25rem; display: flex; flex-direction: column; gap: 6px;
  transition: transform .2s;
}
.summary-card:hover { transform: translateY(-3px); }
.sc-term { display: block; font-size: .82rem; font-weight: 700; color: var(--sc-color); background: none; padding: 0; word-break: break-all; }
.sc-desc { font-size: .8rem; color: var(--text-secondary); line-height: 1.4; }
.sc-note { font-family: 'Fira Code', monospace; font-size: .7rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   CTA
══════════════════════════════════════════ */
.final-cta { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 24px; padding: 3rem 2rem; display: flex; flex-direction: column; align-items: center; gap: 1rem; text-align: center; }
.fca-icon  { width: 72px; height: 72px; background: rgba(96,165,250,.1); border-radius: 20px; display: flex; align-items: center; justify-content: center; }
.fca-title { font-size: 1.5rem; font-weight: 800; color: var(--text-primary); margin: 0; }
.fca-sub   { font-size: .95rem; color: var(--text-secondary); max-width: 520px; line-height: 1.6; margin: 0; }
.fca-actions { margin-top: .5rem; }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1024px) {
  .pkg-flags .pkgf-grid { grid-template-columns: repeat(2, 1fr); }
  .colcon-opts  { grid-template-columns: repeat(2, 1fr); }
  .summary-grid { grid-template-columns: repeat(2, 1fr); }
}
@media (max-width: 900px) {
  .pkg-create-grid { grid-template-columns: 1fr; }
  .pkg-structs     { grid-template-columns: 1fr; }
  .dep-types       { grid-template-columns: 1fr; }
  .test-cmds       { grid-template-columns: 1fr; }
  .wst-folder      { grid-template-columns: 48px 1fr; }
  .wsf-files       { display: none; }
}
@media (max-width: 768px) {
  .colcon-opts        { grid-template-columns: 1fr; }
  .pkg-flags .pkgf-grid { grid-template-columns: 1fr; }
  .so-sequence        { flex-direction: column; }
  .sss-desc           { display: none; }
}
@media (max-width: 480px) {
  .summary-grid { grid-template-columns: 1fr; }
}
</style>
