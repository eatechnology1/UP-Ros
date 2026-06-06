<template>
  <LessonContainer>

    <!-- ══════════════════════════════════════════
         INTRO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <TextBlock>
        C++ es el lenguaje de <strong>alto rendimiento</strong> en ROS 2.
        Mientras Python ejecuta línea por línea, C++ traduce tu código a instrucciones de máquina
        <em>antes</em> de ejecutar. Resultado: 10–100× más rápido, sin overhead de intérprete,
        con acceso directo a hardware. Esencial para drivers, control en tiempo real y
        procesamiento de sensores a alta frecuencia.
      </TextBlock>

      <div class="fact-pills q-mt-lg">
        <div class="fact-pill" v-for="f in facts" :key="f.label">
          <span class="fact-icon">{{ f.icon }}</span>
          <span class="fact-label">{{ f.label }}</span>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         01 PYTHON VS C++
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge green">01</span>
        Python vs C++ — Cuándo Usar Cada Uno
      </SectionTitle>

      <div class="lang-grid q-mt-lg">
        <div v-for="lang in languages" :key="lang.name" class="lang-card"
          :style="{ '--lc-color': lang.color }">
          <div class="lc-header">
            <div class="lc-icon-wrap" :style="{ background: lang.color + '18' }">
              <q-icon :name="lang.icon" size="28px" :style="{ color: lang.color }" />
            </div>
            <div>
              <div class="lc-name" :style="{ color: lang.color }">{{ lang.name }}</div>
              <div class="lc-type">{{ lang.type }}</div>
            </div>
          </div>

          <div class="lc-flow">
            <div v-for="(step, i) in lang.flow" :key="i" class="lc-flow-step">
              <div class="lcf-file">{{ step }}</div>
              <span v-if="i < lang.flow.length - 1" class="lcf-arrow">→</span>
            </div>
          </div>

          <div class="lc-use-cases">
            <div class="lcu-title">Mejor para:</div>
            <div v-for="u in lang.useCases" :key="u" class="lcu-item">
              <q-icon name="check_circle" size="13px" :style="{ color: lang.color }" />
              {{ u }}
            </div>
          </div>

          <div class="lc-pros">
            <div v-for="p in lang.pros" :key="p" class="lcp-item">
              <q-icon name="add" size="12px" color="positive" />{{ p }}
            </div>
          </div>
          <div class="lc-cons">
            <div v-for="c in lang.cons" :key="c" class="lcc-item">
              <q-icon name="remove" size="12px" color="negative" />{{ c }}
            </div>
          </div>
        </div>
      </div>

      <AlertBlock type="info" title="No es uno u otro — ROS 2 usa ambos" class="q-mt-lg">
        En un robot real, usarás Python <em>y</em> C++ juntos.
        El driver del LiDAR en C++ publica datos a 20 Hz;
        el planificador de ruta en Python los consume y toma decisiones.
        Cada lenguaje hace lo que mejor sabe.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         02 PIPELINE DE COMPILACIÓN
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge amber">02</span>
        El Pipeline de Compilación — 3 Pasos Internos
      </SectionTitle>

      <TextBlock>
        Cuando ejecutas <code>colcon build</code>, ocurren 3 etapas invisibles.
        Cada una puede fallar por razones distintas, por eso identificar el tipo de error
        es el primer paso para resolverlo.
      </TextBlock>

      <div class="pipeline q-mt-lg">
        <div v-for="(stage, i) in pipeline" :key="i" class="pp-stage-wrap">
          <div class="pp-stage" :style="{ '--pp-color': stage.color }">
            <div class="pps-num" :style="{ background: stage.color }">{{ i + 1 }}</div>
            <div class="pps-icon-wrap" :style="{ background: stage.color + '18' }">
              <q-icon :name="stage.icon" size="26px" :style="{ color: stage.color }" />
            </div>
            <div class="pps-title">{{ stage.title }}</div>
            <div class="pps-desc">{{ stage.desc }}</div>
            <div class="pps-io">
              <div class="pps-input">
                <q-icon name="input" size="12px" class="q-mr-xs" />
                <span>{{ stage.input }}</span>
              </div>
              <q-icon name="east" size="14px" style="color:var(--text-muted)" />
              <div class="pps-output">
                <q-icon name="output" size="12px" class="q-mr-xs" />
                <span>{{ stage.output }}</span>
              </div>
            </div>
            <div class="pps-errors">
              <div class="pps-err-label">Error típico:</div>
              <code class="pps-err-msg">{{ stage.error }}</code>
            </div>
          </div>
          <div v-if="i < pipeline.length - 1" class="pp-arrow">
            <q-icon name="east" size="22px" style="color:var(--text-muted)" />
          </div>
        </div>
      </div>

      <AlertBlock type="danger" title="El 90% de los errores confusos vienen del Linker" class="q-mt-lg">
        Los errores de linker son los más frustrantes: el código está sintácticamente correcto,
        el compilador pasó sin problema — pero falta una librería en <code>CMakeLists.txt</code>.
        Cuando veas <code>undefined reference to</code>, mira <code>ament_target_dependencies()</code>.
      </AlertBlock>
    </div>

    <!-- ══════════════════════════════════════════
         03 HEADERS Y CÓDIGO FUENTE
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">03</span>
        Headers (.hpp) vs Implementación (.cpp)
      </SectionTitle>

      <TextBlock>
        Una de las grandes diferencias de C++ respecto a Python: el código se divide en
        <strong>dos archivos</strong>. El header declara <em>qué existe</em>;
        el archivo de implementación define <em>cómo funciona</em>.
        Esta separación permite compilar módulos de forma independiente.
      </TextBlock>

      <div class="hpp-cpp-visual q-mt-lg">
        <div class="hc-rule">
          <q-icon name="rule" size="16px" color="primary" />
          Regla: un header por clase, un .cpp por header
        </div>
        <div class="hc-panels">
          <div class="hc-panel hc-hpp">
            <div class="hcp-header">
              <q-icon name="description" size="16px" color="warning" />
              <span>motor.hpp — DECLARACIÓN (el "contrato")</span>
            </div>
            <CodeBlock :hide-header="true" lang="cpp" :content="headerCode" />
          </div>
          <div class="hc-divider">
            <div class="hcd-line"></div>
            <div class="hcd-badge">incluido en</div>
            <q-icon name="east" size="20px" style="color:var(--text-muted)" />
            <div class="hcd-line"></div>
          </div>
          <div class="hc-panel hc-cpp">
            <div class="hcp-header">
              <q-icon name="terminal" size="16px" color="positive" />
              <span>motor.cpp — IMPLEMENTACIÓN (el "cómo")</span>
            </div>
            <CodeBlock :hide-header="true" lang="cpp" :content="implCode" />
          </div>
        </div>
      </div>

      <!-- #pragma once -->
      <div class="pragma-section q-mt-xl">
        <div class="pg-title">
          <q-icon name="shield" size="18px" color="warning" />
          <code>#pragma once</code> — El Include Guard Moderno
        </div>
        <SplitBlock>
          <template #left>
            <div class="pg-panel pg-bad">
              <div class="pgp-header">
                <q-icon name="warning" size="14px" color="warning" />
                Sin include guard — problema
              </div>
              <CodeBlock :hide-header="true" lang="cpp" :content="noGuardCode" />
              <div class="pgp-note pgp-note-bad">
                Si alguien incluye motor.hpp dos veces (indirectamente), el compilador
                ve la clase duplicada → error de redefinición.
              </div>
            </div>
          </template>
          <template #right>
            <div class="pg-panel pg-good">
              <div class="pgp-header">
                <q-icon name="check_circle" size="14px" color="positive" />
                Con #pragma once — seguro
              </div>
              <CodeBlock :hide-header="true" lang="cpp" :content="pragmaCode" />
              <div class="pgp-note pgp-note-good">
                <code>#pragma once</code> le dice al compilador: "incluye este archivo
                máximo una vez por unidad de compilación". Soportado por todos los
                compiladores modernos.
              </div>
            </div>
          </template>
        </SplitBlock>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         04 CMAKELISTS.TXT
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge purple">04</span>
        CMakeLists.txt — La Receta de Compilación
      </SectionTitle>

      <TextBlock>
        CMake lee el archivo <code>CMakeLists.txt</code> de tu paquete y genera las instrucciones
        exactas para compilar tu código con todas sus dependencias.
        Cada directiva tiene un papel específico en el proceso.
      </TextBlock>

      <CodeBlock title="CMakeLists.txt — Paquete C++ ROS 2 Completo"
        lang="cmake" :content="cmakeFullCode" :copyable="true" />

      <!-- CMake directives breakdown -->
      <div class="cmake-breakdown q-mt-xl">
        <div class="cb-title">
          <q-icon name="auto_stories" size="16px" color="primary" />
          Qué hace cada directiva
        </div>
        <div class="cb-grid">
          <div v-for="d in cmakeDirectives" :key="d.cmd" class="cb-card"
            :style="{ '--cbd-color': d.color }">
            <code class="cbd-cmd">{{ d.cmd }}</code>
            <div class="cbd-desc">{{ d.desc }}</div>
            <div v-if="d.note" class="cbd-note">
              <q-icon name="lightbulb" size="12px" color="warning" class="q-mr-xs" />
              {{ d.note }}
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         05 ESTRUCTURA DEL PAQUETE C++
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge red">05</span>
        Estructura de un Paquete C++ ROS 2
      </SectionTitle>

      <TextBlock>
        Un paquete C++ tiene una estructura diferente al Python.
        La clave está en la carpeta <code>include/</code> para headers
        y <code>src/</code> para implementaciones.
      </TextBlock>

      <!-- Create command -->
      <div class="create-cmd q-mt-lg">
        <div class="cc-header">
          <q-icon name="auto_fix_high" size="16px" color="positive" />
          Generar automáticamente
        </div>
        <CodeBlock :hide-header="true" lang="bash" :content="pkgCreateCppCode" :copyable="true" />
      </div>

      <!-- File tree -->
      <div class="file-tree q-mt-lg">
        <div class="ft-header">
          <q-icon name="account_tree" size="16px" color="primary" />
          Estructura generada
        </div>
        <div class="ft-body">
          <div v-for="(item, i) in fileTree" :key="i" class="ft-item"
            :class="{ 'ft-folder': item.type === 'folder', 'ft-critical': item.critical, 'ft-important': item.important }"
            :style="{ '--ft-color': item.color, paddingLeft: (item.level * 24 + 14) + 'px' }">
            <div class="ft-item-main">
              <q-icon :name="item.type === 'folder' ? 'folder' : 'description'"
                size="16px" :style="{ color: item.color }" />
              <span class="ft-name">{{ item.name }}</span>
              <span v-if="item.critical"  class="ft-badge ft-badge-critical">CRÍTICO</span>
              <span v-if="item.important" class="ft-badge ft-badge-important">IMPORTANTE</span>
            </div>
            <div class="ft-desc">{{ item.desc }}</div>
          </div>
        </div>
      </div>

      <!-- package.xml comparison Python vs C++ -->
      <div class="pkgxml-compare q-mt-xl">
        <div class="pxc-title">
          <q-icon name="compare" size="16px" color="primary" />
          package.xml — Python vs C++ (la diferencia clave)
        </div>
        <div class="pxc-panels">
          <div class="pxc-panel">
            <div class="pxcp-header pxcp-python">
              <q-icon name="code" size="14px" />
              Python — ament_python
            </div>
            <CodeBlock :hide-header="true" lang="xml" :content="packageXmlPythonCode" />
          </div>
          <div class="pxc-panel">
            <div class="pxcp-header pxcp-cpp">
              <q-icon name="memory" size="14px" />
              C++ — ament_cmake
            </div>
            <CodeBlock :hide-header="true" lang="xml" :content="packageXmlCppCode" />
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         06 COLCON BUILD
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>
        <span class="cmd-badge cyan">06</span>
        Colcon Build — Ciclo Completo
      </SectionTitle>

      <TextBlock>
        <code>colcon</code> orquesta el proceso completo: llama a CMake, después al compilador,
        y finalmente instala los ejecutables en la carpeta <code>install/</code> para que
        <code>ros2 run</code> los pueda encontrar.
      </TextBlock>

      <!-- Build steps -->
      <div class="build-steps q-mt-lg">
        <div v-for="(step, i) in buildSteps" :key="i" class="bs-step"
          :style="{ '--bs-color': step.color }">
          <div class="bss-num" :style="{ background: step.color }">{{ i + 1 }}</div>
          <div class="bss-body">
            <div class="bss-title">{{ step.title }}</div>
            <CodeBlock :hide-header="true" lang="bash" :content="step.cmd" :copyable="true" />
            <div v-if="step.note" class="bss-note">
              <q-icon name="info" size="13px" class="q-mr-xs" />{{ step.note }}
            </div>
          </div>
        </div>
      </div>

      <!-- Build types -->
      <div class="build-types q-mt-xl">
        <div class="bt-title">
          <q-icon name="tune" size="16px" color="primary" />
          Tipos de Compilación — CMAKE_BUILD_TYPE
        </div>
        <div class="bt-grid">
          <div v-for="bt in buildTypes" :key="bt.type" class="bt-card"
            :class="{ 'bt-recommended': bt.recommended }"
            :style="{ '--bt-color': bt.color }">
            <div class="btc-name" :style="{ color: bt.color }">{{ bt.type }}</div>
            <code class="btc-flag">{{ bt.flag }}</code>
            <div class="btc-opts">{{ bt.opts }}</div>
            <div class="btc-use">{{ bt.use }}</div>
            <div v-if="bt.recommended" class="btc-badge">Recomendado en desarrollo</div>
          </div>
        </div>
      </div>

      <!-- Useful flags -->
      <div class="colcon-flags q-mt-xl">
        <div class="cf-title">
          <q-icon name="flag" size="16px" color="primary" />
          Flags Útiles de Colcon
        </div>
        <div class="cf-grid">
          <div v-for="flag in colconFlags" :key="flag.flag" class="cf-card"
            :style="{ '--cf-color': flag.color }">
            <code class="cfc-flag">{{ flag.flag }}</code>
            <div class="cfc-desc">{{ flag.desc }}</div>
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         ERRORES COMUNES
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Diccionario de Errores de Compilación</SectionTitle>

      <div class="error-list q-mt-lg">
        <div v-for="(err, i) in commonErrors" :key="i" class="error-item"
          :style="{ '--err-color': err.color }">
          <div class="err-header" @click="err.open = !err.open">
            <div class="err-left">
              <div class="err-num" :style="{ background: err.color + '18', color: err.color }">
                {{ i + 1 }}
              </div>
              <div>
                <div class="err-type" :style="{ color: err.color }">{{ err.type }}</div>
                <code class="err-msg">{{ err.msg }}</code>
                <div class="err-summary">{{ err.summary }}</div>
              </div>
            </div>
            <q-icon :name="err.open ? 'expand_less' : 'expand_more'" size="20px"
              style="color:var(--text-muted)" />
          </div>
          <div v-show="err.open" class="err-body">
            <div class="err-cause">
              <q-icon name="search" size="14px" class="q-mr-xs" />
              <strong>Causa:</strong> {{ err.cause }}
            </div>
            <div class="err-fix">
              <q-icon name="build" size="14px" class="q-mr-xs" color="positive" />
              <div>
                <strong>Solución:</strong>
                <ol><li v-for="s in err.steps" :key="s">{{ s }}</li></ol>
              </div>
            </div>
            <CodeBlock v-if="err.code" :hide-header="true" lang="cmake" :content="err.code" />
          </div>
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         RETO PRÁCTICO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Reto — Tu Primer Paquete C++ ROS 2</SectionTitle>

      <div class="challenge-box">
        <div class="challenge-header">
          <div class="challenge-icon">
            <q-icon name="emoji_events" size="28px" color="warning" />
          </div>
          <div>
            <div class="challenge-title">Crea un nodo publicador en C++</div>
            <div class="challenge-subtitle">
              Con headers, CMakeLists.txt y el ciclo completo de compilación
            </div>
          </div>
          <div class="challenge-badge">45 min</div>
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

        <CodeBlock title="Arranque del reto" lang="bash"
          :content="challengeCode" :copyable="true" class="q-mt-md" />

        <q-expansion-item icon="lightbulb" label="Ver pistas"
          header-class="answer-header" class="q-mt-md">
          <div class="answer-body">
            <div v-for="h in challengeHints" :key="h" class="answer-row">
              <q-icon name="chevron_right" size="14px" style="color:#4ade80" />
              {{ h }}
            </div>
          </div>
        </q-expansion-item>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         VIDEO
    ══════════════════════════════════════════ -->
    <div class="section-group">
      <SectionTitle>Video Complementario</SectionTitle>
      <TextBlock>Compilación C++ y CMake en ROS 2 Jazzy:</TextBlock>
      <div class="video-card q-mt-md">
        <div class="video-wrapper">
          <iframe src="https://www.youtube.com/embed/Romc22GgusU"
            title="C++ Compilación ROS 2" frameborder="0"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
            allowfullscreen></iframe>
        </div>
        <div class="video-caption">
          <q-icon name="info" size="16px" color="info" class="q-mr-sm" />
          Video en progreso — será reemplazado con contenido del curso.
        </div>
      </div>
    </div>

    <!-- ══════════════════════════════════════════
         RESUMEN
    ══════════════════════════════════════════ -->
    <div class="section-group q-mb-xl">
      <SectionTitle>Resumen — Comandos y Conceptos Clave</SectionTitle>
      <div class="summary-grid q-mt-lg">
        <div v-for="s in summaryItems" :key="s.cmd" class="summary-card"
          :style="{ '--sc-color': s.color }">
          <code class="sc-cmd">{{ s.cmd }}</code>
          <div class="sc-desc">{{ s.desc }}</div>
          <div class="sc-example">
            <q-icon name="terminal" size="12px" class="q-mr-xs" />{{ s.example }}
          </div>
        </div>
      </div>
    </div>

  </LessonContainer>
</template>

<script setup lang="ts">
import { reactive } from 'vue';
import LessonContainer from 'components/content/LessonContainer.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SplitBlock from 'components/content/SplitBlock.vue';

// ── Fact pills ─────────────────────────────────────────────────
const facts = [
  { icon: '⚡', label: 'C++ — 10–100× más rápido que Python' },
  { icon: '🏗️', label: 'CMake — orquesta todo el proceso de compilación' },
  { icon: '📦', label: 'Colcon — compila múltiples paquetes ROS 2 a la vez' },
];

// ── Language comparison ────────────────────────────────────────
const languages = [
  {
    name: 'Python', type: 'Interpretado', icon: 'code', color: '#fbbf24',
    flow: ['script.py', 'Ejecutar directo'],
    useCases: ['Lógica de alto nivel', 'Planificación de rutas', 'Inteligencia Artificial', 'Scripts de configuración'],
    pros: ['Desarrollo rápido — sin compilación', 'Debugging interactivo', 'Librerías de IA/ML'],
    cons: ['10–100× más lento que C++', 'No apto para tiempo real', 'Errores en ejecución'],
  },
  {
    name: 'C++', type: 'Compilado', icon: 'memory', color: '#60a5fa',
    flow: ['fuente.cpp', 'Compilar', 'binario'],
    useCases: ['Drivers de sensores (LiDAR, IMU)', 'Control de motores', 'Procesamiento de imágenes 60fps+', 'Algoritmos de navegación'],
    pros: ['Máximo rendimiento', 'Errores detectados antes de correr', 'Control total de memoria'],
    cons: ['Ciclo de desarrollo más lento', 'Curva de aprendizaje empinada', 'Errores de linker crípticos'],
  },
];

// ── Compilation pipeline ───────────────────────────────────────
const pipeline = [
  {
    title: 'Preprocesador',
    desc: 'Expande todos los #include copiando el contenido de los headers. Resuelve los #define y macros.',
    icon: 'content_cut', color: '#fbbf24',
    input: '*.cpp + *.hpp',
    output: 'archivo .i (texto)',
    error: '"No such file or directory" — falta un #include',
  },
  {
    title: 'Compilador',
    desc: 'Traduce C++ a código de máquina (assembly). Verifica sintaxis, tipos y semántica. Cada .cpp se compila por separado.',
    icon: 'translate', color: '#60a5fa',
    input: 'archivo .i',
    output: 'archivo .o (binario)',
    error: '"was not declared in this scope" — variable sin declarar',
  },
  {
    title: 'Linker',
    desc: 'Une todos los archivos .o y las librerías externas (rclcpp, etc.) en el ejecutable final. Aquí ocurre el 90% de los errores.',
    icon: 'link', color: '#4ade80',
    input: '*.o + librerías',
    output: 'ejecutable binario',
    error: '"undefined reference to" — librería no enlazada',
  },
];

// ── CMake directives breakdown ─────────────────────────────────
const cmakeDirectives = [
  { cmd: 'cmake_minimum_required()', color: '#94a3b8', desc: 'Versión mínima de CMake requerida. ROS 2 Jazzy requiere 3.8+.', note: undefined },
  { cmd: 'project()', color: '#60a5fa', desc: 'Nombre del paquete. Debe coincidir exactamente con package.xml.', note: 'Genera la variable ${PROJECT_NAME} usable en el resto del archivo.' },
  { cmd: 'find_package()', color: '#fbbf24', desc: 'Busca e importa una librería. REQUIRED hace que el build falle si no la encuentra.', note: 'Un find_package por dependencia. Si falla, verifica que tienes el paquete instalado (apt).' },
  { cmd: 'add_executable()', color: '#4ade80', desc: 'Define un ejecutable y sus archivos fuente. Lista todos los .cpp que lo componen.', note: 'Si agregas un .cpp nuevo, debes añadirlo aquí y recompilar.' },
  { cmd: 'target_include_directories()', color: '#22d3ee', desc: 'Especifica dónde buscar los headers (.hpp). Necesario para que los #include funcionen.', note: undefined },
  { cmd: 'ament_target_dependencies()', color: '#c084fc', desc: 'Enlaza librerías de ROS 2 con el ejecutable. Aquí se resuelven los errores de linker.', note: 'Si ves "undefined reference to rclcpp", probablemente falta aquí.' },
  { cmd: 'install(TARGETS)', color: '#f97316', desc: 'Copia el ejecutable a install/lib/ para que ros2 run lo encuentre.', note: 'Sin esta línea, ros2 run no puede encontrar el nodo aunque compile sin errores.' },
  { cmd: 'ament_package()', color: '#f87171', desc: 'SIEMPRE al final. Genera configuración para que otros paquetes puedan depender de este.', note: undefined },
];

// ── File tree ──────────────────────────────────────────────────
const fileTree = [
  { type: 'folder', name: 'mi_robot_pkg/', level: 0, color: '#60a5fa', desc: 'Raíz del paquete C++ ROS 2' },
  { type: 'file',   name: 'package.xml',   level: 1, color: '#f97316', desc: 'Metadatos y dependencias — usa ament_cmake', important: true },
  { type: 'file',   name: 'CMakeLists.txt', level: 1, color: '#f97316', desc: 'La receta de compilación — el archivo más importante en C++', important: true },
  { type: 'folder', name: 'include/',       level: 1, color: '#fbbf24', desc: 'Carpeta de headers públicos' },
  { type: 'folder', name: 'mi_robot_pkg/',  level: 2, color: '#fbbf24', desc: 'Sub-carpeta con nombre del paquete (convención ROS 2)' },
  { type: 'file',   name: 'control_node.hpp', level: 3, color: '#fbbf24', desc: 'Header con #pragma once — declaraciones de clase', critical: true },
  { type: 'file',   name: 'motor_driver.hpp', level: 3, color: '#fbbf24', desc: 'Header del driver del motor' },
  { type: 'folder', name: 'src/',           level: 1, color: '#4ade80', desc: 'Código fuente C++ — implementaciones' },
  { type: 'file',   name: 'control_node.cpp', level: 2, color: '#4ade80', desc: 'Implementación del nodo + función main()' },
  { type: 'file',   name: 'motor_driver.cpp', level: 2, color: '#4ade80', desc: 'Implementación del driver' },
  { type: 'folder', name: 'launch/',        level: 1, color: '#c084fc', desc: 'Launch files (.py) para lanzar múltiples nodos' },
  { type: 'folder', name: 'config/',        level: 1, color: '#22d3ee', desc: 'Archivos YAML de parámetros' },
];

// ── Build types ────────────────────────────────────────────────
const buildTypes = [
  {
    type: 'Debug', color: '#f87171', recommended: false,
    flag: '-DCMAKE_BUILD_TYPE=Debug',
    opts: '-O0 -g — sin optimización, con símbolos de debug',
    use: 'Para usar GDB y breakpoints. Ejecutable ~5× más grande.',
  },
  {
    type: 'RelWithDebInfo', color: '#4ade80', recommended: true,
    flag: '-DCMAKE_BUILD_TYPE=RelWithDebInfo',
    opts: '-O2 -g — optimizado + símbolos de debug',
    use: 'El mejor balance. Debug cuando algo falla, rendimiento razonable.',
  },
  {
    type: 'Release', color: '#60a5fa', recommended: false,
    flag: '-DCMAKE_BUILD_TYPE=Release',
    opts: '-O3 — máxima optimización, sin debug',
    use: 'Producción. El ejecutable más rápido y pequeño.',
  },
];

// ── Colcon flags ───────────────────────────────────────────────
const colconFlags = [
  { flag: '--packages-select pkg', color: '#4ade80', desc: 'Solo compila el paquete especificado — mucho más rápido' },
  { flag: '--symlink-install', color: '#60a5fa', desc: 'Crea symlinks en lugar de copiar (útil para Python)' },
  { flag: '--parallel-workers N', color: '#fbbf24', desc: 'Compila N paquetes en paralelo. Usa todos los cores' },
  { flag: '--cmake-args -DCMAKE_BUILD_TYPE=Debug', color: '#c084fc', desc: 'Pasa argumentos directamente a CMake' },
  { flag: '--event-handlers console_direct+', color: '#f97316', desc: 'Muestra output del compilador en tiempo real' },
  { flag: 'rm -rf build/ install/ log/', color: '#f87171', desc: 'Limpieza completa — recompilar desde cero' },
];

// ── Build steps ────────────────────────────────────────────────
const buildSteps = [
  {
    title: 'Ir al workspace',
    cmd: 'cd ~/ros2_ws',
    color: '#94a3b8',
    note: 'Siempre ejecuta colcon desde la raíz del workspace, no desde dentro del paquete',
  },
  {
    title: 'Compilar el paquete',
    cmd: 'colcon build --packages-select mi_robot_pkg\n\n# O compilar todo el workspace:\n# colcon build',
    color: '#fbbf24',
    note: 'CMake + compilador + linker. Los archivos van a build/ y los resultados a install/',
  },
  {
    title: 'Activar el workspace (OBLIGATORIO)',
    cmd: 'source install/setup.bash',
    color: '#f97316',
    note: 'Agrega los ejecutables al PATH. Sin esto, ros2 run no encuentra tu nodo. Hazlo en CADA terminal.',
  },
  {
    title: 'Ejecutar el nodo',
    cmd: 'ros2 run mi_robot_pkg control_node\n\n# Ver output de compilación si algo falla:\n# colcon build --event-handlers console_direct+',
    color: '#4ade80',
    note: undefined,
  },
];

// ── Summary items ──────────────────────────────────────────────
const summaryItems = [
  { cmd: 'colcon build',    desc: 'Compila el workspace completo', example: 'colcon build --packages-select mi_pkg', color: '#60a5fa' },
  { cmd: '#pragma once',    desc: 'Include guard moderno — evita doble inclusión', example: 'Primera línea de cada .hpp', color: '#fbbf24' },
  { cmd: 'find_package()',  desc: 'Importa una librería de ROS 2 en CMake', example: 'find_package(rclcpp REQUIRED)', color: '#c084fc' },
  { cmd: 'add_executable()', desc: 'Define el ejecutable y sus fuentes .cpp', example: 'add_executable(nodo src/nodo.cpp)', color: '#4ade80' },
  { cmd: 'ament_target_dependencies()', desc: 'Enlaza librerías ROS 2 — resuelve linker errors', example: 'ament_target_dependencies(nodo rclcpp)', color: '#f97316' },
  { cmd: 'install(TARGETS)', desc: 'Instala el ejecutable para ros2 run', example: 'DESTINATION lib/${PROJECT_NAME}', color: '#f87171' },
];

// ── Challenge ──────────────────────────────────────────────────
const challengeSteps = [
  { num: 1, color: '#60a5fa', text: 'Crea el paquete: ros2 pkg create --build-type ament_cmake mi_cpp_pkg' },
  { num: 2, color: '#fbbf24', text: 'Crea include/mi_cpp_pkg/calculadora.hpp con #pragma once y clase Calculadora' },
  { num: 3, color: '#4ade80', text: 'Crea src/calculadora.cpp con los métodos sumar(), restar(), multiplicar()' },
  { num: 4, color: '#c084fc', text: 'Crea src/talker_node.cpp que use Calculadora e imprima resultados con RCLCPP_INFO' },
  { num: 5, color: '#f97316', text: 'Agrega los 3 archivos .cpp a add_executable() y enlaza rclcpp en CMakeLists.txt' },
  { num: 6, color: '#f87171', text: 'Compila con colcon build, source install/setup.bash, y lanza con ros2 run' },
];

const challengeHints = [
  'La clase Calculadora puede tener métodos estáticos: static double sumar(double a, double b)',
  'En talker_node.cpp: #include "mi_cpp_pkg/calculadora.hpp"',
  'RCLCPP_INFO(this->get_logger(), "Suma: %.2f", Calculadora::sumar(3.0, 4.0))',
  'Si hay "undefined reference", verifica que calculadora.cpp está en add_executable()',
  'Si hay "No such file or directory", verifica target_include_directories() en CMakeLists.txt',
];

// ── Common errors ──────────────────────────────────────────────
const commonErrors = reactive([
  {
    type: 'Error de Compilador',
    msg: "error: 'cout' was not declared in this scope",
    summary: 'Variable, función o tipo usado antes de ser declarado',
    color: '#fbbf24',
    cause: 'Falta un #include, o hay un typo en el nombre, o estás fuera del namespace correcto.',
    steps: [
      'Agrega el #include que corresponde: #include <iostream> para cout',
      'O usa el namespace completo: std::cout',
      'Lee el mensaje de error — indica el archivo y la línea exacta',
    ],
    code: '// Solución para cout:\n#include <iostream>\n// ...\nstd::cout << "valor: " << x << std::endl;\n\n// O bien:\nusing namespace std;\ncout << "valor: " << x << endl;',
    open: false,
  },
  {
    type: 'Error de Linker',
    msg: "undefined reference to 'rclcpp::Node::Node(...)'",
    summary: 'Librería no enlazada en CMakeLists.txt',
    color: '#f87171',
    cause: 'El código está sintácticamente correcto, pero el linker no puede encontrar la implementación de una función. La librería no está declarada en ament_target_dependencies().',
    steps: [
      'Identifica la librería de la función (rclcpp en este caso)',
      'Verifica que find_package(rclcpp REQUIRED) está en CMakeLists.txt',
      'Agrega la librería a ament_target_dependencies() del ejecutable que da error',
      'Recompila con colcon build',
    ],
    code: 'ament_target_dependencies(mi_nodo\n  rclcpp      # ← agrega la librería faltante aquí\n  std_msgs\n)',
    open: false,
  },
  {
    type: 'Error de CMake',
    msg: 'Could not find a package configuration file provided by "rclcpp"',
    summary: 'CMake no encuentra una librería de ROS 2 — falta source o la librería no está instalada',
    color: '#c084fc',
    cause: 'CMake no sabe dónde están las librerías de ROS 2. O no hiciste source del setup.bash, o la librería no está instalada en el sistema.',
    steps: [
      'Activa ROS 2 antes de compilar: source /opt/ros/jazzy/setup.bash',
      'Verifica que la librería está instalada: apt list --installed | grep rclcpp',
      'Instala si falta: sudo apt install ros-jazzy-rclcpp',
      'Vuelve a compilar: colcon build',
    ],
    code: '# Activar ROS 2 primero\nsource /opt/ros/jazzy/setup.bash\n\n# Si falta la librería\nsudo apt install ros-jazzy-rclcpp\n\ncolcon build --packages-select mi_pkg',
    open: false,
  },
  {
    type: 'Error de Preprocesador',
    msg: "fatal error: mi_robot_pkg/motor.hpp: No such file or directory",
    summary: 'El compilador no puede encontrar el archivo header',
    color: '#f97316',
    cause: 'El path del #include no coincide con la estructura de carpetas, o falta target_include_directories() en CMakeLists.txt.',
    steps: [
      'Verifica que el archivo .hpp existe en include/mi_robot_pkg/',
      'El #include debe ser: #include "mi_robot_pkg/motor.hpp"',
      'Agrega target_include_directories() en CMakeLists.txt apuntando a include/',
    ],
    code: 'target_include_directories(mi_nodo PUBLIC\n  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>\n  $<INSTALL_INTERFACE:include>\n)',
    open: false,
  },
  {
    type: 'Error de Linker (Install)',
    msg: 'No executable found matching command "mi_nodo"',
    summary: 'El ejecutable compiló pero ros2 run no lo encuentra',
    color: '#22d3ee',
    cause: 'Falta la directiva install(TARGETS) en CMakeLists.txt. El ejecutable existe en build/ pero no fue copiado a install/lib/ donde ros2 run busca.',
    steps: [
      'Agrega install(TARGETS) al CMakeLists.txt',
      'Recompila: colcon build --packages-select mi_pkg',
      'Haz source de nuevo: source install/setup.bash',
    ],
    code: 'install(TARGETS mi_nodo\n  DESTINATION lib/${PROJECT_NAME}\n)\n# Sin esto: ros2 run nunca encontrará el nodo',
    open: false,
  },
  {
    type: 'Error de Redefinición',
    msg: "error: redefinition of 'class MotorController'",
    summary: 'Un header fue incluido más de una vez en la misma unidad de compilación',
    color: '#94a3b8',
    cause: 'Falta el include guard (#pragma once). Si A incluye B y C, y B incluye C, el compilador ve C dos veces.',
    steps: [
      'Agrega #pragma once como PRIMERA línea de CADA archivo .hpp',
      'Es suficiente con una línea — no necesitas los #ifndef viejos',
    ],
    code: '#pragma once  // ← Primera línea del .hpp, siempre\n\nclass MotorController {\n    // ...\n};',
    open: false,
  },
]);

// ═══════════════════════════════════════════════════════════════
// CODE STRING CONSTANTS
// ═══════════════════════════════════════════════════════════════

const headerCode = [
  '#pragma once  // Include guard — incluir máximo una vez',
  '',
  '#include <string>',
  '',
  'namespace mi_robot {',
  '',
  'class MotorController {',
  'public:',
  '    explicit MotorController(int id, double max_speed);',
  '    void setSpeed(double speed);',
  '    double getSpeed() const;',
  '    bool isRunning() const;',
  '',
  'private:',
  '    int id_;',
  '    double max_speed_;',
  '    double current_speed_;',
  '    bool running_;',
  '};',
  '',
  '}  // namespace mi_robot',
].join('\n');

const implCode = [
  '#include "mi_robot_pkg/motor.hpp"',
  '#include <stdexcept>',
  '',
  'namespace mi_robot {',
  '',
  'MotorController::MotorController(int id, double max_speed)',
  '    : id_(id), max_speed_(max_speed),',
  '      current_speed_(0.0), running_(false) {}',
  '',
  'void MotorController::setSpeed(double speed) {',
  '    if (speed > max_speed_) {',
  '        throw std::out_of_range("Speed exceeds max");',
  '    }',
  '    current_speed_ = speed;',
  '    running_ = speed > 0.0;',
  '}',
  '',
  'double MotorController::getSpeed() const {',
  '    return current_speed_;',
  '}',
  '',
  '}  // namespace mi_robot',
].join('\n');

const noGuardCode = [
  '// motor.hpp SIN include guard',
  '',
  'class MotorController {',
  '    // ...',
  '};',
  '',
  '// Si A.cpp incluye sensor.hpp',
  '// y sensor.hpp incluye motor.hpp',
  '// → A.cpp ve MotorController DOS veces',
  '// → error: redefinition of class',
].join('\n');

const pragmaCode = [
  '#pragma once  // ← una línea, problema resuelto',
  '',
  'class MotorController {',
  '    // ...',
  '};',
  '',
  '// Si motor.hpp se incluye 10 veces',
  '// el compilador lo procesa UNA SOLA VEZ',
  '// → cero errores de redefinición',
].join('\n');

const cmakeFullCode = [
  'cmake_minimum_required(VERSION 3.8)',
  'project(mi_robot_pkg)',
  '',
  '# C++17 — recomendado para ROS 2 Jazzy',
  'if(NOT CMAKE_CXX_STANDARD)',
  '  set(CMAKE_CXX_STANDARD 17)',
  'endif()',
  '',
  '# Activar warnings (buena práctica)',
  'if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")',
  '  add_compile_options(-Wall -Wextra -Wpedantic)',
  'endif()',
  '',
  '# ── Paso 1: Encontrar dependencias ──────────────────────',
  'find_package(ament_cmake REQUIRED)  # Siempre necesario',
  'find_package(rclcpp REQUIRED)       # ROS 2 C++ API',
  'find_package(std_msgs REQUIRED)     # Mensajes estándar',
  '',
  '# ── Paso 2: Definir el ejecutable ───────────────────────',
  'add_executable(control_node',
  '  src/control_node.cpp',
  '  src/motor_driver.cpp',
  ')',
  '',
  '# ── Paso 3: Especificar dónde están los headers ─────────',
  'target_include_directories(control_node PUBLIC',
  '  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>',
  '  $<INSTALL_INTERFACE:include>',
  ')',
  '',
  '# ── Paso 4: Enlazar librerías ROS 2 ─────────────────────',
  'ament_target_dependencies(control_node',
  '  rclcpp',
  '  std_msgs',
  ')',
  '',
  '# ── Paso 5: Instalar para ros2 run ──────────────────────',
  'install(TARGETS control_node',
  '  DESTINATION lib/${PROJECT_NAME}',
  ')',
  '',
  '# ── Paso 6: Instalar headers (si es librería) ───────────',
  'install(DIRECTORY include/',
  '  DESTINATION include',
  ')',
  '',
  'ament_package()  # Siempre al final',
].join('\n');

const pkgCreateCppCode = [
  '# Ir a la carpeta src del workspace',
  'cd ~/ros2_ws/src',
  '',
  '# Crear paquete C++ con un nodo inicial',
  'ros2 pkg create \\',
  '  --build-type ament_cmake \\',
  '  --node-name control_node \\',
  '  mi_robot_pkg',
  '',
  '# La diferencia clave con Python:',
  '# --build-type ament_cmake (no ament_python)',
].join('\n');

const packageXmlPythonCode = [
  '<package format="3">',
  '  <name>mi_python_pkg</name>',
  '',
  '  <!-- Python usa ament_python -->',
  '  <buildtool_depend>ament_python</buildtool_depend>',
  '',
  '  <export>',
  '    <build_type>ament_python</build_type>',
  '  </export>',
  '</package>',
].join('\n');

const packageXmlCppCode = [
  '<package format="3">',
  '  <name>mi_cpp_pkg</name>',
  '',
  '  <!-- C++ usa ament_cmake -->',
  '  <buildtool_depend>ament_cmake</buildtool_depend>',
  '',
  '  <!-- Dependencias de compilación y runtime -->',
  '  <depend>rclcpp</depend>',
  '  <depend>std_msgs</depend>',
  '',
  '  <export>',
  '    <build_type>ament_cmake</build_type>',
  '  </export>',
  '</package>',
].join('\n');

const challengeCode = [
  '# 1. Crear el paquete C++',
  'cd ~/ros2_ws/src',
  'ros2 pkg create --build-type ament_cmake mi_cpp_pkg',
  '',
  '# 2. Crear los archivos',
  'mkdir -p mi_cpp_pkg/include/mi_cpp_pkg',
  'touch mi_cpp_pkg/include/mi_cpp_pkg/calculadora.hpp',
  'touch mi_cpp_pkg/src/calculadora.cpp',
  '',
  '# 3. Compilar y ejecutar',
  'cd ~/ros2_ws',
  'colcon build --packages-select mi_cpp_pkg',
  'source install/setup.bash',
  'ros2 run mi_cpp_pkg talker_node',
].join('\n');
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
.cmd-badge.cyan   { background: rgba( 34,211,238,.15); color: #22d3ee; }
.cmd-badge.amber  { background: rgba(251,191, 36,.15); color: #fbbf24; }
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
.fact-icon { font-size: 1rem; }

/* ══════════════════════════════════════════
   LANGUAGE COMPARISON
══════════════════════════════════════════ */
.lang-grid { display: grid; grid-template-columns: repeat(2,1fr); gap: 16px; }
.lang-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 4px solid var(--lc-color, var(--border-medium));
  border-radius: 16px; padding: 1.5rem; display: flex; flex-direction: column; gap: 12px;
  transition: all .25s;
}
.lang-card:hover { transform: translateY(-4px); box-shadow: var(--shadow-md); }
.lc-header { display: flex; align-items: center; gap: 12px; }
.lc-icon-wrap { width: 48px; height: 48px; border-radius: 12px; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.lc-name { font-size: 1.3rem; font-weight: 800; }
.lc-type { font-size: .8rem; color: var(--text-muted); font-style: italic; }
.lc-flow {
  display: flex; align-items: center; gap: 6px; flex-wrap: wrap;
  background: var(--bg-surface-hover); border-radius: 8px; padding: 8px 12px;
}
.lcf-file { font-family: 'Fira Code', monospace; font-size: .82rem; color: var(--text-secondary); }
.lcf-arrow { color: var(--text-muted); font-size: .9rem; }
.lc-use-cases { display: flex; flex-direction: column; gap: 5px; }
.lcu-title { font-size: .8rem; font-weight: 700; color: var(--text-muted); margin-bottom: 3px; text-transform: uppercase; letter-spacing: .05em; }
.lcu-item { display: flex; align-items: center; gap: 6px; font-size: .84rem; color: var(--text-secondary); }
.lc-pros, .lc-cons { display: flex; flex-direction: column; gap: 4px; }
.lcp-item, .lcc-item {
  display: flex; align-items: center; gap: 5px; font-size: .8rem; color: var(--text-secondary);
}

/* ══════════════════════════════════════════
   PIPELINE
══════════════════════════════════════════ */
.pipeline {
  display: flex; align-items: stretch; gap: 8px;
}
.pp-stage-wrap { display: flex; align-items: center; gap: 8px; flex: 1; min-width: 0; }
.pp-stage {
  flex: 1; background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--pp-color, var(--border-medium));
  border-radius: 14px; padding: 1.25rem; display: flex; flex-direction: column; gap: 10px;
  min-width: 0; transition: all .25s;
}
.pp-stage:hover { box-shadow: var(--shadow-md); transform: translateY(-3px); }
.pps-num {
  width: 36px; height: 36px; border-radius: 50%; display: flex; align-items: center; justify-content: center;
  font-size: 1.1rem; font-weight: 800; color: #1e1e1e; align-self: flex-start;
}
.pps-icon-wrap { width: 44px; height: 44px; border-radius: 10px; display: flex; align-items: center; justify-content: center; }
.pps-title { font-size: 1rem; font-weight: 700; color: var(--text-primary); }
.pps-desc  { font-size: .82rem; color: var(--text-secondary); line-height: 1.5; }
.pps-io {
  display: flex; align-items: center; gap: 6px; font-size: .75rem;
  background: var(--bg-surface-hover); border-radius: 7px; padding: 6px 10px;
}
.pps-input, .pps-output { display: flex; align-items: center; color: var(--text-muted); }
.pps-errors {
  background: rgba(239,68,68,.08); border: 1px solid rgba(239,68,68,.2);
  border-radius: 7px; padding: 7px 10px;
}
.pps-err-label { font-size: .72rem; font-weight: 700; color: #f87171; margin-bottom: 3px; }
.pps-err-msg   { font-family: 'Fira Code', monospace; font-size: .76rem; color: #fca5a5; background: none; padding: 0; word-break: break-word; }
.pp-arrow { flex-shrink: 0; }

/* ══════════════════════════════════════════
   HEADERS VS IMPLEMENTATION
══════════════════════════════════════════ */
.hpp-cpp-visual { }
.hc-rule {
  display: flex; align-items: center; gap: 8px; font-size: .88rem; font-weight: 600;
  color: var(--text-secondary); margin-bottom: 12px;
}
.hc-panels { display: grid; grid-template-columns: 1fr auto 1fr; gap: 10px; align-items: start; }
.hc-panel { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; min-width: 0; }
.hc-hpp { border-top: 3px solid #fbbf24; }
.hc-cpp { border-top: 3px solid #4ade80; }
.hcp-header {
  display: flex; align-items: center; gap: 8px; padding: 10px 14px;
  border-bottom: 1px solid var(--border-subtle); font-size: .84rem; font-weight: 700;
  color: var(--text-secondary);
}
.hc-hpp .hcp-header { background: rgba(251,191,36,.08); }
.hc-cpp .hcp-header { background: rgba( 74,222,128,.08); }
.hc-divider {
  display: flex; flex-direction: column; align-items: center; gap: 6px;
  padding-top: 3rem; flex-shrink: 0; font-size: .72rem; color: var(--text-muted); white-space: nowrap;
}
.hcd-line { width: 1px; height: 20px; background: var(--border-subtle); }
.hcd-badge { background: var(--bg-surface-hover); border: 1px solid var(--border-subtle); border-radius: 999px; padding: 2px 8px; }

/* pragma once */
.pragma-section {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid #fbbf24; border-radius: 14px; padding: 1.5rem;
}
.pg-title {
  display: flex; align-items: center; gap: 8px; font-size: 1rem; font-weight: 700;
  color: var(--text-primary); margin-bottom: 1rem;
}
.pg-panel { height: 100%; display: flex; flex-direction: column; gap: 8px; }
.pgp-header {
  display: flex; align-items: center; gap: 7px; font-size: .84rem; font-weight: 700;
  padding: 7px 12px; border-radius: 8px;
}
.pg-bad  .pgp-header { background: rgba(251,191,36,.08); color: #fbbf24; }
.pg-good .pgp-header { background: rgba( 74,222,128,.08); color: #4ade80; }
.pgp-note {
  font-size: .82rem; color: var(--text-secondary); padding: 8px 12px;
  border-radius: 8px; border: 1px solid var(--border-subtle); line-height: 1.4;
}
.pgp-note-bad  { background: rgba(251,191,36,.04); }
.pgp-note-good { background: rgba( 74,222,128,.04); }

/* ══════════════════════════════════════════
   CMAKE BREAKDOWN
══════════════════════════════════════════ */
.cmake-breakdown { }
.cb-title {
  display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600;
  color: var(--text-secondary); margin-bottom: 12px;
}
.cb-grid { display: grid; grid-template-columns: repeat(2,1fr); gap: 12px; }
.cb-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--cbd-color, var(--border-medium));
  border-radius: 10px; padding: 12px 14px; display: flex; flex-direction: column; gap: 5px;
}
.cbd-cmd  { font-family: 'Fira Code', monospace; font-size: .88rem; font-weight: 700; color: var(--cbd-color); background: none; padding: 0; }
.cbd-desc { font-size: .82rem; color: var(--text-secondary); line-height: 1.4; }
.cbd-note { font-size: .78rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 6px; padding: 5px 9px; }

/* ══════════════════════════════════════════
   FILE TREE
══════════════════════════════════════════ */
.create-cmd { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; }
.cc-header {
  display: flex; align-items: center; gap: 8px; padding: 10px 14px;
  background: rgba(74,222,128,.08); border-bottom: 1px solid var(--border-subtle);
  font-size: .86rem; font-weight: 700; color: #4ade80;
}
.file-tree { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 14px; overflow: hidden; }
.ft-header {
  display: flex; align-items: center; gap: 8px; padding: 12px 16px;
  border-bottom: 1px solid var(--border-subtle); font-size: .88rem; font-weight: 600;
  color: var(--text-secondary); background: var(--bg-surface-solid);
}
.ft-body { padding: 6px 0; }
.ft-item {
  display: flex; align-items: center; padding: 7px 14px 7px;
  border-bottom: 1px solid var(--border-subtle); gap: 8px; transition: background .15s;
}
.ft-item:last-child { border-bottom: none; }
.ft-item:hover { background: var(--bg-surface-hover); }
.ft-folder { font-weight: 700; }
.ft-critical { background: rgba(251,191,36,.04); }
.ft-important { background: rgba(249,115,22,.04); }
.ft-item-main { display: flex; align-items: center; gap: 7px; min-width: 240px; flex-shrink: 0; }
.ft-name { font-family: 'Fira Code', monospace; font-size: .85rem; color: var(--text-primary); }
.ft-badge {
  font-size: .63rem; font-weight: 800; letter-spacing: .07em;
  border-radius: 999px; padding: 1px 7px; border: 1px solid;
}
.ft-badge-critical { background: rgba(251,191,36,.15); color: #fbbf24; border-color: rgba(251,191,36,.35); }
.ft-badge-important { background: rgba(249,115,22,.12); color: #f97316; border-color: rgba(249,115,22,.3); }
.ft-desc { font-size: .78rem; color: var(--text-muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }

/* package.xml compare */
.pkgxml-compare { }
.pxc-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.pxc-panels { display: grid; grid-template-columns: repeat(2,1fr); gap: 14px; }
.pxc-panel { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 12px; overflow: hidden; min-width: 0; }
.pxcp-header {
  display: flex; align-items: center; gap: 8px; padding: 10px 14px;
  border-bottom: 1px solid var(--border-subtle); font-size: .85rem; font-weight: 700;
}
.pxcp-python { background: rgba(251,191,36,.1); color: #fbbf24; }
.pxcp-cpp    { background: rgba( 96,165,250,.1); color: #60a5fa; }

/* ══════════════════════════════════════════
   BUILD STEPS
══════════════════════════════════════════ */
.build-steps { display: flex; flex-direction: column; gap: 8px; }
.bs-step {
  display: flex; align-items: flex-start; gap: 14px;
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--bs-color, var(--border-medium));
  border-radius: 12px; padding: 1.1rem; min-width: 0;
}
.bss-num {
  min-width: 34px; width: 34px; height: 34px; border-radius: 50%; flex-shrink: 0;
  display: flex; align-items: center; justify-content: center;
  font-size: .95rem; font-weight: 800; color: #1e1e1e;
}
.bss-body { flex: 1; min-width: 0; display: flex; flex-direction: column; gap: 7px; }
.bss-title { font-size: .9rem; font-weight: 700; color: var(--text-primary); }
.bss-note  { display: flex; align-items: flex-start; gap: 5px; font-size: .8rem; color: var(--text-muted); background: var(--bg-surface-hover); border-radius: 7px; padding: 6px 10px; }

/* Build types */
.build-types { }
.bt-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.bt-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 12px; }
.bt-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-top: 3px solid var(--bt-color, var(--border-medium));
  border-radius: 12px; padding: 1rem 1.1rem;
  display: flex; flex-direction: column; gap: 7px; transition: all .2s; position: relative;
}
.bt-card.bt-recommended { box-shadow: 0 0 0 2px color-mix(in srgb, var(--bt-color) 30%, transparent); }
.btc-name  { font-size: 1.1rem; font-weight: 800; }
.btc-flag  { font-family: 'Fira Code', monospace; font-size: .76rem; color: var(--text-muted); background: none; padding: 0; word-break: break-all; }
.btc-opts  { font-size: .8rem; color: var(--text-secondary); background: var(--bg-code); border-radius: 6px; padding: 5px 9px; font-family: 'Fira Code', monospace; }
.btc-use   { font-size: .82rem; color: var(--text-secondary); line-height: 1.4; }
.btc-badge {
  font-size: .68rem; font-weight: 800; letter-spacing: .06em;
  background: color-mix(in srgb, var(--bt-color) 15%, transparent);
  color: var(--bt-color); border-radius: 999px; padding: 2px 10px; text-align: center;
}

/* Colcon flags */
.colcon-flags { }
.cf-title { display: flex; align-items: center; gap: 8px; font-size: .9rem; font-weight: 600; color: var(--text-secondary); margin-bottom: 12px; }
.cf-grid { display: grid; grid-template-columns: repeat(2,1fr); gap: 10px; }
.cf-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--cf-color, var(--border-medium));
  border-radius: 10px; padding: 10px 14px; display: flex; flex-direction: column; gap: 5px; min-width: 0;
}
.cfc-flag { font-family: 'Fira Code', monospace; font-size: .82rem; font-weight: 700; color: var(--cf-color); background: none; padding: 0; word-break: break-all; }
.cfc-desc { font-size: .8rem; color: var(--text-secondary); line-height: 1.4; }

/* ══════════════════════════════════════════
   ERROR LIST
══════════════════════════════════════════ */
.error-list { display: flex; flex-direction: column; gap: 10px; }
.error-item {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 3px solid var(--err-color, #ef4444);
  border-radius: 12px; overflow: hidden;
}
.err-header {
  display: flex; align-items: center; justify-content: space-between;
  padding: .9rem 1.25rem; cursor: pointer; gap: 12px; transition: background .2s;
}
.err-header:hover { background: var(--bg-surface-hover); }
.err-left { display: flex; align-items: flex-start; gap: 10px; min-width: 0; }
.err-num {
  min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0;
  font-size: .8rem; font-weight: 800; display: flex; align-items: center; justify-content: center;
}
.err-type    { font-size: .7rem; font-weight: 700; letter-spacing: .05em; text-transform: uppercase; margin-bottom: 1px; }
.err-msg     { font-size: .82rem; display: block; margin-bottom: 2px; color: #f87171; background: none; padding: 0; word-break: break-all; }
.err-summary { font-size: .78rem; color: var(--text-muted); }
.err-body {
  padding: .9rem 1.4rem 1.1rem; border-top: 1px solid var(--border-subtle);
  display: flex; flex-direction: column; gap: 10px;
}
.err-cause, .err-fix {
  font-size: .86rem; color: var(--text-secondary); display: flex; align-items: flex-start; gap: 6px;
}
.err-fix ol { margin: 4px 0 0 14px; }
.err-fix li { margin-bottom: 4px; }

/* ══════════════════════════════════════════
   CHALLENGE BOX
══════════════════════════════════════════ */
.challenge-box {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-radius: 20px; padding: 1.75rem; border-top: 3px solid #f59e0b;
}
.challenge-header { display: flex; align-items: flex-start; gap: 1rem; flex-wrap: wrap; }
.challenge-icon {
  width: 52px; height: 52px; background: rgba(245,158,11,.15); border-radius: 14px;
  display: flex; align-items: center; justify-content: center; flex-shrink: 0;
}
.challenge-title    { font-size: 1.05rem; font-weight: 700; color: var(--text-primary); margin-bottom: 4px; }
.challenge-subtitle { font-size: .9rem; color: var(--text-secondary); }
.challenge-badge {
  margin-left: auto; font-size: .72rem; font-weight: 800; letter-spacing: .07em;
  padding: 4px 12px; border-radius: 999px; white-space: nowrap;
  background: rgba(96,165,250,.12); color: #60a5fa; border: 1px solid rgba(96,165,250,.3);
}
.challenge-steps { background: var(--bg-surface-hover); border-radius: 12px; padding: 1rem 1.25rem; }
.cs-title { font-size: .88rem; font-weight: 700; color: var(--text-secondary); margin-bottom: 10px; }
.cs-list { display: flex; flex-direction: column; gap: 8px; }
.cs-item { display: flex; align-items: flex-start; gap: 10px; }
.cs-num {
  min-width: 26px; width: 26px; height: 26px; border-radius: 50%; flex-shrink: 0;
  font-size: .82rem; font-weight: 800; color: #1e1e1e;
  display: flex; align-items: center; justify-content: center;
}
.cs-text { font-size: .87rem; color: var(--text-secondary); padding-top: 3px; }
:deep(.answer-header) {
  background: rgba(34,197,94,.08); border: 1px solid rgba(34,197,94,.25);
  border-radius: 10px; color: #22c55e;
}
.answer-body {
  background: var(--bg-surface-hover); padding: 1.1rem 1.25rem;
  border-radius: 0 0 10px 10px; display: flex; flex-direction: column; gap: 8px;
}
.answer-row { display: flex; align-items: baseline; gap: 8px; font-size: .88rem; color: var(--text-secondary); }

/* ══════════════════════════════════════════
   VIDEO CARD
══════════════════════════════════════════ */
.video-card { background: var(--bg-surface); border: 1px solid var(--border-subtle); border-radius: 16px; padding: 1.25rem; overflow: hidden; }
.video-wrapper { position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; border-radius: 10px; background: #000; }
.video-wrapper iframe { position: absolute; top: 0; left: 0; width: 100%; height: 100%; }
.video-caption { display: flex; align-items: center; margin-top: 12px; font-size: .82rem; color: var(--text-muted); padding: 8px 12px; background: var(--bg-surface-hover); border-radius: 8px; }

/* ══════════════════════════════════════════
   SUMMARY GRID
══════════════════════════════════════════ */
.summary-grid { display: grid; grid-template-columns: repeat(3,1fr); gap: 14px; }
.summary-card {
  background: var(--bg-surface); border: 1px solid var(--border-subtle);
  border-left: 4px solid var(--sc-color, var(--border-medium));
  border-radius: 12px; padding: 1rem 1.25rem; transition: all .25s;
}
.summary-card:hover { transform: translateY(-3px); box-shadow: var(--shadow-sm); }
.sc-cmd     { display: block; font-family: 'Fira Code', monospace; font-size: .9rem; font-weight: 700; color: var(--sc-color); background: none; padding: 0; margin-bottom: 5px; }
.sc-desc    { font-size: .82rem; color: var(--text-secondary); margin-bottom: 6px; line-height: 1.4; }
.sc-example { display: flex; align-items: center; font-family: 'Fira Code', monospace; font-size: .72rem; color: var(--text-muted); }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 1024px) {
  .ft-desc { display: none; }
}
@media (max-width: 900px) {
  .lang-grid     { grid-template-columns: 1fr; }
  .pipeline      { flex-direction: column; }
  .pp-arrow      { transform: rotate(90deg); align-self: center; }
  .hc-panels     { grid-template-columns: 1fr; }
  .hc-divider    { flex-direction: row; padding-top: 0; justify-content: center; }
  .hcd-line      { width: 20px; height: 1px; }
  .cb-grid       { grid-template-columns: 1fr; }
  .bt-grid       { grid-template-columns: 1fr; }
  .pxc-panels    { grid-template-columns: 1fr; }
  .cf-grid       { grid-template-columns: 1fr; }
  .summary-grid  { grid-template-columns: repeat(2,1fr); }
}
@media (max-width: 768px) {
  .challenge-header { flex-direction: column; }
  .challenge-badge  { margin-left: 0; }
  .fact-pills       { flex-direction: column; gap: 8px; }
  .ft-item-main     { min-width: unset; }
}
@media (max-width: 480px) {
  .summary-grid { grid-template-columns: 1fr; }
  .bt-grid      { grid-template-columns: 1fr; }
}
</style>
