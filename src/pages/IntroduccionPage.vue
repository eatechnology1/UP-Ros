<template>
  <q-page class="intro-page q-pa-lg column items-center">
    <!-- ========== SECCIÓN 1: HERO ÉPICO ========== -->
    <section class="hero-epic self-stretch">
      <!-- Fondo animado de nodos conectándose -->
      <div class="animated-nodes"></div>

      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm animate-fade-in">
          UP-Ros Academy
        </div>
        <h1 class="hero-title animate-slide-up">
          Domina la Robótica Profesional con <span class="text-gradient">ROS 2</span>
        </h1>
        <p class="hero-subtitle animate-slide-up-delay">
          Del "Hello World" al despliegue de flotas autónomas.<br />
          <strong>Sin dolor, sin PDF aburridos.</strong>
        </p>
      </div>
    </section>

    <!-- ========== SECCIÓN 2: ¿QUÉ ES ROS 2? (LA METÁFORA) ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>🧠 ¿Qué es ROS 2?</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            ROS 2 es el <strong class="text-primary">Sistema Nervioso</strong> de los robots
            modernos. No es un sistema operativo, es el <em>pegamento inteligente</em> que une:
          </TextBlock>

          <div class="metaphor-list q-mt-md">
            <div class="metaphor-item">
              <q-icon name="settings" size="sm" color="cyan" />
              <span><strong>Los Músculos</strong> (Motores y actuadores)</span>
            </div>
            <div class="metaphor-item">
              <q-icon name="visibility" size="sm" color="purple" />
              <span><strong>Los Sentidos</strong> (Cámaras, LIDAR, IMU)</span>
            </div>
            <div class="metaphor-item">
              <q-icon name="psychology" size="sm" color="amber" />
              <span><strong>El Cerebro</strong> (Algoritmos de navegación y IA)</span>
            </div>
          </div>

          <AlertBlock title="💡 La Clave" type="info" class="q-mt-md">
            ROS 2 permite que cada componente sea un <strong>programa independiente</strong> que se
            comunica con los demás. Si la cámara falla, las ruedas siguen funcionando.
          </AlertBlock>
        </template>

        <template #right>
          <!-- Visualización animada de componentes conectándose -->
          <div class="robot-nervous-system">
            <div class="component-node camera" data-label="Cámara">
              <q-icon name="videocam" size="lg" />
            </div>
            <div class="component-node lidar" data-label="LIDAR">
              <q-icon name="radar" size="lg" />
            </div>
            <div class="component-node wheel" data-label="Ruedas">
              <q-icon name="settings" size="lg" />
            </div>

            <div class="central-brain">
              <q-icon name="hub" size="xl" color="primary" />
              <div class="brain-label">ROS 2</div>
            </div>

            <!-- Líneas de conexión animadas -->
            <!-- viewBox 0 0 100 100 == unidades porcentuales del contenedor -->
            <svg class="connection-lines" viewBox="0 0 100 100" preserveAspectRatio="none">
              <!-- Cámara (top-left ~20%,15%) → cerebro (50%,50%) -->
              <line x1="20" y1="15" x2="50" y2="50" class="connection-line line-1" />
              <!-- LIDAR (top-right ~80%,15%) → cerebro -->
              <line x1="80" y1="15" x2="50" y2="50" class="connection-line line-2" />
              <!-- Rueda (bottom-center ~50%,87%) → cerebro -->
              <line x1="50" y1="87" x2="50" y2="50" class="connection-line line-3" />
            </svg>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- ========== SECCIÓN 3: EL "GRAN FILTRO" (SISTEMA OPERATIVO) ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>🐧 El "Gran Filtro": Tu Sistema Operativo</SectionTitle>

      <TextBlock class="q-mb-md">
        ROS 2 Jazzy requiere <strong>Linux</strong>. No es negociable, pero tienes opciones según tu
        nivel de experiencia:
      </TextBlock>

      <AlertBlock title="⚠️ Advertencia sobre WSL2" type="warning" class="q-mb-lg">
        Windows Subsystem for Linux (WSL2) <strong>funciona</strong>, pero requiere configuración
        adicional para gráficos (WSLg o X Server). Si eres principiante, usa una VM o dual boot.
      </AlertBlock>

      <!-- Tabs para los 3 caminos -->
      <q-card class="os-setup-card">
        <q-tabs
          v-model="selectedOsTab"
          dense
          :class="isDark ? 'text-grey-3' : 'text-grey-7'"
          active-color="primary"
          indicator-color="primary"
          align="justify"
        >
          <q-tab name="native" class="q-px-lg">
            <div class="column items-center relative-position q-pt-xs">
              <q-icon name="speed" size="24px" class="q-mb-xs" />
              <div class="text-weight-medium">Nativo (Recomendado)</div>

              <q-badge color="positive" floating style="top: -1px; right: 7px">
                Mejor Rendimiento
              </q-badge>
            </div>
          </q-tab>
          <q-tab name="docker" label="Docker (Avanzado)" icon="widgets" />
          <q-tab name="vm" label="Máquina Virtual (Seguro)" icon="computer" />
        </q-tabs>

        <q-separator />

        <q-tab-panels v-model="selectedOsTab" animated class="bg-transparent">
          <!-- Panel Nativo -->
          <q-tab-panel name="native">
            <div class="text-h6 panel-title q-mb-sm">
              <q-icon name="rocket" color="positive" /> Ubuntu 24.04 LTS (Nativo)
            </div>
            <TextBlock>
              Instala Ubuntu directamente en tu máquina (dual boot) o como sistema principal.
              <strong>Máximo rendimiento</strong> para simulación y desarrollo.
            </TextBlock>
            <StepsBlock
              :steps="[
                'Descarga Ubuntu 24.04 LTS desde ubuntu.com',
                'Crea un USB booteable con Rufus o Etcher',
                'Particiona tu disco (mínimo 50GB para ROS 2)',
                'Instala y configura GRUB para dual boot',
              ]"
            />
          </q-tab-panel>

          <!-- Panel Docker -->
          <q-tab-panel name="docker">
            <div class="text-h6 panel-title q-mb-sm">
              <q-icon name="widgets" color="info" /> Docker (Para usuarios avanzados)
            </div>
            <TextBlock>
              Usa contenedores Docker para aislar ROS 2 de tu sistema host. Ideal si ya dominas
              Docker y quieres portabilidad.
            </TextBlock>
            <AlertBlock title="📋 Requisitos" type="info">
              • Docker Desktop instalado<br />
              • Conocimientos de CLI de Docker<br />
              • GPU passthrough configurado (para Gazebo)
            </AlertBlock>
            <CodeBlock
              title="Ejemplo de Dockerfile"
              lang="dockerfile"
              :content="dockerExample"
              :copyable="true"
              class="q-mt-md"
            />
          </q-tab-panel>

          <!-- Panel VM -->
          <q-tab-panel name="vm">
            <div class="text-h6 panel-title q-mb-sm">
              <q-icon name="computer" color="warning" /> Máquina Virtual (Principiantes)
            </div>
            <TextBlock>
              Ejecuta Ubuntu dentro de tu sistema operativo actual usando VirtualBox o VMware.
              <strong>Seguro</strong> pero con menor rendimiento gráfico.
            </TextBlock>
            <StepsBlock
              :steps="[
                'Instala VirtualBox o VMware Workstation Player',
                'Descarga Ubuntu 24.04 ISO',
                'Crea VM con 4GB RAM y 50GB disco',
                'Habilita aceleración 3D en configuración',
                'Instala Guest Additions para mejor rendimiento',
              ]"
            />
          </q-tab-panel>
        </q-tab-panels>
      </q-card>
    </div>

    <!-- ========== SECCIÓN 4: ROADMAP INTERACTIVO ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>🗺️ Tu Roadmap de Aprendizaje</SectionTitle>

      <TextBlock class="q-mb-lg text-center">
        Este es el camino que recorrerás: desde la terminal de Linux hasta robots autónomos
        navegando en simulación.
      </TextBlock>

      <!-- Timeline Vertical -->
      <div class="roadmap-timeline">
        <div
          v-for="(module, index) in roadmapModules"
          :key="index"
          class="timeline-item"
          :class="{ 'timeline-item-even': index % 2 === 1 }"
        >
          <div class="timeline-marker" :style="{ backgroundColor: getModuleColor(index) }">
            <q-icon :name="getModuleIcon(index)" size="md" color="white" />
          </div>

          <div class="timeline-content">
            <div class="timeline-card" :style="{ borderLeftColor: getModuleColor(index) }">
              <div class="timeline-header">
                <q-icon :name="getModuleIcon(index)" size="sm" :color="getModuleColor(index)" />
                <h3>{{ module.title }}</h3>
              </div>
              <p class="timeline-desc">{{ module.description }}</p>
              <div class="timeline-meta">
                <q-chip size="sm" :color="getModuleColor(index)" text-color="white">
                  {{ module.lessons }} lecciones
                </q-chip>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- ========== SECCIÓN 5: PRE-FLIGHT CHECK ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>✈️ Pre-Flight Check</SectionTitle>

      <TextBlock class="q-mb-md text-center">
        Antes de despegar, asegúrate de cumplir con estos requisitos mínimos:
      </TextBlock>

      <div class="preflight-card">
        <div class="preflight-header">
          <q-icon name="checklist" size="lg" color="primary" />
          <h3>Checklist de Preparación</h3>
        </div>

        <div class="preflight-items">
          <div
            v-for="(item, idx) in preflightItems"
            :key="idx"
            class="preflight-item"
            @click="togglePreflight(idx)"
          >
            <q-checkbox
              v-model="item.checked"
              :label="item.label"
              color="primary"
              class="preflight-checkbox"
            />
            <div class="preflight-detail">{{ item.detail }}</div>
          </div>
        </div>

        <div class="preflight-progress q-mt-lg">
          <div class="progress-label">
            Progreso: {{ preflightProgress }}% ({{ checkedCount }}/{{ preflightItems.length }})
          </div>
          <q-linear-progress
            :value="preflightProgress / 100"
            color="primary"
            size="12px"
            rounded
            class="q-mt-sm"
          />
        </div>

        <AlertBlock
          v-if="preflightProgress === 100"
          title="🎉 ¡Listo para comenzar!"
          type="success"
          class="q-mt-md"
        >
          Has completado todos los requisitos. Estás preparado para iniciar tu viaje en ROS 2.
        </AlertBlock>
      </div>
    </div>

    <!-- ========== SECCIÓN 6: VALIDACIÓN DE ENTORNO ========== -->
    <div class="section-group self-stretch">
      <SectionTitle>🔍 Validación de Entorno</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Una vez que tengas Ubuntu instalado, ejecuta estos comandos en tu terminal para
            verificar que las herramientas básicas están disponibles.
          </TextBlock>
          <AlertBlock title="💻 Cómo abrir la terminal" type="info">
            Presiona <kbd>Ctrl + Alt + T</kbd> en Ubuntu para abrir la terminal.
          </AlertBlock>
        </template>
        <template #right>
          <CodeBlock
            title="Verificación de Herramientas Base"
            lang="bash"
            :content="checkCommand"
            :copyable="true"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- ========== SECCIÓN 7: CTA FINAL ========== -->
    <SectionTitle>🔍 Plan de estudios detallado</SectionTitle>
    <div class="section-group self-stretch column items-center q-mt-xl">
      <div class="full-width q-mt-xl">
        <PdfBlock
          title="Syllabus Completo — UP-Ros Academy"
          description="Descarga el plan de estudios oficial con todos los módulos, lecciones, objetivos y criterios de evaluación del curso."
          size="PDF · Documento oficial"
          :src="syllabusUrl"
        />
      </div>
    </div>
    <q-btn
      unelevated
      rounded
      color="primary"
      class="cta-btn shadow-glow"
      padding="12px 32px"
      to="/instalacion"
    >
      <div class="column items-start">
        <span class="text-weight-bold">Instalación</span>
        <span class="cta-caption" style="font-size: 0.7rem">Instala ROS 2</span>
      </div>
      <q-icon name="arrow_forward" class="q-ml-md" /> </q-btn
    ><br />
    <br />
    <br />
  </q-page>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue';
import { useTheme } from 'src/composables/useTheme';
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import StepsBlock from 'components/content/StepsBlock.vue';
import PdfBlock from 'components/content/PdfBlock.vue';
import syllabusUrl from 'src/assets/pdf/Syllabus.pdf';

// ========== TEMA ==========
const { currentTheme } = useTheme();
const isDark = computed(() => currentTheme.value === 'dark');

// ========== TAB DEL SISTEMA OPERATIVO ==========
const selectedOsTab = ref('native');

const dockerExample = `FROM osrf/ros:jazzy-desktop
RUN apt-get update && apt-get install -y \\
    python3-pip \\
    git
WORKDIR /workspace
CMD ["/bin/bash"]`.trim();

// ========== ROADMAP MODULES ==========
const roadmapModules = [
  {
    title: 'Módulo 0: Fundamentos Linux',
    description:
      'Domina la terminal, gestión de archivos, permisos y variables de entorno. La base para todo.',
    lessons: 6,
  },
  {
    title: 'Módulo 1: Programación Esencial',
    description: 'Python, C++ y Bash. Los lenguajes que hablan los robots profesionales.',
    lessons: 5,
  },
  {
    title: 'Módulo 2: Formatos de Datos',
    description: 'XML, JSON y YAML. Aprende a configurar robots sin hardcodear valores.',
    lessons: 4,
  },
  {
    title: 'Módulo 3: Git y GitHub',
    description:
      'Control de versiones profesional. Trabaja en equipo sin perder código ni cordura.',
    lessons: 7,
  },
  {
    title: 'Módulo 4: ROS 2 Fundamentos',
    description: 'Nodos, tópicos, servicios y acciones. El corazón de la arquitectura distribuida.',
    lessons: 5,
  },
  {
    title: 'Módulo 5: Herramientas de Desarrollo',
    description:
      'RViz, RQT, Rosbag y debugging de TF. Visualiza y diagnostica como un profesional.',
    lessons: 5,
  },
  {
    title: 'Módulo 6: Simulación (Lab Virtual)',
    description: 'URDF, Gazebo y sensores simulados. Construye robots sin gastar en hardware.',
    lessons: 4,
  },
  {
    title: 'Módulo 7: Navegación Autónoma (Nav2)',
    description: 'SLAM, localización y planificación de rutas. Robots que navegan solos.',
    lessons: 4,
  },
  {
    title: 'Módulo 8: Ingeniería de Software',
    description: 'Launch files avanzados, Docker y proyecto final integrador. Nivel ingeniero.',
    lessons: 4,
  },
];

// Colores e iconos por módulo (extraídos de courseStructure.ts)
const moduleColors = [
  '#10b981', // Módulo 0 - Green (terminal)
  '#3b82f6', // Módulo 1 - Blue (code)
  '#8b5cf6', // Módulo 2 - Purple (data_object)
  '#ec4899', // Módulo 3 - Pink (call_split)
  '#f59e0b', // Módulo 4 - Amber (smart_toy)
  '#06b6d4', // Módulo 5 - Cyan (handyman)
  '#6366f1', // Módulo 6 - Indigo (science)
  '#14b8a6', // Módulo 7 - Teal (explore)
  '#a855f7', // Módulo 8 - Purple (integration_instructions)
];

const moduleIcons = [
  'terminal',
  'code',
  'data_object',
  'call_split',
  'smart_toy',
  'handyman',
  'science',
  'explore',
  'integration_instructions',
];

const getModuleColor = (index: number) => moduleColors[index] || '#64748b';
const getModuleIcon = (index: number) => moduleIcons[index] || 'circle';

// ========== PRE-FLIGHT CHECKLIST ==========
const preflightItems = ref([
  {
    label: 'Conocimientos básicos de Python',
    detail: 'Variables, funciones, condicionales y bucles',
    checked: false,
  },
  {
    label: 'Al menos 20GB de espacio libre en disco',
    detail: 'Para Ubuntu, ROS 2, Gazebo y workspaces',
    checked: false,
  },
  {
    label: 'Máquina virtual configurada o equipo con Linux',
    detail: 'Ubuntu 24.04 LTS recomendado',
    checked: false,
  },
  {
    label: 'Conexión a internet estable',
    detail: 'Para descargar paquetes y dependencias',
    checked: false,
  },
]);

const togglePreflight = (index: number) => {
  if (preflightItems.value[index]) {
    preflightItems.value[index].checked = !preflightItems.value[index].checked;
  }
};

const checkedCount = computed(() => preflightItems.value.filter((item) => item.checked).length);

const preflightProgress = computed(() => {
  return Math.round((checkedCount.value / preflightItems.value.length) * 100);
});

// ========== VERIFICACIÓN DE ENTORNO ==========
const checkCommand = `# Verifica herramientas base
python3 --version
gcc --version
git --version
echo "¡Todo listo para ROS 2 Jazzy!"`.trim();
</script>

<style scoped>
/* ========== PÁGINA BASE ========== */
.intro-page {
  background: var(--bg-surface);
}

.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 4rem auto;
}

/* ========== HERO ÉPICO ========== */
.hero-epic {
  position: relative;
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 4rem auto;
  padding: 5rem 2rem;
  background:
    radial-gradient(circle at top center, rgba(99, 102, 241, 0.2), transparent 70%),
    radial-gradient(circle at bottom left, rgba(236, 72, 153, 0.15), transparent 60%),
    var(--bg-surface);
  backdrop-filter: blur(30px);
  border-radius: 32px;
  border: 1px solid var(--border-subtle);
  text-align: center;
  overflow: hidden;
}

/* Fondo animado de nodos */
.animated-nodes {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background-image:
    radial-gradient(circle, rgba(99, 102, 241, 0.4) 2px, transparent 2px),
    radial-gradient(circle, rgba(236, 72, 153, 0.4) 2px, transparent 2px);
  background-size: 80px 80px;
  background-position:
    0 0,
    40px 40px;
  opacity: 0.3;
  animation: nodesFloat 20s linear infinite;
}

@keyframes nodesFloat {
  0% {
    transform: translate(0, 0);
  }
  100% {
    transform: translate(40px, 40px);
  }
}

.hero-content {
  position: relative;
  z-index: 1;
}

.hero-title {
  font-size: 3.5rem;
  font-weight: 900;
  margin: 0 0 1rem 0;
  line-height: 1.1;
  color: var(--text-primary);
  letter-spacing: -0.02em;
}

.text-gradient {
  background: linear-gradient(135deg, #06b6d4 0%, #8b5cf6 50%, #ec4899 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.hero-subtitle {
  font-size: 1.25rem;
  color: var(--text-secondary);
  line-height: 1.6;
  max-width: 700px;
  margin: 0 auto;
}

.hero-cta {
  margin-top: 2rem;
}

.cta-pulse {
  animation: pulse 2s ease-in-out infinite;
}

@keyframes pulse {
  0%,
  100% {
    box-shadow: 0 0 0 0 rgba(99, 102, 241, 0.7);
  }
  50% {
    box-shadow: 0 0 0 20px rgba(99, 102, 241, 0);
  }
}

/* Animaciones de entrada */
.animate-fade-in {
  animation: fadeIn 0.8s ease-out;
}

.animate-slide-up {
  animation: slideUp 0.8s ease-out 0.2s both;
}

.animate-slide-up-delay {
  animation: slideUp 0.8s ease-out 0.4s both;
}

.animate-fade-in-delay {
  animation: fadeIn 0.8s ease-out 0.6s both;
}

@keyframes fadeIn {
  from {
    opacity: 0;
  }
  to {
    opacity: 1;
  }
}

@keyframes slideUp {
  from {
    opacity: 0;
    transform: translateY(30px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

/* ========== METÁFORA DEL SISTEMA NERVIOSO ========== */
.metaphor-list {
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.metaphor-item {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 12px;
  background: var(--bg-surface-solid);
  border-radius: 8px;
  border-left: 3px solid currentColor;
  color: var(--text-secondary);
}

/* Visualización del sistema nervioso */
.robot-nervous-system {
  position: relative;
  width: 100%;
  height: 350px;
  background: var(--bg-surface);
  border-radius: 16px;
  border: 1px solid var(--border-subtle);
  display: flex;
  align-items: center;
  justify-content: center;
}

.component-node {
  position: absolute;
  width: 70px;
  height: 70px;
  background: var(--bg-surface);
  border: 2px solid var(--border-hover);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: var(--text-code);
  animation: nodePulse 3s ease-in-out infinite;
}

.component-node.camera {
  top: 16px;
  left: 16px;
  animation-delay: 0s;
}

.component-node.lidar {
  top: 16px;
  right: 16px;
  animation-delay: 1s;
}

/* wheel usa translate para centrar; el keyframe de nodePulse debe preservar eso */
.component-node.wheel {
  bottom: 16px;
  left: 50%;
  transform: translateX(-50%);
  animation-delay: 2s;
}

/* Sobrescribe la animación para wheel para mantener el translateX */
.component-node.wheel:is(.component-node) {
  animation-name: nodePulseWheel;
}

@keyframes nodePulse {
  0%, 100% {
    box-shadow: 0 0 0 0 rgba(99, 102, 241, 0.3);
    border-color: rgba(99, 102, 241, 0.5);
  }
  50% {
    box-shadow: 0 0 12px 4px rgba(236, 72, 153, 0.4);
    border-color: rgba(236, 72, 153, 0.8);
  }
}

@keyframes nodePulseWheel {
  0%, 100% {
    box-shadow: 0 0 0 0 rgba(99, 102, 241, 0.3);
    border-color: rgba(99, 102, 241, 0.5);
    transform: translateX(-50%) scale(1);
  }
  50% {
    box-shadow: 0 0 12px 4px rgba(236, 72, 153, 0.4);
    border-color: rgba(236, 72, 153, 0.8);
    transform: translateX(-50%) scale(1.05);
  }
}

.central-brain {
  width: 100px;
  height: 100px;
  background: radial-gradient(circle, rgba(99, 102, 241, 0.3), transparent 70%);
  border: 3px solid #6366f1;
  border-radius: 50%;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  animation: brainGlow 2s ease-in-out infinite;
}

@keyframes brainGlow {
  0%,
  100% {
    box-shadow: 0 0 20px rgba(99, 102, 241, 0.5);
  }
  50% {
    box-shadow: 0 0 40px rgba(99, 102, 241, 0.8);
  }
}

.brain-label {
  margin-top: 8px;
  font-size: 0.75rem;
  font-weight: 700;
  color: #6366f1;
}

.connection-lines {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}

.connection-line {
  stroke: #06b6d4;
  stroke-width: 2;
  stroke-dasharray: 5, 5;
  opacity: 0.6;
  animation: lineDash 2s linear infinite;
}

.line-1 {
  animation-delay: 0s;
}
.line-2 {
  animation-delay: 0.5s;
}
.line-3 {
  animation-delay: 1s;
}

@keyframes lineDash {
  to {
    stroke-dashoffset: -20;
  }
}

/* ========== TARJETA DE SETUP DE OS ========== */
.os-setup-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 16px;
  overflow: hidden;
}

kbd {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-medium);
  border-radius: 4px;
  padding: 2px 6px;
  font-family: 'Fira Code', monospace;
  font-size: 0.9em;
  color: var(--text-code);
}

/* ========== ROADMAP TIMELINE ========== */
.roadmap-timeline {
  position: relative;
  padding: 2rem 0;
}

.roadmap-timeline::before {
  content: '';
  position: absolute;
  left: 50%;
  top: 0;
  bottom: 0;
  width: 3px;
  background: linear-gradient(
    to bottom,
    rgba(99, 102, 241, 0.5),
    rgba(236, 72, 153, 0.5),
    rgba(251, 191, 36, 0.5)
  );
  transform: translateX(-50%);
}

.timeline-item {
  position: relative;
  display: flex;
  align-items: center;
  margin-bottom: 3rem;
}

.timeline-item:last-child {
  margin-bottom: 0;
}

.timeline-marker {
  position: absolute;
  left: 50%;
  transform: translateX(-50%);
  width: 60px;
  height: 60px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  border: 4px solid var(--bg-surface);
  z-index: 2;
  transition: transform 0.3s ease;
}

.timeline-item:hover .timeline-marker {
  transform: translateX(-50%) scale(1.15);
}

.timeline-content {
  width: 45%;
}

.timeline-item:nth-child(odd) .timeline-content {
  margin-left: auto;
  padding-left: 3rem;
}

.timeline-item:nth-child(even) .timeline-content {
  margin-right: auto;
  padding-right: 3rem;
  text-align: right;
}

.timeline-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-left: 4px solid;
  border-radius: 12px;
  padding: 1.5rem;
  transition:
    transform 0.3s ease,
    box-shadow 0.3s ease;
}

.timeline-card:hover {
  transform: translateY(-5px);
  box-shadow: var(--shadow-lg);
}

.timeline-header {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 0.75rem;
}

.timeline-header h3 {
  margin: 0;
  font-size: 1.25rem;
  font-weight: 700;
  color: var(--text-primary);
}

.timeline-desc {
  color: var(--text-secondary);
  line-height: 1.6;
  margin: 0 0 1rem 0;
}

.timeline-meta {
  display: flex;
  gap: 8px;
}

/* ========== PRE-FLIGHT CARD ========== */
.preflight-card {
  background: var(--bg-surface);
  backdrop-filter: blur(20px);
  border: 1px solid var(--border-subtle);
  border-radius: 16px;
  padding: 2rem;
  max-width: 800px;
  margin: 0 auto;
}

.preflight-header {
  display: flex;
  align-items: center;
  gap: 16px;
  margin-bottom: 2rem;
  padding-bottom: 1rem;
  border-bottom: 1px solid var(--border-subtle);
}

.preflight-header h3 {
  margin: 0;
  font-size: 1.75rem;
  font-weight: 700;
  color: var(--text-primary);
}

.preflight-items {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.preflight-item {
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  border-radius: 12px;
  padding: 1rem;
  cursor: pointer;
  transition: all 0.3s ease;
}

.preflight-item:hover {
  background: var(--bg-surface-hover);
  border-color: var(--border-hover);
  transform: translateX(5px);
}

.preflight-checkbox {
  font-size: 1.1rem;
  font-weight: 600;
  color: var(--text-primary);
}

.preflight-detail {
  margin-top: 0.5rem;
  margin-left: 2rem;
  font-size: 0.9rem;
  color: var(--text-muted);
}

.preflight-progress {
  text-align: center;
}

.progress-label {
  font-size: 1rem;
  font-weight: 600;
  color: var(--text-secondary);
}

/* ========== PANEL TITLE (reactive theme) ========== */
.panel-title {
  color: var(--text-primary);
}

.cta-caption {
  color: rgba(255, 255, 255, 0.75);
}

/* ========== CTA FINAL ========== */
.final-cta {
  text-align: center;
  padding: 3rem 2rem;
  background: var(--bg-surface);
  border-radius: 24px;
  border: 1px solid var(--border-subtle);
  max-width: 800px;
  margin: 0 auto;
}

/* ========== RESPONSIVE ========== */
@media (max-width: 1024px) {
  .roadmap-timeline::before {
    left: 30px;
  }

  .timeline-marker {
    left: 30px;
  }

  .timeline-item:nth-child(odd) .timeline-content,
  .timeline-item:nth-child(even) .timeline-content {
    width: calc(100% - 100px);
    margin-left: 100px;
    padding-left: 0;
    padding-right: 0;
    text-align: left;
  }
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.5rem;
  }

  .hero-subtitle {
    font-size: 1.1rem;
  }

  .robot-nervous-system {
    height: 260px;
  }

  .component-node {
    width: 54px;
    height: 54px;
  }

  .central-brain {
    width: 80px;
    height: 80px;
  }

  .timeline-card {
    padding: 1rem;
  }

  .timeline-header h3 {
    font-size: 1.1rem;
  }
}

@media (max-width: 480px) {
  .hero-title { font-size: 2rem; }
  .hero-epic  { padding: 3rem 1rem; }
  .section-group { margin-bottom: 2.5rem; }

  .robot-nervous-system {
    height: 220px;
  }

  .component-node {
    width: 44px;
    height: 44px;
  }

  .central-brain {
    width: 64px;
    height: 64px;
  }

  .preflight-card { padding: 1.25rem; }
  .final-cta      { padding: 2rem 1rem; }
}
</style>
