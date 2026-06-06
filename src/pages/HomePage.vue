<template>
  <q-page class="bg-slate-dark">
    <div class="page-content-wrapper">

      <!-- ══════════════════════════════════════════
           1. HERO SECTION
      ══════════════════════════════════════════ -->
      <section class="hero-section">
        <div class="cyber-grid"></div>
        <div class="hero-glow"></div>

        <div class="content-container hero-inner">
          <!-- TEXTO IZQUIERDA -->
          <div class="hero-text">
            <div class="terminal-badge q-mb-lg">
              <span class="text-accent">UP-Ros </span>Plataforma Educativa v1.0
              <br />
              <span class="text-accent">admin@upros:~$</span> ./iniciar_aprendizaje.sh<span class="cursor-blink">_</span>
            </div>

            <h1 class="hero-title q-mb-md">
              Aprende <span class="text-gradient">ROS 2 Jazzy</span><br />
              de Cero a Robot Autónomo
            </h1>

            <p class="hero-desc q-mb-xl">
              El primer curso en español que lleva a estudiantes de Mecatrónica
              <strong>sin experiencia en Linux</strong> hasta la navegación autónoma real.
              Metodología progresiva, 100% práctica y gratuita.
            </p>

            <!-- CTAs -->
            <div class="hero-ctas q-mb-xl">
              <q-btn
                unelevated
                color="primary"
                class="cta-primary"
                padding="14px 32px"
                to="/instalacion"
                no-caps
              >
                <q-icon name="rocket_launch" class="q-mr-sm" />
                Empezar Ahora
              </q-btn>
              <q-btn
                outline
                class="cta-secondary"
                padding="14px 32px"
                :href="syllabusUrl"
                target="_blank"
                rel="noopener noreferrer"
                no-caps
                type="a"
              >
                <q-icon name="picture_as_pdf" class="q-mr-sm" />
                Ver Plan de Estudios
              </q-btn>
            </div>

            <!-- STATS REALES -->
            <div class="hero-stats">
              <div class="stat-pill">
                <span class="stat-num">9</span>
                <span class="stat-label">Módulos</span>
              </div>
              <div class="stat-divider"></div>
              <div class="stat-pill">
                <span class="stat-num">50+</span>
                <span class="stat-label">Lecciones</span>
              </div>
              <div class="stat-divider"></div>
              <div class="stat-pill">
                <span class="stat-num">~59h</span>
                <span class="stat-label">de Contenido</span>
              </div>
              <div class="stat-divider"></div>
              <div class="stat-pill">
                <span class="stat-num">100%</span>
                <span class="stat-label">Gratuito</span>
              </div>
            </div>
          </div>

          <!-- ROBOT DERECHA -->
          <div class="hero-robot-col">
            <div class="robot-wrapper">
              <div class="robot-ring ring-1"></div>
              <div class="robot-ring ring-2"></div>
              <img
                src="~assets/images/jazzy_robot.png"
                alt="Robot ROS 2 Jazzy"
                class="hero-robot floating-anim"
              />
              <div class="robot-glow-base"></div>
            </div>
          </div>
        </div>
      </section>

      <!-- ══════════════════════════════════════════
           2. TECH MARQUEE
      ══════════════════════════════════════════ -->
      <div class="tech-marquee-container">
        <div class="tech-marquee-track">
          <div class="tech-group" v-for="_ in 2" :key="_">
            <q-icon :name="`img:${imgRos2}`" />
            <span class="tech-text">ROS 2 JAZZY</span>
            <q-icon :name="`img:${imgPython}`" />
            <span class="tech-text">PYTHON 3</span>
            <q-icon :name="`img:${imgCpp}`" />
            <span class="tech-text">C++17</span>
            <q-icon :name="`img:${imgDds}`" />
            <span class="tech-text">DDS / FASTDDS</span>
            <q-icon name="terminal" />
            <span class="tech-text">BASH / LINUX</span>
            <q-icon :name="`img:${imgCmake}`" />
            <span class="tech-text">CMAKE</span>
            <q-icon :name="`img:${imgGazebo}`" />
            <span class="tech-text">GAZEBO HARMONIC</span>
            <q-icon :name="`img:${imgNav2}`" />
            <span class="tech-text">NAV 2</span>
            <q-icon :name="`img:${imgDocker}`" />
            <span class="tech-text">DOCKER</span>
            <q-icon name="hub" />
            <span class="tech-text">SLAM TOOLBOX</span>
          </div>
        </div>
      </div>

      <!-- ══════════════════════════════════════════
           3. ¿PARA QUIÉN ES ESTE CURSO?
      ══════════════════════════════════════════ -->
      <section class="section-padding">
        <div class="content-container">
          <div class="text-center q-mb-xl">
            <div class="section-eyebrow">DISEÑADO PARA TI</div>
            <h2 class="section-title">¿Para quién es este curso?</h2>
            <p class="section-subtitle">
              No importa tu punto de partida — si quieres programar robots reales, este es tu lugar.
            </p>
          </div>

          <div class="audience-grid">
            <div class="audience-card" v-for="a in audiences" :key="a.id">
              <div class="audience-icon-wrap" :style="{ background: a.color + '22', borderColor: a.color + '55' }">
                <q-icon :name="a.icon" size="2rem" :style="{ color: a.color }" />
              </div>
              <h3 class="audience-title">{{ a.title }}</h3>
              <p class="audience-desc">{{ a.desc }}</p>
              <ul class="audience-list">
                <li v-for="item in a.items" :key="item">
                  <q-icon name="check_circle" size="xs" :style="{ color: a.color }" />
                  {{ item }}
                </li>
              </ul>
            </div>
          </div>
        </div>
      </section>

      <!-- ══════════════════════════════════════════
           4. METODOLOGÍA (3 PILARES)
      ══════════════════════════════════════════ -->
      <section class="section-padding bg-dark-alt">
        <div class="content-container">
          <div class="text-center q-mb-xl">
            <div class="section-eyebrow">CÓMO APRENDEMOS</div>
            <h2 class="section-title">La metodología UP-Ros</h2>
            <p class="section-subtitle">
              Cada módulo sigue el mismo ciclo para que el aprendizaje sea natural y efectivo.
            </p>
          </div>

          <div class="methodology-row">
            <div class="method-step" v-for="(m, i) in methodology" :key="m.title">
              <div class="method-number">{{ i + 1 }}</div>
              <div class="method-icon-box" :class="m.bg">
                <q-icon :name="m.icon" color="white" size="28px" />
              </div>
              <h3 class="method-title">{{ m.title }}</h3>
              <p class="method-desc">{{ m.desc }}</p>
              <div v-if="i < methodology.length - 1" class="method-connector"></div>
            </div>
          </div>
        </div>
      </section>

      <!-- ══════════════════════════════════════════
           5. RUTA DE APRENDIZAJE
      ══════════════════════════════════════════ -->
      <section class="section-padding">
        <div class="content-container">
          <div class="text-center q-mb-xl">
            <div class="section-eyebrow">DE CERO A EXPERTO</div>
            <h2 class="section-title">Ruta de Aprendizaje Completa</h2>
            <p class="section-subtitle">
              9 módulos estructurados en 3 niveles progresivos.
              Cada uno construye sobre el anterior.
            </p>
          </div>

          <div class="roadmap-card">
            <img
              src="~assets/images/Ruta_aprendizaje.png"
              alt="Ruta de Aprendizaje UP-Ros"
              class="roadmap-img"
            />
          </div>

          <!-- Nivel leyenda -->
          <div class="level-legend q-mt-lg">
            <div class="level-badge level-basico">
              <q-icon name="circle" size="10px" /> BÁSICO — Módulos 0 al 3
            </div>
            <div class="level-badge level-intermedio">
              <q-icon name="circle" size="10px" /> INTERMEDIO — Módulos 4 y 5
            </div>
            <div class="level-badge level-avanzado">
              <q-icon name="circle" size="10px" /> AVANZADO — Módulos 6 al 8
            </div>
          </div>
        </div>
      </section>

      <!-- ══════════════════════════════════════════
           6. PLAN DE ESTUDIOS (MÓDULOS)
      ══════════════════════════════════════════ -->
      <section class="section-padding bg-dark-alt" id="plan-estudios">
        <div class="content-container">
          <div class="text-center q-mb-xl">
            <div class="section-eyebrow">CONTENIDO DEL CURSO</div>
            <h2 class="section-title">Plan de Estudios</h2>
            <p class="section-subtitle">
              Selecciona cualquier módulo para comenzar. Cada lección incluye teoría, código funcional y ejercicios prácticos.
            </p>
          </div>

          <div class="modules-grid">
            <template v-for="module in courseStructure" :key="module.path || 'unknown'">
              <router-link
                v-if="module.path && module.path !== 'home' && module.children && module.children.length > 0"
                :to="`/${module.path}/${module.children[0]?.path || ''}`"
                class="module-card"
                :class="{ 'full-width-card': module.path.includes('modulo-8') }"
                :style="{ '--module-color': getModuleColor(module.path) }"
              >
                <div class="card-glow"></div>

                <div class="card-top">
                  <div class="card-icon-box" :class="getModuleBgColor(module.path)">
                    <q-icon :name="module.icon || 'folder'" color="white" size="22px" />
                  </div>
                  <div class="card-badges">
                    <span class="badge-level" :class="getLevelClass(module.path)">
                      {{ getLevel(module.path) }}
                    </span>
                  </div>
                </div>

                <div class="card-content">
                  <div class="card-module-id" :class="getModuleTextColor(module.path)">
                    {{ formatModuleId(module.title || '') }}
                  </div>
                  <h3 class="card-title">{{ formatModuleTitle(module.title || '') }}</h3>
                  <p class="card-desc">{{ getModuleDescription(module.path) }}</p>

                  <div class="card-meta">
                    <div class="meta-item">
                      <q-icon name="menu_book" size="14px" />
                      <span>{{ module.children.length }} lecciones</span>
                    </div>
                    <div class="meta-item">
                      <q-icon name="schedule" size="14px" />
                      <span>{{ getModuleTime(module.path) }}</span>
                    </div>
                  </div>
                </div>

                <div class="card-footer">
                  <span class="card-start-text">Comenzar módulo</span>
                  <q-icon name="arrow_forward" class="card-arrow" />
                </div>
              </router-link>
            </template>
          </div>
        </div>
      </section>

      <!-- ══════════════════════════════════════════
           6.5. DESCARGA DEL SYLLABUS
      ══════════════════════════════════════════ -->
      <section class="section-padding syllabus-section">
        <div class="content-container">
          <div class="text-center q-mb-xl">
            <div class="section-eyebrow">DOCUMENTO OFICIAL</div>
            <h2 class="section-title">Plan de Estudios Oficial</h2>
            <p class="section-subtitle">
              Descarga el Syllabus completo con módulos, lecciones, objetivos y criterios de evaluación.
            </p>
          </div>

          <div class="syllabus-card">
            <div class="syllabus-icon-wrap">
              <q-icon name="picture_as_pdf" size="3.5rem" color="red-4" />
            </div>
            <div class="syllabus-info">
              <h3 class="syllabus-title">UP-Ros Academy — Syllabus Oficial</h3>
              <p class="syllabus-desc">
                Incluye todos los módulos del curso, lecciones detalladas, tiempos estimados,
                objetivos de aprendizaje y criterios de evaluación.
                Ideal para presentar a docentes o para organizar tu plan de estudio.
              </p>
              <div class="syllabus-meta">
                <span class="meta-chip">
                  <q-icon name="folder" size="14px" /> 9 Módulos
                </span>
                <span class="meta-chip">
                  <q-icon name="menu_book" size="14px" /> 50+ Lecciones
                </span>
                <span class="meta-chip">
                  <q-icon name="schedule" size="14px" /> ~59 Horas
                </span>
              </div>
            </div>
            <div class="syllabus-actions">
              <a
                :href="syllabusUrl"
                target="_blank"
                rel="noopener noreferrer"
                class="syllabus-btn-view"
                id="btn-ver-syllabus"
              >
                <q-icon name="open_in_new" size="18px" class="q-mr-xs" />
                Ver PDF
              </a>
              <a
                :href="syllabusUrl"
                :download="'UP-Ros_Syllabus.pdf'"
                class="syllabus-btn-download"
                id="btn-descargar-syllabus"
              >
                <q-icon name="download" size="18px" class="q-mr-xs" />
                Descargar
              </a>
            </div>
          </div>
        </div>
      </section>

      <!-- ══════════════════════════════════════════
           7. CTA FINAL
      ══════════════════════════════════════════ -->
      <section class="cta-section">
        <div class="cta-glow-1"></div>
        <div class="cta-glow-2"></div>
        <div class="content-container cta-inner">
          <div class="cta-icon-wrap">
            <q-icon name="rocket_launch" size="3rem" color="primary" />
          </div>
          <h2 class="cta-title">¿Listo para programar tu primer robot?</h2>
          <p class="cta-subtitle">
            Empieza por la instalación de ROS 2 Jazzy en Ubuntu 24.04.
            Es gratis, paso a paso, y lo hacemos juntos.
          </p>
          <div class="cta-buttons">
            <q-btn
              unelevated
              color="primary"
              padding="16px 40px"
              class="cta-main-btn"
              to="/instalacion"
              no-caps
            >
              <q-icon name="download" class="q-mr-sm" />
              Instalar ROS 2 Jazzy
            </q-btn>
            <q-btn
              flat
              padding="16px 32px"
              class="cta-secondary-btn"
              to="/introduccion"
              no-caps
            >
              Leer introducción primero
              <q-icon name="arrow_forward" class="q-ml-sm" />
            </q-btn>
          </div>
          <div class="cta-reassurance">
            <q-icon name="check_circle" size="16px" color="positive" />
            Sin cuenta requerida
            <span class="dot-sep">·</span>
            <q-icon name="check_circle" size="16px" color="positive" />
            Ubuntu 24.04 LTS
            <span class="dot-sep">·</span>
            <q-icon name="check_circle" size="16px" color="positive" />
            Guía completa incluida
          </div>
        </div>
      </section>

      <!-- FOOTER -->
      <div class="text-center q-py-lg text-caption border-t page-footer">
        © {{ new Date().getFullYear() }} Alexander Calderón Leal — UP-Ros Academy &nbsp;•&nbsp;
        Ingeniería Mecatrónica &nbsp;•&nbsp; Universidad de Pamplona &nbsp;•&nbsp; ROS 2 Jazzy
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import { computed } from 'vue';
import { useTheme } from 'src/composables/useTheme';
import { courseStructure } from 'src/data/courseStructure';

import imgRos2 from 'src/assets/images/ros2.png';
import imgPython from 'src/assets/images/Python.png';
import imgCpp from 'src/assets/images/ISO_C++_Logo.png';
import imgDds from 'src/assets/images/dds.png';
import imgCmake from 'src/assets/images/Cmake.png';
import imgGazebo from 'src/assets/images/gazebo.png';
import imgNav2 from 'src/assets/images/nav2.png';
import imgDocker from 'src/assets/images/docker.png';
import syllabusUrl from 'src/assets/pdf/Syllabus.pdf';

const { currentTheme } = useTheme();
const isDark = computed(() => currentTheme.value === 'dark');

// ── Audiencias ──────────────────────────────────────────────────
const audiences = [
  {
    id: 'student',
    icon: 'school',
    color: '#38bdf8',
    title: 'Estudiantes de Mecatrónica',
    desc: 'Nunca tocaste Linux ni ROS 2 en la universidad. Este curso empieza desde el principio absoluto.',
    items: ['Sin experiencia en terminal', 'Sin conocimientos de ROS', 'Con ganas de aprender robots reales'],
  },
  {
    id: 'engineer',
    icon: 'engineering',
    color: '#a78bfa',
    title: 'Ingenieros en Formación',
    desc: 'Sabes programar pero quieres dar el salto a la robótica profesional con estándares industriales.',
    items: ['Python básico o intermedio', 'Quieres proyectos en tu portafolio', 'Buscas empleo en robótica/IoT'],
  },
  {
    id: 'hobbyist',
    icon: 'build_circle',
    color: '#34d399',
    title: 'Entusiastas y Makers',
    desc: 'Tienes un robot o quieres construir uno. ROS 2 es el framework que usan los profesionales.',
    items: ['Tienes hardware (Raspberry Pi, Arduino)', 'Quieres automatizar algo real', 'Amas el open source'],
  },
];

// ── Metodología ──────────────────────────────────────────────────
const methodology = [
  {
    title: 'Conceptos Claros',
    desc: 'Cada tema arranca con la analogía correcta. Sin jerga innecesaria, con contexto real.',
    icon: 'lightbulb',
    bg: 'bg-indigo-8',
  },
  {
    title: 'Código Funcional',
    desc: 'Cada lección tiene comandos reales que puedes ejecutar ahora mismo en tu terminal.',
    icon: 'code',
    bg: 'bg-blue-8',
  },
  {
    title: 'Practica y Verifica',
    desc: 'Resultado esperado incluido. Sabrás exactamente si lo hiciste bien antes de avanzar.',
    icon: 'fact_check',
    bg: 'bg-teal-8',
  },
  {
    title: 'Proyectos Reales',
    desc: 'Cada módulo termina con un mini-proyecto que puedes mostrar en tu portafolio.',
    icon: 'rocket_launch',
    bg: 'bg-purple-8',
  },
];

// ── Helpers de módulos ──────────────────────────────────────────
const formatModuleId = (title: string) => (title.split(':')[0] || 'MOD').toUpperCase();
const formatModuleTitle = (title: string) => title.split(':')[1]?.trim() || title;

const moduleColors: Record<string, string> = {
  'modulo-0': '#38bdf8',
  'modulo-1': '#818cf8',
  'modulo-2': '#22d3ee',
  'modulo-3': '#f59e0b',
  'modulo-4': '#f87171',
  'modulo-5': '#60a5fa',
  'modulo-6': '#fb923c',
  'modulo-7': '#f472b6',
  'modulo-8': '#4ade80',
};

const getModuleColor = (path: string) => moduleColors[path] ?? '#64748b';

const getModuleBgColor = (path: string) => {
  const map: Record<string, string> = {
    'modulo-0': 'bg-blue-8', 'modulo-1': 'bg-purple-8', 'modulo-2': 'bg-cyan-8',
    'modulo-3': 'bg-amber-8', 'modulo-4': 'bg-red-8', 'modulo-5': 'bg-indigo-8',
    'modulo-6': 'bg-orange-8', 'modulo-7': 'bg-pink-8', 'modulo-8': 'bg-green-8',
  };
  return map[path] ?? 'bg-grey-7';
};

const getModuleTextColor = (path: string) => {
  const suffix = isDark.value ? '4' : '8';
  const map: Record<string, string> = {
    'modulo-0': `text-blue-${suffix}`, 'modulo-1': `text-purple-${suffix}`,
    'modulo-2': `text-cyan-${suffix}`, 'modulo-3': `text-amber-${suffix}`,
    'modulo-4': `text-red-${suffix}`, 'modulo-5': `text-indigo-${suffix}`,
    'modulo-6': `text-orange-${suffix}`, 'modulo-7': `text-pink-${suffix}`,
    'modulo-8': `text-green-${suffix}`,
  };
  return map[path] ?? (isDark.value ? 'text-grey-4' : 'text-grey-8');
};

const getLevel = (path: string) => {
  if (['modulo-0', 'modulo-1', 'modulo-2', 'modulo-3'].some(m => path.includes(m))) return 'BÁSICO';
  if (['modulo-4', 'modulo-5'].some(m => path.includes(m))) return 'INTERMEDIO';
  return 'AVANZADO';
};

const getLevelClass = (path: string) => {
  const level = getLevel(path);
  if (level === 'BÁSICO') return 'level-basico';
  if (level === 'INTERMEDIO') return 'level-intermedio';
  return 'level-avanzado';
};

const getModuleDescription = (path: string) => {
  const desc: Record<string, string> = {
    'modulo-0': 'Terminal, archivos, permisos, editores y variables de entorno. La base para todo lo que viene.',
    'modulo-1': 'Python 3, C++17 y Bash. Los tres lenguajes que hablan los robots profesionales.',
    'modulo-2': 'XML, JSON y YAML: aprende a configurar sistemas complejos sin hardcodear un solo valor.',
    'modulo-3': 'Git y GitHub para trabajo en equipo. Nunca pierdas código y colabora como profesional.',
    'modulo-4': 'Nodos, tópicos, servicios y acciones: la arquitectura distribuida de ROS 2 desde cero.',
    'modulo-5': 'RViz2, RQT, Rosbag y TF. Las herramientas de diagnóstico que usan los ingenieros reales.',
    'modulo-6': 'URDF, Gazebo Harmonic y sensores virtuales. Construye y prueba robots sin gastar en hardware.',
    'modulo-7': 'SLAM, mapeo y Nav2. Robots que se ubican solos y navegan de forma autónoma.',
    'modulo-8': 'Launch files, Docker, CI/CD y proyecto integrador. El nivel de una ingeniería profesional.',
  };
  return desc[path] ?? 'Contenido avanzado de robótica profesional con ROS 2.';
};

const getModuleTime = (path: string) => {
  const times: Record<string, string> = {
    'modulo-0': '~6 h', 'modulo-1': '~5 h', 'modulo-2': '~4 h',
    'modulo-3': '~7 h', 'modulo-4': '~10 h', 'modulo-5': '~5 h',
    'modulo-6': '~8 h', 'modulo-7': '~8 h', 'modulo-8': '~6 h',
  };
  return times[path] ?? '~4 h';
};
</script>

<style scoped>
/* ══════════════════════════════════════════
   BASE
══════════════════════════════════════════ */
.bg-slate-dark { background-color: var(--bg-page); }
.bg-dark-alt   { background-color: var(--bg-deep); }

.page-content-wrapper {
  width: 100%;
  overflow-x: hidden;
  display: flex;
  flex-direction: column;
}

.content-container {
  width: 100%;
  max-width: 1200px;
  margin: 0 auto;
  padding: 0 24px;
  box-sizing: border-box;
  position: relative;
  z-index: 2;
}

.section-padding { padding: 5rem 0; }

.border-t    { border-top: 1px solid var(--border-subtle); }
.page-footer { color: var(--text-muted); }

/* Section headers */
.section-eyebrow {
  font-size: 0.75rem;
  font-weight: 700;
  letter-spacing: 0.15em;
  text-transform: uppercase;
  color: var(--text-code);
  margin-bottom: 0.75rem;
}
.section-title {
  font-size: 2.4rem;
  font-weight: 800;
  color: var(--text-primary);
  margin: 0 0 0.75rem 0;
  line-height: 1.15;
}
.section-subtitle {
  font-size: 1.1rem;
  color: var(--text-secondary);
  max-width: 640px;
  margin: 0 auto;
  line-height: 1.6;
}

/* ══════════════════════════════════════════
   HERO
══════════════════════════════════════════ */
.hero-section {
  position: relative;
  padding: 7rem 0 5rem 0;
  overflow: hidden;
  min-height: 90vh;
  display: flex;
  align-items: center;
}

.cyber-grid {
  position: absolute;
  width: 200%; height: 200%;
  top: -50%; left: -50%;
  background-image:
    linear-gradient(rgba(56, 189, 248, 0.05) 1px, transparent 1px),
    linear-gradient(90deg, rgba(56, 189, 248, 0.05) 1px, transparent 1px);
  background-size: 50px 50px;
  transform: perspective(500px) rotateX(60deg);
  animation: grid-move 20s linear infinite;
  z-index: 0;
  mask-image: linear-gradient(to bottom, transparent, black 40%, transparent);
}
@keyframes grid-move {
  from { transform: perspective(500px) rotateX(60deg) translateY(0); }
  to   { transform: perspective(500px) rotateX(60deg) translateY(50px); }
}

.hero-glow {
  position: absolute; top: 0; right: 0;
  width: 60%; height: 100%;
  background: radial-gradient(circle at center, rgba(56, 189, 248, 0.08), transparent 70%);
  pointer-events: none; z-index: 0;
}

.hero-inner {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 4rem;
  align-items: center;
  position: relative;
  z-index: 2;
}

.terminal-badge {
  font-family: 'Fira Code', monospace;
  font-size: 0.85rem;
  color: var(--text-secondary);
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-medium);
  padding: 10px 16px;
  border-radius: 10px;
  display: inline-block;
  line-height: 1.7;
}
.cursor-blink { animation: blink 1s step-end infinite; }
@keyframes blink { 50% { opacity: 0; } }

.text-gradient {
  background: linear-gradient(135deg, #38bdf8 0%, #818cf8 60%, #ec4899 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.hero-title {
  font-size: 3.2rem;
  font-weight: 900;
  line-height: 1.1;
  letter-spacing: -0.03em;
  color: var(--text-primary);
  margin: 0 0 1rem 0;
}

.hero-desc {
  font-size: 1.15rem;
  line-height: 1.7;
  color: var(--text-secondary);
  max-width: 560px;
}

/* CTAs */
.hero-ctas {
  display: flex;
  gap: 1rem;
  flex-wrap: wrap;
}

.cta-primary {
  font-weight: 700;
  font-size: 1rem;
  border-radius: 12px;
  box-shadow: 0 0 20px rgba(56, 189, 248, 0.35);
  transition: all 0.3s ease;
}
.cta-primary:hover {
  transform: translateY(-2px);
  box-shadow: 0 0 32px rgba(56, 189, 248, 0.55);
}
.cta-secondary {
  font-weight: 600;
  font-size: 1rem;
  border-radius: 12px;
  color: var(--text-secondary);
  border-color: var(--border-medium);
  transition: all 0.3s ease;
}
.cta-secondary:hover {
  color: var(--text-primary);
  border-color: var(--border-hover);
  background: var(--bg-surface-hover);
  transform: translateY(-2px);
}

/* Stats */
.hero-stats {
  display: flex;
  align-items: center;
  gap: 1rem;
  flex-wrap: wrap;
}
.stat-pill {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 2px;
}
.stat-num {
  font-size: 1.4rem;
  font-weight: 900;
  color: var(--text-primary);
  line-height: 1;
}
.stat-label {
  font-size: 0.75rem;
  color: var(--text-muted);
  text-transform: uppercase;
  letter-spacing: 0.05em;
}
.stat-divider {
  width: 1px;
  height: 36px;
  background: var(--border-subtle);
}

/* Robot */
.hero-robot-col {
  display: flex;
  justify-content: center;
  align-items: center;
}
.robot-wrapper {
  position: relative;
  width: 100%;
  max-width: 420px;
}
.robot-ring {
  position: absolute;
  border-radius: 50%;
  border: 1px solid rgba(56, 189, 248, 0.15);
  top: 50%; left: 50%;
  transform: translate(-50%, -50%);
  animation: ring-pulse 4s ease-in-out infinite;
}
.ring-1 { width: 85%; padding-top: 85%; animation-delay: 0s; }
.ring-2 { width: 110%; padding-top: 110%; animation-delay: 2s; }
@keyframes ring-pulse {
  0%, 100% { opacity: 0.3; transform: translate(-50%, -50%) scale(1); }
  50%       { opacity: 0.7; transform: translate(-50%, -50%) scale(1.05); }
}
.hero-robot {
  width: 100%; height: auto;
  position: relative; z-index: 2;
  filter: drop-shadow(0 0 24px rgba(56, 189, 248, 0.25));
}
.floating-anim { animation: float 8s ease-in-out infinite; }
@keyframes float {
  0%, 100% { transform: translateY(0); }
  50%       { transform: translateY(-18px); }
}
.robot-glow-base {
  position: absolute;
  bottom: -10px; left: 50%;
  transform: translateX(-50%);
  width: 60%; height: 24px;
  background: rgba(56, 189, 248, 0.2);
  border-radius: 50%;
  filter: blur(16px);
  animation: shadow-pulse 8s ease-in-out infinite;
}
@keyframes shadow-pulse {
  0%, 100% { opacity: 0.6; width: 60%; }
  50%       { opacity: 0.2; width: 40%; }
}

/* ══════════════════════════════════════════
   TECH MARQUEE
══════════════════════════════════════════ */
.tech-marquee-container {
  background: var(--bg-deep);
  padding: 20px 0;
  border-top: 1px solid var(--border-subtle);
  border-bottom: 1px solid var(--border-subtle);
  width: 100%;
  overflow: hidden;
  display: flex;
  align-items: center;
  position: relative;
  z-index: 1;
}
.tech-marquee-container::before,
.tech-marquee-container::after {
  content: '';
  position: absolute;
  top: 0; width: 120px; height: 100%;
  z-index: 2; pointer-events: none;
}
.tech-marquee-container::before {
  left: 0;
  background: linear-gradient(to right, var(--bg-deep), transparent);
}
.tech-marquee-container::after {
  right: 0;
  background: linear-gradient(to left, var(--bg-deep), transparent);
}
.tech-marquee-track {
  display: flex;
  width: max-content;
  flex-wrap: nowrap;
  animation: scroll 45s linear infinite;
}
.tech-marquee-track:hover { animation-play-state: paused; }
.tech-group {
  display: flex;
  gap: 3rem;
  align-items: center;
  flex-shrink: 0;
  padding-right: 3rem;
}
.tech-text {
  font-weight: 700;
  color: var(--text-muted);
  font-size: 0.9rem;
  letter-spacing: 1px;
  white-space: nowrap;
}
.tech-marquee-track .q-icon {
  font-size: 26px;
  color: var(--text-muted);
  flex-shrink: 0;
}
@keyframes scroll {
  from { transform: translateX(0); }
  to   { transform: translateX(-50%); }
}

/* ══════════════════════════════════════════
   AUDIENCIAS
══════════════════════════════════════════ */
.audience-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 24px;
}
.audience-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 20px;
  padding: 2rem;
  transition: all 0.3s ease;
  border-top: 3px solid var(--border-subtle);
}
.audience-card:hover {
  transform: translateY(-6px);
  box-shadow: var(--shadow-lg);
  border-color: var(--border-hover);
}
.audience-icon-wrap {
  width: 60px; height: 60px;
  border-radius: 16px;
  border: 1px solid;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 1.25rem;
}
.audience-title {
  font-size: 1.15rem;
  font-weight: 700;
  color: var(--text-primary);
  margin: 0 0 0.75rem 0;
}
.audience-desc {
  font-size: 0.95rem;
  color: var(--text-secondary);
  line-height: 1.6;
  margin: 0 0 1rem 0;
}
.audience-list {
  list-style: none;
  padding: 0; margin: 0;
  display: flex;
  flex-direction: column;
  gap: 6px;
}
.audience-list li {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 0.9rem;
  color: var(--text-secondary);
}

/* ══════════════════════════════════════════
   METODOLOGÍA
══════════════════════════════════════════ */
.methodology-row {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
  gap: 2rem;
  position: relative;
}
.method-step {
  position: relative;
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 20px;
  padding: 2rem 1.5rem;
  text-align: center;
  transition: transform 0.3s, box-shadow 0.3s;
}
.method-step:hover {
  transform: translateY(-4px);
  box-shadow: var(--shadow-md);
}
.method-number {
  position: absolute;
  top: -14px; left: 50%;
  transform: translateX(-50%);
  width: 28px; height: 28px;
  border-radius: 50%;
  background: var(--bg-page);
  border: 2px solid var(--border-hover);
  color: var(--text-code);
  font-size: 0.8rem;
  font-weight: 800;
  display: flex;
  align-items: center;
  justify-content: center;
}
.method-icon-box {
  width: 56px; height: 56px;
  border-radius: 16px;
  display: flex;
  align-items: center;
  justify-content: center;
  margin: 0 auto 1rem auto;
}
.method-title {
  font-size: 1.05rem;
  font-weight: 700;
  color: var(--text-primary);
  margin: 0 0 0.5rem 0;
}
.method-desc {
  font-size: 0.9rem;
  color: var(--text-secondary);
  line-height: 1.6;
  margin: 0;
}

/* ══════════════════════════════════════════
   ROADMAP
══════════════════════════════════════════ */
.roadmap-card {
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 20px;
  padding: 12px;
  display: flex;
  justify-content: center;
  align-items: center;
  overflow: hidden;
}
.roadmap-img {
  width: 100%;
  border-radius: 12px;
  max-width: 100%;
  min-width: 320px;
}
.level-legend {
  display: flex;
  gap: 1rem;
  justify-content: center;
  flex-wrap: wrap;
}
.level-badge {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  padding: 6px 14px;
  border-radius: 999px;
  font-size: 0.8rem;
  font-weight: 600;
  border: 1px solid;
}
.level-basico     { color: #22c55e; border-color: #22c55e44; background: #22c55e11; }
.level-intermedio { color: #f59e0b; border-color: #f59e0b44; background: #f59e0b11; }
.level-avanzado   { color: #f87171; border-color: #f8717144; background: #f8717111; }

/* ══════════════════════════════════════════
   MÓDULOS GRID
══════════════════════════════════════════ */
.modules-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(340px, 1fr));
  gap: 20px;
}
.module-card {
  position: relative;
  background: var(--bg-surface);
  border: 1px solid var(--border-medium);
  border-radius: 20px;
  padding: 24px;
  display: flex;
  flex-direction: column;
  gap: 16px;
  text-decoration: none;
  overflow: hidden;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  border-top: 3px solid var(--module-color, var(--border-medium));
}
.module-card:hover {
  transform: translateY(-5px);
  background: var(--bg-surface-hover);
  border-color: var(--module-color, var(--border-hover));
  box-shadow: var(--shadow-lg);
}
.card-glow {
  position: absolute;
  top: 0; left: 0;
  width: 100%; height: 4px;
  background: linear-gradient(90deg, transparent, var(--module-color, var(--border-hover)), transparent);
  opacity: 0;
  transition: opacity 0.3s;
}
.module-card:hover .card-glow { opacity: 1; }

.full-width-card {
  grid-column: 1 / -1;
  flex-direction: row;
  align-items: center;
  gap: 24px;
}

.card-top {
  display: flex;
  justify-content: space-between;
  align-items: center;
}
.card-icon-box {
  width: 44px; height: 44px;
  border-radius: 12px;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: var(--shadow-sm);
}

.badge-level {
  font-size: 0.7rem;
  font-weight: 700;
  letter-spacing: 0.08em;
  padding: 3px 10px;
  border-radius: 999px;
  border: 1px solid;
}

.card-module-id {
  font-size: 0.75rem;
  font-weight: 700;
  letter-spacing: 0.1em;
  text-transform: uppercase;
}
.card-title {
  font-size: 1.1rem;
  font-weight: 700;
  color: var(--text-primary);
  margin: 4px 0 6px 0;
  line-height: 1.3;
}
.card-desc {
  font-size: 0.88rem;
  color: var(--text-secondary);
  line-height: 1.55;
  margin: 0;
  flex: 1;
}
.card-meta {
  display: flex;
  gap: 1rem;
  margin-top: auto;
}
.meta-item {
  display: flex;
  align-items: center;
  gap: 4px;
  font-size: 0.8rem;
  color: var(--text-muted);
}
.card-footer {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding-top: 12px;
  border-top: 1px solid var(--border-subtle);
  margin-top: 4px;
}
.card-start-text {
  font-size: 0.82rem;
  font-weight: 600;
  color: var(--text-muted);
  transition: color 0.3s;
}
.module-card:hover .card-start-text { color: var(--text-link); }
.card-arrow {
  color: var(--text-muted);
  transition: all 0.3s;
}
.module-card:hover .card-arrow {
  color: var(--text-link);
  transform: translateX(4px);
}

/* ══════════════════════════════════════════
   CTA FINAL
══════════════════════════════════════════ */
.cta-section {
  position: relative;
  padding: 6rem 0;
  overflow: hidden;
  background:
    radial-gradient(ellipse at top left, rgba(56, 189, 248, 0.1), transparent 55%),
    radial-gradient(ellipse at bottom right, rgba(129, 140, 248, 0.1), transparent 55%),
    var(--bg-surface);
  border-top: 1px solid var(--border-subtle);
}
.cta-glow-1 {
  position: absolute; top: -100px; left: -100px;
  width: 400px; height: 400px;
  border-radius: 50%;
  background: rgba(56, 189, 248, 0.05);
  filter: blur(60px);
  pointer-events: none;
}
.cta-glow-2 {
  position: absolute; bottom: -100px; right: -100px;
  width: 500px; height: 500px;
  border-radius: 50%;
  background: rgba(129, 140, 248, 0.05);
  filter: blur(80px);
  pointer-events: none;
}
.cta-inner {
  text-align: center;
  max-width: 760px;
}
.cta-icon-wrap {
  margin-bottom: 1.5rem;
}
.cta-title {
  font-size: 2.8rem;
  font-weight: 900;
  color: var(--text-primary);
  margin: 0 0 1rem 0;
  line-height: 1.1;
  letter-spacing: -0.03em;
}
.cta-subtitle {
  font-size: 1.15rem;
  color: var(--text-secondary);
  line-height: 1.65;
  margin: 0 0 2.5rem 0;
  max-width: 560px;
  margin-left: auto;
  margin-right: auto;
}
.cta-buttons {
  display: flex;
  gap: 1rem;
  justify-content: center;
  flex-wrap: wrap;
  margin-bottom: 1.5rem;
}
.cta-main-btn {
  font-weight: 700;
  font-size: 1rem;
  border-radius: 12px;
  box-shadow: 0 0 24px rgba(56, 189, 248, 0.4);
  transition: all 0.3s;
}
.cta-main-btn:hover {
  transform: translateY(-2px);
  box-shadow: 0 0 36px rgba(56, 189, 248, 0.6);
}
.cta-secondary-btn {
  font-weight: 600;
  font-size: 1rem;
  border-radius: 12px;
  color: var(--text-secondary);
  transition: all 0.3s;
}
.cta-secondary-btn:hover {
  color: var(--text-primary);
  background: var(--bg-surface-hover);
}
.cta-reassurance {
  display: flex;
  align-items: center;
  gap: 8px;
  justify-content: center;
  flex-wrap: wrap;
  font-size: 0.85rem;
  color: var(--text-muted);
}
.dot-sep { color: var(--border-hover); }

/* ══════════════════════════════════════════
   RESPONSIVE
══════════════════════════════════════════ */
@media (max-width: 900px) {
  .hero-inner {
    grid-template-columns: 1fr;
    gap: 2rem;
    text-align: center;
  }
  .hero-title { font-size: 2.6rem; }
  .hero-desc  { max-width: 100%; }
  .hero-ctas  { justify-content: center; }
  .hero-stats { justify-content: center; }
  .hero-robot-col { order: -1; }
  .robot-wrapper { max-width: 280px; margin: 0 auto; }

  .methodology-row { grid-template-columns: 1fr 1fr; }
  .full-width-card  { flex-direction: column; }
  .cta-title { font-size: 2rem; }

  .syllabus-card {
    flex-direction: column;
    align-items: flex-start;
    gap: 1.5rem;
  }
  .syllabus-actions {
    flex-direction: row;
    width: 100%;
    justify-content: flex-start;
  }
}

@media (max-width: 600px) {
  .hero-section { padding: 6rem 0 4rem 0; min-height: auto; }
  .hero-title   { font-size: 2rem; }
  .section-title { font-size: 1.8rem; }
  .modules-grid  { grid-template-columns: 1fr; }
  .methodology-row { grid-template-columns: 1fr; }
  .audience-grid { grid-template-columns: 1fr; }
  .hero-ctas { flex-direction: column; align-items: center; }
  .cta-buttons { flex-direction: column; align-items: center; }

  .syllabus-card { padding: 1.5rem; }
  .syllabus-actions { flex-direction: column; gap: 0.75rem; }
  .syllabus-btn-view,
  .syllabus-btn-download { width: 100%; justify-content: center; }
}

/* ══════════════════════════════════════════
   SYLLABUS CARD
══════════════════════════════════════════ */
.syllabus-section {
  background: var(--bg-page);
}

.syllabus-card {
  display: flex;
  align-items: center;
  gap: 2rem;
  background: var(--bg-surface);
  border: 1px solid var(--border-subtle);
  border-radius: 20px;
  padding: 2.5rem;
  position: relative;
  overflow: hidden;
  transition: all 0.3s ease;

  /* Borde izquierdo rojo como PDF */
  border-left: 4px solid #ef4444;
}

.syllabus-card::before {
  content: '';
  position: absolute;
  top: 0; left: 0; right: 0;
  height: 1px;
  background: linear-gradient(90deg, #ef4444, rgba(239, 68, 68, 0), transparent);
}

.syllabus-card:hover {
  transform: translateY(-3px);
  box-shadow: 0 12px 40px var(--shadow-md);
  border-color: rgba(239, 68, 68, 0.5);
}

.syllabus-icon-wrap {
  flex-shrink: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  background: rgba(239, 68, 68, 0.1);
  border-radius: 16px;
  width: 80px;
  height: 80px;
  border: 1px solid rgba(239, 68, 68, 0.2);
}

.syllabus-info {
  flex: 1;
  min-width: 0;
}

.syllabus-title {
  font-size: 1.25rem;
  font-weight: 700;
  color: var(--text-primary);
  margin: 0 0 0.5rem 0;
}

.syllabus-desc {
  font-size: 0.95rem;
  color: var(--text-secondary);
  line-height: 1.6;
  margin: 0 0 1rem 0;
}

.syllabus-meta {
  display: flex;
  gap: 0.75rem;
  flex-wrap: wrap;
}

.meta-chip {
  display: inline-flex;
  align-items: center;
  gap: 4px;
  font-size: 0.78rem;
  font-weight: 600;
  color: var(--text-muted);
  background: var(--bg-surface-solid);
  border: 1px solid var(--border-subtle);
  padding: 4px 10px;
  border-radius: 20px;
  letter-spacing: 0.02em;
}

.syllabus-actions {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
  flex-shrink: 0;
}

.syllabus-btn-view,
.syllabus-btn-download {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
  padding: 10px 22px;
  border-radius: 10px;
  font-size: 0.9rem;
  font-weight: 600;
  text-decoration: none;
  cursor: pointer;
  transition: all 0.25s ease;
  white-space: nowrap;
}

.syllabus-btn-view {
  background: transparent;
  border: 2px solid var(--border-hover);
  color: var(--text-code);
}

.syllabus-btn-view:hover {
  background: var(--bg-code);
  border-color: var(--text-code);
  transform: translateY(-2px);
  box-shadow: 0 4px 16px var(--shadow-sm);
}

.syllabus-btn-download {
  background: #ef4444;
  border: 2px solid transparent;
  color: #ffffff;
}

.syllabus-btn-download:hover {
  background: #dc2626;
  transform: translateY(-2px);
  box-shadow: 0 4px 20px rgba(239, 68, 68, 0.4);
}

</style>
