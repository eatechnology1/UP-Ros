<template>
  <q-page class="bg-slate-dark text-white">
    <div class="page-content-wrapper">
      <!-- 1. HERO SECTION (CON CYBER-GRID BG) -->
      <section class="hero-section relative-position">
        <!-- Fondo Dinámico -->
        <div class="cyber-grid"></div>
        <div class="hero-glow"></div>

        <div class="content-container row items-center q-col-gutter-xl relative-position">
          <!-- COLUMNA IZQUIERDA: TEXTO -->
          <div class="col-12 col-md-7">
            <!-- Terminal Badge Animado -->

            <div class="terminal-badge q-mb-md">
              <span class="text-accent">UP-Ros </span>Plataforma Educativa v1.0 <br /><br />
              <span class="text-accent">admin@upros:~$</span> ./iniciar_aprendizaje.sh<span
                class="cursor-blink"
                >_</span
              >
            </div>

            <h1 class="hero-title q-mb-md">
              Aprende
              <span class="text-gradient">ROS 2 Jazzy</span>
              de forma Progresiva y Práctica
            </h1>

            <p class="hero-desc text-grey-4 q-mb-lg">
              La arquitectura educativa definitiva. Pasa de la teoría a la
              <strong>navegación autónoma real</strong> mediante una metodología progresiva basada
              en proyectos industriales.
            </p>

            <!-- STATS -->
            <div class="row q-mt-xl q-gutter-x-lg text-grey-5 stats-row">
              <div class="row items-center stat-item">
                <q-icon name="code" color="accent" class="q-mr-sm" />
                <span>100% Open Source</span>
              </div>
              <div class="row items-center stat-item">
                <q-icon name="view_in_ar" color="accent" class="q-mr-sm" />
                <span>Enfoque Práctico</span>
              </div>
              <div class="row items-center stat-item">
                <q-icon name="school" color="accent" class="q-mr-sm" />
                <span>Académico y documentación</span>
              </div>
            </div>
          </div>

          <!-- COLUMNA DERECHA: ROBOT -->
          <div class="col-12 col-md-5 column flex-center">
            <div class="robot-wrapper">
              <div class="code-decoration"></div>
              <!-- Decoración de código flotante -->
              <img
                src="~assets/images/jazzy_robot.png"
                alt="Robot UP-Ros"
                class="hero-robot floating-anim"
              />
              <div class="robot-shadow"></div>
            </div>
          </div>
        </div>
      </section>

      <!-- 2. INFINITE TECH MARQUEE (CORREGIDO) -->
      <div class="tech-marquee-container">
        <div class="tech-marquee-track">
          <!-- GRUPO 1 -->
          <div class="tech-group">
            <q-icon :name="`img:${imgRos2}`" />
            <span class="tech-text">ROS 2 JAZZY</span>
            <q-icon :name="`img:${imgPython}`" />
            <span class="tech-text">PYTHON</span>
            <q-icon :name="`img:${imgCpp}`" />
            <span class="tech-text">C++ </span>
            <q-icon :name="`img:${imgDds}`" />
            <span class="tech-text">DDS</span>
            <q-icon name="terminal" />
            <span class="tech-text">BASH / LINUX</span>
            <q-icon :name="`img:${imgCmake}`" />
            <span class="tech-text">CMAKE</span>
            <q-icon :name="`img:${imgGazebo}`" />
            <span class="tech-text">GAZEBO</span>
            <q-icon :name="`img:${imgNav2}`" />
            <span class="tech-text">NAV 2</span>
            <q-icon :name="`img:${imgDocker}`" />
            <span class="tech-text">DOCKER</span>
            <q-icon :name="`img:${imgQuasar}`" />
            <span class="tech-text">QUASAR</span>
          </div>

          <!-- GRUPO 2 (DUPLICADO PARA LOOP INFINITO) -->
          <div class="tech-group">
            <q-icon :name="`img:${imgRos2}`" />
            <span class="tech-text">ROS 2 JAZZY</span>
            <q-icon :name="`img:${imgPython}`" />
            <span class="tech-text">PYTHON</span>
            <q-icon :name="`img:${imgCpp}`" />
            <span class="tech-text">C++</span>
            <q-icon :name="`img:${imgDds}`" />
            <span class="tech-text">DDS</span>
            <q-icon name="terminal" />
            <span class="tech-text">BASH / LINUX</span>
            <q-icon :name="`img:${imgCmake}`" />
            <span class="tech-text">CMAKE</span>
            <q-icon :name="`img:${imgGazebo}`" />
            <span class="tech-text">GAZEBO</span>
            <q-icon :name="`img:${imgNav2}`" />
            <span class="tech-text">NAV 2</span>
            <q-icon :name="`img:${imgDocker}`" />
            <span class="tech-text">DOCKER</span>
            <q-icon :name="`img:${imgQuasar}`" />
            <span class="tech-text">QUASAR</span>
          </div>
        </div>
      </div>

      <!-- 3. PROPUESTA DE VALOR (NUEVO) -->
      <section class="section-padding">
        <div class="content-container">
          <div class="row q-col-gutter-lg">
            <div class="col-12 col-md-4">
              <div class="feature-card">
                <div class="icon-box bg-purple-9"><q-icon name="view_in_ar" color="white" /></div>
                <h3>1. Simula</h3>
                <p>
                  Aprende sin riesgos. Domina la física y los sensores en mundos virtuales antes de
                  tocar el hardware.
                </p>
              </div>
            </div>
            <div class="col-12 col-md-4">
              <div class="feature-card">
                <div class="icon-box bg-blue-9"><q-icon name="code" color="white" /></div>
                <h3>2. Codifica</h3>
                <p>
                  Desarrolla nodos en Python moderno. Estructura tu código con Programación
                  Orientada a Objetos.
                </p>
              </div>
            </div>
            <div class="col-12 col-md-4">
              <div class="feature-card">
                <div class="icon-box bg-green-9"><q-icon name="rocket_launch" color="white" /></div>
                <h3>3. Despliega</h3>
                <p>
                  Lleva tu inteligencia al mundo real. Navegación, mapeo y control en robots
                  físicos.
                </p>
              </div>
            </div>
          </div>
        </div>
      </section>

      <!-- 4. ROADMAP VISUAL -->
      <section class="section-padding bg-dark-alt">
        <div class="content-container">
          <div class="text-center q-mb-xl">
            <h2 class="section-title">Mapa de Arquitectura</h2>
            <p class="section-subtitle">Visión global del ecosistema UP-Ros.</p>
          </div>

          <div class="roadmap-card">
            <img
              src="~assets/images/Ruta_aprendizaje.png"
              alt="Roadmap del Curso"
              class="roadmap-img"
            />
          </div>
        </div>
      </section>
      <!-- 5. DASHBOARD DE MÓDULOS (DINÁMICO) -->
      <section class="section-padding">
        <div class="content-container">
          <div class="text-center q-mb-xl">
            <h2 class="section-title">Plan de Estudios</h2>
            <p class="section-subtitle">Selecciona un módulo para comenzar.</p>
          </div>

          <div class="modules-grid">
            <!-- Bucle Dinámico sobre los Módulos -->
            <template v-for="module in courseStructure" :key="module.path || 'unknown'">
              <!-- VALIDACIÓN 1: Aseguramos que path y children existen antes de renderizar -->
              <router-link
                v-if="module.path && module.children && module.children.length > 0"
                :to="`/${module.path}/${module.children[0]?.path || ''}`"
                class="module-card group"
                :class="{ 'full-width-card': module.path.includes('modulo-8') }"
              >
                <div class="card-glow"></div>

                <!-- Icono -->
                <div class="card-icon-box" :class="getModuleBgColor(module.path)">
                  <q-icon :name="module.icon || 'folder'" color="white" size="24px" />
                </div>

                <div class="card-content">
                  <!-- Título Pequeño -->
                  <div class="text-overline" :class="getModuleTextColor(module.path)">
                    {{ formatModuleId(module.title || 'Módulo') }}
                  </div>

                  <!-- Título Principal -->
                  <h3 class="card-title">{{ formatModuleTitle(module.title || 'Sin Título') }}</h3>

                  <!-- Descripción -->
                  <p class="card-desc">
                    {{ getModuleDescription(module.path) }}
                  </p>

                  <!-- Tags -->
                  <div class="card-tags">
                    <q-chip
                      dense
                      color="blue-grey-9"
                      :text-color="getChipColor(module.path)"
                      size="sm"
                    >
                      {{ module.children.length }} Lecciones
                    </q-chip>
                    <q-chip
                      dense
                      color="blue-grey-9"
                      :text-color="getChipColor(module.path)"
                      size="sm"
                    >
                      Práctico
                    </q-chip>
                  </div>
                </div>

                <div v-if="module.path.includes('modulo-8')" class="action-arrow">
                  <span class="q-mr-sm text-caption text-weight-bold">IR AL DESAFÍO</span>
                  <q-icon name="arrow_forward" />
                </div>
                <q-icon v-else name="arrow_forward" class="card-arrow" />
              </router-link>
            </template>
          </div>
        </div>
      </section>

      <!-- FOOTER SIMPLE -->
      <div class="text-center text-grey-7 q-py-lg text-caption border-t border-white-5">
        UP-Ros v1.0 • Ingeniería Mecatrónica • Universidad de Pamplona
      </div>
    </div>
  </q-page>
</template>

<script setup lang="ts">
// En <script setup lang="ts">
import { courseStructure } from 'src/data/courseStructure';

import imgRos2 from 'src/assets/images/ros2.png';
import imgPython from 'src/assets/images/Python.png';
import imgCpp from 'src/assets/images/ISO_C++_Logo.png';
import imgDds from 'src/assets/images/dds.png';
import imgCmake from 'src/assets/images/Cmake.png';
import imgGazebo from 'src/assets/images/gazebo.png';
import imgNav2 from 'src/assets/images/nav2.png';
import imgDocker from 'src/assets/images/docker.png';
import imgQuasar from 'src/assets/images/quasar.png';

// --- HELPERS PARA LAS TARJETAS ---

// VALIDACIÓN 3: Manejo ultra-seguro de strings y arrays
const formatModuleId = (title?: string) => {
  if (!title) return 'MOD';
  // Agregamos ( ... || '') para garantizar que toUpperCase siempre tenga un string
  return (title.split(':')[0] || 'MOD').toUpperCase();
};

const formatModuleTitle = (title?: string) => {
  if (!title) return 'Sin Título';
  // Ya usabas ?. aquí, está bien, pero aseguramos el fallback final
  return title.split(':')[1]?.trim() || title;
};

// Descripciones
const getModuleDescription = (path: string) => {
  const descriptions: Record<string, string> = {
    'modulo-0': 'Fundamentos de Linux, Python, Git y configuración del Workspace.',
    'modulo-1': 'Core de ROS 2: Nodos, Tópicos, Servicios, Acciones y Grafo.',
    'modulo-2': 'Arquitectura: Launch files, Parámetros y Composición compleja.',
    'modulo-3': 'Herramientas avanzadas de desarrollo y depuración.', // Ajusta según tu temario
    'modulo-4': 'Sistemas embebidos y control de hardware.',
    'modulo-5': 'Percepción y procesamiento de sensores.',
    'modulo-6': 'Simulación física con Gazebo, URDF y RViz.',
    'modulo-7': 'Navegación autónoma (Nav2) y SLAM.',
    'modulo-8': 'Despliegue profesional e Integración Final.',
  };
  return descriptions[path] || 'Contenido avanzado de robótica profesional.';
};

// Colores de Fondo
const getModuleBgColor = (path: string) => {
  if (path.includes('modulo-0')) return 'bg-blue-9'; // Azul (Setup)
  if (path.includes('modulo-1')) return 'bg-purple-9'; // Morado (Core)
  if (path.includes('modulo-2')) return 'bg-cyan-9'; // Cyan (Arch)
  if (path.includes('modulo-3')) return 'bg-amber-9';
  if (path.includes('modulo-4')) return 'bg-red-9';
  if (path.includes('modulo-5')) return 'bg-indigo-9';
  if (path.includes('modulo-6')) return 'bg-orange-9'; // Naranja (Sim)
  if (path.includes('modulo-7')) return 'bg-pink-9';
  if (path.includes('modulo-8')) return 'bg-green-9'; // Verde (Capstone)
  return 'bg-grey-8';
};

// Colores de Texto
const getModuleTextColor = (path: string) => {
  if (path.includes('modulo-0')) return 'text-blue-4';
  if (path.includes('modulo-1')) return 'text-purple-4';
  if (path.includes('modulo-2')) return 'text-cyan-4';
  if (path.includes('modulo-3')) return 'text-amber-4';
  if (path.includes('modulo-4')) return 'text-red-4';
  if (path.includes('modulo-5')) return 'text-indigo-4';
  if (path.includes('modulo-6')) return 'text-orange-4';
  if (path.includes('modulo-7')) return 'text-pink-4';
  if (path.includes('modulo-8')) return 'text-green-4';
  return 'text-grey-4';
};

// Colores Chips
const getChipColor = (path: string) => {
  if (path.includes('modulo-0')) return 'blue-2';
  if (path.includes('modulo-1')) return 'purple-2';
  if (path.includes('modulo-2')) return 'cyan-2';
  if (path.includes('modulo-3')) return 'amber-2';
  if (path.includes('modulo-4')) return 'red-2';
  if (path.includes('modulo-6')) return 'orange-2';
  if (path.includes('modulo-8')) return 'green-2';
  return 'white';
};
</script>

<style scoped>
/* --- UTILS --- */
.bg-slate-dark {
  background-color: #0f172a;
}

.bg-dark-alt {
  background-color: #0b1120;
} /* Fondo alternativo más oscuro */
.content-container {
  width: 100%;
  max-width: 1200px;
  margin: 0 auto;
  padding: 0 24px;
  box-sizing: border-box;
  position: relative;
  z-index: 2; /* Solo por encima del grid, no del menú */
}

.text-gradient {
  background: linear-gradient(to right, #38bdf8, #818cf8);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
}
.section-padding {
  padding: 5rem 0;
}
.border-t {
  border-top: 1px solid rgba(255, 255, 255, 0.05);
}

.hero-section {
  padding: 8rem 0 5rem 0;
  position: relative;
  width: 100%;
  max-width: 100%;
  overflow: hidden; /* Cambia overflow-x por overflow simple */
  z-index: 1; /* Mantenerlo bajo */
}

/* Efecto Grid de Fondo */
.cyber-grid {
  position: absolute;
  width: 200%;
  height: 200%;
  top: -50%;
  left: -50%;
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
  0% {
    transform: perspective(500px) rotateX(60deg) translateY(0);
  }
  100% {
    transform: perspective(500px) rotateX(60deg) translateY(50px);
  }
}

.hero-glow {
  position: absolute;
  top: 0;
  right: 0;
  width: 60%;
  height: 100%;
  background: radial-gradient(circle at center, rgba(56, 189, 248, 0.08) 0%, transparent 70%);
  pointer-events: none;
  z-index: 0;
}

/* Typography & Badges */
.terminal-badge {
  font-family: 'Fira Code', monospace;
  font-size: 0.9rem;
  color: #cbd5e1;
  background: rgba(15, 23, 42, 0.8);
  border: 1px solid rgba(255, 255, 255, 0.1);
  padding: 8px 16px;
  border-radius: 8px;
  display: inline-block;
}
.cursor-blink {
  animation: blink 1s step-end infinite;
}
@keyframes blink {
  50% {
    opacity: 0;
  }
}

.hero-title {
  font-size: 3.5rem;
  font-weight: 800;
  line-height: 1.1;
  letter-spacing: -1px;
  z-index: 2;
  position: relative;
}
.hero-desc {
  font-size: 1.15rem;
  line-height: 1.6;
  max-width: 550px;
}

.cta-btn.shadow-glow {
  box-shadow: 0 0 20px rgba(56, 189, 248, 0.4);
}
.cta-btn:hover {
  transform: translateY(-2px);
  box-shadow: 0 0 30px rgba(56, 189, 248, 0.6);
}
/* Agrega esto a tu CSS */
.page-content-wrapper {
  width: 100%;
  overflow-x: hidden; /* Aquí sí es seguro usarlo */
  display: flex;
  flex-direction: column;
}

/* Robot */
.robot-wrapper {
  position: relative;
  width: 100%;
  max-width: 480px;
}
.hero-robot {
  width: 100%;
  height: auto;
  filter: drop-shadow(0 0 30px rgba(0, 0, 0, 0.5));
  position: relative;
  z-index: 0.3;
}
.floating-anim {
  animation: float 20s ease-in-out infinite;
}
@keyframes float {
  0%,
  100% {
    transform: translateY(0);
  }
  50% {
    transform: translateY(-20px);
  }
}

/* --- INFINITE MARQUEE --- */
/* --- INFINITE MARQUEE --- */
.tech-marquee-container {
  background: #0b1120;
  padding: 24px 0;
  border-top: 1px solid rgba(255, 255, 255, 0.05);
  border-bottom: 1px solid rgba(255, 255, 255, 0.05);
  position: relative;

  /* CONTENEDOR PADRE: */
  width: 100%;
  overflow: hidden; /* Corta lo que sobra visualmente */
  display: flex; /* Alineación vertical centrada */
  align-items: center;
  z-index: 1;
}

/* Sombras laterales (Opcional, mantienen el estilo) */
.tech-marquee-container::before,
.tech-marquee-container::after {
  content: '';
  position: absolute;
  top: 0;
  width: 100px;
  height: 100%;
  z-index: 2;
  pointer-events: none;
}
.tech-marquee-container::before {
  left: 0;
  background: linear-gradient(to right, #0b1120, transparent);
}
.tech-marquee-container::after {
  right: 0;
  background: linear-gradient(to left, #0b1120, transparent);
}

.tech-marquee-track {
  display: flex;
  /* CLAVE 1: El ancho debe ser el contenido total, ignorando el ancho de pantalla */
  width: max-content;
  /* CLAVE 2: Forzamos a que nunca haga salto de línea */
  flex-wrap: nowrap;
  animation: scroll 40s linear infinite; /* Ajusté a 40s para que sea más suave */
}

/* Si pones el mouse encima, se pausa (Opcional, buen UX) */
.tech-marquee-track:hover {
  animation-play-state: paused;
}

.tech-group {
  display: flex;
  gap: 4rem;
  align-items: center;
  /* CLAVE 3: Impedir que este grupo se encoja por flexbox */
  flex-shrink: 0;
  padding-right: 4rem; /* Añadimos espacio al final para el loop */
}

.tech-text {
  font-weight: 700;
  color: #475569;
  font-size: 1rem;
  letter-spacing: 1px;
  /* CLAVE 4: Asegurar que el texto nunca se parta en dos líneas */
  white-space: nowrap;
}

.tech-marquee-track .q-icon {
  font-size: 28px;
  color: #64748b;
  /* Asegura que el icono tampoco se encoja */
  flex-shrink: 0;
}

@keyframes scroll {
  0% {
    transform: translateX(0);
  }
  100% {
    /* Mueve la mitad exacta del contenido (asumiendo que tienes 2 grupos idénticos) */
    transform: translateX(-50%);
  }
}

/* --- FEATURE CARDS (Propuesta Valor) --- */
.feature-card {
  background: rgba(30, 41, 59, 0.3);
  padding: 30px;
  border-radius: 16px;
  border: 1px solid rgba(255, 255, 255, 0.05);
  transition: transform 0.3s;
}
.feature-card:hover {
  transform: translateY(-5px);
  background: rgba(30, 41, 59, 0.5);
}
.icon-box {
  width: 48px;
  height: 48px;
  border-radius: 10px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 24px;
  margin-bottom: 16px;
}
.feature-card h3 {
  font-size: 1.25rem;
  font-weight: 700;
  margin-bottom: 8px;
  color: white;
}
.feature-card p {
  font-size: 0.95rem;
  color: #94a3b8;
  line-height: 1.5;
}

/* --- MODULES GRID --- */
.section-title {
  font-size: 2.5rem;
  font-weight: 800;
  margin-bottom: 0.5rem;
}
.section-subtitle {
  color: #94a3b8;
  font-size: 1.1rem;
}

.modules-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
  gap: 24px;
}

.module-card {
  position: relative;
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 20px;
  padding: 30px;
  display: flex;
  flex-direction: column;
  gap: 20px;
  text-decoration: none;
  overflow: hidden;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}
.full-width-card {
  grid-column: 1 / -1;
  flex-direction: row;
  align-items: center;
}

.module-card:hover {
  transform: translateY(-5px);
  background: rgba(30, 41, 59, 0.8);
  border-color: rgba(56, 189, 248, 0.3);
  box-shadow: 0 15px 30px -10px rgba(0, 0, 0, 0.5);
}

.card-glow {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 5px;
  background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.2), transparent);
  opacity: 0;
  transition: opacity 0.3s;
}
.module-card:hover .card-glow {
  opacity: 1;
}

.card-icon-box {
  width: 50px;
  height: 50px;
  border-radius: 12px;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0 10px 20px rgba(0, 0, 0, 0.2);
}
.card-content {
  flex: 1;
}
.card-title {
  font-size: 1.25rem;
  font-weight: 700;
  color: white;
  margin: 4px 0 8px 0;
}
.card-desc {
  color: #94a3b8;
  font-size: 0.9rem;
  line-height: 1.5;
  margin-bottom: 12px;
}
.card-tags {
  display: flex;
  gap: 8px;
  flex-wrap: wrap;
}
.card-arrow {
  color: #475569;
  transition: all 0.3s;
  align-self: flex-start;
}
.module-card:hover .card-arrow {
  color: #38bdf8;
  transform: translateX(5px);
}
.action-arrow {
  display: flex;
  align-items: center;
  color: #4ade80;
  font-weight: 700;
}

/* --- ROADMAP --- */
.roadmap-card {
  background: rgba(30, 41, 59, 0.3);
  border: 1px solid rgba(255, 255, 255, 0.05);
  border-radius: 20px;
  padding: 10px;
  backdrop-filter: blur(10px);
  display: flex;
  justify-content: center; /* Centra horizontalmente (eje principal) */
  align-items: center; /* Centra verticalmente (eje secundario) */
}

.roadmap-img {
  width: 99%;
  border-radius: 12px;
  min-width: 350px;
  max-width: 100%;
}

/* RESPONSIVE */
@media (max-width: 900px) {
  .hero-title {
    font-size: 3rem;
  }
  .full-width-card {
    flex-direction: column;
    align-items: flex-start;
  }
}
@media (max-width: 600px) {
  .hero-section {
    padding: 8rem 0 5rem 0;
    position: relative;
    /* IMPORTANTE: Forzar al contenedor a no exceder el ancho disponible */
    width: 100%;
    max-width: 100%;
    overflow-x: hidden; /* Corta el grid sobrante */
  }

  .hero-title {
    font-size: 2.5rem;
  }
  .content-container .row.justify-between {
    justify-content: center;
    gap: 1rem;
  }
  .stats-row {
    justify-content: center;
    gap: 1rem;
  }
}
</style>
