<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO SECTION -->
    <!-- Usamos SplitBlock para dividir Contenido (Izq) y Visual (Der) -->
    <div class="section-group self-stretch q-mt-md">
      <SplitBlock>
        <!-- IZQUIERDA: Mensaje Principal -->
        <template #left>
          <div class="hero-header">
            <div class="text-overline text-accent text-weight-bold q-mb-xs">
              Plataforma Educativa v1.0
            </div>
            <h1 class="hero-title">
              Aprende <span class="text-primary">ROS 2</span> de forma progresiva
            </h1>
          </div>

          <TextBlock>
            <strong>UP-Ros</strong> te guía desde la instalación hasta el control de robots reales.
            Explora conceptos, practica con simuladores y domina la robótica moderna con un enfoque
            estructurado y documentación de rigor técnico.
          </TextBlock>

          <div class="row q-gutter-md q-mt-md">
            <q-btn
              color="primary"
              unelevated
              rounded
              padding="10px 24px"
              icon-right="arrow_forward"
              label="Empezar Introducción"
              to="/introduccion"
              class="hero-btn"
            />
            <q-btn
              color="dark"
              text-color="white"
              outline
              rounded
              padding="10px 24px"
              icon="download"
              label="Instalar ROS 2"
              to="/instalacion"
              class="hero-btn"
            />
          </div>
        </template>

        <!-- DERECHA: Robot + Ruta Sugerida -->
        <template #right>
          <div class="column q-gutter-y-md">
            <!-- Imagen del Robot -->
            <ImageBlock
              src="src/assets/images/jazzy_robot.png"
              alt="Robot educativo UP-Ros"
              :zoomable="false"
              class="hero-image-block"
            />

            <!-- Checklist rápido (Ahora es un AlertBlock) -->
            <AlertBlock type="info" title="🚀 Ruta Sugerida">
              1. <strong>Introducción:</strong> Filosofía y Conceptos.<br />
              2. <strong>Instalación:</strong> Configura tu Ubuntu.<br />
              3. <strong>Fundamentos:</strong> Nodos y Tópicos.<br />
              4. <strong>Proyecto:</strong> Simulación y Hardware.
            </AlertBlock>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 2. HUB DE NAVEGACIÓN -->
    <div class="section-group self-stretch">
      <SectionTitle>Explora el Contenido</SectionTitle>

      <div class="modules-grid">
        <router-link
          v-for="section in mainSections"
          :key="section.to"
          :to="section.to"
          class="module-card"
        >
          <div class="module-icon-box">
            <q-icon :name="section.icon" size="28px" />
          </div>
          <div class="module-content">
            <div class="module-title">{{ section.title }}</div>
            <div class="module-desc">{{ section.description }}</div>
          </div>
          <div class="module-arrow">
            <q-icon name="chevron_right" />
          </div>
        </router-link>
      </div>
    </div>

    <!-- 3. METODOLOGÍA (Flow) -->
    <div class="section-group self-stretch">
      <SectionTitle>¿Cómo está organizado?</SectionTitle>

      <div class="flow-grid">
        <div v-for="(step, index) in learningFlow" :key="index" class="flow-card">
          <div class="flow-number">{{ index + 1 }}</div>
          <h3 class="flow-title">{{ step.title }}</h3>
          <p class="flow-desc">{{ step.description }}</p>
        </div>
      </div>
    </div>

    <!-- 4. MANIFIESTO (Enfoque) -->
    <div class="section-group self-stretch q-mb-xl">
      <TextBlock highlighted>
        <div class="text-h6 text-white q-mb-sm">🎓 Enfoque de UP-Ros</div>
        UP-Ros está diseñado para integrar <strong>rigor técnico</strong>,
        <strong>documentación clara</strong> y <strong>visualización efectiva</strong>. Priorizamos
        una curva de aprendizaje progresiva para facilitar la transferencia de conocimiento real a
        estudiantes de ingeniería y profesionales.
      </TextBlock>
    </div>
  </q-page>
</template>

<script setup lang="ts">
import SplitBlock from 'components/content/SplitBlock.vue';
import TextBlock from 'components/content/TextBlock.vue';
import ImageBlock from 'components/content/ImageBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';

// DATOS
const mainSections = [
  {
    title: 'Introducción',
    description: 'Filosofía, arquitectura y enfoque.',
    to: '/introduccion',
    icon: 'school',
  },
  {
    title: 'Fundamentos',
    description: 'Nodos, tópicos, servicios y acciones.',
    to: '/fundamentos',
    icon: 'memory', // Icono de chip/proceso
  },
  {
    title: 'Instalación',
    description: 'Guía paso a paso para Ubuntu 24.04.',
    to: '/instalacion',
    icon: 'terminal', // Más técnico que download
  },
  {
    title: 'Ejemplos Prácticos',
    description: 'Código Python explicado línea a línea.',
    to: '/ejemplos',
    icon: 'code',
  },
  {
    title: 'Simulación',
    description: 'Gazebo y RViz sin riesgos.',
    to: '/simulacion',
    icon: 'view_in_ar', // Icono de 3D/AR
  },
  {
    title: 'Proyecto Robot',
    description: 'Integración final en hardware.',
    to: '/proyecto',
    icon: 'smart_toy',
  },
];

const learningFlow = [
  {
    title: 'Contexto',
    description: 'Entiende qué resuelve ROS 2 antes de escribir código.',
  },
  {
    title: 'Entorno',
    description: 'Prepara tu sistema Linux con las herramientas profesionales.',
  },
  {
    title: 'Experimentación',
    description: 'Usa simuladores para fallar rápido y aprender seguro.',
  },
  {
    title: 'Aplicación',
    description: 'Transfiere el código a un robot físico real.',
  },
];
</script>

<style scoped>
/* GENERAL LAYOUT */
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3rem auto;
}

/* HERO */
.hero-header {
  margin-bottom: 24px;
}

.hero-title {
  font-size: 3rem;
  font-weight: 800;
  line-height: 1.1;
  color: #f8fafc; /* Slate-50 */
  margin: 0;
}

.hero-btn {
  font-weight: 600;
  letter-spacing: 0.5px;
}

/* Ajuste específico para la imagen del hero para que no sea gigante */
:deep(.hero-image-block .image-container) {
  background: transparent !important;
  box-shadow: none !important;
  border: none !important;
}

:deep(.hero-image-block img) {
  max-height: 300px; /* Control de altura */
  object-fit: contain;
  filter: drop-shadow(0 10px 20px rgba(0, 0, 0, 0.3)); /* Sombra a la silueta PNG */
}

/* GRID DE MÓDULOS (HUB) */
.modules-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
  gap: 20px;
}

.module-card {
  display: flex;
  align-items: center;
  gap: 16px;
  padding: 20px;
  background: rgba(30, 41, 59, 0.4); /* Slate-800 Glass */
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  text-decoration: none;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  backdrop-filter: blur(8px);
}

.module-card:hover {
  background: rgba(30, 41, 59, 0.6);
  border-color: rgba(56, 189, 248, 0.4); /* Sky-400 */
  transform: translateY(-4px);
  box-shadow: 0 12px 24px -8px rgba(0, 0, 0, 0.3);
}

.module-icon-box {
  width: 48px;
  height: 48px;
  border-radius: 12px;
  background: rgba(56, 189, 248, 0.1);
  color: #38bdf8;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
  transition: background 0.3s;
}

.module-card:hover .module-icon-box {
  background: rgba(56, 189, 248, 0.2);
  color: #7dd3fc;
}

.module-content {
  flex: 1;
}

.module-title {
  color: #f1f5f9;
  font-weight: 600;
  font-size: 1rem;
  margin-bottom: 4px;
}

.module-desc {
  color: #94a3b8;
  font-size: 0.85rem;
  line-height: 1.4;
}

.module-arrow {
  color: #475569;
  transition:
    transform 0.3s,
    color 0.3s;
}

.module-card:hover .module-arrow {
  color: #38bdf8;
  transform: translateX(4px);
}

/* GRID DE FLUJO (STEPS VISUALES) */
.flow-grid {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 24px;
}

.flow-card {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  padding: 24px;
  position: relative;
  overflow: hidden;
}

.flow-number {
  font-size: 4rem;
  font-weight: 900;
  color: rgba(255, 255, 255, 0.03);
  position: absolute;
  top: -10px;
  right: -10px;
  line-height: 1;
}

.flow-title {
  color: #38bdf8;
  font-size: 1.1rem;
  font-weight: 700;
  margin: 0 0 12px 0;
  line-height: 1.3;
}

.flow-desc {
  color: #cbd5e1;
  font-size: 0.9rem;
  line-height: 1.5;
  margin: 0;
}

/* RESPONSIVE */
@media (max-width: 900px) {
  .hero-title {
    font-size: 2.2rem;
  }

  .flow-grid {
    grid-template-columns: 1fr 1fr; /* 2 columnas en tablets */
  }
}

@media (max-width: 600px) {
  .hero-title {
    font-size: 1.8rem;
  }

  .flow-grid {
    grid-template-columns: 1fr; /* 1 columna en móvil */
  }
}
</style>
