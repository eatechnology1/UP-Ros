<template>
  <q-page class="q-pa-xl column items-center justify-start up-hero-page">
    <!-- HERO PRINCIPAL -->
    <section class="up-hero q-pa-xl q-mb-xl">
      <div class="up-hero-content">
        <div class="up-hero-text">
          <div class="text-overline text-primary text-weight-bold q-mb-sm">
            Plataforma educativa para ROS 2
          </div>

          <h1 class="text-h3 text-weight-bold q-mb-md">
            UP-Ros: aprende ROS 2 de forma progresiva y guiada
          </h1>

          <p class="text-subtitle1 text-grey-4 q-mb-lg">
            Explora conceptos, instala tu entorno, practica con ejemplos y lleva ROS 2 a robots
            reales a través de un recorrido estructurado, visualmente claro y orientado a la
            práctica.
          </p>

          <div class="row q-gutter-sm q-gutter-md-sm q-mt-sm">
            <q-btn
              color="primary"
              unelevated
              no-caps
              class="up-btn-main"
              label="Empezar por la Introducción"
              to="/introduccion"
            />
            <q-btn
              color="secondary"
              outline
              no-caps
              class="up-btn-secondary"
              label="Ir a Instalación de ROS 2"
              to="/instalacion"
            />
          </div>
        </div>

        <div class="up-hero-visual">
          <q-card flat bordered class="up-hero-card">
            <q-card-section class="up-hero-image-wrapper">
              <img
                src="src/assets/images/jazzy_robot.png"
                alt="Robot educativo UP-Ros"
                class="up-hero-image"
              />
            </q-card-section>

            <q-separator color="grey-8" />

            <q-card-section>
              <div class="text-caption text-primary text-weight-bold q-mb-xs">Ruta sugerida</div>
              <div class="text-body2 text-grey-4">
                1. Introducción a ROS 2 <br />
                2. Instalación del entorno <br />
                3. Fundamentos y nodos básicos <br />
                4. Ejemplos, simulación y proyecto con robot
              </div>
            </q-card-section>
          </q-card>
        </div>
      </div>
    </section>

    <!-- SECCIONES PRINCIPALES COMO HUB -->
    <section class="up-sections q-mb-xl full-width">
      <div class="text-subtitle1 text-grey-4 q-mb-md text-center">
        Navega por los bloques principales del contenido
      </div>

      <div class="row q-col-gutter-lg q-row-gutter-lg justify-center items-stretch">
        <div
          v-for="section in mainSections"
          :key="section.to"
          class="col-12 col-sm-6 col-md-3 flex"
        >
          <q-card
            class="up-section-card cursor-pointer full-width"
            flat
            bordered
            @click="$router.push(section.to)"
          >
            <q-card-section class="column justify-between up-section-inner">
              <div>
                <div class="row items-center no-wrap q-mb-sm">
                  <q-icon :name="section.icon" size="md" class="q-mr-sm text-primary" />
                  <div class="text-h6 text-weight-medium">
                    {{ section.title }}
                  </div>
                </div>
                <div class="text-body2 text-grey-5">
                  {{ section.description }}
                </div>
              </div>
            </q-card-section>
          </q-card>
        </div>
      </div>
    </section>

    <!-- CÓMO ESTÁ ORGANIZADO EL RECORRIDO -->
    <section class="up-flow q-pa-lg q-mb-xl">
      <div class="text-subtitle1 text-weight-bold q-mb-md text-primary">
        ¿Cómo está organizado el recorrido de aprendizaje?
      </div>

      <div class="row q-col-gutter-md q-row-gutter-sm">
        <div
          v-for="(step, index) in learningFlow"
          :key="index"
          class="col-12 col-md-3 up-flow-step"
        >
          <div class="text-h5 text-primary text-weight-bold q-mb-xs">{{ index + 1 }}.</div>
          <div class="text-body1 text-grey-3 text-weight-medium q-mb-xs">
            {{ step.title }}
          </div>
          <div class="text-body2 text-grey-5">
            {{ step.description }}
          </div>
        </div>
      </div>
    </section>

    <!-- MENSAJE DOCENTE / FILOSOFÍA -->
    <section class="up-message q-pa-lg q-mb-xl">
      <div class="text-subtitle1 text-weight-bold q-mb-sm">Enfoque de UP-Ros</div>
      <p class="text-body2 text-grey-4">
        UP-Ros está diseñado para integrar rigor técnico, documentación clara y visualización
        efectiva de conceptos clave en ROS 2. La plataforma prioriza una curva de aprendizaje
        progresiva, apoyada en ejemplos prácticos, simulación y proyectos con robots reales, para
        facilitar la transferencia de conocimiento a estudiantes y profesionales.
      </p>
    </section>
  </q-page>
</template>

<script lang="ts">
import { defineComponent } from 'vue';

interface MainSection {
  title: string;
  description: string;
  to: string;
  icon: string;
}

interface LearningStep {
  title: string;
  description: string;
}

export default defineComponent({
  name: 'IndexPage',

  setup() {
    const mainSections: MainSection[] = [
      {
        title: 'Introducción',
        description:
          'Comprende la filosofía de ROS 2, su arquitectura y el enfoque de la plataforma UP-Ros.',
        to: '/introduccion',
        icon: 'school',
      },
      {
        title: 'Instalación',
        description:
          'Prepara tu entorno de desarrollo con una guía paso a paso para instalar ROS 2 correctamente.',
        to: '/instalacion',
        icon: 'download',
      },
      {
        title: 'Fundamentos',
        description:
          'Aprende los conceptos esenciales: nodos, tópicos, servicios, acciones y paquetes en ROS 2.',
        to: '/fundamentos',
        icon: 'memory',
      },
      {
        title: 'Ejemplos prácticos',
        description:
          'Desarrolla y ejecuta ejemplos guiados para afianzar los conceptos clave de ROS 2.',
        to: '/ejemplos',
        icon: 'play_circle_filled',
      },
      {
        title: 'Simulación',
        description:
          'Trabaja con simuladores para probar tus nodos y paquetes antes de llevarlos a hardware real.',
        to: '/simulacion',
        icon: 'videogame_asset',
      },
      {
        title: 'Proyecto con Robot',
        description:
          'Integra ROS 2 en un proyecto real con robot, aplicando los conceptos aprendidos.',
        to: '/proyecto',
        icon: 'precision_manufacturing',
      },
      {
        title: 'Glosario',
        description:
          'Consulta términos clave, acrónimos y conceptos relacionados con ROS 2 y robótica.',
        to: '/glosario',
        icon: 'book',
      },
      {
        title: 'Créditos',
        description:
          'Conoce la autoría, referencias y recursos utilizados para el desarrollo de UP-Ros.',
        to: '/creditos',
        icon: 'info',
      },
    ];

    const learningFlow: LearningStep[] = [
      {
        title: 'Comprender el contexto de ROS_2',
        description:
          'Revisa la introducción y los fundamentos para entender qué resuelve ROS 2 y cómo se estructura.',
      },
      {
        title: 'Preparar el entorno',
        description:
          'Sigue la guía de instalación para configurar tu sistema y verificar que todo funciona correctamente.',
      },
      {
        title: 'Explorar ejemplos y simulación',
        description:
          'Desarrolla ejemplos guiados y utiliza simuladores para experimentar sin riesgo sobre hardware real.',
      },
      {
        title: 'Aplicar en un proyecto con robot',
        description:
          'Transfiere lo aprendido a un proyecto con robot, integrando control, comunicación y herramientas ROS 2.',
      },
    ];

    return {
      mainSections,
      learningFlow,
    };
  },
});
</script>

<style scoped>
.up-hero-page {
  gap: 2rem;
}

/* HERO */
.up-hero {
  width: 100%;
  max-width: 1100px;
  border-radius: 24px;
  backdrop-filter: blur(18px);
  -webkit-backdrop-filter: blur(18px);
  background: rgba(15, 23, 42, 0.7);
  border: 1px solid rgba(148, 163, 184, 0.3);
}

.up-hero-content {
  display: grid;
  grid-template-columns: minmax(0, 2fr) minmax(0, 1.3fr);
  gap: 2.5rem;
  align-items: center;
}

.up-hero-text {
  max-width: 620px;
}

.up-hero-visual {
  display: flex;
  justify-content: flex-end;
}

.up-hero-card {
  min-width: 260px;
  border-radius: 18px;
  background: rgba(15, 23, 42, 0.8);
  overflow: hidden;
}

/* IMAGEN JAZZY_ROBOT */
.up-hero-image-wrapper {
  padding: 0;
}

.up-hero-image {
  display: block;
  width: 100%;
  height: auto; /* ← CAMBIA: altura automática */
  max-height: 240px; /* ← Aumenta un poco el espacio disponible */
  object-fit: contain; /* ← MUESTRA COMPLETA SIN CORTAR */
  object-position: center; /* ← Centra la imagen */
}

/* BOTONES HERO */
.up-btn-main {
  border-radius: 999px;
  padding-inline: 1.5rem;
}

.up-btn-secondary {
  border-radius: 999px;
  padding-inline: 1.2rem;
}

/* CARDS SECCIONES - ALINEADAS EN ALTO */
.up-sections {
  max-width: 1100px;
}

.up-section-card {
  width: 100%;
  height: 100%;
  border-radius: 18px;
  backdrop-filter: blur(14px);
  -webkit-backdrop-filter: blur(14px);
  background: rgba(15, 23, 42, 0.65);
  transition:
    transform 0.18s ease,
    box-shadow 0.18s ease,
    background 0.18s ease;
}

.up-section-card:hover {
  transform: translateY(-4px);
  box-shadow: 0 18px 40px rgba(15, 23, 42, 0.7);
  background: rgba(15, 23, 42, 0.85);
}

.up-section-inner {
  height: 100%;
}

/* FLUJO DE APRENDIZAJE */
.up-flow {
  width: 100%;
  max-width: 1100px;
  border-radius: 18px;
  backdrop-filter: blur(16px);
  -webkit-backdrop-filter: blur(16px);
  background: rgba(15, 23, 42, 0.75);
  border: 1px solid rgba(148, 163, 184, 0.25);
}

.up-flow-step {
  padding: 0.4rem 0.2rem;
}

/* MENSAJE DOCENTE */
.up-message {
  width: 100%;
  max-width: 900px;
  border-radius: 18px;
  border: 1px dashed rgba(148, 163, 184, 0.5);
  background: rgba(15, 23, 42, 0.6);
}

/* RESPONSIVE */
@media (max-width: 960px) {
  .up-hero-content {
    grid-template-columns: minmax(0, 1fr);
  }

  .up-hero-visual {
    justify-content: flex-start;
  }

  .up-hero {
    padding: 1.5rem;
  }
}
</style>
