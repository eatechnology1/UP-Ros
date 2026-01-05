<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO: PROPÓSITO -->
    <!-- Usamos 'intro-hero' con backdrop-filter para un inicio impactante -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">Módulo 1</div>
        <h1 class="hero-title">Introducción a <span class="text-primary">ROS 2</span></h1>

        <TextBlock>
          <strong>UpROS Academy</strong> es tu puerta de entrada al desarrollo robótico profesional.
          Diseñamos este recorrido para reducir la curva de aprendizaje mediante una metodología
          <strong>teórico-práctica</strong>. <br /><br />
          Desde la instalación del entorno hasta la simulación y control de robots reales, aquí
          superarás las barreras de Linux y la terminal con acompañamiento paso a paso.
        </TextBlock>
      </div>
    </section>

    <!-- 2. OBJETIVOS (SplitBlock para balancear texto y visual) -->
    <div class="section-group self-stretch">
      <SectionTitle>🎯 Objetivos del Módulo</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Al finalizar este recorrido introductorio, habrás dominado las competencias clave para
            iniciar tu carrera en robótica:
          </TextBlock>
          <StepsBlock :steps="learningObjectives" />
        </template>

        <template #right>
          <AlertBlock title="🧠 Metodología UP-Ros" type="info">
            Aprendizaje cíclico y progresivo:<br />
            1. <strong>Concepto:</strong> Entiende el "por qué".<br />
            2. <strong>Código:</strong> Analiza el "cómo".<br />
            3. <strong>Simulación:</strong> Prueba sin riesgos.<br />
            4. <strong>Hardware:</strong> Despliega en el mundo real.
          </AlertBlock>

          <!-- Imagen decorativa pequeña para llenar espacio -->
          <div class="q-mt-md opacity-80">
            <ImageBlock
              src="src/assets/images/jazzy_robot.png"
              style="max-width: 200px; margin: 0 auto"
              :zoomable="false"
              alt="Robot concept"
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 3. GLOSARIO VISUAL (Conceptos Clave) -->
    <div class="section-group self-stretch">
      <SectionTitle>Conceptos Fundamentales</SectionTitle>

      <SplitBlock>
        <template #left>
          <ImageBlock
            src="src/assets/images/Nodes-TopicandService.gif"
            caption="Arquitectura de comunicación en ROS 2"
            credit="ROS 2 Documentation"
          />
        </template>

        <template #right>
          <TextBlock>
            Antes de escribir tu primera línea de código, es vital hablar el mismo idioma. La
            arquitectura de ROS 2 se basa en grafos de comunicación:
          </TextBlock>

          <div class="q-mt-md">
            <AlertBlock title="🔑 Diccionario Esencial" type="success">
              • <strong>Nodo:</strong> Tu programa (ej: driver de cámara).<br />
              • <strong>Tópico:</strong> Streaming de datos (TV broadcast).<br />
              • <strong>Servicio:</strong> Cliente/Servidor (Llamada telefónica).<br />
              • <strong>Paquete:</strong> La carpeta que agrupa tu código.
            </AlertBlock>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. JUSTIFICACIÓN TECNOLÓGICA -->
    <div class="section-group self-stretch">
      <SectionTitle>¿Por qué ROS 2 Jazzy (LTS)?</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            No usamos versiones antiguas. Apostamos por <strong>Jazzy Jalisco</strong>, la versión
            con soporte a largo plazo (LTS) hasta 2029.
          </TextBlock>
          <AlertBlock title="🚫 ¿Por qué no ROS 1?" type="warning">
            • <strong>Fin de vida:</strong> Sin soporte oficial.<br />
            • <strong>No Real-Time:</strong> No apto para industria crítica.<br />
            • <strong>Inseguro:</strong> Sin encriptación nativa.
          </AlertBlock>
        </template>
        <template #right>
          <TextBlock highlighted>
            <div class="text-h6 text-white q-mb-sm">Ventajas de ROS 2</div>
            El estándar industrial moderno ofrece:<br /><br />
            ✅ <strong>DDS (Data Distribution Service):</strong> Comunicación robusta.<br />
            ✅ <strong>Tiempo Real:</strong> Control determinista.<br />
            ✅ <strong>Multi-plataforma:</strong> Linux, Windows, macOS.
          </TextBlock>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. RUTA DE APRENDIZAJE (Roadmap Visual Mejorado) -->
    <div class="section-group self-stretch">
      <SectionTitle>Tu Ruta de Aprendizaje</SectionTitle>

      <!-- Reemplazamos la imagen estática por bloques visuales interactivos -->
      <div class="roadmap-container">
        <div class="roadmap-card" v-for="(step, i) in roadmapSteps" :key="i">
          <div class="step-badge">{{ i + 1 }}</div>
          <div class="step-icon">
            <q-icon :name="step.icon" />
          </div>
          <div class="step-info">
            <h4>{{ step.title }}</h4>
            <p>{{ step.desc }}</p>
          </div>
        </div>
      </div>
    </div>

    <!-- 6. VALIDACIÓN INICIAL -->
    <div class="section-group self-stretch">
      <SectionTitle>Validación Inmediata</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            ¿Listo para empezar? Abre tu terminal y ejecuta este comando. Si ves las versiones, tu
            sistema base está listo.
          </TextBlock>
          <AlertBlock title="✅ Checklist Previo" type="success">
            • Ubuntu 22.04 / 24.04 instalado.<br />
            • Terminal abierta (Ctrl+Alt+T).<br />
            • Ganas de aprender.
          </AlertBlock>
        </template>
        <template #right>
          <CodeBlock
            title="Verificación de Entorno"
            lang="bash"
            :content="checkCommand"
            :copyable="true"
          />
        </template>
      </SplitBlock>
    </div>

    <!-- 7. CTA / NAVEGACIÓN -->
    <div class="section-group self-stretch column items-center q-mt-xl">
      <div class="row q-gutter-md justify-center">
        <q-btn
          color="primary"
          unelevated
          rounded
          padding="12px 32px"
          to="/instalacion"
          icon="terminal"
          label="Comenzar Instalación"
          class="text-weight-bold"
        />
        <q-btn flat rounded color="white" icon="arrow_back" label="Volver al Inicio" to="/" />
      </div>
    </div>

    <!-- 8. RECURSOS DESCARGABLES -->
    <div class="section-group self-stretch q-mb-xl">
      <PdfBlock
        title="Syllabus y Cronograma"
        description="Descarga el plan de estudios detallado y la guía de referencia rápida."
        size="2.4 MB"
        src="src/assets/upros-roadmap.pdf"
      />
    </div>
  </q-page>
</template>

<script setup lang="ts">
import TextBlock from 'components/content/TextBlock.vue';
import AlertBlock from 'components/content/AlertBlock.vue';
import ImageBlock from 'components/content/ImageBlock.vue';
import CodeBlock from 'components/content/CodeBlock.vue';
import SectionTitle from 'components/content/SectionTitle.vue';
import SplitBlock from 'components/content/SplitBlock.vue';
import StepsBlock from 'components/content/StepsBlock.vue';
import PdfBlock from 'components/content/PdfBlock.vue';

const learningObjectives = [
  'Comprender la arquitectura distribuida de ROS 2 (Nodos, DDS).',
  'Diferenciar patrones de comunicación (Tópicos vs Servicios).',
  'Configurar un entorno de desarrollo profesional en Ubuntu.',
  'Desarrollar y depurar primeros nodos en Python.',
  'Integrar simulación (Gazebo) y hardware real.',
];

const checkCommand = `
# Verifica herramientas base
python3 --version
gcc --version
git --version
echo "¡Todo listo para ROS 2 Jazzy!"
`.trim();

const roadmapSteps = [
  { title: 'Fundamentos', desc: 'Teoría base y conceptos.', icon: 'school' },
  { title: 'Instalación', desc: 'Setup del entorno Linux.', icon: 'settings_suggest' },
  { title: 'Código', desc: 'Python, Nodos y Tópicos.', icon: 'code' },
  { title: 'Proyecto', desc: 'Simulación y Robot Real.', icon: 'smart_toy' },
];
</script>

<style scoped>
/* GENERAL LAYOUT */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1000px; /* Un poco más estrecho para facilitar lectura */
  margin: 0 auto 3.5rem auto;
}

/* HERO SECTION */
.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at top right, rgba(56, 189, 248, 0.1), transparent 40%),
    rgba(15, 23, 42, 0.6);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.1);
  text-align: center;
}

.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  line-height: 1.1;
  color: #f8fafc;
}

/* ROADMAP CARDS (Mejorado) */
.roadmap-container {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 20px;
}

.roadmap-card {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  padding: 24px;
  position: relative;
  overflow: hidden;
  transition: transform 0.3s ease;
}

.roadmap-card:hover {
  transform: translateY(-4px);
  background: rgba(30, 41, 59, 0.6);
  border-color: rgba(56, 189, 248, 0.3);
}

.step-badge {
  position: absolute;
  top: -10px;
  right: -10px;
  width: 60px;
  height: 60px;
  background: rgba(255, 255, 255, 0.03);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 2rem;
  font-weight: 900;
  color: rgba(255, 255, 255, 0.1);
  pointer-events: none;
}

.step-icon {
  width: 48px;
  height: 48px;
  background: rgba(56, 189, 248, 0.1);
  color: #38bdf8;
  border-radius: 12px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.5rem;
  margin-bottom: 16px;
}

.step-info h4 {
  color: #e2e8f0;
  font-size: 1.1rem;
  margin: 0 0 4px 0;
  font-weight: 700;
}

.step-info p {
  color: #94a3b8;
  font-size: 0.9rem;
  margin: 0;
  line-height: 1.4;
}

/* RESPONSIVE */
@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
  .intro-hero {
    padding: 2rem 1rem;
  }
}
</style>
