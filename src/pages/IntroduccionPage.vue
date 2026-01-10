<template>
  <q-page class="q-pa-lg column items-center">
    <!-- 1. HERO: PROPÓSITO -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <div class="text-overline text-accent text-weight-bold q-mb-sm">Módulo 1</div>
        <h1 class="hero-title">Introducción a <span class="text-primary">ROS 2</span></h1>

        <TextBlock>
          <strong>UpROS Academy</strong> es tu puerta de entrada al desarrollo robótico profesional.
          Diseñamos este recorrido para reducir la curva de aprendizaje mediante una metodología
          <strong>teórico-práctica</strong>. Supera las barreras de Linux y la terminal con
          acompañamiento paso a paso.
        </TextBlock>
      </div>
    </section>

    <!-- 2. OBJETIVOS -->
    <div class="section-group self-stretch">
      <SectionTitle>🎯 Objetivos del Módulo</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Al finalizar este recorrido, habrás dominado las competencias clave para iniciar tu
            carrera:
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

          <!-- Imagen ajustada al estilo -->
          <div class="row justify-center q-mt-md">
            <q-img
              src="src/assets/images/jazzy_robot.png"
              style="width: 180px; opacity: 0.8"
              alt="Robot Concept"
            />
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 3. GLOSARIO VISUAL -->
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
            Antes de escribir código, es vital hablar el mismo idioma. La arquitectura se basa en
            grafos:
          </TextBlock>

          <!-- Usamos el estilo trouble-grid para definiciones rápidas -->
          <div class="trouble-grid q-mt-md">
            <div class="trouble-item">
              <div class="trouble-q text-primary">Nodo</div>
              <div class="trouble-a">Tu programa ejecutable (ej: driver de cámara).</div>
            </div>
            <div class="trouble-item">
              <div class="trouble-q text-primary">Tópico</div>
              <div class="trouble-a">Streaming de datos continuo (TV broadcast).</div>
            </div>
            <div class="trouble-item">
              <div class="trouble-q text-primary">Servicio</div>
              <div class="trouble-a">Cliente/Servidor síncrono (Llamada telefónica).</div>
            </div>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 4. JUSTIFICACIÓN TECNOLÓGICA -->
    <div class="section-group self-stretch">
      <SectionTitle>¿Por qué ROS 2 Jazzy?</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            Apostamos por <strong>Jazzy Jalisco</strong>, la versión con soporte a largo plazo (LTS)
            hasta 2029.
          </TextBlock>
          <AlertBlock title="🚫 ¿Por qué no ROS 1?" type="warning">
            • <strong>Fin de vida:</strong> Sin soporte oficial.<br />
            • <strong>No Real-Time:</strong> No apto para industria crítica.<br />
            • <strong>Inseguro:</strong> Sin encriptación nativa.
          </AlertBlock>
        </template>

        <template #right>
          <!-- Convertido a Tool Card para destacar ventajas -->
          <div class="tool-card advantages">
            <div class="tool-header">
              <q-icon name="verified" size="md" color="white" />
              <h2>Ventajas Modernas</h2>
            </div>
            <ul class="tool-list">
              <li>✅ <strong>DDS:</strong> Comunicación industrial robusta.</li>
              <li>✅ <strong>Real-Time:</strong> Control determinista.</li>
              <li>✅ <strong>Multi-plataforma:</strong> Linux, Windows, macOS.</li>
              <li>✅ <strong>Seguridad:</strong> SROS2 nativo.</li>
            </ul>
          </div>
        </template>
      </SplitBlock>
    </div>

    <!-- 5. RUTA DE APRENDIZAJE (ROADMAP) -->
    <div class="section-group self-stretch">
      <SectionTitle>Tu Ruta de Aprendizaje</SectionTitle>

      <div class="row q-col-gutter-md">
        <div class="col-12 col-md-3" v-for="(step, i) in roadmapSteps" :key="i">
          <!-- Tool Card adaptada para pasos -->
          <div class="tool-card" :style="{ borderTopColor: step.color }">
            <div class="tool-header">
              <q-icon :name="step.icon" size="md" />
              <div class="text-h6 text-weight-bold">Paso {{ i + 1 }}</div>
            </div>
            <h3 class="text-subtitle1 text-white q-my-none">{{ step.title }}</h3>
            <p class="q-mt-sm text-caption text-grey-4">{{ step.desc }}</p>
          </div>
        </div>
      </div>
    </div>

    <!-- 6. VALIDACIÓN -->
    <div class="section-group self-stretch">
      <SectionTitle>Validación Inmediata</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            ¿Listo para empezar? Abre tu terminal y ejecuta este comando para verificar tu sistema
            base.
          </TextBlock>
          <AlertBlock title="✅ Checklist Previo" type="success">
            • Ubuntu 22.04 / 24.04 instalado.<br />
            • Terminal abierta (Ctrl+Alt+T).
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

    <!-- 7. CTA & RECURSOS -->
    <div class="section-group self-stretch column items-center q-mt-xl">
      <div class="row q-gutter-md justify-center q-mb-xl">
        <q-btn
          color="primary"
          unelevated
          rounded
          padding="12px 32px"
          to="/modulo-0/01nav-sistema"
          icon="terminal"
          label="Comenzar Aprendizaje"
          class="text-weight-bold"
        />
        <q-btn flat rounded color="white" icon="arrow_back" label="Volver al Inicio" to="/" />
      </div>

      <div class="full-width" style="max-width: 800px">
        <PdfBlock
          title="Syllabus y Cronograma"
          description="Descarga el plan de estudios detallado y la guía de referencia rápida."
          size="2.4 MB"
          src="src/assets/upros-roadmap.pdf"
        />
      </div>
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
  { title: 'Fundamentos', desc: 'Teoría base y conceptos.', icon: 'school', color: '#4ade80' }, // Green
  {
    title: 'Instalación',
    desc: 'Setup del entorno Linux.',
    icon: 'settings_suggest',
    color: '#38bdf8',
  }, // Blue
  { title: 'Código', desc: 'Python, Nodos y Tópicos.', icon: 'code', color: '#a78bfa' }, // Purple
  { title: 'Proyecto', desc: 'Simulación y Robot Real.', icon: 'smart_toy', color: '#f97316' }, // Orange
];
</script>

<style scoped>
/* --- ESTILOS MAESTROS --- */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px; /* Estandarizado a 1100px */
  margin: 0 auto 3.5rem auto;
}

.intro-hero {
  padding: 3rem 2rem;
  background:
    radial-gradient(circle at top left, rgba(236, 72, 153, 0.15), transparent 60%),
    rgba(15, 23, 42, 0.8);
  backdrop-filter: blur(20px);
  border-radius: 24px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  text-align: center;
}

.hero-title {
  font-size: 3rem;
  font-weight: 800;
  margin: 0 0 1.5rem 0;
  line-height: 1.1;
  color: #f8fafc;
}

/* CARDS ESTÁNDAR */
.tool-card {
  height: 100%;
  padding: 24px;
  border-radius: 16px;
  border: 1px solid rgba(255, 255, 255, 0.05);
  transition: transform 0.3s ease;
  background: linear-gradient(145deg, rgba(30, 41, 59, 0.6), rgba(15, 23, 42, 0.8));
  display: flex;
  flex-direction: column;
}

.tool-card:hover {
  transform: translateY(-5px);
}

.tool-header {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 12px;
}

.tool-header h2 {
  margin: 0;
  font-size: 1.5rem;
  font-weight: 700;
  color: white;
}

.tool-list {
  list-style: none;
  padding: 0;
  margin-top: 16px;
  color: #cbd5e1;
}

.tool-list li {
  margin-bottom: 8px;
  font-size: 0.95rem;
}

/* VARIANTE SPECIFICA PARA ESTA PAGINA */
.tool-card.advantages {
  border-top: 4px solid #4ade80; /* Green highlight */
}

/* GRID DE DEFINICIONES (Trouble-grid reutilizado) */
.trouble-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 16px;
}

.trouble-item {
  background: rgba(30, 41, 59, 0.4);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 12px;
  padding: 16px;
}

.trouble-q {
  font-weight: 700;
  margin-bottom: 8px;
  font-size: 0.95rem;
}

.trouble-a {
  color: #94a3b8;
  font-size: 0.85rem;
  line-height: 1.4;
}

@media (max-width: 768px) {
  .hero-title {
    font-size: 2.2rem;
  }
}
</style>
