<template>
  <q-page class="q-pa-lg column items-center">
    <!-- HERO: Propósito y Motivación -->
    <section class="intro-hero self-stretch">
      <div class="hero-content">
        <h1 class="hero-title">Introducción a <span class="accent">ROS 2</span></h1>
        <TextBlock>
          Bienvenido a <strong>UP-Ros</strong>. Esta plataforma es una
          <strong>ruta de aprendizaje estructurada</strong> y diseñada para que domines el estándar
          mundial en robótica (ROS 2 Jazzy), desde los conceptos fundamentales hasta la
          implementación en robots reales.
        </TextBlock>
      </div>
    </section>

    <!-- OBJETIVOS DE APRENDIZAJE (Pedagogía Explícita) -->
    <div class="section-group self-stretch">
      <SectionTitle>🎯 Objetivos de aprendizaje</SectionTitle>
      <div class="row q-col-gutter-md items-stretch">
        <div class="col-12 col-md-6">
          <TextBlock>
            Al completar este recorrido introductorio y la ruta UP-Ros, serás capaz de:
          </TextBlock>
          <StepsBlock :steps="learningObjectives" />
        </div>
        <div class="col-12 col-md-6">
          <AlertBlock title="🧠 Metodología UP-Ros" type="info">
            Usamos un enfoque <strong>teórico-práctico progresivo</strong>:<br />
            1. <strong>Concepto:</strong> Explicación visual clara.<br />
            2. <strong>Ejemplo:</strong> Código real reproducible.<br />
            3. <strong>Práctica:</strong> Ejercicios de simulación.<br />
            4. <strong>Proyecto:</strong> Integración en hardware.
          </AlertBlock>
        </div>
      </div>
    </div>

    <!-- GLOSARIO VISUAL (Reducción de carga cognitiva) -->
    <div class="section-group self-stretch">
      <SectionTitle>Conceptos Fundamentales (Glosario Visual)</SectionTitle>
      <div class="row q-col-gutter-lg items-stretch">
        <div class="col-12 col-md-7">
          <ImageBlock
            src="src/assets/images/Nodes-TopicandService.gif"
            caption="Comunicación en tiempo real: Nodos, Tópicos y Servicios"
            credit="ROS 2 Documentation"
            class="full-height-image"
          />
        </div>
        <div class="col-12 col-md-5 column justify-center">
          <TextBlock> Antes de programar, necesitas dominar este vocabulario esencial: </TextBlock>
          <AlertBlock title="🔑 Términos Clave" type="success">
            • <strong>Nodo:</strong> Un proceso (programa) que realiza una tarea específica (ej:
            leer cámara).<br />
            • <strong>Tópico:</strong> Canal de datos tipo "streaming" (pub/sub).<br />
            • <strong>Servicio:</strong> Comunicación sincrónica (pregunta/respuesta).<br />
            • <strong>Paquete:</strong> Contenedor de código, dependencias y configuración.
          </AlertBlock>
        </div>
      </div>
    </div>

    <!-- JUSTIFICACIÓN TECNOLÓGICA (Rigor Académico) -->
    <div class="section-group self-stretch">
      <SectionTitle>¿Por qué ROS 2 Jazzy (LTS)?</SectionTitle>
      <SplitBlock>
        <template #left>
          <TextBlock>
            ROS 2 no es una actualización menor; es una <strong>reescritura completa</strong> basada
            en el estándar industrial <strong>DDS</strong>.
          </TextBlock>
          <AlertBlock title="🚫 Limitaciones ROS 1" type="warning">
            • Sin tiempo real garantizado.<br />
            • Master centralizado (punto único de fallo).<br />
            • Inseguro por defecto (sin encriptación).
          </AlertBlock>
        </template>
        <template #right>
          <TextBlock highlighted>
            <strong>Selección Tecnológica: ROS 2 Jazzy Jalisco</strong><br />
            Elegimos esta versión por ser <strong>LTS (Long Term Support)</strong> hasta 2029,
            garantizando estabilidad académica y profesional. Ofrece: <br />•
            <strong>Tiempo Real:</strong> Crítico para control robótico. <br />•
            <strong>Seguridad (SROS2):</strong> Estándar industrial. <br />•
            <strong>Compatibilidad:</strong> Ubuntu 24.04 y 22.04.
          </TextBlock>
        </template>
      </SplitBlock>
    </div>

    <!-- GUÍA DE USO Y NAVEGACIÓN -->
    <div class="section-group self-stretch">
      <SectionTitle>¿Cómo usar UP-Ros?</SectionTitle>
      <ImageBlock
        src="src/assets/images/upros_roadmap.png"
        caption="Tu ruta de aprendizaje estructurada"
      />

      <div class="roadmap-grid row q-col-gutter-md q-mt-md">
        <div class="roadmap-step col-12 col-sm-6 col-md-3">
          <div class="step-icon">1</div>
          <h3>Fundamentos</h3>
          <p>Base conceptual sólida.</p>
        </div>
        <div class="roadmap-step col-12 col-sm-6 col-md-3">
          <div class="step-icon">2</div>
          <h3>Instalación</h3>
          <p>Entorno listo y verificado.</p>
        </div>
        <div class="roadmap-step col-12 col-sm-6 col-md-3">
          <div class="step-icon">3</div>
          <h3>Ejemplos</h3>
          <p>Código Python/C++ explicado.</p>
        </div>
        <div class="roadmap-step col-12 col-sm-6 col-md-3">
          <div class="step-icon">4</div>
          <h3>Proyecto</h3>
          <p>Integración final en robot.</p>
        </div>
      </div>
    </div>

    <!-- VALIDACIÓN INICIAL (Práctica inmediata) -->
    <div class="section-group self-stretch">
      <SectionTitle>Validación Inmediata</SectionTitle>
      <div class="row q-col-gutter-lg items-center">
        <div class="col-12 col-md-6">
          <TextBlock>
            Para garantizar que estás listo, ejecuta este comando en tu terminal. Si funciona, tu
            entorno tiene las herramientas básicas.
          </TextBlock>
          <AlertBlock title="✅ Checklist de entrada" type="success">
            • Ubuntu 22.04 / 24.04 instalado.<br />
            • Terminal abierta.<br />
            • Conexión a internet activa.
          </AlertBlock>
        </div>
        <div class="col-12 col-md-6">
          <CodeBlock title="Prueba de entorno" lang="bash" :content="checkCommand" />
          <div class="text-caption text-grey-5 q-mt-sm text-center">
            Este comando verifica versiones de Python y compiladores.
          </div>
        </div>
      </div>
    </div>

    <!-- CTA / NAVEGACIÓN FINAL -->
    <div class="cta-final row justify-center q-mt-xl q-mb-lg">
      <q-btn
        color="primary"
        unelevated
        size="lg"
        to="/instalacion"
        icon="download"
        icon-after
        class="q-mr-lg"
      >
        🚀 Ir a Instalación
      </q-btn>
      <q-btn flat icon="arrow_back" label="Volver al Home" @click="$router.go(-1)" />
    </div>

    <!-- MATERIAL DESCARGABLE -->
    <div class="section-group self-stretch q-mb-none">
      <PdfBlock label="📥 Descargar Syllabus y Roadmap (PDF)" src="src/assets/upros-roadmap.pdf" />
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
  'Diferenciar patrones de comunicación (Tópicos vs Servicios vs Acciones).',
  'Configurar un entorno de desarrollo profesional en Ubuntu.',
  'Desarrollar y depurar paquetes ROS 2 en Python.',
  'Integrar simulación y hardware real en proyectos robóticos.',
];

const checkCommand = `
# Verifica que tienes las herramientas base
python3 --version
gcc --version
git --version
echo "¡Sistema base listo para instalar ROS 2!"`.trim();
</script>

<style scoped>
/* =========================================
   ESTRUCTURA GENERAL (Homogeneidad)
========================================= */

/* Hero y Secciones comparten ancho máximo y centrado */
.intro-hero,
.section-group {
  width: 100%;
  max-width: 1100px;
  margin: 0 auto 3rem auto;
}

.section-group > * + * {
  margin-top: 1.5rem;
}

/* =========================================
   ESTILOS ESPECÍFICOS
======================================== */

/* HERO */
.intro-hero {
  padding: 2.5rem;
  background: rgba(15, 23, 42, 0.85);
  backdrop-filter: blur(20px);
  border-radius: 20px;
  border: 1px solid rgba(148, 163, 184, 0.3);
  text-align: center;
}

.hero-title {
  font-size: 2.75rem;
  font-weight: 800;
  margin-bottom: 1.5rem;
  line-height: 1.2;
  background: linear-gradient(135deg, #6ecbff 0%, #ffffff 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.accent {
  background: linear-gradient(135deg, #6ecbff, #a5b4fc);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
}

/* IMAGEN AJUSTADA */
:deep(.full-height-image .image-container img) {
  max-height: 380px;
  width: 100%;
  object-fit: contain;
  background: rgba(0, 0, 0, 0.1);
}

/* ROADMAP STEPS */
.roadmap-step {
  background: rgba(20, 30, 50, 0.8);
  border-radius: 16px;
  padding: 1.5rem;
  text-align: center;
  border: 1px solid rgba(148, 163, 184, 0.2);
  transition: all 0.3s ease;
  height: 100%;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
}

.roadmap-step:hover {
  transform: translateY(-4px);
  border-color: #6ecbff;
  box-shadow: 0 12px 32px rgba(110, 203, 255, 0.15);
}

.step-icon {
  width: 48px;
  height: 48px;
  background: linear-gradient(135deg, #6ecbff, #a5b4fc);
  color: white;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.25rem;
  font-weight: 700;
  margin: 0 auto 1rem;
}

.roadmap-step h3 {
  color: #e6edf7;
  font-size: 1.2rem;
  margin-bottom: 0.5rem;
  font-weight: 600;
}

.roadmap-step p {
  color: #9ca3af;
  line-height: 1.4;
  margin: 0;
  font-size: 0.95rem;
}

.cta-final {
  max-width: 600px;
  gap: 1.5rem;
  margin: 4rem auto 2rem auto !important;
}

/* RESPONSIVE */
@media (max-width: 768px) {
  .hero-title {
    font-size: 2rem;
  }
  .intro-hero {
    padding: 1.5rem;
    margin-bottom: 2rem;
  }
  .section-group {
    margin-bottom: 2rem;
  }
}
</style>
